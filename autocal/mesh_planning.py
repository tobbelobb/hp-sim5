"""Sphere-sector meshes: hold one line, drive two, measure the rest.

Holding one line keeps the effector on a sphere around its anchor. Two more lines
in position control pick a point on that sphere, and the remaining lines measure
it under force control. One mesh therefore spreads its points over a surface,
where a sweep holding two lines can only follow a circle, and it needs one
positioning instead of a sweep's positioning, pullout and reversal. In synthetic
HP4 tests with exactly held lines, meshes gave about 35% smaller anchor errors
than circle sweeps with the same number of points.

Every planned pose, including poses along each move and the coupling probe, keeps
the commanded and predicted lengths inside the lengths already measured on that
axis, keeps the sweep planner's pretension and anchor clearance rules, lets the
three position lines fix the pose well, and keeps all line tensions positive at
the sensor force in effect. Point sensor forces are set for equal strain and never
exceed the largest sensor force the dataset already applied. Points are picked
greedily by information and visited as a closed tour from the point nearest home.

Poses are planned with rigid lengths. Line stretch moves the reached pose by a
fraction of a millimetre; measurements record the encoders, and the fit corrects
their stretch.
"""
from __future__ import annotations

import itertools
from typing import Dict, List, Optional

import numpy as np

from autocal.dataset_roles import measurement_groups
from autocal.equal_strain import (PROFILE_STRAIN_MODEL, equal_strain_tensions, force_planning_parameters,
                                  tensions_at_sensor_force)
from autocal.pose_fit import HOME_OFFSET_PRIOR_SIGMA_MM, dataset_line_lengths
from autocal.pose_planning import (ANCHOR_CLEARANCE_SHARE, SWEEP_PRETENSION_SHARE, point_informations,
                                   pose_information)
from autocal.sweep_types import MachineConfig, MachineType

MESH_POINTS = 20
# A mesh with fewer feasible points than this is not planned, and a collector
# that loses sensor coupling earlier rejects the mesh.
MIN_MESH_POINTS = 10
# Meshes extend sweeps; the first sweeps establish forces, ranges and the rough geometry.
MIN_SWEEPS_BEFORE_MESHES = 3
# Only a few percent of a held line's sphere lies in the measured workspace.
_CLOUD_POINTS = 20000
_HELD_LENGTHS = 7
# Smallest singular value of the three position lines' unit directions. Below it
# the pose is poorly fixed, as when the three anchors and the effector are coplanar.
_POSITION_CONDITION = 0.25
# Poses checked along each straight move in commanded drive lengths.
_EDGE_SAMPLES = 8


def _sphere_directions(count: int) -> np.ndarray:
    """Evenly spread unit vectors (Fibonacci lattice)."""
    index = np.arange(count) + 0.5
    polar = np.arccos(1.0 - 2.0 * index / count)
    azimuth = np.pi * (1.0 + 5.0 ** 0.5) * index
    return np.column_stack([np.cos(azimuth) * np.sin(polar), np.sin(azimuth) * np.sin(polar), np.cos(polar)])


def _pretension_shares(anchors: np.ndarray, positions: np.ndarray, lines: List[int]) -> np.ndarray:
    """Smallest line's share of the largest in the d+1 lines' self-stress, per position."""
    directions = anchors[lines][None] - positions[:, None, :]
    directions /= np.linalg.norm(directions, axis=2)[:, :, None]
    pretension = np.linalg.svd(np.transpose(directions, (0, 2, 1)))[2][:, -1, :]
    pretension *= np.sign(np.sum(pretension, axis=1))[:, None]
    return np.min(pretension, axis=1) / np.max(pretension, axis=1)


def _greedy_points(base: np.ndarray, informations: np.ndarray, count: int):
    """Indices that greedily maximise log det(base + sum of chosen informations)."""
    total = base.copy()
    chosen: List[int] = []
    available = np.ones(len(informations), dtype=bool)
    for _ in range(count):
        # Determinant lemma: each gain is log det(I + total^-1 F_i), batched.
        relative = np.linalg.solve(total, informations)
        gains = np.linalg.slogdet(np.eye(total.shape[0]) + relative)[1]
        gains[~available] = -np.inf
        best = int(np.argmax(gains))
        if not np.isfinite(gains[best]):
            break
        chosen.append(best)
        available[best] = False
        total = total + informations[best]
    return chosen, float(np.linalg.slogdet(total)[1] - np.linalg.slogdet(base)[1])


def _path_order(positions: np.ndarray) -> List[int]:
    """A short closed tour from the point nearest home back to it (nearest neighbour, then 2-opt).

    The collector returns to the first point before going home, because the
    one-motor-at-a-time return to origin is only reliable from poses near home.
    """
    start = int(np.argmin(np.linalg.norm(positions, axis=1)))
    distance = np.linalg.norm(positions[:, None, :] - positions[None], axis=2)
    tour, left = [start], set(range(len(positions))) - {start}
    while left:
        tour.append(min(left, key=lambda index: distance[tour[-1], index]))
        left.remove(tour[-1])
    improved = True
    while improved:
        improved = False
        for i in range(1, len(tour) - 1):
            for j in range(i + 1, len(tour)):
                a, b, c, d = tour[i - 1], tour[i], tour[j], tour[(j + 1) % len(tour)]
                if distance[a, c] + distance[b, d] < distance[a, b] + distance[c, d] - 1e-9:
                    tour[i:j + 1] = tour[i:j + 1][::-1]
                    improved = True
    return tour


def mesh_key(mesh: dict):
    """Identity of a planned mesh, to skip ones the collector found unmeasurable."""
    return (int(mesh["held_anchor"]), round(float(mesh["held_length_mm"]), 1),
            tuple(int(axis) for axis in mesh["drive_anchors"]), int(mesh["sensor_anchor"]))


class _Feasibility:
    """Pose checks shared by candidate points and the poses along each planned move."""

    def __init__(self, anchors, offsets, measured_range, held, drives, sensor, checked, force, max_sensor_force_n):
        self.anchors, self.offsets, self.range = anchors, offsets, measured_range
        self.home = np.linalg.norm(anchors, axis=1)
        self.held, self.drives, self.sensor, self.checked = held, list(drives), sensor, checked
        self.active = [held, *drives, sensor]
        self.force, self.max_sensor_force_n = force, max_sensor_force_n

    def predicted(self, positions: np.ndarray) -> np.ndarray:
        """Encoder lengths of every line at the positions."""
        return np.linalg.norm(positions[:, None, :] - self.anchors[None], axis=2) - self.home - self.offsets

    def geometric(self, positions: np.ndarray) -> np.ndarray:
        """Range, clearance, position-line conditioning and pretension checks."""
        predicted = self.predicted(positions)
        keep = np.all((predicted[:, self.checked] >= self.range[self.checked, 0])
                      & (predicted[:, self.checked] <= self.range[self.checked, 1]), axis=1)
        clearance = np.linalg.norm(positions[:, None, :] - self.anchors[None], axis=2) / self.home
        keep &= np.all(clearance >= ANCHOR_CLEARANCE_SHARE, axis=1)
        position_lines = self.anchors[[self.held, *self.drives]][None] - positions[:, None, :]
        position_lines /= np.linalg.norm(position_lines, axis=2)[:, :, None]
        keep &= np.linalg.svd(position_lines, compute_uv=False)[:, -1] >= _POSITION_CONDITION
        if np.any(keep):
            keep[keep] = _pretension_shares(self.anchors, positions[keep], self.active) >= SWEEP_PRETENSION_SHARE
        return keep

    def equal_strain_force(self, position: np.ndarray) -> Optional[float]:
        """The point's equal-strain sensor force, or None when infeasible or above the applied ceiling."""
        tensions = equal_strain_tensions(self.anchors, position, self.active, **{
            key: value for key, value in self.force.items() if key != "minimum_force_n"})
        if (np.all(np.isfinite(tensions) & (tensions > 0.0)) and tensions[-1] >= self.force["minimum_force_n"]
                and tensions[-1] <= self.max_sensor_force_n):
            return float(tensions[-1])
        return None

    def pose_for(self, held_length: float, drive_lengths, guess) -> np.ndarray:
        """Effector position where the three position lines reach their commanded lengths."""
        lines = np.array([self.held, *self.drives])
        target = self.home[lines] + self.offsets[lines] + np.array([held_length, *drive_lengths])
        position = np.asarray(guess, dtype=float).copy()
        for _ in range(30):
            offset = position - self.anchors[lines]
            distance = np.linalg.norm(offset, axis=1)
            step = np.linalg.lstsq(offset / distance[:, None], target - distance, rcond=None)[0]
            position += step
            if np.linalg.norm(step) < 1e-9:
                break
        return position

    def move_feasible(self, held_length, start, end, sensor_force_n, guess) -> bool:
        """Every pose along a straight move in drive lengths, under the sensor force set before the move."""
        position = np.asarray(guess, dtype=float)
        start, end = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
        for fraction in np.linspace(0.0, 1.0, _EDGE_SAMPLES + 2)[1:]:
            position = self.pose_for(held_length, start + fraction * (end - start), position)
            if not self.geometric(position[None])[0]:
                return False
            tensions = tensions_at_sensor_force(self.anchors, position, self.active, sensor_force_n=sensor_force_n,
                                                mover_weight_n=self.force["mover_weight_n"],
                                                compliant_force_n=self.force["compliant_force_n"])
            if not np.all(np.isfinite(tensions) & (tensions > 0.0)):
                return False
        return True

    def tour_feasible(self, held_length, planned) -> bool:
        """Each move of the tour, and the move back to its first point."""
        for previous, current in zip(planned, planned[1:] + planned[:1]):
            if not self.move_feasible(held_length, previous["drive_mm"], current["drive_mm"],
                                      current["sensor_force_n"], previous["position_mm"]):
                return False
        return True

    def probe_targets(self, held_length, first, radius_mm) -> Optional[Dict[str, float]]:
        """Coupling-probe targets for the second drive at the first point, shrunk until both moves are feasible."""
        drive = np.asarray(first["drive_mm"], dtype=float)
        for radius in (radius_mm, radius_mm / 2.0, radius_mm / 4.0):
            moves = [drive + np.array([0.0, sign * radius]) for sign in (1.0, -1.0)]
            if all(self.move_feasible(held_length, drive, moved, first["sensor_force_n"], first["position_mm"])
                   and self.move_feasible(held_length, moved, drive, first["sensor_force_n"],
                                          self.pose_for(held_length, moved, first["position_mm"]))
                   for moved in moves):
                return {"away_mm": float(moves[0][1]), "toward_mm": float(moves[1][1])}
        return None


def _spool_radius_mm(dataset: dict, axis: int) -> float:
    """The collector's coupling-probe step uses this spool radius."""
    config = dataset.get("config") or {}
    m666 = config.get("m666_adjusted_by_data_collector") or config.get("m666_before_data_collection") or {}
    radius = m666.get("R")
    radius = radius[axis] if isinstance(radius, list) else radius
    return float(radius) if isinstance(radius, (int, float)) and radius > 0 else 30.0


def plan_sphere_mesh(
    anchors: np.ndarray,
    dataset: dict,
    estimation_dataset: dict,
    *,
    machine_type: str,
    home_offsets_mm: np.ndarray,
    low_anchor_z: Optional[float],
    sigma_mm: float,
    regularization: float,
    measure_extra_ranges: bool,
    observed_stretch=None,
    points: int = MESH_POINTS,
) -> Optional[Dict[str, object]]:
    """The most informative feasible sphere mesh, or None when none fits the measured workspace.

    `dataset` holds the raw encoder lengths that bound the workspace and the
    force tuning; `estimation_dataset` the offset-corrected lengths the anchors
    were fitted to. Commanded lengths are returned in encoder coordinates.
    """
    anchors = np.asarray(anchors, dtype=float)
    num_anchors, dimensions = anchors.shape
    offsets = np.asarray(home_offsets_mm, dtype=float)
    force = force_planning_parameters(dataset)
    if force is None or len(measurement_groups(dataset)) < MIN_SWEEPS_BEFORE_MESHES:
        return None
    extra_force = force.pop("extra_sensor_force_n")
    measure_extras = bool(measure_extra_ranges and extra_force is not None)
    if not measure_extras:
        # Unmeasured spare lines stay compliant at the low force during collection.
        force["compliant_force_n"] = float((dataset["config"].get("force_tuning") or {}).get("force_low_n", 0.0))
    applied = [point.get("sensor_force_n") for sweep in dataset.get("sweeps", []) for point in sweep.get("data_points", [])]
    applied = [float(value) for value in applied if isinstance(value, (int, float)) and np.isfinite(value)]
    if not applied:
        return None
    # Never command a sensor load this dataset has not already applied.
    max_sensor_force_n = max(applied)

    raw = dataset_line_lengths(dataset)["lengths_mm"]
    measured_range = np.column_stack([np.nanmin(raw, axis=0), np.nanmax(raw, axis=0)])
    options = dict(machine_type=machine_type, low_anchor_z=low_anchor_z, sigma_mm=sigma_mm)
    base = pose_information(anchors, dataset_line_lengths(estimation_dataset)["lengths_mm"],
                            stretch=observed_stretch, **options)
    prior = np.full(base.shape[0], float(regularization))
    prior[base.shape[0] - (num_anchors - dimensions):] += 1.0 / HOME_OFFSET_PRIOR_SIGMA_MM ** 2
    base = base + np.diag(prior)

    home = np.linalg.norm(anchors, axis=1)
    directions = _sphere_directions(_CLOUD_POINTS)
    held_anchors = MachineConfig.from_type(MachineType(machine_type)).must_be_fixed_anchors or list(range(num_anchors))
    rejected = {mesh_key(entry["mesh"]) for entry in dataset.get("acquisition_rejections", []) if entry.get("mesh")}
    best: Optional[Dict[str, object]] = None
    for held in held_anchors:
        low, high = measured_range[held]
        # Held lines never pay out beyond their encoder origin.
        high = min(high, 0.0)
        if not np.isfinite(low) or high <= low:
            continue
        others = [axis for axis in range(num_anchors) if axis != held]
        for sensor in others:
            for drives in itertools.combinations([axis for axis in others if axis != sensor], 2):
                extras = [axis for axis in others if axis != sensor and axis not in drives]
                if len(drives) + 2 != dimensions + 1:
                    continue
                checked = [held, *drives, sensor, *(extras if measure_extras else [])]
                feasibility = _Feasibility(anchors, offsets, measured_range, held, drives, sensor, checked,
                                           force, max_sensor_force_n)
                for held_raw in np.linspace(low, high, _HELD_LENGTHS):
                    if mesh_key(dict(held_anchor=held, held_length_mm=held_raw, drive_anchors=drives,
                                     sensor_anchor=sensor)) in rejected:
                        continue
                    cloud = anchors[held] + (home[held] + held_raw + offsets[held]) * directions
                    candidates, forces = [], []
                    for index in np.flatnonzero(feasibility.geometric(cloud)):
                        sensor_force = feasibility.equal_strain_force(cloud[index])
                        if sensor_force is not None:
                            candidates.append(int(index))
                            forces.append(sensor_force)
                    if len(candidates) < MIN_MESH_POINTS:
                        continue
                    predicted = feasibility.predicted(cloud[candidates])
                    lengths = np.full((len(candidates), num_anchors), np.nan)
                    lengths[:, checked] = (predicted + offsets)[:, checked]
                    informations = point_informations(anchors, lengths, cloud[candidates], **options)
                    chosen, gain = _greedy_points(base, informations, min(points, len(candidates)))
                    if len(chosen) < MIN_MESH_POINTS or (best is not None and gain <= best["gain"]):
                        continue
                    order = [chosen[index] for index in _path_order(cloud[[candidates[i] for i in chosen]])]
                    planned = [{
                        "drive_mm": [float(predicted[index, axis]) for axis in drives],
                        "sensor_force_n": forces[index],
                        "predicted_mm": [float(value) for value in predicted[index]],
                        "position_mm": [float(value) for value in cloud[candidates[index]]],
                    } for index in order]
                    if not feasibility.tour_feasible(held_raw, planned):
                        continue
                    probe = feasibility.probe_targets(held_raw, planned[0],
                                                      np.pi * _spool_radius_mm(dataset, drives[1]) / 4.0)
                    if probe is None:
                        continue
                    best = {
                        "gain": gain, "held_anchor": int(held), "held_length_mm": float(held_raw),
                        "drive_anchors": [int(axis) for axis in drives], "sensor_anchor": int(sensor),
                        "extra_anchors": [int(axis) for axis in extras] if measure_extras else [],
                        "extra_sensor_force_n": extra_force if measure_extras else None,
                        "compliant_anchors": [] if measure_extras else [int(axis) for axis in extras],
                        "strand_tension_n": float(force["strand_tension_n"]), "strain_model": PROFILE_STRAIN_MODEL,
                        "max_sensor_force_n": max_sensor_force_n, "min_points": MIN_MESH_POINTS,
                        "probe": probe, "points": planned,
                    }
    return best
