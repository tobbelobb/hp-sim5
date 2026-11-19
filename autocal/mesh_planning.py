"""Sphere-sector meshes as a sweep of sweeps: hold one line, step two drives.

Holding one line keeps the effector on a sphere around its anchor. A mesh holds
that line (D, or the top line on HP5) and collects several rows: each row also
holds a first drive line at one level, so it is an ordinary circle sweep in which
the second drive steps the effector and the remaining lines measure under force.
Each row's lengths trace an ellipse, and all rows of a mesh lie on one ellipsoid:
the ellipse reference fit fits that ellipsoid, the point fit every point.

A mesh needs one positioning of the held line instead of a sweep's positioning,
pullout and reversal per circle. In synthetic HP4 tests with exactly held lines,
meshes gave about 35% smaller anchor errors than circle sweeps with the same
number of points.

The planner keeps every commanded and predicted length inside the lengths already
measured on that axis, keeps the sweep planner's pretension and anchor clearance
rules, lets the three position lines fix the pose, sets each point's sensor force
for equal strain below MESH_MAX_SENSOR_FORCE_N, and picks rows by information.
"""
from __future__ import annotations

import itertools
from typing import Dict, List, Optional

import numpy as np

from autocal.active_learning import SweepConfig
from autocal.dataset_roles import measurement_groups
from autocal.equal_strain import PROFILE_STRAIN_MODEL, equal_strain_tensions, force_planning_parameters
from autocal.pose_fit import HOME_OFFSET_PRIOR_SIGMA_MM, dataset_line_lengths
from autocal.pose_planning import (ANCHOR_CLEARANCE_SHARE, SWEEP_PRETENSION_SHARE, _fixed_length_circle,
                                   point_informations, pose_information)
from autocal.sweep_types import MachineConfig, MachineType

ROWS = 4
ROW_POINTS = 5
# Equal-strain sensor forces above this are not planned. Ordinary line forces on a
# Hangprinter; sweeps' own force profiles carry no ceiling.
MESH_MAX_SENSOR_FORCE_N = 16.0
# A mesh is planned and kept only with at least this many points.
MIN_MESH_POINTS = 10
# Meshes extend sweeps; the first sweeps establish forces, ranges and the rough geometry.
MIN_SWEEPS_BEFORE_MESHES = 3
_HELD_LENGTHS = 7
_ROW_LEVELS = 9
# Smallest singular value of the three position lines' unit directions. Below it
# the pose is poorly fixed, as when the three anchors and the effector are coplanar.
_POSITION_CONDITION = 0.25


def mesh_key(mesh: dict):
    """Identity of a planned mesh, to skip ones the collector found unmeasurable."""
    return (int(mesh["held_anchor"]), round(float(mesh["held_length_mm"]), 1),
            tuple(int(axis) for axis in mesh["drive_anchors"]), int(mesh["sensor_anchor"]))


def _spool_radius_mm(dataset: dict, axis: int) -> float:
    config = dataset.get("config") or {}
    m666 = config.get("m666_adjusted_by_data_collector") or config.get("m666_before_data_collection") or {}
    radius = m666.get("R")
    radius = radius[axis] if isinstance(radius, list) else radius
    return float(radius) if isinstance(radius, (int, float)) and radius > 0 else 30.0


def _greedy_rows(base: np.ndarray, row_informations: List[np.ndarray], count: int):
    """Rows that greedily maximise log det(base + their summed information)."""
    total, chosen = base.copy(), []
    for _ in range(count):
        gains = [np.linalg.slogdet(total + information)[1] if index not in chosen else -np.inf
                 for index, information in enumerate(row_informations)]
        best = int(np.argmax(gains))
        if not np.isfinite(gains[best]):
            break
        chosen.append(best)
        total = total + row_informations[best]
    return chosen, float(np.linalg.slogdet(total)[1] - np.linalg.slogdet(base)[1])


def _serpentine(rows: List[List[dict]]) -> List[List[dict]]:
    """Rows in level order from the end nearest home, alternating direction so each
    row starts beside where the previous one ended."""
    rows = sorted(rows, key=lambda row: row[0]["drive_mm"][0])
    nearest = lambda row: min(np.linalg.norm(point["position_mm"]) for point in row)
    if nearest(rows[-1]) < nearest(rows[0]):
        rows = rows[::-1]
    ordered, end = [], np.zeros(3)
    for row in rows:
        if np.linalg.norm(np.asarray(row[-1]["position_mm"]) - end) < np.linalg.norm(np.asarray(row[0]["position_mm"]) - end):
            row = row[::-1]
        ordered.append(row)
        end = np.asarray(row[-1]["position_mm"])
    return ordered


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
    rows: int = ROWS,
    row_points: int = ROW_POINTS,
) -> Optional[Dict[str, object]]:
    """The most informative feasible mesh of circle-sweep rows, or None.

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
    minimum_force = force.pop("minimum_force_n")
    measure_extras = bool(measure_extra_ranges and extra_force is not None)
    if not measure_extras:
        # Unmeasured spare lines stay compliant at the low force during collection.
        force["compliant_force_n"] = float((dataset["config"].get("force_tuning") or {}).get("force_low_n", 0.0))

    raw = dataset_line_lengths(dataset)["lengths_mm"]
    measured_range = np.column_stack([np.nanmin(raw, axis=0), np.nanmax(raw, axis=0)])
    options = dict(machine_type=machine_type, low_anchor_z=low_anchor_z, sigma_mm=sigma_mm)
    base = pose_information(anchors, dataset_line_lengths(estimation_dataset)["lengths_mm"],
                            stretch=observed_stretch, **options)
    prior = np.full(base.shape[0], float(regularization))
    prior[base.shape[0] - (num_anchors - dimensions):] += 1.0 / HOME_OFFSET_PRIOR_SIGMA_MM ** 2
    base = base + np.diag(prior)
    home = np.linalg.norm(anchors, axis=1)
    held_anchors = MachineConfig.from_type(MachineType(machine_type)).must_be_fixed_anchors or list(range(num_anchors))
    rejected = {mesh_key(entry["mesh"]) for entry in dataset.get("acquisition_rejections", []) if entry.get("mesh")}

    def row(held, held_raw, level_drive, level_raw, step_drive, sensor, checked):
        """One circle-sweep row: feasible points evenly over its longest feasible arc."""
        cfg = SweepConfig(fixed_anchors=(held, level_drive), fixed_deltas_mm=(held_raw + offsets[held],
                          level_raw + offsets[level_drive]), drive_anchor=step_drive, sensor_anchor=sensor)
        positions = _fixed_length_circle(anchors, cfg)
        if positions.shape[0] == 0:
            return None
        predicted = np.linalg.norm(positions[:, None, :] - anchors[None], axis=2) - home - offsets
        keep = np.all((predicted[:, checked] >= measured_range[checked, 0])
                      & (predicted[:, checked] <= measured_range[checked, 1]), axis=1)
        keep &= np.all(np.linalg.norm(positions[:, None, :] - anchors[None], axis=2) / home >= ANCHOR_CLEARANCE_SHARE, axis=1)
        lines = anchors[[held, level_drive, step_drive]][None] - positions[:, None, :]
        keep &= np.linalg.svd(lines / np.linalg.norm(lines, axis=2)[:, :, None], compute_uv=False)[:, -1] >= _POSITION_CONDITION
        active = [held, level_drive, step_drive, sensor]
        if keep.any():
            directions = anchors[active][None] - positions[keep][:, None, :]
            directions /= np.linalg.norm(directions, axis=2)[:, :, None]
            pretension = np.linalg.svd(np.transpose(directions, (0, 2, 1)))[2][:, -1, :]
            pretension *= np.sign(np.sum(pretension, axis=1))[:, None]
            keep[keep] = np.min(pretension, axis=1) >= SWEEP_PRETENSION_SHARE * np.max(pretension, axis=1)
        forces = np.full(len(positions), np.nan)
        for index in np.flatnonzero(keep):
            tensions = equal_strain_tensions(anchors, positions[index], active, **force)
            if np.all(np.isfinite(tensions) & (tensions > 0)) and minimum_force <= tensions[-1] <= MESH_MAX_SENSOR_FORCE_N:
                forces[index] = tensions[-1]
        # The longest feasible arc on the closed circle, allowed to wrap around.
        feasible = np.isfinite(forces)
        if not feasible.any():
            return None
        shift = int(np.argmin(feasible)) if not feasible.all() else 0
        rolled = np.roll(np.arange(len(feasible)), -shift)
        indices = rolled[feasible[rolled]]
        breaks = np.flatnonzero(np.diff(np.flatnonzero(feasible[rolled])) > 1) + 1
        run = max(np.split(indices, breaks), key=len)
        if len(run) < row_points:
            return None
        chosen = run[np.round(np.linspace(0, len(run) - 1, row_points)).astype(int)]
        return [{"drive_mm": [float(level_raw), float(predicted[index, step_drive])],
                 "sensor_force_n": float(forces[index]),
                 "predicted_mm": [float(value) for value in predicted[index]],
                 "position_mm": [float(value) for value in positions[index]]} for index in chosen]

    best: Optional[Dict[str, object]] = None
    for held in held_anchors:
        low, high = measured_range[held]
        high = min(high, 0.0)  # Held lines never pay out beyond their encoder origin.
        if not np.isfinite(low) or high <= low:
            continue
        others = [axis for axis in range(num_anchors) if axis != held]
        for sensor in others:
            for level_drive, step_drive in itertools.permutations([axis for axis in others if axis != sensor], 2):
                extras = [axis for axis in others if axis not in (sensor, level_drive, step_drive)]
                if 4 != dimensions + 1:
                    continue
                checked = [held, level_drive, step_drive, sensor, *(extras if measure_extras else [])]
                level_low, level_high = measured_range[level_drive]
                for held_raw in np.linspace(low, high, _HELD_LENGTHS):
                    if mesh_key(dict(held_anchor=held, held_length_mm=held_raw,
                                     drive_anchors=(level_drive, step_drive), sensor_anchor=sensor)) in rejected:
                        continue
                    candidates = [r for level in np.linspace(level_low, level_high, _ROW_LEVELS + 2)[1:-1]
                                  if (r := row(held, held_raw, level_drive, level, step_drive, sensor, checked))]
                    if len(candidates) * row_points < MIN_MESH_POINTS:
                        continue
                    informations = []
                    for r in candidates:
                        lengths = np.full((len(r), num_anchors), np.nan)
                        lengths[:, checked] = (np.array([point["predicted_mm"] for point in r]) + offsets)[:, checked]
                        informations.append(np.sum(point_informations(
                            anchors, lengths, np.array([point["position_mm"] for point in r]), **options), axis=0))
                    chosen, gain = _greedy_rows(base, informations, min(rows, len(candidates)))
                    if len(chosen) * row_points < MIN_MESH_POINTS or (best is not None and gain <= best["gain"]):
                        continue
                    ordered = _serpentine([candidates[index] for index in chosen])
                    best = {
                        "gain": gain, "held_anchor": int(held), "held_length_mm": float(held_raw),
                        "drive_anchors": [int(level_drive), int(step_drive)], "sensor_anchor": int(sensor),
                        "extra_anchors": [int(axis) for axis in extras] if measure_extras else [],
                        "extra_sensor_force_n": extra_force if measure_extras else None,
                        "compliant_anchors": [] if measure_extras else [int(axis) for axis in extras],
                        "strand_tension_n": float(force["strand_tension_n"]), "strain_model": PROFILE_STRAIN_MODEL,
                        "max_sensor_force_n": MESH_MAX_SENSOR_FORCE_N, "min_points": MIN_MESH_POINTS,
                        "probe_radius_mm": float(np.pi * _spool_radius_mm(dataset, step_drive) / 4.0),
                        "points": [{**point, "row": row_index} for row_index, r in enumerate(ordered) for point in r],
                    }
    return best
