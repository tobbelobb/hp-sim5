"""Sensor force targeting a constant weighted mean tension per strand.

At a position the active d+1 line tensions are fixed by gravity and the
compliant lines' loads up to one free amount of pretension. The torque-controlled
sensor line sets that amount.
Stretch biases the fit through the pretension-weighted mean strain of the lines.
The target makes shared compliance resemble a common scale plus a home length
offset in the first-order model, without using nominal stiffness. Actual strain
also depends on force tracking, friction, routing and unequal compliance.
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, List, Sequence

import numpy as np

from autocal.active_learning import SweepConfig, dataset_sweep_configs
from autocal.pose_planning import predict_sweep_positions
from autocal.projection_mode import resolve_projection_mode

GRAVITY_M_PER_S2 = 9.81
# Predict a broad arc; collection is restricted to the resulting feasible table.
_PROFILE_PRETENSION_SHARE = 0.05
_PROFILE_POINTS = 60
# The collector's constant measurement force is this multiple of the tuned start force.
_COLLECTION_FORCE_MULTIPLE = 5.0
# Planned line tensions stay below this share of the firmware's max planned force
# (M666 X). The multiple above scales with the measured breakaway, which motor
# ripple or stiction can inflate; held motors then slip or deflect.
_MAX_PLANNED_FORCE_SHARE = 0.75
# A capped strand target must leave every planned sweep this share of its poses.
_CAPPED_POSE_SHARE = 0.5
PROFILE_STRAIN_MODEL = "reeving-aware-v2"


class InfeasibleForceProfileError(ValueError):
    """A candidate has no usable interval under the requested line loads."""


def _carrying_and_pretension(anchors, position, lines, mover_weight_n, compliant_force_n):
    """Static line tensions of the d+1 active lines: one particular solution and the self-stress mode."""
    if len(lines) != np.asarray(anchors).shape[1] + 1:
        raise ValueError("Equal-strain sweeps require d+1 active lines with known loads on the remaining lines")
    to_anchor = np.asarray(anchors, dtype=float)[lines] - np.asarray(position, dtype=float)
    distance = np.linalg.norm(to_anchor, axis=1)
    directions = (to_anchor / distance[:, None]).T
    load = np.zeros(directions.shape[0])
    load[-1] = mover_weight_n
    # Redundant lines hold a known force during measurement. Their
    # known load removes the extra pretension freedoms before solving the
    # active d+1 lines' single self-stress mode.
    unused = [axis for axis in range(len(anchors)) if axis not in lines]
    if unused:
        passive = np.asarray(anchors, dtype=float)[unused] - np.asarray(position, dtype=float)
        load -= compliant_force_n * np.sum(passive / np.linalg.norm(passive, axis=1)[:, None], axis=0)
    carrying = np.linalg.lstsq(directions, load, rcond=None)[0]
    pretension = np.linalg.svd(directions)[2][-1]
    return carrying, pretension * np.sign(np.sum(pretension)), distance


def tensions_at_sensor_force(anchors, position, lines, *, sensor_force_n: float, mover_weight_n: float,
                             compliant_force_n: float = 0.0) -> np.ndarray:
    """Static tensions of the active lines when the last one (the sensor) pulls with `sensor_force_n`."""
    carrying, pretension, _ = _carrying_and_pretension(anchors, position, list(lines), mover_weight_n, compliant_force_n)
    return carrying + (sensor_force_n - carrying[-1]) / pretension[-1] * pretension


def equal_strain_tensions(
    anchors: np.ndarray,
    position: np.ndarray,
    lines: Sequence[int],
    *,
    strand_tension_n: float,
    mover_weight_n: float,
    strands: np.ndarray,
    extra_line_length_mm: np.ndarray,
    mechanical_advantage: np.ndarray | None = None,
    compliant_force_n: float = 0.0,
) -> np.ndarray:
    """Tensions of the given lines at a position whose weighted mean tension per strand is `strand_tension_n`."""
    lines = list(lines)
    carrying, pretension, distance = _carrying_and_pretension(anchors, position, lines, mover_weight_n, compliant_force_n)
    advantage = np.ones(len(anchors)) if mechanical_advantage is None else np.broadcast_to(
        np.asarray(mechanical_advantage, dtype=float), (len(anchors),))
    advantage = advantage[lines]
    # Firmware flex is F*(U*distance+Y)/(EA*U²*N). The common-strain
    # invariant therefore uses effective N*U and carriage-equivalent Y/U.
    strands = np.asarray(strands, dtype=float)[lines] * advantage
    weight = pretension * (distance + np.asarray(extra_line_length_mm, dtype=float)[lines] / advantage)
    amount = (strand_tension_n * np.sum(weight) - np.sum(weight * carrying / strands)) / np.sum(weight * pretension / strands)
    return carrying + amount * pretension


def sensor_force_profiles(
    anchors: np.ndarray,
    cfg: SweepConfig,
    *,
    strand_tension_n: float,
    mover_weight_n: float,
    strands: np.ndarray,
    extra_line_length_mm: np.ndarray,
    minimum_force_n: float,
    mechanical_advantage: np.ndarray | None = None,
    compliant_force_n: float = 0.0,
    maximum_force_n: np.ndarray | None = None,
) -> List[Dict[str, object]]:
    """Sensor force against drive length change, for both directions of a sweep."""
    anchors = np.asarray(anchors, dtype=float)
    positions = predict_sweep_positions(anchors, cfg, _PROFILE_POINTS, _PROFILE_PRETENSION_SHARE)
    lines = [*cfg.fixed_anchors, cfg.drive_anchor, cfg.sensor_anchor]
    tensions = _planned_tensions(anchors, positions, lines, strand_tension_n=strand_tension_n,
                                 mover_weight_n=mover_weight_n, strands=strands,
                                 extra_line_length_mm=extra_line_length_mm,
                                 mechanical_advantage=mechanical_advantage, compliant_force_n=compliant_force_n)
    feasible = _feasible(tensions, lines, cfg, minimum_force_n, maximum_force_n)
    # Never interpolate across an infeasible gap in the predicted arc.
    runs = np.split(np.flatnonzero(feasible), np.flatnonzero(np.diff(np.flatnonzero(feasible)) > 1) + 1)
    keep = max(runs, key=len)
    positions, tensions = positions[keep], tensions[keep]
    if len(positions) < 2:
        raise InfeasibleForceProfileError("Equal-strain profile needs reachable positions with positive finite line tensions")
    profiles = []
    for drive, sensor in ((cfg.drive_anchor, cfg.sensor_anchor), (cfg.sensor_anchor, cfg.drive_anchor)):
        drive_mm = np.linalg.norm(anchors[drive] - positions, axis=1) - np.linalg.norm(anchors[drive])
        order = np.argsort(drive_mm)
        force = tensions[:, lines.index(sensor)]
        profiles.append({
            "strain_model": PROFILE_STRAIN_MODEL,
            "strand_tension_n": float(strand_tension_n),
            "drive_anchor": int(drive),
            "sensor_anchor": int(sensor),
            "fixed_anchors": list(cfg.fixed_anchors),
            "fixed_lengths": list(cfg.fixed_deltas_mm),
            "drive_mm": drive_mm[order].tolist(),
            "force_n": force[order].tolist(),
        })
    return profiles


def force_planning_parameters(dataset: dict):
    """Equal-strain inputs from the firmware line description and tuned forces, or None.

    Keys: strand_tension_n, mover_weight_n, strands, extra_line_length_mm,
    mechanical_advantage, compliant_force_n, minimum_force_n, maximum_force_n,
    extra_sensor_force_n, target_recorded.
    """
    config = dataset["config"]
    line_description = config.get("m666_before_data_collection") or {}
    tuning = config.get("force_tuning") or {}
    required = [line_description.get(key) for key in ("W", "N", "Y", "U")] + [tuning.get("force_mid_n")]
    if any(value is None for value in required):
        return None
    strands = np.asarray(line_description["N"], dtype=float)
    advantage = np.broadcast_to(np.asarray(line_description["U"], dtype=float), strands.shape)
    if not np.all(np.isfinite(advantage) & (advantage > 0)):
        raise ValueError("Equal-strain force planning requires positive finite mechanical advantages")
    start_force_n = float(tuning["force_mid_n"])
    recorded_targets = [(sweep.get("metadata") or {}).get("strand_tension_n")
                        for sweep in dataset.get("sweeps", [])]
    recorded_targets = [float(value) for value in recorded_targets if value is not None]
    if recorded_targets and (not np.all(np.isfinite(recorded_targets))
                             or min(recorded_targets) <= 0
                             or not np.allclose(recorded_targets, recorded_targets[0], rtol=1e-9, atol=0)):
        raise ValueError("Cannot combine different equal-strain targets in one refinement dataset")
    # Pulley reduction must not lower the strand tension. Parallel lines on one
    # spool share load through effector roll, and a lightly loaded line goes slack;
    # the encoder then follows only the taut line, which no point model describes.
    strand_target = (recorded_targets[0] if recorded_targets else
                     _COLLECTION_FORCE_MULTIPLE * start_force_n / float(np.min(strands)))
    extra_force = None
    if config.get("extra_sensors_enabled") and int(dataset["num_anchors"]) == 5:
        extra_force = min(float(tuning.get("force_max_n", np.inf)),
                          _COLLECTION_FORCE_MULTIPLE * start_force_n)
    return dict(
        strand_tension_n=strand_target,
        mover_weight_n=float(line_description["W"]) * GRAVITY_M_PER_S2,
        strands=strands,
        extra_line_length_mm=np.asarray(line_description["Y"], dtype=float),
        mechanical_advantage=advantage,
        compliant_force_n=(extra_force if extra_force is not None else float(tuning.get("force_low_n", 0.0))),
        minimum_force_n=start_force_n,
        maximum_force_n=(None if line_description.get("X") is None else
                         _MAX_PLANNED_FORCE_SHARE * np.broadcast_to(np.asarray(line_description["X"], dtype=float), strands.shape)),
        extra_sensor_force_n=extra_force,
        target_recorded=bool(recorded_targets),
    )


def _planned_tensions(anchors, positions, lines, **options):
    return np.array([equal_strain_tensions(anchors, position, lines, **options)
                     for position in positions]).reshape(-1, len(lines))


def _feasible(tensions, lines, cfg, minimum_force_n, maximum_force_n):
    # Gravity can make a line go slack at the tails of the self-stress curve.
    # Never tabulate those poses as realizable equal-strain measurements.
    feasible = np.all(np.isfinite(tensions) & (tensions > 0.0), axis=1)
    sensors = [lines.index(cfg.drive_anchor), lines.index(cfg.sensor_anchor)]
    feasible &= np.all(tensions[:, sensors] >= minimum_force_n, axis=1)
    if maximum_force_n is not None:
        feasible &= np.all(tensions <= np.asarray(maximum_force_n)[lines], axis=1)
    return feasible


def _capped_strand_tension(anchors, cfgs, params):
    """Largest strand target, at most the planned one, that leaves every sweep enough poses under the force cap."""
    target = params["strand_tension_n"]
    if params["maximum_force_n"] is None or not cfgs:
        return target
    options = {key: params[key] for key in ("mover_weight_n", "strands", "extra_line_length_mm",
                                            "mechanical_advantage", "compliant_force_n")}
    sweeps = []
    for cfg in cfgs:
        lines = [*cfg.fixed_anchors, cfg.drive_anchor, cfg.sensor_anchor]
        positions = predict_sweep_positions(anchors, cfg, _PROFILE_POINTS, _PROFILE_PRETENSION_SHARE)
        uncapped = _feasible(_planned_tensions(anchors, positions, lines, strand_tension_n=target, **options),
                             lines, cfg, params["minimum_force_n"], None)
        sweeps.append((cfg, lines, positions, _CAPPED_POSE_SHARE * np.sum(uncapped)))
    for scale in np.linspace(1.0, 0.1, 37):
        if all(np.sum(_feasible(_planned_tensions(anchors, positions, lines, strand_tension_n=scale * target, **options),
                                lines, cfg, params["minimum_force_n"], params["maximum_force_n"])) >= needed
               for cfg, lines, positions, needed in sweeps):
            return scale * target
    return target


def write_sensor_force_profile(path: Path, dataset: dict, anchors: np.ndarray, cfgs: Sequence[SweepConfig]) -> bool:
    """Write the collector's force table for the sweeps. Needs the firmware's line description and tuned forces."""
    params = force_planning_parameters(dataset)
    if params is None:
        return False
    extra_force = params.pop("extra_sensor_force_n")
    if not params.pop("target_recorded"):
        # The first profiles fix the dataset's target; plan it for the rough sweeps too.
        params["strand_tension_n"] = _capped_strand_tension(anchors, [*dataset_sweep_configs(dataset), *cfgs], params)
    profiles = [profile for cfg in cfgs for profile in sensor_force_profiles(anchors, cfg, **params)]
    if extra_force is not None:
        for profile in profiles:
            profile["extra_sensor_force_n"] = extra_force
    Path(path).write_text(json.dumps(profiles), encoding="utf-8")
    return True


def is_equal_strain(dataset: dict) -> bool:
    """Whether every tensioned sweep commanded a sensor force profile."""
    sweeps = dataset.get("sweeps", [])
    return (resolve_projection_mode(dataset) is not True and bool(sweeps)
            and all((sweep.get("metadata") or {}).get("sensor_force_profile") for sweep in sweeps))


def profile_policy_error(dataset: dict):
    """Refuse continuation across incompatible force-table policies."""
    flags = [(sweep.get("metadata") or {}).get("sensor_force_profile", False) for sweep in dataset.get("sweeps", [])]
    if not any(flags) or resolve_projection_mode(dataset) is True:
        return None
    if not all(flags):
        return "Cannot continue mixed profiled and constant-force measurement data"
    advantage = ((dataset.get("config") or {}).get("m666_before_data_collection") or {}).get("U")
    if advantage is None:
        return "Profiled collection requires M666 U for mechanical-advantage-aware forces"
    try:
        advantage = np.asarray(advantage, dtype=float)
        targets = [float((sweep.get("metadata") or {})['strand_tension_n']) for sweep in dataset['sweeps']
                   if (sweep.get("metadata") or {}).get('strand_tension_n') is not None]
    except (TypeError, ValueError):
        return "Invalid mechanical advantage or equal-strain target in recorded data"
    if not np.all(np.isfinite(advantage) & (advantage > 0)):
        return "Profiled collection requires positive finite mechanical advantages"
    if targets and (not np.all(np.isfinite(targets)) or min(targets) <= 0
                    or not np.allclose(targets, targets[0], rtol=1e-9, atol=0)):
        return "Cannot combine different equal-strain targets in one refinement dataset"
    if any((sweep.get("metadata") or {}).get("sensor_force_profile_model") != PROFILE_STRAIN_MODEL
           for sweep in dataset["sweeps"]):
        return "Legacy profiled data requires a fresh dataset for corrected equal-strain collection; offline replay remains available"
    return None
