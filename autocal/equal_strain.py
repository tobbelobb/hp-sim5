"""Sensor force targeting a constant weighted mean tension per strand.

At a position the line tensions are fixed by the mover weight up to one free
amount of pretension, and the torque-controlled sensor line sets that amount.
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

from autocal.active_learning import SweepConfig
from autocal.pose_planning import predict_sweep_positions
from autocal.projection_mode import resolve_projection_mode

GRAVITY_M_PER_S2 = 9.81
# Predict a broad arc; collection is restricted to the resulting feasible table.
_PROFILE_PRETENSION_SHARE = 0.05
_PROFILE_POINTS = 60
# The collector's constant measurement force is this multiple of the tuned start force.
_COLLECTION_FORCE_MULTIPLE = 5.0
PROFILE_STRAIN_MODEL = "reeving-aware-v2"


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
) -> np.ndarray:
    """Tensions of the given lines at a position whose weighted mean tension per strand is `strand_tension_n`."""
    lines = list(lines)
    to_anchor = np.asarray(anchors, dtype=float)[lines] - np.asarray(position, dtype=float)
    distance = np.linalg.norm(to_anchor, axis=1)
    directions = (to_anchor / distance[:, None]).T
    load = np.zeros(directions.shape[0])
    load[-1] = mover_weight_n
    carrying = np.linalg.lstsq(directions, load, rcond=None)[0]
    pretension = np.linalg.svd(directions)[2][-1]
    pretension = pretension * np.sign(np.sum(pretension))
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
) -> List[Dict[str, object]]:
    """Sensor force against drive length change, for both directions of a sweep."""
    anchors = np.asarray(anchors, dtype=float)
    positions = predict_sweep_positions(anchors, cfg, _PROFILE_POINTS, _PROFILE_PRETENSION_SHARE)
    lines = [*cfg.fixed_anchors, cfg.drive_anchor, cfg.sensor_anchor]
    tensions = np.array([
        equal_strain_tensions(anchors, position, lines, strand_tension_n=strand_tension_n,
                              mover_weight_n=mover_weight_n, strands=strands,
                              extra_line_length_mm=extra_line_length_mm,
                              mechanical_advantage=mechanical_advantage)
        for position in positions
    ]).reshape(-1, len(lines))
    # Gravity can make a line go slack at the tails of the self-stress curve.
    # Never tabulate those poses as realizable equal-strain measurements.
    feasible = np.all(np.isfinite(tensions) & (tensions > 0.0), axis=1)
    sensors = [lines.index(cfg.drive_anchor), lines.index(cfg.sensor_anchor)]
    feasible &= np.all(tensions[:, sensors] >= minimum_force_n, axis=1)
    # Never interpolate across an infeasible gap in the predicted arc.
    runs = np.split(np.flatnonzero(feasible), np.flatnonzero(np.diff(np.flatnonzero(feasible)) > 1) + 1)
    keep = max(runs, key=len)
    positions, tensions = positions[keep], tensions[keep]
    if len(positions) < 2:
        raise ValueError("Equal-strain profile needs reachable positions with positive finite line tensions")
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


def write_sensor_force_profile(path: Path, dataset: dict, anchors: np.ndarray, cfgs: Sequence[SweepConfig]) -> bool:
    """Write the collector's force table for the sweeps. Needs the firmware's line description and tuned forces."""
    config = dataset["config"]
    line_description = config.get("m666_before_data_collection") or {}
    tuning = config.get("force_tuning") or {}
    required = [line_description.get(key) for key in ("W", "N", "Y", "U")] + [tuning.get("force_mid_n")]
    if any(value is None for value in required):
        return False
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
    strand_target = (recorded_targets[0] if recorded_targets else
                     _COLLECTION_FORCE_MULTIPLE * start_force_n / float(np.min(strands * advantage)))
    profiles = [
        profile
        for cfg in cfgs
        for profile in sensor_force_profiles(
            anchors, cfg,
            strand_tension_n=strand_target,
            mover_weight_n=float(line_description["W"]) * GRAVITY_M_PER_S2,
            strands=strands,
            extra_line_length_mm=np.asarray(line_description["Y"], dtype=float),
            mechanical_advantage=advantage,
            minimum_force_n=start_force_n,
        )
    ]
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
