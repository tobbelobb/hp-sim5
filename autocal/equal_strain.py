"""Sensor force that keeps the line strain the same at every measured position.

At a position the line tensions are fixed by the mover weight up to one free
amount of pretension, and the torque-controlled sensor line sets that amount.
Stretch biases the fit through the pretension-weighted mean strain of the lines.
Choosing the sensor force so that this mean is the same at every position makes
the stretch act like a uniform scale plus a home length offset, whatever the
line stiffness is.
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, List, Sequence

import numpy as np

from autocal.active_learning import SweepConfig
from autocal.pose_planning import predict_sweep_positions

GRAVITY_M_PER_S2 = 9.81
# The table reaches past where sweeps turn, so the collector rarely has to extrapolate.
_PROFILE_PRETENSION_SHARE = 0.05
_PROFILE_POINTS = 60
# The collector's constant measurement force is this multiple of the tuned start force.
_COLLECTION_FORCE_MULTIPLE = 5.0


def equal_strain_tensions(
    anchors: np.ndarray,
    position: np.ndarray,
    lines: Sequence[int],
    *,
    strand_tension_n: float,
    mover_weight_n: float,
    strands: np.ndarray,
    extra_line_length_mm: np.ndarray,
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
    strands = np.asarray(strands, dtype=float)[lines]
    weight = pretension * (distance + np.asarray(extra_line_length_mm, dtype=float)[lines])
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
) -> List[Dict[str, object]]:
    """Sensor force against drive length change, for both directions of a sweep."""
    anchors = np.asarray(anchors, dtype=float)
    positions = predict_sweep_positions(anchors, cfg, _PROFILE_POINTS, _PROFILE_PRETENSION_SHARE)
    lines = [*cfg.fixed_anchors, cfg.drive_anchor, cfg.sensor_anchor]
    tensions = np.array([
        equal_strain_tensions(anchors, position, lines, strand_tension_n=strand_tension_n,
                              mover_weight_n=mover_weight_n, strands=strands,
                              extra_line_length_mm=extra_line_length_mm)
        for position in positions
    ]).reshape(-1, len(lines))
    profiles = []
    for drive, sensor in ((cfg.drive_anchor, cfg.sensor_anchor), (cfg.sensor_anchor, cfg.drive_anchor)):
        drive_mm = np.linalg.norm(anchors[drive] - positions, axis=1) - np.linalg.norm(anchors[drive])
        order = np.argsort(drive_mm)
        force = np.maximum(tensions[:, lines.index(sensor)], minimum_force_n)
        profiles.append({
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
    required = [line_description.get(key) for key in ("W", "N", "Y")] + [tuning.get("force_mid_n")]
    if any(value is None for value in required):
        return False
    strands = np.asarray(line_description["N"], dtype=float)
    start_force_n = float(tuning["force_mid_n"])
    profiles = [
        profile
        for cfg in cfgs
        for profile in sensor_force_profiles(
            anchors, cfg,
            strand_tension_n=_COLLECTION_FORCE_MULTIPLE * start_force_n / float(np.min(strands)),
            mover_weight_n=float(line_description["W"]) * GRAVITY_M_PER_S2,
            strands=strands,
            extra_line_length_mm=np.asarray(line_description["Y"], dtype=float),
            minimum_force_n=start_force_n,
        )
    ]
    Path(path).write_text(json.dumps(profiles), encoding="utf-8")
    return True


def is_equal_strain(dataset: dict) -> bool:
    """Whether every sweep of the dataset was measured with a sensor force profile."""
    sweeps = dataset.get("sweeps", [])
    return bool(sweeps) and all((sweep.get("metadata") or {}).get("sensor_force_profile") for sweep in sweeps)
