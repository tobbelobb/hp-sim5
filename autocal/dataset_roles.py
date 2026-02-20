from __future__ import annotations

"""Shared helpers for normalizing sweep dataset point roles."""

from typing import Dict, List, Optional

import numpy as np


def normalize_dataset_point_roles(dataset: Dict[str, object]) -> int:
    """
    Ensure every point in a sweep uses the sweep-level drive/sensor roles.

    Important semantics:
    - `drive_anchor`/`sensor_anchor` (sweep-level) define the canonical orientation consumed by
      the ellipse solver.
    - `source_drive_anchor`/`source_sensor_anchor` (point-level) describe the physical sub-sweep
      direction used during collection and may legitimately be the reverse of the canonical pair.
    - During collection we may already remap `l_drive`/`l_sensor` into canonical orientation while
      keeping `source_*` as physical metadata. In this valid case, reversed `source_*` is expected
      and must NOT be swapped again.

    For points with reversed `source_*`, use `drive_setpoint_mm` as the disambiguator:
    - if it is closer to `l_drive`, the point is still in physical orientation and should be
      swapped to canonical;
    - if it is closer to `l_sensor`, the point is already canonical and must be left untouched.
    """
    sweeps = dataset.get("sweeps")
    if not isinstance(sweeps, list):
        return 0

    swapped_points = 0
    for sweep in sweeps:
        if not isinstance(sweep, dict):
            continue
        try:
            drive_idx = int(sweep.get("drive_anchor"))
            sensor_idx = int(sweep.get("sensor_anchor"))
        except (TypeError, ValueError):
            continue
        points = sweep.get("data_points")
        if not isinstance(points, list):
            continue

        for point in points:
            if not isinstance(point, dict):
                continue

            src_drive = point.get("source_drive_anchor")
            src_sensor = point.get("source_sensor_anchor")
            try:
                src_drive_idx = int(src_drive)
                src_sensor_idx = int(src_sensor)
            except (TypeError, ValueError):
                continue

            if src_drive_idx == drive_idx and src_sensor_idx == sensor_idx:
                continue
            if src_drive_idx != sensor_idx or src_sensor_idx != drive_idx:
                continue

            try:
                l_drive = float(point.get("l_drive"))
                l_sensor = float(point.get("l_sensor"))
            except (TypeError, ValueError):
                continue
            if not np.isfinite(l_drive) or not np.isfinite(l_sensor):
                continue

            raw_setpoint = point.get("drive_setpoint_mm")
            try:
                drive_setpoint = float(raw_setpoint)
            except (TypeError, ValueError):
                drive_setpoint = float("nan")
            should_swap = False
            if np.isfinite(drive_setpoint):
                err_drive = abs(l_drive - drive_setpoint)
                err_sensor = abs(l_sensor - drive_setpoint)
                # `drive_setpoint_mm` always refers to the physical source drive axis.
                # Therefore:
                # - err_drive < err_sensor => point still in physical orientation (swap now)
                # - err_sensor <= err_drive => point already canonical (do not swap)
                should_swap = bool(err_drive + 1e-6 < err_sensor)
            if not should_swap:
                continue

            point["l_drive"], point["l_sensor"] = point.get("l_sensor"), point.get("l_drive")
            if "l_drive_mu" in point or "l_sensor_mu" in point:
                point["l_drive_mu"], point["l_sensor_mu"] = (
                    point.get("l_sensor_mu"),
                    point.get("l_drive_mu"),
                )
            if "assumed_tension_drive_n" in point or "assumed_tension_sensor_n" in point:
                point["assumed_tension_drive_n"], point["assumed_tension_sensor_n"] = (
                    point.get("assumed_tension_sensor_n"),
                    point.get("assumed_tension_drive_n"),
                )
            swapped_points += 1

    return int(swapped_points)
