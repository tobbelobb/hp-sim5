"""Rough anchor positions to start every fit from, for all machine types.

They come from the dataset's motor angles, spool radii and machine type, before any
sweep has been fitted.
"""
from __future__ import annotations

from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from autocal.sweep_types import MachineConfig, MachineType
from autocal.theoretical_ellipse import canonicalize_anchor_gauge

def _extract_m666(config: Optional[dict]) -> Optional[dict]:
    if not isinstance(config, dict):
        return None
    for key in (
        "m666_adjusted_by_data_collector",
        "m666_before_data_collection",
        "m666",
        "m666_after",
        "m666_before",
    ):
        val = config.get(key)
        if isinstance(val, dict):
            return val
    return None

def _resolve_mm_per_degree_for_seed(
    dataset: dict,
    *,
    num_axes: int,
    base_radii_mm: Optional[Sequence[float]],
) -> np.ndarray:
    config = dataset.get("config")
    if not isinstance(config, dict):
        config = {}
    m666 = _extract_m666(config)

    if base_radii_mm is not None and len(base_radii_mm) > 0:
        radii = np.asarray(_expand_to_num_axes(base_radii_mm, int(num_axes), float("nan")), dtype=float)
    else:
        radii = np.asarray(
            _expand_to_num_axes(m666.get("R") if isinstance(m666, dict) else None, int(num_axes), float("nan")),
            dtype=float,
        )
    fallback_radius = float(np.nanmedian(radii)) if np.any(np.isfinite(radii)) else 30.0
    if not np.isfinite(fallback_radius) or fallback_radius <= 0.0:
        fallback_radius = 30.0
    radii = np.where(np.isfinite(radii) & (radii > 0.0), radii, fallback_radius)

    mech_adv = np.asarray(
        _expand_to_num_axes(m666.get("U") if isinstance(m666, dict) else None, int(num_axes), 1.0),
        dtype=float,
    )
    mech_adv = np.where(np.isfinite(mech_adv) & (np.abs(mech_adv) > 1e-12), mech_adv, 1.0)
    motor_gear = np.asarray(
        _expand_to_num_axes(m666.get("L") if isinstance(m666, dict) else None, int(num_axes), 1.0),
        dtype=float,
    )
    motor_gear = np.where(np.isfinite(motor_gear) & (np.abs(motor_gear) > 1e-12), motor_gear, 1.0)
    spool_gear = np.asarray(
        _expand_to_num_axes(m666.get("H") if isinstance(m666, dict) else None, int(num_axes), 1.0),
        dtype=float,
    )
    spool_gear = np.where(np.isfinite(spool_gear) & (np.abs(spool_gear) > 1e-12), spool_gear, 1.0)
    spool_to_motor = spool_gear / motor_gear

    denom = spool_to_motor * mech_adv * 360.0
    with np.errstate(divide="ignore", invalid="ignore"):
        mm_per_degree = (2.0 * np.pi * radii) / denom
    mm_per_degree = np.where(np.isfinite(mm_per_degree) & (mm_per_degree > 0.0), mm_per_degree, np.nan)
    return np.asarray(mm_per_degree, dtype=float)

def _estimate_anchor_distance_from_raw_angles(
    dataset: dict,
    *,
    base_radii_mm: Optional[Sequence[float]],
) -> Optional[float]:
    sweeps = dataset.get("sweeps")
    if not isinstance(sweeps, list) or not sweeps:
        return None

    num_axes = int(dataset.get("num_anchors", 0))
    if num_axes <= 0:
        max_len = 0
        for sweep in sweeps:
            if not isinstance(sweep, dict):
                continue
            points = sweep.get("data_points")
            if not isinstance(points, list):
                continue
            for point in points:
                if not isinstance(point, dict):
                    continue
                raw = point.get("raw_angles_deg")
                if isinstance(raw, list):
                    max_len = max(max_len, len(raw))
        num_axes = int(max_len)
    if num_axes <= 0:
        return None

    mm_per_degree = _resolve_mm_per_degree_for_seed(
        dataset,
        num_axes=int(num_axes),
        base_radii_mm=base_radii_mm,
    )
    if mm_per_degree.size != int(num_axes):
        return None

    travel_mm: List[float] = []
    for sweep in sweeps:
        if not isinstance(sweep, dict):
            continue
        points = sweep.get("data_points")
        if not isinstance(points, list):
            continue
        for point in points:
            if not isinstance(point, dict):
                continue
            raw = point.get("raw_angles_deg")
            if not isinstance(raw, list):
                continue
            count = min(int(num_axes), len(raw))
            for axis in range(count):
                try:
                    angle = float(raw[axis])
                except (TypeError, ValueError):
                    continue
                if not np.isfinite(angle) or angle >= 0.0:
                    continue
                mm_per = float(mm_per_degree[axis])
                if not np.isfinite(mm_per) or mm_per <= 0.0:
                    continue
                mm = -angle * mm_per
                if np.isfinite(mm) and mm > 0.0:
                    travel_mm.append(float(mm))

    if not travel_mm:
        return None

    a = float(np.max(np.asarray(travel_mm, dtype=float)))
    if not np.isfinite(a) or a <= 0.0:
        return None
    return float(a)

def _rotate_about_z(vec: np.ndarray, angle_rad: float) -> np.ndarray:
    vec = np.asarray(vec, dtype=float).reshape(-1)
    c = float(np.cos(float(angle_rad)))
    s = float(np.sin(float(angle_rad)))
    if vec.size == 2:
        x, y = float(vec[0]), float(vec[1])
        return np.asarray([c * x - s * y, s * x + c * y], dtype=float)
    if vec.size == 3:
        x, y, z = float(vec[0]), float(vec[1]), float(vec[2])
        return np.asarray([c * x - s * y, s * x + c * y, z], dtype=float)
    raise ValueError("rotation vector must be 2D or 3D")

def build_anchor_initial_guess(
    dataset: dict,
    *,
    machine_type: Optional[str] = None,
    base_radii_mm: Optional[Sequence[float]] = None,
    low_anchor_z: Optional[float] = None,
) -> Optional[np.ndarray]:
    machine = str(machine_type or dataset.get("machine_type", "")).strip().lower()
    if not machine:
        return None
    try:
        num_anchors = int(dataset.get("num_anchors", 0))
    except (TypeError, ValueError):
        num_anchors = 0
    try:
        dimensions = int(dataset.get("dimensions", 0))
    except (TypeError, ValueError):
        dimensions = 0
    if num_anchors <= 0:
        return None

    a = _estimate_anchor_distance_from_raw_angles(
        dataset,
        base_radii_mm=base_radii_mm,
    )
    if a is None:
        return None
    a = float(a)

    if machine == "slideprinter":
        if dimensions not in (0, 2):
            return None
        base = np.asarray([0.0, -a], dtype=float)
        step = (2.0 * np.pi) / float(num_anchors)
        out = np.vstack([_rotate_about_z(base, step * idx) for idx in range(num_anchors)])
        return out

    if machine in ("hangprinter_3", "hangprinter_4", "hangprinter_5", "hangprinter_5_pulley_reduced"):
        if dimensions not in (0, 3) or num_anchors not in (4, 5):
            return None
        low_z = low_anchor_z
        if low_z is None or not np.isfinite(float(low_z)):
            low_z = -a * np.sin(np.pi / 10.0)
        low_z = float(low_z)
        low_xy = float(np.sqrt(max(a * a - low_z * low_z, 0.0)))
        low_base = np.asarray([0.0, -low_xy, low_z], dtype=float)
        low_count = num_anchors - 1
        low = [_rotate_about_z(low_base, (2.0 * np.pi / low_count) * idx) for idx in range(low_count)]
        top = np.asarray([0.0, 0.0, a], dtype=float)
        return np.vstack(low + [top])

    if machine == "cubecorners":
        if dimensions not in (0, 3) or num_anchors != 8:
            return None
        low_base = np.asarray([0.0, -a * np.cos(np.pi / 4.0), -a * np.sin(np.pi / 4.0)], dtype=float)
        high_base = np.asarray([0.0, -a * np.cos(np.pi / 4.0), a * np.sin(np.pi / 4.0)], dtype=float)
        low = [_rotate_about_z(low_base, (2.0 * np.pi / 4.0) * idx) for idx in range(4)]
        high = [_rotate_about_z(high_base, (2.0 * np.pi / 4.0) * idx) for idx in range(4)]
        return np.vstack(low + high)

    if machine == "skycam":
        if dimensions not in (0, 3):
            return None
        base = np.asarray([0.0, -a * np.cos(np.pi / 4.0), a * np.sin(np.pi / 4.0)], dtype=float)
        step = (2.0 * np.pi) / float(num_anchors)
        out = np.vstack([_rotate_about_z(base, step * idx) for idx in range(num_anchors)])
        return out

    return None

def _apply_low_anchor_z_constraint(dataset: dict, low_anchor_z: Optional[float]) -> dict:
    try:
        value = float(low_anchor_z) if low_anchor_z is not None else None
    except (TypeError, ValueError):
        value = None
    if value is None or not np.isfinite(value):
        return dataset
    dataset_out = dict(dataset)
    dataset_out["_low_anchor_z"] = float(value)
    return dataset_out

def _expand_to_num_axes(value: Any, num_axes: int, default: float) -> List[float]:
    if value is None:
        return [float(default)] * num_axes
    if isinstance(value, (int, float)):
        try:
            v = float(value)
        except Exception:
            v = float(default)
        return [v] * num_axes
    if isinstance(value, (list, tuple)):
        out: List[float] = []
        for item in value[:num_axes]:
            try:
                out.append(float(item))
            except Exception:
                out.append(float(default))
        if len(out) < num_axes:
            out.extend([float(out[0] if out else default)] * (num_axes - len(out)))
        return out
    return [float(default)] * num_axes

def _first_float(value: Any, default: float) -> float:
    if isinstance(value, (list, tuple)) and value:
        value = value[0]
    try:
        return float(value)
    except Exception:
        return float(default)
