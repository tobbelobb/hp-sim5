from __future__ import annotations

"""JAX-native objective for ellipse anchor optimization."""

from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple
import logging
import os

import numpy as np

from autocal.ellipse_cost import EllipseCostFunction
from autocal.theoretical_ellipse import (
    _ANCHOR_OPT_MODE_FULL,
    _ANCHOR_OPT_MODE_HP4_AX0_FIXED_LOW_Z,
    _ANCHOR_OPT_MODE_HP4_AX0_SHARED_LOW_Z,
    _ANCHOR_OPT_MODE_SLIDEPRINTER_AX0,
    anchor_opt_constraint_mode,
    anchor_opt_vector_size,
)

# Always force CPU backend for reproducible behavior and consistent user performance.
os.environ["JAX_PLATFORMS"] = "cpu"
os.environ["JAX_PLATFORM_NAME"] = "cpu"
os.environ["CUDA_VISIBLE_DEVICES"] = ""
logging.getLogger("jax._src.xla_bridge").setLevel(logging.CRITICAL)

try:
    import jax
    import jax.numpy as jnp
    try:
        # CPU-only mode: avoid probing third-party PJRT plugins (e.g. CUDA).
        from jax._src import xla_bridge as _xla_bridge

        _xla_bridge.discover_pjrt_plugins = lambda: None
    except Exception:
        pass

    try:
        jax.config.update("jax_platform_name", "cpu")
    except Exception:
        pass
    try:
        jax.config.update("jax_enable_x64", True)
    except Exception:
        pass
    _JAX_AVAILABLE = True
except Exception:  # pragma: no cover - optional dependency
    jax = None  # type: ignore[assignment]
    jnp = None  # type: ignore[assignment]
    _JAX_AVAILABLE = False

_EPS_LEN_MM = 1.0
_MAD_SCALE = 1.4826
_POINTWISE_TRIM_K = 3.0
_POINTWISE_MIN_INLIERS = 3
_SWEEP_WISE_K = 3.0
_SWEEP_WISE_MIN_KEEP = 2
_SWEEP_WISE_MIN_KEEP_RATIO = 0.5
_UNDERCONSTRAINED_PENALTY = 100.0
_SMOOTH_POINT_TRIM_BAND = 0.10
_SMOOTH_INLIER_BLEND_BAND = 0.50
_SAFE_NORM_EPS = 1e-12


@dataclass(frozen=True)
class _PackedSweeps:
    fixed_anchor_primary: np.ndarray
    fixed_delta_primary: np.ndarray
    fixed_anchor_secondary: np.ndarray
    fixed_delta_secondary: np.ndarray
    drive_anchor: np.ndarray
    sensor_anchor: np.ndarray
    l_drive: np.ndarray
    l_sensor: np.ndarray
    sigma_drive: np.ndarray
    sigma_sensor: np.ndarray
    point_mask: np.ndarray
    has_sigma: np.ndarray
    num_points: np.ndarray


def _metric_mode_code(mode: str) -> int:
    out = str(mode or "").strip().lower()
    if out in ("outlier_ratio", "outlier-ratio", "outliers"):
        return 1
    if out in ("mad", "median_abs_dev", "median-abs-dev"):
        return 2
    return 0


def _can_use_jax_objective(cost_fn: EllipseCostFunction) -> bool:
    if not _JAX_AVAILABLE:
        return False
    dims = int(cost_fn.dimensions)
    machine_type = str(cost_fn.machine_type or "").strip().lower()
    if dims == 2:
        if machine_type != "slideprinter":
            return False
    elif dims == 3:
        if machine_type != "hangprinter_4":
            return False
    else:
        return False
    if str(cost_fn.pointwise_residual_mode or "").strip().lower() not in ("sampson", ""):
        return False
    if cost_fn.flex_model is not None:
        return False
    return True


def _prepare_sweeps(cost_fn: EllipseCostFunction) -> Optional[_PackedSweeps]:
    expected_fixed = 1 if int(cost_fn.dimensions) == 2 else 2 if int(cost_fn.dimensions) == 3 else 0
    if expected_fixed <= 0:
        return None

    rows: List[Tuple[List[int], List[float], int, int, np.ndarray, np.ndarray, np.ndarray, np.ndarray, bool]] = []
    max_points = 0

    for sweep in cost_fn.sweeps:
        (
            fixed_indices,
            fixed_deltas,
            drive_idx,
            sensor_idx,
            l_drive,
            l_sensor,
            _sweep_id,
            sigma_drive,
            sigma_sensor,
        ) = cost_fn._extract_sweep_arrays(sweep)

        if len(fixed_indices) != expected_fixed or len(fixed_deltas) != expected_fixed:
            return None

        l_drive_arr = np.asarray(l_drive, dtype=float).reshape(-1)
        l_sensor_arr = np.asarray(l_sensor, dtype=float).reshape(-1)
        if l_drive_arr.size != l_sensor_arr.size:
            return None

        if sigma_drive is None or sigma_sensor is None:
            sigma_drive_arr = np.full(l_drive_arr.shape, np.nan, dtype=float)
            sigma_sensor_arr = np.full(l_sensor_arr.shape, np.nan, dtype=float)
            has_sigma = False
        else:
            sigma_drive_arr = np.asarray(sigma_drive, dtype=float).reshape(-1)
            sigma_sensor_arr = np.asarray(sigma_sensor, dtype=float).reshape(-1)
            has_sigma = (
                sigma_drive_arr.size == l_drive_arr.size
                and sigma_sensor_arr.size == l_sensor_arr.size
                and np.all(np.isfinite(sigma_drive_arr))
                and np.all(np.isfinite(sigma_sensor_arr))
            )
            if not has_sigma:
                sigma_drive_arr = np.full(l_drive_arr.shape, np.nan, dtype=float)
                sigma_sensor_arr = np.full(l_sensor_arr.shape, np.nan, dtype=float)

        max_points = max(max_points, int(l_drive_arr.size))
        rows.append(
            (
                [int(v) for v in fixed_indices],
                [float(v) for v in fixed_deltas],
                int(drive_idx),
                int(sensor_idx),
                l_drive_arr,
                l_sensor_arr,
                sigma_drive_arr,
                sigma_sensor_arr,
                bool(has_sigma),
            )
        )

    if not rows or max_points <= 0:
        return None

    n_sweeps = len(rows)
    fixed_anchor_primary = np.zeros(n_sweeps, dtype=np.int32)
    fixed_delta_primary = np.zeros(n_sweeps, dtype=float)
    fixed_anchor_secondary = np.zeros(n_sweeps, dtype=np.int32)
    fixed_delta_secondary = np.zeros(n_sweeps, dtype=float)
    drive_anchor = np.zeros(n_sweeps, dtype=np.int32)
    sensor_anchor = np.zeros(n_sweeps, dtype=np.int32)
    l_drive = np.zeros((n_sweeps, max_points), dtype=float)
    l_sensor = np.zeros((n_sweeps, max_points), dtype=float)
    sigma_drive = np.zeros((n_sweeps, max_points), dtype=float)
    sigma_sensor = np.zeros((n_sweeps, max_points), dtype=float)
    point_mask = np.zeros((n_sweeps, max_points), dtype=bool)
    has_sigma = np.zeros(n_sweeps, dtype=bool)
    num_points = np.zeros(n_sweeps, dtype=np.int32)

    for i, row in enumerate(rows):
        (
            fixed_indices,
            fixed_deltas,
            drive_i,
            sensor_i,
            drive_vals,
            sensor_vals,
            sigma_drive_vals,
            sigma_sensor_vals,
            sigma_ok,
        ) = row
        n = int(drive_vals.size)
        fixed_anchor_primary[i] = int(fixed_indices[0])
        fixed_delta_primary[i] = float(fixed_deltas[0])
        if expected_fixed > 1:
            fixed_anchor_secondary[i] = int(fixed_indices[1])
            fixed_delta_secondary[i] = float(fixed_deltas[1])
        drive_anchor[i] = int(drive_i)
        sensor_anchor[i] = int(sensor_i)
        num_points[i] = int(n)
        has_sigma[i] = bool(sigma_ok)
        if n > 0:
            l_drive[i, :n] = drive_vals
            l_sensor[i, :n] = sensor_vals
            sigma_drive[i, :n] = sigma_drive_vals
            sigma_sensor[i, :n] = sigma_sensor_vals
            point_mask[i, :n] = True

    return _PackedSweeps(
        fixed_anchor_primary=fixed_anchor_primary,
        fixed_delta_primary=fixed_delta_primary,
        fixed_anchor_secondary=fixed_anchor_secondary,
        fixed_delta_secondary=fixed_delta_secondary,
        drive_anchor=drive_anchor,
        sensor_anchor=sensor_anchor,
        l_drive=l_drive,
        l_sensor=l_sensor,
        sigma_drive=sigma_drive,
        sigma_sensor=sigma_sensor,
        point_mask=point_mask,
        has_sigma=has_sigma,
        num_points=num_points,
    )


def _pseudo_huber_loss_jax(residuals: "jnp.ndarray", delta: "jnp.ndarray") -> "jnp.ndarray":
    d = jnp.where(jnp.logical_and(jnp.isfinite(delta), delta > 0.0), delta, 1.0)
    scaled = residuals / d
    return 2.0 * (d**2) * (jnp.sqrt(1.0 + scaled * scaled) - 1.0)


def _masked_mean_axis1(values: "jnp.ndarray", mask: "jnp.ndarray") -> "jnp.ndarray":
    denom = jnp.maximum(jnp.sum(mask.astype(values.dtype), axis=1), 1.0)
    total = jnp.sum(jnp.where(mask, values, 0.0), axis=1)
    return total / denom


def _weighted_mean_axis1(values: "jnp.ndarray", weights: "jnp.ndarray") -> "jnp.ndarray":
    safe_values = jnp.where(jnp.isfinite(values), values, 0.0)
    safe_weights = jnp.where(jnp.isfinite(weights), weights, 0.0)
    denom = jnp.maximum(jnp.sum(safe_weights, axis=1), 1e-6)
    total = jnp.sum(safe_values * safe_weights, axis=1)
    return total / denom


def _nanmedian_1d(values: "jnp.ndarray") -> "jnp.ndarray":
    arr = jnp.asarray(values).reshape(-1)
    n = int(arr.shape[0])
    if n <= 0:
        return jnp.asarray(jnp.nan)
    finite = jnp.isfinite(arr)
    count = jnp.sum(finite.astype(jnp.int32))
    sorted_vals = jnp.sort(jnp.where(finite, arr, jnp.inf))
    low = jnp.clip((count - 1) // 2, 0, n - 1)
    high = jnp.clip(count // 2, 0, n - 1)
    med = 0.5 * (sorted_vals[low] + sorted_vals[high])
    return jnp.where(count > 0, med, jnp.asarray(jnp.nan))


def _nanmedian_axis1(values: "jnp.ndarray") -> "jnp.ndarray":
    arr = jnp.asarray(values)
    n_rows = int(arr.shape[0])
    n_cols = int(arr.shape[1])
    if n_rows <= 0:
        return jnp.asarray([], dtype=arr.dtype)
    if n_cols <= 0:
        return jnp.full((n_rows,), jnp.nan, dtype=arr.dtype)

    finite = jnp.isfinite(arr)
    count = jnp.sum(finite.astype(jnp.int32), axis=1)
    sorted_vals = jnp.sort(jnp.where(finite, arr, jnp.inf), axis=1)
    rows = jnp.arange(n_rows, dtype=jnp.int32)
    low = jnp.clip((count - 1) // 2, 0, n_cols - 1)
    high = jnp.clip(count // 2, 0, n_cols - 1)
    med = 0.5 * (sorted_vals[rows, low] + sorted_vals[rows, high])
    return jnp.where(count > 0, med, jnp.full_like(med, jnp.nan))


def _mad_scale_1d(values: "jnp.ndarray") -> "jnp.ndarray":
    med = _nanmedian_1d(values)
    mad = _nanmedian_1d(jnp.abs(values - med))
    scale = _MAD_SCALE * mad
    abs_med = _nanmedian_1d(jnp.abs(values))
    invalid = jnp.logical_or(jnp.logical_not(jnp.isfinite(scale)), scale <= 0.0)
    return jnp.where(invalid, abs_med, scale)


def _mad_scale_axis1(values: "jnp.ndarray") -> "jnp.ndarray":
    med = _nanmedian_axis1(values)
    mad = _nanmedian_axis1(jnp.abs(values - med[:, None]))
    scale = _MAD_SCALE * mad
    abs_med = _nanmedian_axis1(jnp.abs(values))
    invalid = jnp.logical_or(jnp.logical_not(jnp.isfinite(scale)), scale <= 0.0)
    return jnp.where(invalid, abs_med, scale)


def _pointwise_scale_1d(values: "jnp.ndarray", *, floor_norm: float, override_scale: "jnp.ndarray") -> "jnp.ndarray":
    scale_data = _mad_scale_1d(values)
    scale = jnp.where(jnp.isfinite(override_scale), override_scale, scale_data)
    floor = jnp.asarray(max(float(floor_norm), 1e-12), dtype=scale.dtype)
    invalid = jnp.logical_or(jnp.logical_not(jnp.isfinite(scale)), scale <= 0.0)
    scale = jnp.where(invalid, floor, scale)
    return jnp.maximum(scale, floor)


def _pointwise_scale_axis1(
    values: "jnp.ndarray",
    *,
    floor_norm: float,
    override_scale: "jnp.ndarray",
) -> "jnp.ndarray":
    scale_data = _mad_scale_axis1(values)
    scale = jnp.where(jnp.isfinite(override_scale), override_scale, scale_data)
    floor = jnp.asarray(max(float(floor_norm), 1e-12), dtype=scale.dtype)
    invalid = jnp.logical_or(jnp.logical_not(jnp.isfinite(scale)), scale <= 0.0)
    scale = jnp.where(invalid, floor, scale)
    return jnp.maximum(scale, floor)


def _safe_positive_sqrt(
    values: "jnp.ndarray",
    *,
    eps: float,
    fallback: float = 0.0,
) -> Tuple["jnp.ndarray", "jnp.ndarray"]:
    arr = jnp.asarray(values)
    positive = arr > float(eps)
    safe_arg = jnp.where(positive, arr, jnp.ones_like(arr))
    sqrt_safe = jnp.sqrt(safe_arg)
    out = jnp.where(positive, sqrt_safe, jnp.full_like(arr, fallback))
    return out, positive


def _safe_vector_norm(
    values: "jnp.ndarray",
    *,
    eps: float,
) -> Tuple["jnp.ndarray", "jnp.ndarray", "jnp.ndarray"]:
    arr = jnp.asarray(values)
    sq_norm = jnp.sum(arr * arr, axis=1)
    norm, positive = _safe_positive_sqrt(sq_norm, eps=float(eps) * float(eps), fallback=0.0)
    norm_safe = jnp.where(positive, norm, jnp.ones_like(norm))
    return norm, norm_safe, positive


def _anchors_from_opt_vec(
    anchor_vec: "jnp.ndarray",
    lb: "jnp.ndarray",
    ub: "jnp.ndarray",
    low_anchor_z: "jnp.ndarray",
    *,
    num_anchors: int,
    dimensions: int,
    constraint_mode: int,
) -> "jnp.ndarray":
    x = jnp.clip(jnp.asarray(anchor_vec).reshape(-1), lb, ub)
    if constraint_mode == _ANCHOR_OPT_MODE_FULL:
        return x.reshape((num_anchors, dimensions))
    if constraint_mode == _ANCHOR_OPT_MODE_SLIDEPRINTER_AX0:
        x = jnp.concatenate([jnp.zeros((1,), dtype=x.dtype), x], axis=0)
        return x.reshape((num_anchors, dimensions))
    if constraint_mode == _ANCHOR_OPT_MODE_HP4_AX0_SHARED_LOW_Z:
        low_z = x[1]
        zero = jnp.zeros((), dtype=x.dtype)
        return jnp.stack(
            [
                jnp.stack([zero, x[0], low_z]),
                jnp.stack([x[2], x[3], low_z]),
                jnp.stack([x[4], x[5], low_z]),
                jnp.stack([x[6], x[7], x[8]]),
            ],
            axis=0,
        )
    if constraint_mode == _ANCHOR_OPT_MODE_HP4_AX0_FIXED_LOW_Z:
        low_z = jnp.asarray(low_anchor_z, dtype=x.dtype)
        zero = jnp.zeros((), dtype=x.dtype)
        return jnp.stack(
            [
                jnp.stack([zero, x[0], low_z]),
                jnp.stack([x[1], x[2], low_z]),
                jnp.stack([x[3], x[4], low_z]),
                jnp.stack([x[5], x[6], x[7]]),
            ],
            axis=0,
        )
    return x.reshape((num_anchors, dimensions))


def _parametric_to_algebraic_coeffs(
    k_d: "jnp.ndarray",
    m_d: "jnp.ndarray",
    n_d: "jnp.ndarray",
    k_s: "jnp.ndarray",
    m_s: "jnp.ndarray",
    n_s: "jnp.ndarray",
) -> "jnp.ndarray":
    det = m_d * n_s - n_d * m_s

    cand1 = jnp.stack([m_s, -m_d], axis=1)
    cand2 = jnp.stack([n_s, -n_d], axis=1)
    use_cand2 = jnp.sum(cand2 * cand2, axis=1) > jnp.sum(cand1 * cand1, axis=1)
    p = jnp.where(use_cand2, cand2[:, 0], cand1[:, 0])
    q = jnp.where(use_cand2, cand2[:, 1], cand1[:, 1])
    valid_pq = jnp.logical_and(jnp.isfinite(p), jnp.isfinite(q))
    valid_pq = jnp.logical_and(valid_pq, (jnp.abs(p) + jnp.abs(q)) >= 1e-12)
    p = jnp.where(valid_pq, p, 1.0)
    q = jnp.where(valid_pq, q, 0.0)
    norm_pq = jnp.hypot(p, q)
    norm_safe = jnp.where(norm_pq > 1e-12, norm_pq, 1.0)
    p = p / norm_safe
    q = q / norm_safe
    f_line = -(p * k_d + q * k_s)
    coeff_line = jnp.stack(
        [
            jnp.zeros_like(p),
            jnp.zeros_like(p),
            jnp.zeros_like(p),
            p,
            q,
            f_line,
        ],
        axis=1,
    )

    a_xy = n_s * n_s + m_s * m_s
    b_xy = -2.0 * (n_s * n_d + m_s * m_d)
    c_xy = n_d * n_d + m_d * m_d
    f_xy = -(det * det)

    a = a_xy
    b = b_xy
    c = c_xy
    d = -2.0 * a_xy * k_d - b_xy * k_s
    e = -2.0 * c_xy * k_s - b_xy * k_d
    f = a_xy * k_d * k_d + b_xy * k_d * k_s + c_xy * k_s * k_s + f_xy
    coeff_ellipse = jnp.stack([a, b, c, d, e, f], axis=1)
    _abc_norm, norm_div, _abc_nonzero = _safe_vector_norm(coeff_ellipse[:, :3], eps=1e-10)
    coeff_ellipse = coeff_ellipse / norm_div[:, None]

    use_line = jnp.abs(det) < 1e-10
    return jnp.where(use_line[:, None], coeff_line, coeff_ellipse)


def _objective_core(
    anchor_vec: "jnp.ndarray",
    fixed_anchor_primary: "jnp.ndarray",
    fixed_delta_primary: "jnp.ndarray",
    fixed_anchor_secondary: "jnp.ndarray",
    fixed_delta_secondary: "jnp.ndarray",
    drive_anchor: "jnp.ndarray",
    sensor_anchor: "jnp.ndarray",
    l_drive: "jnp.ndarray",
    l_sensor: "jnp.ndarray",
    sigma_drive: "jnp.ndarray",
    sigma_sensor: "jnp.ndarray",
    point_mask: "jnp.ndarray",
    has_sigma: "jnp.ndarray",
    num_points: "jnp.ndarray",
    low_anchor_z: "jnp.ndarray",
    lb: "jnp.ndarray",
    ub: "jnp.ndarray",
    *,
    num_anchors: int,
    dimensions: int,
    constraint_mode: int,
    pointwise_filtering: bool,
    pointwise_global_mad: bool,
    sweep_wise_filtering: bool,
    noise_norm_available: bool,
    robust_loss_unfiltered: bool,
    hard_cut: bool,
    metric_mode: int,
    pointwise_cost_weight: float,
    l2_scale: float,
    min_points: int,
    min_sweeps_after_trim: int,
    huber_delta_unfiltered: float,
    floor_norm: float,
    huber_mult: float,
) -> "jnp.ndarray":
    anchors = _anchors_from_opt_vec(
        anchor_vec,
        lb,
        ub,
        low_anchor_z,
        num_anchors=num_anchors,
        dimensions=dimensions,
        constraint_mode=constraint_mode,
    )
    x = anchors.reshape(-1)

    anchor_norms = jnp.linalg.norm(anchors, axis=1)
    fixed_abs_primary = anchor_norms[fixed_anchor_primary] + fixed_delta_primary
    fixed_abs_secondary = anchor_norms[fixed_anchor_secondary] + fixed_delta_secondary
    drive_abs = l_drive + anchor_norms[drive_anchor][:, None]
    sensor_abs = l_sensor + anchor_norms[sensor_anchor][:, None]

    neg_fixed_primary = jnp.maximum(_EPS_LEN_MM - fixed_abs_primary, 0.0)
    neg_fixed_secondary = jnp.maximum(_EPS_LEN_MM - fixed_abs_secondary, 0.0)
    neg_drive = jnp.maximum(_EPS_LEN_MM - drive_abs, 0.0)
    neg_sensor = jnp.maximum(_EPS_LEN_MM - sensor_abs, 0.0)

    fixed_abs_primary = jnp.maximum(fixed_abs_primary, _EPS_LEN_MM)
    fixed_abs_secondary = jnp.maximum(fixed_abs_secondary, _EPS_LEN_MM)
    drive_abs = jnp.maximum(drive_abs, _EPS_LEN_MM)
    sensor_abs = jnp.maximum(sensor_abs, _EPS_LEN_MM)

    drive_pen = jnp.sum(jnp.where(point_mask, neg_drive * neg_drive, 0.0), axis=1)
    sensor_pen = jnp.sum(jnp.where(point_mask, neg_sensor * neg_sensor, 0.0), axis=1)
    mean_drive = _masked_mean_axis1(drive_abs, point_mask)
    mean_sensor = _masked_mean_axis1(sensor_abs, point_mask)
    typical = jnp.maximum(jnp.maximum(mean_drive, mean_sensor), jnp.maximum(fixed_abs_primary, 1.0))
    penalties = neg_fixed_primary * neg_fixed_primary + drive_pen + sensor_pen

    if dimensions == 3:
        typical = jnp.maximum(typical, fixed_abs_secondary)
        penalties = penalties + neg_fixed_secondary * neg_fixed_secondary

    penalties = penalties / (typical * typical)

    center = anchors[fixed_anchor_primary]
    radius = fixed_abs_primary
    if dimensions == 2:
        u = jnp.broadcast_to(jnp.asarray([1.0, 0.0], dtype=anchors.dtype), center.shape)
        v = jnp.broadcast_to(jnp.asarray([0.0, 1.0], dtype=anchors.dtype), center.shape)
        geom_valid = jnp.ones_like(radius, dtype=bool)
    else:
        anchor_secondary = anchors[fixed_anchor_secondary]
        delta_center = anchor_secondary - center
        center_dist, center_dist_safe, center_dist_ok = _safe_vector_norm(delta_center, eps=1e-10)
        radius_secondary = fixed_abs_secondary

        h = (
            center_dist * center_dist
            + radius * radius
            - radius_secondary * radius_secondary
        ) / (2.0 * center_dist_safe)
        radius_sq = radius * radius - h * h
        radius, _radius_positive = _safe_positive_sqrt(radius_sq, eps=_SAFE_NORM_EPS, fallback=0.0)
        normal = delta_center / center_dist_safe[:, None]
        center = center + h[:, None] * normal

        temp_x = jnp.asarray([1.0, 0.0, 0.0], dtype=anchors.dtype)
        temp_y = jnp.asarray([0.0, 1.0, 0.0], dtype=anchors.dtype)
        temp = jnp.where(jnp.abs(normal[:, :1]) < 0.9, temp_x[None, :], temp_y[None, :])
        u = jnp.cross(normal, temp)
        _u_norm, _u_norm_safe, u_ok_initial = _safe_vector_norm(u, eps=1e-12)
        temp_z = jnp.broadcast_to(jnp.asarray([0.0, 0.0, 1.0], dtype=anchors.dtype), u.shape)
        u_alt = jnp.cross(normal, temp_z)
        use_alt = jnp.logical_not(u_ok_initial)
        u = jnp.where(use_alt[:, None], u_alt, u)
        _u_norm, u_norm_safe, u_ok = _safe_vector_norm(u, eps=1e-12)
        u = u / u_norm_safe[:, None]
        v = jnp.cross(normal, u)
        _v_norm, v_norm_safe, v_ok = _safe_vector_norm(v, eps=1e-12)
        v = v / v_norm_safe[:, None]

        geom_valid = center_dist_ok
        geom_valid = jnp.logical_and(geom_valid, center_dist <= (fixed_abs_primary + fixed_abs_secondary))
        geom_valid = jnp.logical_and(geom_valid, center_dist >= jnp.abs(fixed_abs_primary - fixed_abs_secondary))
        geom_valid = jnp.logical_and(geom_valid, radius_sq >= -1e-10)
        geom_valid = jnp.logical_and(geom_valid, u_ok)
        geom_valid = jnp.logical_and(geom_valid, v_ok)

    drive = anchors[drive_anchor]
    sensor = anchors[sensor_anchor]
    delta_d = center - drive
    delta_s = center - sensor

    k_d = jnp.sum(delta_d * delta_d, axis=1) + radius * radius
    m_d = 2.0 * radius * jnp.sum(u * delta_d, axis=1)
    n_d = 2.0 * radius * jnp.sum(v * delta_d, axis=1)
    k_s = jnp.sum(delta_s * delta_s, axis=1) + radius * radius
    m_s = 2.0 * radius * jnp.sum(u * delta_s, axis=1)
    n_s = 2.0 * radius * jnp.sum(v * delta_s, axis=1)

    coeffs = _parametric_to_algebraic_coeffs(k_d, m_d, n_d, k_s, m_s, n_s)

    x_sq = drive_abs * drive_abs
    y_sq = sensor_abs * sensor_abs
    a_c = coeffs[:, 0][:, None]
    b_c = coeffs[:, 1][:, None]
    c_c = coeffs[:, 2][:, None]
    d_c = coeffs[:, 3][:, None]
    e_c = coeffs[:, 4][:, None]
    f_c = coeffs[:, 5][:, None]

    algebraic = a_c * x_sq * x_sq + b_c * x_sq * y_sq + c_c * y_sq * y_sq + d_c * x_sq + e_c * y_sq + f_c
    grad_x = 2.0 * a_c * x_sq + b_c * y_sq + d_c
    grad_y = b_c * x_sq + 2.0 * c_c * y_sq + e_c
    denom_sq = grad_x * grad_x + grad_y * grad_y
    denom, _denom_positive = _safe_positive_sqrt(denom_sq, eps=1e-24, fallback=1e-12)
    residuals = algebraic / denom
    residuals = jnp.where(point_mask, residuals, jnp.nan)

    if noise_norm_available:
        sigma_x = 2.0 * drive_abs * sigma_drive
        sigma_y = 2.0 * sensor_abs * sigma_sensor
        n_x = grad_x / denom
        n_y = grad_y / denom
        sigma_l2_sq = (n_x * n_x) * (sigma_x * sigma_x) + (n_y * n_y) * (sigma_y * sigma_y)
        sigma_l2, _sigma_l2_positive = _safe_positive_sqrt(sigma_l2_sq, eps=1e-24, fallback=1e-12)
        sigma_l2 = jnp.where(point_mask, sigma_l2, jnp.nan)
        sigma_ok = jnp.logical_and(
            has_sigma,
            jnp.all(jnp.where(point_mask, jnp.isfinite(sigma_l2), True), axis=1),
        )
        r_norm = jnp.where(sigma_ok[:, None], residuals / sigma_l2, residuals / l2_scale)
    else:
        r_norm = residuals / l2_scale

    r_norm = jnp.where(point_mask, r_norm, jnp.nan)
    residual_ok = jnp.logical_and(
        jnp.all(jnp.where(point_mask, jnp.isfinite(residuals), True), axis=1),
        jnp.all(jnp.where(point_mask, jnp.isfinite(r_norm), True), axis=1),
    )
    residual_ok = jnp.logical_and(residual_ok, geom_valid)

    if pointwise_filtering and pointwise_global_mad:
        global_values = jnp.where(residual_ok[:, None], r_norm, jnp.nan).reshape(-1)
        global_scale = _pointwise_scale_1d(
            global_values,
            floor_norm=floor_norm,
            override_scale=jnp.asarray(jnp.nan, dtype=x.dtype),
        )
    else:
        global_scale = jnp.asarray(jnp.nan, dtype=x.dtype)

    abs_r = jnp.abs(r_norm)
    valid_point_mask = point_mask

    if pointwise_filtering:
        scale_local = _pointwise_scale_axis1(
            r_norm,
            floor_norm=floor_norm,
            override_scale=global_scale,
        )
        huber_delta = jnp.maximum(huber_mult * scale_local, 1e-12)
        trim_threshold = jnp.maximum(_POINTWISE_TRIM_K * scale_local, 1e-12)
        losses = _pseudo_huber_loss_jax(r_norm, huber_delta[:, None])

        if hard_cut:
            trim_band = jnp.maximum(_SMOOTH_POINT_TRIM_BAND * trim_threshold, 1e-6)
            inlier_soft = jax.nn.sigmoid((trim_threshold[:, None] - abs_r) / trim_band[:, None])
            inlier_weights = valid_point_mask.astype(x.dtype) * inlier_soft
            inlier_count = jnp.sum(inlier_weights, axis=1)
            min_inliers = jnp.minimum(jnp.asarray(_POINTWISE_MIN_INLIERS, dtype=x.dtype), num_points.astype(x.dtype))
            use_inliers = jax.nn.sigmoid((inlier_count - min_inliers) / _SMOOTH_INLIER_BLEND_BAND)
            cost_all = _masked_mean_axis1(losses, valid_point_mask)
            cost_inliers = _weighted_mean_axis1(losses, inlier_weights)
            cost_unit = use_inliers * cost_inliers + (1.0 - use_inliers) * cost_all
            inlier_ratio = inlier_count / jnp.maximum(num_points.astype(x.dtype), 1.0)

            inlier_mask = inlier_weights > 0.5
            abs_masked = jnp.where(inlier_mask, abs_r, jnp.nan)
            metric_median = _nanmedian_axis1(jnp.where(valid_point_mask, abs_r, jnp.nan))
            metric_mad = _mad_scale_axis1(jnp.where(valid_point_mask, abs_r, jnp.nan))
        else:
            cost_unit = _masked_mean_axis1(losses, valid_point_mask)
            inlier_ratio = jnp.ones_like(cost_unit)
            metric_median = _nanmedian_axis1(jnp.where(valid_point_mask, abs_r, jnp.nan))
            metric_mad = _mad_scale_axis1(jnp.where(valid_point_mask, abs_r, jnp.nan))

        cost = cost_unit * pointwise_cost_weight
        if metric_mode == 1:
            sweep_metric = jnp.maximum(0.0, 1.0 - inlier_ratio)
        elif metric_mode == 2:
            sweep_metric = metric_mad
        else:
            sweep_metric = metric_median
    else:
        if robust_loss_unfiltered:
            losses = _pseudo_huber_loss_jax(r_norm, jnp.asarray(huber_delta_unfiltered, dtype=x.dtype))
        else:
            losses = r_norm * r_norm
        cost_unit = _masked_mean_axis1(losses, valid_point_mask)
        cost = cost_unit * pointwise_cost_weight
        if metric_mode == 1:
            sweep_metric = jnp.full_like(cost, jnp.inf)
        elif metric_mode == 2:
            sweep_metric = _mad_scale_axis1(jnp.where(valid_point_mask, abs_r, jnp.nan))
        else:
            sweep_metric = _nanmedian_axis1(jnp.where(valid_point_mask, abs_r, jnp.nan))

    enough_points = num_points >= int(min_points)
    valid = jnp.logical_and(enough_points, residual_ok)
    valid = jnp.logical_and(valid, jnp.isfinite(cost))
    costs = jnp.where(valid, cost, jnp.asarray(jnp.inf, dtype=x.dtype))
    metrics = jnp.where(valid, sweep_metric, jnp.asarray(jnp.inf, dtype=x.dtype))

    keep_mask = jnp.ones_like(valid, dtype=bool)
    use_keep = jnp.asarray(False)
    if sweep_wise_filtering:
        finite_mask = jnp.isfinite(metrics)
        count = jnp.sum(finite_mask.astype(jnp.int32))
        metrics_nan = jnp.where(finite_mask, metrics, jnp.nan)
        med = _nanmedian_1d(metrics_nan)
        mad = _nanmedian_1d(jnp.abs(metrics_nan - med))
        scale = _MAD_SCALE * mad
        abs_med = _nanmedian_1d(jnp.abs(metrics_nan))
        invalid_scale = jnp.logical_or(jnp.logical_not(jnp.isfinite(scale)), scale <= 0.0)
        scale = jnp.where(invalid_scale, abs_med, scale)
        scale = jnp.maximum(scale, jnp.maximum(floor_norm, 1e-12))
        threshold = jnp.maximum(med + _SWEEP_WISE_K * scale, 1e-12)
        keep = metrics <= threshold
        keep_count = jnp.sum(jnp.logical_and(keep, finite_mask).astype(jnp.int32))
        min_keep = jnp.maximum(
            _SWEEP_WISE_MIN_KEEP,
            jnp.ceil(_SWEEP_WISE_MIN_KEEP_RATIO * count.astype(x.dtype)).astype(jnp.int32),
        )
        sorted_metrics = jnp.sort(jnp.where(finite_mask, metrics, jnp.inf))
        relaxed_idx = jnp.clip(min_keep - 1, 0, int(sorted_metrics.shape[0]) - 1)
        relaxed = sorted_metrics[relaxed_idx]
        threshold = jnp.where(keep_count < min_keep, jnp.maximum(threshold, relaxed), threshold)
        keep = metrics <= threshold
        any_keep = jnp.any(jnp.logical_and(keep, finite_mask))
        use_keep = jnp.logical_and(count > 0, any_keep)
        keep_mask = keep

    effective_keep = jnp.where(use_keep, keep_mask, jnp.ones_like(keep_mask, dtype=bool))
    accepted = jnp.logical_and(valid, effective_keep)
    weights = accepted.astype(x.dtype)
    finite_costs = jnp.where(jnp.isfinite(costs), costs, 0.0)
    weighted_costs = jnp.where(accepted, finite_costs + penalties, 0.0)
    kept_count = jnp.sum(accepted.astype(jnp.int32))
    weight_sum = jnp.maximum(jnp.sum(weights), 1.0)
    total = jnp.sum(weighted_costs * (weights / weight_sum))
    return jnp.where(kept_count < int(min_sweeps_after_trim), _UNDERCONSTRAINED_PENALTY, total)


if _JAX_AVAILABLE:
    _COMPILED_VALUE_AND_GRAD = jax.jit(
        jax.value_and_grad(_objective_core),
        static_argnames=(
            "num_anchors",
            "dimensions",
            "constraint_mode",
            "pointwise_filtering",
            "pointwise_global_mad",
            "sweep_wise_filtering",
            "noise_norm_available",
            "robust_loss_unfiltered",
            "hard_cut",
            "metric_mode",
            "pointwise_cost_weight",
            "l2_scale",
            "min_points",
            "min_sweeps_after_trim",
            "huber_delta_unfiltered",
            "floor_norm",
            "huber_mult",
        ),
    )
else:  # pragma: no cover - optional dependency
    _COMPILED_VALUE_AND_GRAD = None


def _build_kwargs_for_objective(cost_fn: EllipseCostFunction, *, constraint_mode: int) -> dict:
    metric_mode = _metric_mode_code(str(cost_fn.sweep_metric))
    _, _stage_name, huber_mult, hard_cut = cost_fn._pointwise_stage_settings()
    return {
        "num_anchors": int(cost_fn.num_anchors),
        "dimensions": int(cost_fn.dimensions),
        "constraint_mode": int(constraint_mode),
        "pointwise_filtering": bool(cost_fn.pointwise_filtering),
        "pointwise_global_mad": bool(cost_fn.pointwise_global_mad),
        "sweep_wise_filtering": bool(cost_fn.sweep_wise_filtering),
        "noise_norm_available": bool(cost_fn._noise_norm_available),
        "robust_loss_unfiltered": bool(cost_fn.robust_loss),
        "hard_cut": bool(hard_cut),
        "metric_mode": int(metric_mode),
        "pointwise_cost_weight": float(cost_fn.pointwise_cost_weight),
        "l2_scale": float(max(cost_fn._l2_scale, 1.0)),
        "min_points": int(cost_fn.min_points),
        "min_sweeps_after_trim": int(cost_fn._min_sweeps_after_trim),
        "huber_delta_unfiltered": float(cost_fn.huber_delta),
        "floor_norm": float(cost_fn._pointwise_sigma_floor_norm),
        "huber_mult": float(huber_mult),
    }


def _build_packed_jax_args(
    packed: _PackedSweeps,
    low_anchor_z: float,
    lb_arr: np.ndarray,
    ub_arr: np.ndarray,
) -> tuple:
    return (
        jnp.asarray(packed.fixed_anchor_primary),
        jnp.asarray(packed.fixed_delta_primary),
        jnp.asarray(packed.fixed_anchor_secondary),
        jnp.asarray(packed.fixed_delta_secondary),
        jnp.asarray(packed.drive_anchor),
        jnp.asarray(packed.sensor_anchor),
        jnp.asarray(packed.l_drive),
        jnp.asarray(packed.l_sensor),
        jnp.asarray(packed.sigma_drive),
        jnp.asarray(packed.sigma_sensor),
        jnp.asarray(packed.point_mask),
        jnp.asarray(packed.has_sigma),
        jnp.asarray(packed.num_points),
        jnp.asarray(low_anchor_z, dtype=float),
        jnp.asarray(lb_arr),
        jnp.asarray(ub_arr),
    )


def _jax_constraint_mode(
    cost_fn: EllipseCostFunction,
    lb_arr: np.ndarray,
    ub_arr: np.ndarray,
) -> Optional[Tuple[int, float]]:
    if lb_arr.size != ub_arr.size:
        return None
    full_size = int(cost_fn.num_anchors) * int(cost_fn.dimensions)
    opt_size = int(
        anchor_opt_vector_size(
            cost_fn.machine_type,
            cost_fn.num_anchors,
            cost_fn.dimensions,
            cost_fn.low_anchor_z,
        )
    )
    if lb_arr.size == full_size:
        return _ANCHOR_OPT_MODE_FULL, float("nan")
    if lb_arr.size == opt_size:
        mode = int(
            anchor_opt_constraint_mode(
                cost_fn.machine_type,
                cost_fn.num_anchors,
                cost_fn.dimensions,
                cost_fn.low_anchor_z,
            )
        )
        low_anchor_z = float(cost_fn.low_anchor_z) if cost_fn.low_anchor_z is not None else float("nan")
        if mode == _ANCHOR_OPT_MODE_HP4_AX0_FIXED_LOW_Z and not np.isfinite(low_anchor_z):
            return None
        return mode, low_anchor_z
    return None


def build_compiled_value_and_grad(
    cost_fn: EllipseCostFunction,
    lb: np.ndarray,
    ub: np.ndarray,
) -> Optional[Callable[[np.ndarray], Tuple[float, np.ndarray]]]:
    """
    Build a compiled objective+gradient callable for SciPy L-BFGS-B.

    Returns None when the current objective configuration is not supported by
    the JAX-native path.
    """
    if not _can_use_jax_objective(cost_fn):
        return None
    packed = _prepare_sweeps(cost_fn)
    if packed is None:
        return None
    if _COMPILED_VALUE_AND_GRAD is None:
        return None

    lb_arr = np.asarray(lb, dtype=float).reshape(-1)
    ub_arr = np.asarray(ub, dtype=float).reshape(-1)
    layout = _jax_constraint_mode(cost_fn, lb_arr, ub_arr)
    if layout is None:
        return None
    constraint_mode, low_anchor_z = layout
    kwargs = _build_kwargs_for_objective(cost_fn, constraint_mode=constraint_mode)
    packed_args = _build_packed_jax_args(packed, low_anchor_z, lb_arr, ub_arr)

    def _wrapped(anchor_vec: np.ndarray) -> Tuple[float, np.ndarray]:
        x_np = np.clip(np.asarray(anchor_vec, dtype=float).reshape(-1), lb_arr, ub_arr)
        value, grad = _COMPILED_VALUE_AND_GRAD(
            jnp.asarray(x_np),
            *packed_args,
            **kwargs,
        )
        return float(value), np.asarray(grad, dtype=float).reshape(-1)

    return _wrapped


if _JAX_AVAILABLE:
    _COMPILED_VALUE = jax.jit(
        _objective_core,
        static_argnames=(
            "num_anchors",
            "dimensions",
            "constraint_mode",
            "pointwise_filtering",
            "pointwise_global_mad",
            "sweep_wise_filtering",
            "noise_norm_available",
            "robust_loss_unfiltered",
            "hard_cut",
            "metric_mode",
            "pointwise_cost_weight",
            "l2_scale",
            "min_points",
            "min_sweeps_after_trim",
            "huber_delta_unfiltered",
            "floor_norm",
            "huber_mult",
        ),
    )
else:  # pragma: no cover - optional dependency
    _COMPILED_VALUE = None


def build_compiled_objective(
    cost_fn: EllipseCostFunction,
    lb: np.ndarray,
    ub: np.ndarray,
) -> Optional[Callable[[np.ndarray], float]]:
    if not _can_use_jax_objective(cost_fn):
        return None
    packed = _prepare_sweeps(cost_fn)
    if packed is None:
        return None
    if _COMPILED_VALUE is None:
        return None

    lb_arr = np.asarray(lb, dtype=float).reshape(-1)
    ub_arr = np.asarray(ub, dtype=float).reshape(-1)
    layout = _jax_constraint_mode(cost_fn, lb_arr, ub_arr)
    if layout is None:
        return None
    constraint_mode, low_anchor_z = layout
    kwargs = _build_kwargs_for_objective(cost_fn, constraint_mode=constraint_mode)
    packed_args = _build_packed_jax_args(packed, low_anchor_z, lb_arr, ub_arr)

    def _wrapped(anchor_vec: np.ndarray) -> float:
        x_np = np.clip(np.asarray(anchor_vec, dtype=float).reshape(-1), lb_arr, ub_arr)
        value = _COMPILED_VALUE(
            jnp.asarray(x_np),
            *packed_args,
            **kwargs,
        )
        return float(value)

    return _wrapped
