from __future__ import annotations

"""JAX-native objective for ellipse anchor optimization."""

from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple
import logging
import os

import numpy as np

from autocal.ellipse_cost import EllipseCostFunction

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


@dataclass(frozen=True)
class _PackedSweeps:
    fixed_anchor: np.ndarray
    fixed_delta: np.ndarray
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
    if int(cost_fn.dimensions) != 2:
        return False
    if str(cost_fn.pointwise_residual_mode or "").strip().lower() not in ("sampson", ""):
        return False
    if cost_fn.flex_model is not None:
        return False
    return True


def _prepare_sweeps(cost_fn: EllipseCostFunction) -> Optional[_PackedSweeps]:
    rows: List[Tuple[int, float, int, int, np.ndarray, np.ndarray, np.ndarray, np.ndarray, bool]] = []
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

        if len(fixed_indices) != 1 or len(fixed_deltas) != 1:
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
                int(fixed_indices[0]),
                float(fixed_deltas[0]),
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
    fixed_anchor = np.zeros(n_sweeps, dtype=np.int32)
    fixed_delta = np.zeros(n_sweeps, dtype=float)
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
            fixed_i,
            fixed_d,
            drive_i,
            sensor_i,
            drive_vals,
            sensor_vals,
            sigma_drive_vals,
            sigma_sensor_vals,
            sigma_ok,
        ) = row
        n = int(drive_vals.size)
        fixed_anchor[i] = int(fixed_i)
        fixed_delta[i] = float(fixed_d)
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
        fixed_anchor=fixed_anchor,
        fixed_delta=fixed_delta,
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


def _objective_core(
    anchor_vec: "jnp.ndarray",
    fixed_anchor: "jnp.ndarray",
    fixed_delta: "jnp.ndarray",
    drive_anchor: "jnp.ndarray",
    sensor_anchor: "jnp.ndarray",
    l_drive: "jnp.ndarray",
    l_sensor: "jnp.ndarray",
    sigma_drive: "jnp.ndarray",
    sigma_sensor: "jnp.ndarray",
    point_mask: "jnp.ndarray",
    has_sigma: "jnp.ndarray",
    num_points: "jnp.ndarray",
    lb: "jnp.ndarray",
    ub: "jnp.ndarray",
    *,
    num_anchors: int,
    dimensions: int,
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
    x = jnp.clip(jnp.asarray(anchor_vec).reshape(-1), lb, ub)
    anchors = x.reshape((num_anchors, dimensions))

    anchor_norms = jnp.linalg.norm(anchors, axis=1)
    fixed_abs = anchor_norms[fixed_anchor] + fixed_delta
    drive_abs = l_drive + anchor_norms[drive_anchor][:, None]
    sensor_abs = l_sensor + anchor_norms[sensor_anchor][:, None]

    neg_fixed = jnp.maximum(_EPS_LEN_MM - fixed_abs, 0.0)
    neg_drive = jnp.maximum(_EPS_LEN_MM - drive_abs, 0.0)
    neg_sensor = jnp.maximum(_EPS_LEN_MM - sensor_abs, 0.0)

    fixed_abs = jnp.maximum(fixed_abs, _EPS_LEN_MM)
    drive_abs = jnp.maximum(drive_abs, _EPS_LEN_MM)
    sensor_abs = jnp.maximum(sensor_abs, _EPS_LEN_MM)

    drive_pen = jnp.sum(jnp.where(point_mask, neg_drive * neg_drive, 0.0), axis=1)
    sensor_pen = jnp.sum(jnp.where(point_mask, neg_sensor * neg_sensor, 0.0), axis=1)
    mean_drive = _masked_mean_axis1(drive_abs, point_mask)
    mean_sensor = _masked_mean_axis1(sensor_abs, point_mask)
    typical = jnp.maximum(jnp.maximum(mean_drive, mean_sensor), jnp.maximum(fixed_abs, 1.0))
    penalties = (neg_fixed * neg_fixed + drive_pen + sensor_pen) / (typical * typical)

    center = anchors[fixed_anchor]
    drive = anchors[drive_anchor]
    sensor = anchors[sensor_anchor]
    delta_d = center - drive
    delta_s = center - sensor

    radius = fixed_abs
    k_d = jnp.sum(delta_d * delta_d, axis=1) + radius * radius
    m_d = 2.0 * radius * delta_d[:, 0]
    n_d = 2.0 * radius * delta_d[:, 1]
    k_s = jnp.sum(delta_s * delta_s, axis=1) + radius * radius
    m_s = 2.0 * radius * delta_s[:, 0]
    n_s = 2.0 * radius * delta_s[:, 1]

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
    abc_norm = jnp.linalg.norm(coeff_ellipse[:, :3], axis=1)
    norm_div = jnp.where(abc_norm > 1e-10, abc_norm, 1.0)
    coeff_ellipse = coeff_ellipse / norm_div[:, None]

    use_line = jnp.abs(det) < 1e-10
    coeffs = jnp.where(use_line[:, None], coeff_line, coeff_ellipse)

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
    denom = jnp.sqrt(grad_x * grad_x + grad_y * grad_y)
    denom = jnp.where(denom < 1e-12, 1e-12, denom)
    residuals = algebraic / denom
    residuals = jnp.where(point_mask, residuals, jnp.nan)

    if noise_norm_available:
        sigma_x = 2.0 * drive_abs * sigma_drive
        sigma_y = 2.0 * sensor_abs * sigma_sensor
        n_x = grad_x / denom
        n_y = grad_y / denom
        sigma_l2 = jnp.sqrt((n_x * n_x) * (sigma_x * sigma_x) + (n_y * n_y) * (sigma_y * sigma_y))
        sigma_l2 = jnp.where(sigma_l2 < 1e-12, 1e-12, sigma_l2)
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


def _build_kwargs_for_objective(cost_fn: EllipseCostFunction) -> dict:
    metric_mode = _metric_mode_code(str(cost_fn.sweep_metric))
    _, _stage_name, huber_mult, hard_cut = cost_fn._pointwise_stage_settings()
    return {
        "num_anchors": int(cost_fn.num_anchors),
        "dimensions": int(cost_fn.dimensions),
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
    lb_arr: np.ndarray,
    ub_arr: np.ndarray,
) -> tuple:
    return (
        jnp.asarray(packed.fixed_anchor),
        jnp.asarray(packed.fixed_delta),
        jnp.asarray(packed.drive_anchor),
        jnp.asarray(packed.sensor_anchor),
        jnp.asarray(packed.l_drive),
        jnp.asarray(packed.l_sensor),
        jnp.asarray(packed.sigma_drive),
        jnp.asarray(packed.sigma_sensor),
        jnp.asarray(packed.point_mask),
        jnp.asarray(packed.has_sigma),
        jnp.asarray(packed.num_points),
        jnp.asarray(lb_arr),
        jnp.asarray(ub_arr),
    )


def build_compiled_value_and_grad(
    cost_fn: EllipseCostFunction,
    lb: np.ndarray,
    ub: np.ndarray,
) -> Optional[Callable[[np.ndarray], Tuple[float, np.ndarray]]]:
    """
    Build a compiled objective+gradient callable for SciPy SLSQP.

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
    kwargs = _build_kwargs_for_objective(cost_fn)
    packed_args = _build_packed_jax_args(packed, lb_arr, ub_arr)

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
    kwargs = _build_kwargs_for_objective(cost_fn)
    packed_args = _build_packed_jax_args(packed, lb_arr, ub_arr)

    def _wrapped(anchor_vec: np.ndarray) -> float:
        x_np = np.clip(np.asarray(anchor_vec, dtype=float).reshape(-1), lb_arr, ub_arr)
        value = _COMPILED_VALUE(
            jnp.asarray(x_np),
            *packed_args,
            **kwargs,
        )
        return float(value)

    return _wrapped
