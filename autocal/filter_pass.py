from __future__ import annotations

import importlib.util

from autocal.theoretical_ellipse import (
    anchor_opt_vec_to_matrix,
    anchors_matrix_to_opt_vec,
    canonicalize_anchor_gauge,
    get_anchor_opt_bounds,
    predict_ellipse_coefficients,
)

from autocal._autocal_common import *  # noqa: F401,F403
from autocal.alternating_refinement import run_alternating_refinement
from autocal.initialize_pass import run_initialize_pass
from autocal.scale_polish import apply_final_scale_polish, run_uniform_scale_polish

_OBJECTIVE_NAMES = {
    0: "ellipse_prefit",
    1: "pointwise_forward_model",
    2: "position_reconstruction",
}
_LEGACY_FORWARD_TRANSFORM_FN = None


def _objective_name(objective_id: int) -> str:
    return str(_OBJECTIVE_NAMES.get(int(objective_id), f"objective_{int(objective_id)}"))


def _resolve_objective_schedule(
    objective_schedule: Optional[Sequence[Any]],
    *,
    num_passes: int,
    label: str = "objective_schedule",
) -> Tuple[int, ...]:
    target_passes = max(1, int(num_passes))
    if objective_schedule is None:
        return tuple(1 for _ in range(target_passes))

    parsed = tuple(
        int(v) for v in _parse_objective_schedule(objective_schedule, label=label)
    )
    if len(parsed) == target_passes:
        return parsed
    if len(parsed) == 1:
        return tuple(int(parsed[0]) for _ in range(target_passes))
    if len(set(parsed)) == 1:
        return tuple(int(parsed[0]) for _ in range(target_passes))
    raise ValueError(
        f"{label} must contain either 1 entry or exactly {target_passes} entries; "
        f"got {len(parsed)}"
    )


def _load_legacy_forward_transform_fn():
    global _LEGACY_FORWARD_TRANSFORM_FN
    if callable(_LEGACY_FORWARD_TRANSFORM_FN):
        return _LEGACY_FORWARD_TRANSFORM_FN

    legacy_path = (
        REPO_ROOT
        / "autocal"
        / "auto-calibration-simulation-for-hangprinter"
        / "hangprinter_forward_transform.py"
    )
    if not legacy_path.exists():
        raise FileNotFoundError(f"Legacy forward transform not found at {legacy_path}")

    spec = importlib.util.spec_from_file_location(
        "autocal_legacy_forward_transform",
        legacy_path,
    )
    if spec is None or spec.loader is None:
        raise ImportError(f"Could not load legacy forward transform from {legacy_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    forward_transform_fn = getattr(module, "forward_transform", None)
    if not callable(forward_transform_fn):
        raise AttributeError("Legacy forward transform module has no forward_transform")
    _LEGACY_FORWARD_TRANSFORM_FN = forward_transform_fn
    return _LEGACY_FORWARD_TRANSFORM_FN


def _modeled_point_length(
    point: dict,
    *,
    primary_key: str,
    mean_key: str,
    use_noise_mean: bool,
) -> Optional[float]:
    if bool(use_noise_mean):
        raw_value = point.get(mean_key, point.get(primary_key))
    else:
        raw_value = point.get(primary_key, point.get(mean_key))
    try:
        value = float(raw_value)
    except (TypeError, ValueError):
        return None
    if not np.isfinite(value):
        return None
    return float(value)


def simulation_position_objective_score(
    transformed_dataset: dict,
    anchors_eval: np.ndarray,
    *,
    use_noise_mean: bool,
) -> Tuple[float, int, int]:
    anchors_arr = np.asarray(anchors_eval, dtype=float)
    if anchors_arr.ndim != 2 or anchors_arr.shape[0] <= 0 or not np.all(np.isfinite(anchors_arr)):
        return float("inf"), 0, 0
    if anchors_arr.shape[1] == 2:
        anchors_arr = np.column_stack(
            [anchors_arr, np.zeros(anchors_arr.shape[0], dtype=float)]
        )
    if anchors_arr.shape[1] != 3 or anchors_arr.shape[0] < 4:
        return float("inf"), 0, 0

    sweeps = transformed_dataset.get("sweeps")
    if not isinstance(sweeps, list) or len(sweeps) <= 0:
        return float("inf"), 0, 0

    try:
        forward_transform = _load_legacy_forward_transform_fn()
    except Exception:
        return float("inf"), 0, 0

    num_anchors = int(anchors_arr.shape[0])
    total_cost = 0.0
    valid_points = 0
    invalid_points = 0

    for sweep in sweeps:
        if not isinstance(sweep, dict):
            continue
        points = sweep.get("data_points")
        if not isinstance(points, list) or len(points) <= 0:
            continue

        try:
            drive_idx = int(sweep.get("drive_anchor", 0))
            sensor_idx = int(sweep.get("sensor_anchor", 0))
        except (TypeError, ValueError):
            invalid_points += int(len(points))
            continue
        fixed_axes = [int(v) for v in (sweep.get("fixed_anchors", []) or [])]
        fixed_lengths = [float(v) for v in (sweep.get("fixed_lengths", []) or [])]
        fixed_by_axis = {
            int(axis): float(length)
            for axis, length in zip(fixed_axes, fixed_lengths)
            if 0 <= int(axis) < num_anchors and np.isfinite(float(length))
        }
        last_pos = None
        for point in points:
            if not isinstance(point, dict):
                invalid_points += 1
                continue
            drive_len = _modeled_point_length(
                point,
                primary_key="l_drive",
                mean_key="l_drive_mu",
                use_noise_mean=bool(use_noise_mean),
            )
            sensor_len = _modeled_point_length(
                point,
                primary_key="l_sensor",
                mean_key="l_sensor_mu",
                use_noise_mean=bool(use_noise_mean),
            )
            if drive_len is None or sensor_len is None:
                invalid_points += 1
                continue
            line_positions = np.full(num_anchors, np.nan, dtype=float)
            line_positions[drive_idx] = float(drive_len)
            line_positions[sensor_idx] = float(sensor_len)
            for axis, fixed_delta in fixed_by_axis.items():
                line_positions[int(axis)] = float(fixed_delta)
            if not np.all(np.isfinite(line_positions)):
                invalid_points += 1
                continue
            try:
                pos_est, spread = forward_transform(
                    anchors_arr,
                    line_positions,
                    seed=last_pos,
                    return_spread=True,
                )
            except Exception:
                invalid_points += 1
                continue
            pos_arr = np.asarray(pos_est, dtype=float).reshape(-1)
            spread_f = float(spread)
            if pos_arr.size != 3 or not np.all(np.isfinite(pos_arr)) or not np.isfinite(spread_f):
                invalid_points += 1
                continue
            total_cost += float(spread_f)
            last_pos = np.asarray(pos_arr, dtype=float)
            valid_points += 1

    if valid_points <= 0:
        return float("inf"), 0, int(invalid_points)
    return float(total_cost + 100.0 * float(invalid_points)), int(valid_points), int(invalid_points)


def ellipse_prediction_objective_score(
    transformed_dataset: dict,
    anchors_eval: np.ndarray,
    *,
    cost_fn: Optional[EllipseCostFunction],
) -> Tuple[float, int, int]:
    _ = transformed_dataset
    anchors_arr = np.asarray(anchors_eval, dtype=float)
    if anchors_arr.ndim != 2 or anchors_arr.shape[0] <= 0 or not np.all(np.isfinite(anchors_arr)):
        return float("inf"), 0, 0
    if cost_fn is None:
        return float("inf"), 0, 0

    sweeps = getattr(cost_fn, "sweeps", None)
    if not isinstance(sweeps, list) or len(sweeps) <= 0:
        return float("inf"), 0, 0

    total_cost = 0.0
    valid_sweeps = 0
    invalid_sweeps = 0
    dimensions = int(getattr(cost_fn, "dimensions", anchors_arr.shape[1] if anchors_arr.ndim == 2 else 0))

    for sweep in sweeps:
        try:
            (
                fixed_lengths_abs,
                _drive_idx,
                _sensor_idx,
                l_drive_abs,
                l_sensor_abs,
                _sweep_id,
                violation_penalty,
                _sigma_drive_mm,
                _sigma_sensor_mm,
            ) = cost_fn._reconstruct_lengths(sweep, anchors_arr)
            (
                fixed_indices,
                _fixed_deltas,
                drive_idx2,
                sensor_idx2,
                _l_drive,
                _l_sensor,
                _packed_sweep_id,
                _packed_sigma_drive_mm,
                _packed_sigma_sensor_mm,
            ) = cost_fn._extract_sweep_arrays(sweep)
            fit = fit_ellipse_from_sweep(
                np.asarray(l_drive_abs, dtype=float),
                np.asarray(l_sensor_abs, dtype=float),
                residual_threshold=float("inf"),
                min_points=5,
                square_inputs=True,
            )
            coeffs_pred = predict_ellipse_coefficients(
                anchors_arr,
                [int(v) for v in fixed_indices],
                [float(v) for v in fixed_lengths_abs],
                int(drive_idx2),
                int(sensor_idx2),
                dimensions=int(dimensions),
            )
            coeffs_obs = np.asarray(getattr(fit, "coefficients", None), dtype=float).reshape(-1)
            if (
                coeffs_pred is None
                or coeffs_obs.size != 6
                or np.asarray(coeffs_pred, dtype=float).reshape(-1).size != 6
                or not np.all(np.isfinite(coeffs_obs))
                or not np.all(np.isfinite(np.asarray(coeffs_pred, dtype=float)))
                or not bool(getattr(fit, "valid", False))
            ):
                invalid_sweeps += 1
                continue
            coeffs_pred_arr = np.asarray(coeffs_pred, dtype=float).reshape(-1)
            obs_scale = float(np.linalg.norm(coeffs_obs))
            pred_scale = float(np.linalg.norm(coeffs_pred_arr))
            if (not np.isfinite(obs_scale)) or (not np.isfinite(pred_scale)) or obs_scale <= 0.0 or pred_scale <= 0.0:
                invalid_sweeps += 1
                continue
            obs_norm = coeffs_obs / obs_scale
            pred_norm = coeffs_pred_arr / pred_scale
            mismatch = min(
                float(np.linalg.norm(obs_norm - pred_norm)),
                float(np.linalg.norm(obs_norm + pred_norm)),
            )
            if not np.isfinite(mismatch):
                invalid_sweeps += 1
                continue
            total_cost += float(2.0 * (np.sqrt(1.0 + mismatch * mismatch) - 1.0))
            total_cost += float(max(float(violation_penalty), 0.0))
            valid_sweeps += 1
        except Exception:
            invalid_sweeps += 1

    if valid_sweeps <= 0:
        return float("inf"), 0, int(invalid_sweeps)
    return float(total_cost + 4.0 * float(invalid_sweeps)), int(valid_sweeps), int(invalid_sweeps)


def _normalize_anchor_solver_method(method: str) -> str:
    method_raw = str(method or "L-BFGS-B")
    method_norm = method_raw.strip().replace("_", "-").lower()
    if method_norm in ("slsqp", "sqp"):
        return "L-BFGS-B"
    if method_norm == "l-bfgs-b":
        return "L-BFGS-B"
    return str(method_raw)


def _solve_local_anchor_objective(
    transformed_dataset: dict,
    initial_guess: np.ndarray,
    *,
    objective_id: int,
    objective_name: str,
    objective_cost: Callable[[dict, np.ndarray], float],
    solve_restarts: int,
    solve_iterations: int,
    solve_optimizer: str,
    low_anchor_z: Optional[float],
) -> Optional[Dict[str, object]]:
    try:
        machine_type = _require_machine_type(
            transformed_dataset,
            context="anchor proposal dataset",
        )
    except Exception:
        return None

    anchors_seed = np.asarray(initial_guess, dtype=float)
    if anchors_seed.ndim != 2 or anchors_seed.shape[0] <= 0 or not np.all(np.isfinite(anchors_seed)):
        return None
    num_anchors = int(anchors_seed.shape[0])
    dimensions = int(anchors_seed.shape[1]) if anchors_seed.shape[1] > 0 else 0
    if dimensions <= 0:
        return None

    try:
        lb, ub = get_anchor_opt_bounds(
            machine_type,
            int(num_anchors),
            int(dimensions),
            low_anchor_z,
        )
        x0 = np.clip(
            np.asarray(
                anchors_matrix_to_opt_vec(anchors_seed, machine_type, low_anchor_z),
                dtype=float,
            ).reshape(-1),
            lb,
            ub,
        )
    except Exception:
        return None
    if x0.size <= 0:
        return None

    method_norm = _normalize_anchor_solver_method(str(solve_optimizer))
    bounds = list(zip(lb.tolist(), ub.tolist()))
    rng = np.random.default_rng(0)
    guesses = [np.asarray(x0, dtype=float)]
    span = np.maximum(ub - lb, 1.0)
    while len(guesses) < max(1, int(solve_restarts)):
        jitter = rng.normal(loc=0.0, scale=0.05 * span, size=x0.shape)
        guesses.append(np.clip(np.asarray(x0 + jitter, dtype=float), lb, ub))

    objective_cache: Dict[Tuple[float, ...], float] = {}
    eval_counter = 0

    def _objective(anchor_vec: np.ndarray) -> float:
        nonlocal eval_counter
        clipped = np.clip(np.asarray(anchor_vec, dtype=float).reshape(-1), lb, ub)
        key = tuple(float(v) for v in np.round(clipped, decimals=9).tolist())
        cached = objective_cache.get(key)
        if cached is not None and np.isfinite(cached):
            return float(cached)
        eval_counter += 1
        try:
            anchors_try = anchor_opt_vec_to_matrix(
                clipped,
                machine_type,
                int(num_anchors),
                int(dimensions),
                low_anchor_z,
            )
            value = float(objective_cost(transformed_dataset, anchors_try))
        except Exception:
            value = float("inf")
        if not np.isfinite(value):
            value = 1e12
        objective_cache[key] = float(value)
        return float(value)

    best_result = None
    best_fun = float("inf")
    best_x = np.asarray(x0, dtype=float)
    total_nit = 0
    for guess in guesses:
        try:
            result = minimize(
                _objective,
                np.asarray(guess, dtype=float),
                method=str(method_norm),
                bounds=bounds,
                options={"maxiter": int(max(1, solve_iterations))},
            )
        except Exception:
            continue
        result_x = np.clip(np.asarray(getattr(result, "x", guess), dtype=float).reshape(-1), lb, ub)
        result_fun = float(_objective(result_x))
        total_nit += int(getattr(result, "nit", 0) or 0)
        if result_fun + 1e-12 < best_fun:
            best_fun = float(result_fun)
            best_x = np.asarray(result_x, dtype=float)
            best_result = result

    if not np.isfinite(best_fun):
        return None

    try:
        anchors_best = anchor_opt_vec_to_matrix(
            best_x,
            machine_type,
            int(num_anchors),
            int(dimensions),
            low_anchor_z,
        )
        anchors_best = canonicalize_anchor_gauge(
            machine_type,
            np.asarray(anchors_best, dtype=float),
            low_anchor_z=low_anchor_z,
        )
    except Exception:
        return None

    return {
        "anchors": np.asarray(anchors_best, dtype=float),
        "cost": float(best_fun),
        "success": bool(best_result is not None and getattr(best_result, "success", True)),
        "objective_id": int(objective_id),
        "objective_name": str(objective_name),
        "solver": "scheduled_local_objective",
        "nfev": int(eval_counter),
        "nit": int(total_nit),
    }


def estimate_effective_radii_with_spool_model(
    dataset: dict,
    anchors: np.ndarray,
    *,
    find_radii_mode: str,
    find_buildup_mode: str,
    base_radii_mm: np.ndarray,
    modeled_buildup_factor: np.ndarray,
    spool_to_motor_gearing_factor: np.ndarray,
    mechanical_advantage: np.ndarray,
    lines_per_spool: np.ndarray,
    r0_bounds: Optional[Tuple[float, float]],
    b_bounds: Optional[Tuple[float, float]],
    r0_prior_sigma_mm: Optional[float],
    b_prior_sigma: Optional[float],
    spool_outer_iters: int,
    spool_inner_iters: int,
    theta0_mode: str,
    solve_restarts: int,
    solve_iterations: int,
    solve_optimizer: str,
    residual_threshold: float,
    spring_k_multiplier: float,
    use_flex: bool,
    pointwise_residual_mode: str,
    pointwise_filtering: bool,
    pointwise_global_mad: bool,
    sweep_wise_filtering: bool,
    sweep_metric: str,
    use_noise_mean: bool,
    sigma_source: str,
    robust_debug: bool,
    low_anchor_z: Optional[float] = None,
    prefer_zero_tension_angles: bool = False,
    scale_fix_levels: Optional[Sequence[int]] = None,
    fit_structure_levels: Optional[Sequence[int]] = None,
    filter_schedule: Optional[Sequence[Any]] = None,
    objective_schedule: Optional[Sequence[Any]] = None,
    initial_radii_mm: Optional[np.ndarray] = None,
    initial_buildup_factor: Optional[np.ndarray] = None,
    enable_prefit: bool = True,
    enable_bootstrap_anchor_refresh: bool = True,
) -> Tuple[np.ndarray, np.ndarray, SpoolModelParams, dict, Dict[str, object]]:
    base = np.asarray(base_radii_mm, dtype=float).reshape(-1)
    num_anchors = int(base.size)
    if num_anchors <= 0 or not np.all(np.isfinite(base)) or np.any(base <= 0.0):
        raise ValueError("Cannot estimate radii without finite positive base radii.")
    mode_r = _normalize_spool_find_mode(find_radii_mode)
    mode_b = _normalize_spool_find_mode(find_buildup_mode)
    theta0_mode_norm = _normalize_theta0_mode(theta0_mode)
    search_r = _spool_mode_enabled(mode_r)
    search_b = _spool_mode_enabled(mode_b)
    scale_fix_set = set(_parse_scale_fix_levels(scale_fix_levels))
    fit_structure_set = set(_parse_fit_structure_levels(fit_structure_levels))
    use_scale_fix_1 = 1 in scale_fix_set
    use_scale_fix_2 = 2 in scale_fix_set
    use_scale_fix_3 = 3 in scale_fix_set
    if not (search_r or search_b):
        raise ValueError("spool estimation requires find_radii and/or find_buildup_factor")

    modeled_b = np.asarray(modeled_buildup_factor, dtype=float).reshape(-1)
    spool_to_motor = np.asarray(spool_to_motor_gearing_factor, dtype=float).reshape(-1)
    mech_adv = np.asarray(mechanical_advantage, dtype=float).reshape(-1)
    lines = np.asarray(lines_per_spool, dtype=float).reshape(-1)
    if any(arr.size != num_anchors for arr in (modeled_b, spool_to_motor, mech_adv, lines)):
        raise ValueError("spool parameter length mismatch for radii estimation")

    lo_parts: List[np.ndarray] = []
    hi_parts: List[np.ndarray] = []
    lo_r = np.zeros(0, dtype=float)
    hi_r = np.zeros(0, dtype=float)
    if search_r:
        lo_r, hi_r = _resolve_r0_bounds(base, find_radii_mode=mode_r, r0_bounds=r0_bounds)
        lo_parts.append(lo_r)
        hi_parts.append(hi_r)
    lo_b = np.zeros(0, dtype=float)
    hi_b = np.zeros(0, dtype=float)
    if search_b:
        lo_b, hi_b = _resolve_b_bounds(modeled_b, find_buildup_mode=mode_b, b_bounds=b_bounds)
        lo_parts.append(lo_b)
        hi_parts.append(hi_b)

    if lo_parts:
        lo = np.concatenate(lo_parts).astype(float)
        hi = np.concatenate(hi_parts).astype(float)
    else:
        lo = np.zeros(0, dtype=float)
        hi = np.zeros(0, dtype=float)

    sigma_r = None
    if r0_prior_sigma_mm is not None:
        try:
            sigma_raw = float(r0_prior_sigma_mm)
        except (TypeError, ValueError):
            sigma_raw = float("nan")
        if np.isfinite(sigma_raw) and sigma_raw > 0.0:
            sigma_r = sigma_raw

    sigma_b = None
    if b_prior_sigma is not None:
        try:
            sigma_raw = float(b_prior_sigma)
        except (TypeError, ValueError):
            sigma_raw = float("nan")
        if np.isfinite(sigma_raw) and sigma_raw > 0.0:
            sigma_b = sigma_raw
    elif search_b:
        sigma_b = float(_DEFAULT_B_PRIOR_SIGMA)

    sigma_rpair = None
    if search_r and mode_r == "per-anchor" and num_anchors > 1:
        if sigma_r is not None:
            sigma_rpair = float(max(sigma_r, 1e-6))
        else:
            sigma_rpair = float(_DEFAULT_RADIUS_PAIR_SIGMA_MM)
        if not np.isfinite(sigma_rpair) or sigma_rpair <= 0.0:
            sigma_rpair = float(_DEFAULT_RADIUS_PAIR_SIGMA_MM)

    anchors_current = np.asarray(anchors, dtype=float)
    if anchors_current.ndim != 2 or anchors_current.shape[0] != num_anchors:
        raise ValueError("anchor estimate shape mismatch")
    if not np.all(np.isfinite(anchors_current)):
        raise ValueError("anchor estimate contains non-finite values")
    requested_filter_schedule = _parse_filter_schedule(
        filter_schedule if filter_schedule is not None else ("warmup",),
        label="filter_schedule",
    )
    requested_objective_schedule = _resolve_objective_schedule(
        objective_schedule,
        num_passes=len(requested_filter_schedule),
        label="objective_schedule",
    )

    def _build_locked_filter_dataset(
        source_dataset: dict,
        anchors_eval: np.ndarray,
        *,
        pointwise_enabled: bool,
        sweep_enabled: bool,
    ) -> Tuple[Optional[dict], Dict[str, object]]:
        info: Dict[str, object] = {
            "attempted": True,
            "success": False,
            "message": "not_run",
            "pointwise_enabled": bool(pointwise_enabled),
            "sweep_wise_enabled": bool(sweep_enabled),
            "sweeps_in": 0,
            "sweeps_out": 0,
            "points_in": 0,
            "points_out": 0,
            "sweeps_dropped_by_sweep_mask": 0,
            "sweeps_dropped_empty_after_point_mask": 0,
            "point_mask_mismatch_sweeps": 0,
        }
        if not (bool(pointwise_enabled) or bool(sweep_enabled)):
            info["message"] = "mask lock skipped (no filtering enabled)"
            return None, info
        sweeps_in = source_dataset.get("sweeps")
        if not isinstance(sweeps_in, list):
            info["message"] = "mask lock skipped (dataset has no sweeps list)"
            return None, info
        info["sweeps_in"] = int(len(sweeps_in))
        anchors_arr = np.asarray(anchors_eval, dtype=float)
        if (
            anchors_arr.ndim != 2
            or anchors_arr.shape[0] != num_anchors
            or not np.all(np.isfinite(anchors_arr))
        ):
            info["message"] = "mask lock skipped (invalid anchors)"
            return None, info

        try:
            cost_fn = _build_ellipse_cost_function(
                source_dataset,
                residual_threshold=float(residual_threshold),
                spring_k_multiplier=float(spring_k_multiplier),
                use_flex=bool(use_flex),
                pointwise_residual_mode=str(pointwise_residual_mode),
                pointwise_filtering=bool(pointwise_enabled),
                pointwise_global_mad=bool(pointwise_global_mad),
                sweep_wise_filtering=bool(sweep_enabled),
                sweep_metric=str(sweep_metric),
                use_noise_mean=bool(use_noise_mean),
                noise_normalized=True,
                sigma_source=str(sigma_source),
            )
            if not hasattr(cost_fn, "_pointwise_entries"):
                info["message"] = "mask lock skipped (cost function has no pointwise entries API)"
                return None, info
            entries_obj = cost_fn._pointwise_entries(anchors_arr)  # type: ignore[attr-defined]
            if not isinstance(entries_obj, tuple) or len(entries_obj) < 1:
                info["message"] = "mask lock skipped (pointwise entries unavailable)"
                return None, info
            entries_raw = entries_obj[0]
            entries = list(entries_raw) if isinstance(entries_raw, (list, tuple)) else []

            keep_mask_arr: Optional[np.ndarray] = None
            if bool(sweep_enabled):
                if not hasattr(cost_fn, "_sweep_wise_keep_mask"):
                    info["message"] = "mask lock skipped (cost function has no sweep mask API)"
                    return None, info
                sweep_metrics: List[float] = []
                for entry in entries:
                    if isinstance(entry, dict):
                        sweep_metrics.append(float(entry.get("sweep_metric", float("inf"))))
                    else:
                        sweep_metrics.append(float("inf"))
                keep_mask_obj = cost_fn._sweep_wise_keep_mask(sweep_metrics)  # type: ignore[attr-defined]
                if isinstance(keep_mask_obj, tuple) and len(keep_mask_obj) >= 1:
                    keep_mask = keep_mask_obj[0]
                else:
                    keep_mask = None
                if keep_mask is not None:
                    keep_mask_arr = np.asarray(keep_mask, dtype=bool).reshape(-1)

            sweeps_out: List[dict] = []
            sweeps_dropped_sweep = 0
            sweeps_dropped_empty = 0
            point_mask_mismatch_sweeps = 0
            points_in = 0
            points_out = 0
            for idx, sweep in enumerate(sweeps_in):
                if not isinstance(sweep, dict):
                    continue
                if keep_mask_arr is not None and idx < keep_mask_arr.size and not bool(keep_mask_arr[idx]):
                    sweeps_dropped_sweep += 1
                    continue
                sweep_out = dict(sweep)
                points_raw = sweep.get("data_points")
                if not isinstance(points_raw, list):
                    sweeps_out.append(sweep_out)
                    continue
                points_in += int(len(points_raw))
                points_keep = list(points_raw)
                if bool(pointwise_enabled):
                    entry = entries[idx] if idx < len(entries) and isinstance(entries[idx], dict) else None
                    inlier_mask = None if entry is None else entry.get("_inlier_mask")
                    if isinstance(inlier_mask, np.ndarray):
                        inlier_arr = np.asarray(inlier_mask, dtype=bool).reshape(-1)
                        if inlier_arr.size == len(points_raw):
                            points_keep = [pt for pt, keep in zip(points_raw, inlier_arr.tolist()) if bool(keep)]
                        else:
                            point_mask_mismatch_sweeps += 1
                if len(points_raw) > 0 and len(points_keep) <= 0:
                    sweeps_dropped_empty += 1
                    continue
                points_out += int(len(points_keep))
                sweep_out["data_points"] = points_keep
                sweeps_out.append(sweep_out)

            if len(sweeps_out) <= 0:
                info["message"] = "mask lock skipped (all sweeps dropped)"
                info["sweeps_dropped_by_sweep_mask"] = int(sweeps_dropped_sweep)
                info["sweeps_dropped_empty_after_point_mask"] = int(sweeps_dropped_empty)
                info["point_mask_mismatch_sweeps"] = int(point_mask_mismatch_sweeps)
                info["points_in"] = int(points_in)
                info["points_out"] = int(points_out)
                return None, info

            out_dataset = dict(source_dataset)
            out_dataset["sweeps"] = sweeps_out
            info["success"] = True
            info["message"] = "mask lock dataset created"
            info["sweeps_out"] = int(len(sweeps_out))
            info["points_in"] = int(points_in)
            info["points_out"] = int(points_out)
            info["sweeps_dropped_by_sweep_mask"] = int(sweeps_dropped_sweep)
            info["sweeps_dropped_empty_after_point_mask"] = int(sweeps_dropped_empty)
            info["point_mask_mismatch_sweeps"] = int(point_mask_mismatch_sweeps)
            return out_dataset, info
        except Exception as exc:
            info["message"] = f"mask lock failed: {exc}"
            return None, info

    if len(requested_filter_schedule) > 1:
        anchors_seed = np.asarray(anchors_current, dtype=float)
        radii_seed = (
            None
            if initial_radii_mm is None
            else np.asarray(initial_radii_mm, dtype=float).reshape(-1)
        )
        buildup_seed = (
            None
            if initial_buildup_factor is None
            else np.asarray(initial_buildup_factor, dtype=float).reshape(-1)
        )
        schedule_history: List[Dict[str, object]] = []
        final_tuple: Optional[Tuple[np.ndarray, np.ndarray, SpoolModelParams, dict, Dict[str, object]]] = None
        final_calibration: Optional[Dict[str, object]] = None
        schedule_pointwise_filtering = bool(pointwise_filtering)
        schedule_sweep_wise_filtering = bool(sweep_wise_filtering)
        constant_mask_dataset: Optional[dict] = None
        constant_mask_info: Dict[str, object] = {
            "attempted": False,
            "success": False,
            "message": "not_attempted",
        }

        for schedule_idx, pass_kind_raw in enumerate(requested_filter_schedule):
            pass_kind = _normalize_filter_schedule_pass(
                pass_kind_raw,
                label="filter_schedule",
            )
            pass_objective_id = int(requested_objective_schedule[schedule_idx])
            pass_objective_name = _objective_name(pass_objective_id)
            pass_enable_prefit = bool(enable_prefit) if schedule_idx == 0 else False
            pass_enable_bootstrap = (
                bool(enable_bootstrap_anchor_refresh) if schedule_idx == 0 else False
            )
            pass_dataset = dataset
            if pass_kind == "warmup":
                constant_mask_dataset = None
                constant_mask_info = {
                    "attempted": False,
                    "success": False,
                    "message": "mask cleared by warmup pass",
                }
                pass_pointwise_filtering = False
                pass_sweep_wise_filtering = False
            elif pass_kind == "dynamic":
                pass_pointwise_filtering = bool(schedule_pointwise_filtering)
                pass_sweep_wise_filtering = bool(schedule_sweep_wise_filtering)
            else:
                # Constant-mask phase: run on the locked filtered dataset and
                # keep runtime filters off so the mask remains fixed.
                pass_pointwise_filtering = False
                pass_sweep_wise_filtering = False
                if isinstance(constant_mask_dataset, dict):
                    pass_dataset = constant_mask_dataset
                else:
                    raise ValueError(
                        "filter_schedule constant pass requires a locked mask from a prior "
                        "dynamic pass"
                    )
            pass_scale_fix_set = set(scale_fix_set)
            # scale_fix 2: final polish only on the last filter-schedule pass.
            # scale_fix 3: final polish on every filter-schedule pass.
            if (2 in pass_scale_fix_set) and (3 not in pass_scale_fix_set) and (
                schedule_idx < len(requested_filter_schedule) - 1
            ):
                pass_scale_fix_set.discard(2)
            (
                eff_r_pass,
                fit_anchors_pass,
                spool_params_pass,
                transformed_pass,
                fit_info_pass,
            ) = estimate_effective_radii_with_spool_model(
                pass_dataset,
                anchors_seed,
                find_radii_mode=find_radii_mode,
                find_buildup_mode=find_buildup_mode,
                base_radii_mm=base_radii_mm,
                modeled_buildup_factor=modeled_buildup_factor,
                spool_to_motor_gearing_factor=spool_to_motor_gearing_factor,
                mechanical_advantage=mechanical_advantage,
                lines_per_spool=lines_per_spool,
                r0_bounds=r0_bounds,
                b_bounds=b_bounds,
                r0_prior_sigma_mm=r0_prior_sigma_mm,
                b_prior_sigma=b_prior_sigma,
                spool_outer_iters=spool_outer_iters,
                spool_inner_iters=spool_inner_iters,
                theta0_mode=theta0_mode,
                solve_restarts=solve_restarts,
                solve_iterations=solve_iterations,
                solve_optimizer=solve_optimizer,
                residual_threshold=residual_threshold,
                spring_k_multiplier=spring_k_multiplier,
                use_flex=use_flex,
                pointwise_residual_mode=pointwise_residual_mode,
                pointwise_filtering=pass_pointwise_filtering,
                pointwise_global_mad=pointwise_global_mad,
                sweep_wise_filtering=pass_sweep_wise_filtering,
                sweep_metric=sweep_metric,
                use_noise_mean=use_noise_mean,
                sigma_source=sigma_source,
                robust_debug=robust_debug,
                prefer_zero_tension_angles=prefer_zero_tension_angles,
                scale_fix_levels=sorted(int(v) for v in pass_scale_fix_set),
                fit_structure_levels=sorted(int(v) for v in fit_structure_set),
                filter_schedule=None,
                objective_schedule=[int(pass_objective_id)],
                initial_radii_mm=radii_seed,
                initial_buildup_factor=buildup_seed,
                enable_prefit=pass_enable_prefit,
                enable_bootstrap_anchor_refresh=pass_enable_bootstrap,
            )
            cal_pass = calibrate_elliptical(
                transformed_pass,
                output_path=None,
                residual_threshold=float(residual_threshold),
                num_restarts=int(solve_restarts),
                max_iterations=int(solve_iterations),
                method=str(solve_optimizer),
                spring_k_multiplier=float(spring_k_multiplier),
                use_flex=bool(use_flex),
                verbose=False,
                use_parallel=False,
                pointwise_residual_mode=str(pointwise_residual_mode),
                robust_debug=bool(robust_debug),
                pointwise_filtering=bool(pass_pointwise_filtering),
                pointwise_global_mad=bool(pointwise_global_mad),
                sweep_wise_filtering=bool(pass_sweep_wise_filtering),
                sweep_metric=str(sweep_metric),
                use_noise_mean=bool(use_noise_mean),
                sigma_source=str(sigma_source),
                generate_report=False,
                residuals_csv=None,
                initial_guess=np.asarray(fit_anchors_pass, dtype=float),
                low_anchor_z=low_anchor_z,
            )
            plan_tmp = {
                "length_model": {
                    "find_radii": bool(search_r),
                    "find_buildup_factor": bool(search_b),
                    "fit_structure_levels": [int(v) for v in sorted(fit_structure_set)],
                },
                "calibration": cal_pass,
                "cost_raw": cal_pass.get("cost"),
                "cost_noise_normalized": cal_pass.get("cost"),
                "fit_structure_levels": [int(v) for v in sorted(fit_structure_set)],
            }
            pass_score_ui, pass_rank_score, pass_score_basis = _plan_score_ui(plan_tmp)
            if pass_kind == "dynamic":
                constant_mask_info["attempted"] = True
                mask_dataset, mask_info = _build_locked_filter_dataset(
                    pass_dataset,
                    np.asarray(cal_pass.get("anchors"), dtype=float),
                    pointwise_enabled=bool(schedule_pointwise_filtering),
                    sweep_enabled=bool(schedule_sweep_wise_filtering),
                )
                constant_mask_info.update(mask_info)
                if isinstance(mask_dataset, dict):
                    constant_mask_dataset = mask_dataset
                else:
                    constant_mask_dataset = None
            schedule_history.append(
                {
                    "filter_pass_index": int(schedule_idx + 1),
                    "filter_pass": str(pass_kind),
                    "objective_id": int(pass_objective_id),
                    "objective_name": str(pass_objective_name),
                    "prefit_enabled": bool(pass_enable_prefit),
                    "bootstrap_enabled": bool(pass_enable_bootstrap),
                    "pointwise_filtering": bool(pass_pointwise_filtering),
                    "sweep_wise_filtering": bool(pass_sweep_wise_filtering),
                    "constant_mask_available": bool(isinstance(constant_mask_dataset, dict)),
                    "constant_mask_applied": bool(
                        pass_kind == "constant" and isinstance(constant_mask_dataset, dict)
                    ),
                    "scale_fix_levels": [int(v) for v in sorted(pass_scale_fix_set)],
                    "scale_fix_2_active": bool(2 in pass_scale_fix_set),
                    "scale_fix_3_active": bool(3 in pass_scale_fix_set),
                    "final_scale_polish_attempted": bool(
                        ((fit_info_pass.get("final_scale_polish") or {}).get("attempted", False))
                        if isinstance(fit_info_pass, dict)
                        else False
                    ),
                    "score_ui": float(pass_score_ui) if np.isfinite(pass_score_ui) else None,
                    "rank_score": float(pass_rank_score) if np.isfinite(pass_rank_score) else None,
                    "score_basis": str(pass_score_basis),
                    "cost_noise_normalized": (
                        float(cal_pass.get("cost"))
                        if isinstance(cal_pass.get("cost"), (int, float))
                        and np.isfinite(float(cal_pass.get("cost")))
                        else None
                    ),
                    "anchors": np.asarray(cal_pass.get("anchors"), dtype=float).tolist(),
                    "effective_radii_mm": np.asarray(eff_r_pass, dtype=float).tolist(),
                }
            )
            anchors_seed = np.asarray(cal_pass.get("anchors"), dtype=float)
            radii_seed = np.asarray(eff_r_pass, dtype=float)
            buildup_seed = np.asarray(
                fit_info_pass.get("best_modeled_buildup_factor", modeled_b.tolist()),
                dtype=float,
            ).reshape(-1)
            final_tuple = (
                np.asarray(eff_r_pass, dtype=float),
                np.asarray(fit_anchors_pass, dtype=float),
                spool_params_pass,
                transformed_pass,
                fit_info_pass,
            )
            final_calibration = cal_pass

        if final_tuple is None:
            raise RuntimeError("filter-schedule loop failed to produce a spool-fit result")
        eff_r_final, fit_anchors_final, spool_params_final, transformed_final, fit_info_final = final_tuple
        fit_info_out = dict(fit_info_final)
        fit_info_out["filter_schedule_requested"] = [str(v) for v in requested_filter_schedule]
        fit_info_out["objective_schedule_requested"] = [
            int(v) for v in requested_objective_schedule
        ]
        fit_info_out["filter_schedule_history"] = list(schedule_history)
        fit_info_out["filter_schedule_constant_mask"] = dict(constant_mask_info)
        if isinstance(final_calibration, dict):
            fit_info_out["filter_schedule_final_calibration"] = final_calibration
        return (
            np.asarray(eff_r_final, dtype=float),
            np.asarray(fit_anchors_final, dtype=float),
            spool_params_final,
            transformed_final,
            fit_info_out,
        )

    kinds = _spool_opt_kinds(
        num_anchors=num_anchors,
        find_radii_mode=mode_r,
        find_buildup_mode=mode_b,
    )
    outer_iters = max(1, int(spool_outer_iters))
    inner_iters = max(1, int(spool_inner_iters))

    def _build_dataset_and_params(
        radii_mm: np.ndarray,
        buildup_factor: np.ndarray,
    ) -> Tuple[SpoolModelParams, dict]:
        spool_params = build_spool_model_params(
            dataset,
            base_radii_mm=base,
            modeled_radii_mm=radii_mm,
            modeled_buildup_factor=buildup_factor,
            spool_to_motor_gearing_factor=spool_to_motor,
            mechanical_advantage=mech_adv,
            lines_per_spool=lines,
            base_buildup_factor=np.zeros(num_anchors, dtype=float),
            theta0_mode=theta0_mode_norm,
            prefer_zero_tension_angles=bool(prefer_zero_tension_angles),
        )
        transformed = dataset_with_modeled_lengths(
            dataset,
            spool_params,
            prefer_zero_tension_angles=bool(prefer_zero_tension_angles),
        )
        return spool_params, transformed

    def _prior_cost(radii_mm: np.ndarray, buildup_factor: np.ndarray) -> float:
        prior = 0.0
        if search_r and sigma_r is not None:
            radii_arr = np.asarray(radii_mm, dtype=float).reshape(-1)
            if mode_r == "global":
                prior += float(((float(np.median(radii_arr)) - float(np.median(base))) / sigma_r) ** 2.0)
            else:
                prior += float(np.sum(((radii_arr - base) / sigma_r) ** 2.0))
        if search_b and sigma_b is not None:
            b_arr = np.asarray(buildup_factor, dtype=float).reshape(-1)
            if mode_b == "global":
                prior += float(((float(np.median(b_arr)) - float(np.median(modeled_b))) / sigma_b) ** 2.0)
            else:
                prior += float(np.sum(((b_arr - modeled_b) / sigma_b) ** 2.0))
        if sigma_rpair is not None:
            radii_arr = np.asarray(radii_mm, dtype=float).reshape(-1)
            r_mean = float(np.mean(radii_arr))
            prior += float(np.sum(((radii_arr - r_mean) / sigma_rpair) ** 2.0))
        return float(prior)

    def _spool_anchor_step_gate(
        start_total: float,
        fitted_total: float,
    ) -> Tuple[bool, float, float]:
        if not np.isfinite(start_total) or not np.isfinite(fitted_total):
            return False, float("nan"), float("nan")
        improvement = float(start_total) - float(fitted_total)
        threshold = max(
            1e-9,
            float(_SPOOL_ANCHOR_STEP_IMPROVE_REL) * max(1.0, abs(float(start_total))),
        )
        return bool(improvement > threshold), float(improvement), float(threshold)

    # Keep spool-step data term aligned with the main optimization metric.
    spool_noise_normalized = True
    raw_machine_type = dataset.get("machine_type")
    machine_type_for_risk = (
        str(raw_machine_type)
        if isinstance(raw_machine_type, str) and str(raw_machine_type) in MACHINE_TYPE_CHOICES
        else None
    )
    dimensions_for_risk = None
    try:
        dims_raw = int(dataset.get("dimensions"))
    except Exception:
        dims_raw = 0
    if dims_raw > 0:
        dimensions_for_risk = int(dims_raw)
    elif anchors_current.ndim == 2 and anchors_current.shape[1] > 0:
        dimensions_for_risk = int(anchors_current.shape[1])

    eval_bundle_cache: List[Tuple[dict, Tuple[float, ...], Dict[str, object]]] = []
    cost_fn_cache: List[Tuple[dict, EllipseCostFunction]] = []

    def _cached_cost_fn(transformed_dataset: dict) -> Optional[EllipseCostFunction]:
        for ds_ref, cost_fn_ref in reversed(cost_fn_cache):
            if ds_ref is transformed_dataset:
                return cost_fn_ref
        try:
            cost_fn = _build_ellipse_cost_function(
                transformed_dataset,
                residual_threshold=float(residual_threshold),
                spring_k_multiplier=float(spring_k_multiplier),
                use_flex=bool(use_flex),
                pointwise_residual_mode=str(pointwise_residual_mode),
                pointwise_filtering=bool(pointwise_filtering),
                pointwise_global_mad=bool(pointwise_global_mad),
                sweep_wise_filtering=bool(sweep_wise_filtering),
                sweep_metric=str(sweep_metric),
                use_noise_mean=bool(use_noise_mean),
                noise_normalized=bool(spool_noise_normalized),
                sigma_source=str(sigma_source),
            )
        except Exception:
            return None
        cost_fn_cache.append((transformed_dataset, cost_fn))
        if len(cost_fn_cache) > 16:
            del cost_fn_cache[: (len(cost_fn_cache) - 16)]
        return cost_fn

    def _cached_eval_state_get(
        transformed_dataset: dict,
        anchor_key: Tuple[float, ...],
    ) -> Optional[Dict[str, object]]:
        for ds_ref, key_ref, state_ref in reversed(eval_bundle_cache):
            if ds_ref is transformed_dataset and key_ref == anchor_key:
                return state_ref
        return None

    def _cached_eval_state_set(
        transformed_dataset: dict,
        anchor_key: Tuple[float, ...],
        state: Dict[str, object],
    ) -> None:
        for idx in range(len(eval_bundle_cache) - 1, -1, -1):
            ds_ref, key_ref, _state_ref = eval_bundle_cache[idx]
            if ds_ref is transformed_dataset and key_ref == anchor_key:
                eval_bundle_cache[idx] = (transformed_dataset, anchor_key, state)
                return
        eval_bundle_cache.append((transformed_dataset, anchor_key, state))
        if len(eval_bundle_cache) > 64:
            del eval_bundle_cache[: (len(eval_bundle_cache) - 64)]

    def _get_eval_state(
        transformed_dataset: dict,
        anchors_eval: np.ndarray,
    ) -> Dict[str, object]:
        anchor_vec = np.asarray(anchors_eval, dtype=float).ravel()
        anchor_key = tuple(float(v) for v in np.asarray(anchor_vec, dtype=float).tolist())
        cached = _cached_eval_state_get(transformed_dataset, anchor_key)
        if cached is not None:
            return cached

        state: Dict[str, object] = {
            "anchor_vec": np.asarray(anchor_vec, dtype=float),
            "legacy_cost": float("inf"),
            "legacy_ready": False,
            "cost_fn": None,
            "detailed_total_cost": None,
            "base_noise_metrics": {},
            "base_risk_metric": None,
            "rows": None,
            "rescored_noise_metrics": None,
            "rescored_risk_metric": None,
        }

        cost_fn = _cached_cost_fn(transformed_dataset)
        state["cost_fn"] = cost_fn
        if cost_fn is not None:
            try:
                detailed = cost_fn.evaluate_detailed(anchor_vec)
                detailed_total = float(detailed.total_cost)
                base_noise = (
                    dict(detailed.noise_metrics)
                    if isinstance(detailed.noise_metrics, dict)
                    else {}
                )
                state["detailed_total_cost"] = detailed_total
                state["base_noise_metrics"] = base_noise
                state["base_risk_metric"] = _trimmed_risk_metric_from_components(base_noise)
            except Exception:
                pass

        _cached_eval_state_set(transformed_dataset, anchor_key, state)
        return state

    def _ensure_bundle_legacy_cost(
        state: Dict[str, object],
        transformed_dataset: dict,
        anchors_eval: np.ndarray,
    ) -> float:
        if bool(state.get("legacy_ready", False)):
            return float(state.get("legacy_cost", float("inf")))

        legacy_cost = float("inf")
        cost_fn = state.get("cost_fn")
        default_eval_cost_fn = globals().get("_DEFAULT_EVALUATE_COST_AT_ANCHORS", None)
        eval_cost_overridden = (
            callable(default_eval_cost_fn)
            and _evaluate_cost_at_anchors is not default_eval_cost_fn
        )
        if (
            not eval_cost_overridden
            and cost_fn is not None
            and hasattr(cost_fn, "evaluate")
            and callable(getattr(cost_fn, "evaluate", None))
        ):
            try:
                anchor_vec = np.asarray(state.get("anchor_vec"), dtype=float).ravel()
                legacy_cost = float(cost_fn.evaluate(anchor_vec))
            except Exception:
                legacy_cost = float("inf")
        if eval_cost_overridden or not np.isfinite(legacy_cost):
            legacy_cost = float(
                _evaluate_cost_at_anchors(
                    transformed_dataset,
                    anchors_eval,
                    residual_threshold=float(residual_threshold),
                    spring_k_multiplier=float(spring_k_multiplier),
                    use_flex=bool(use_flex),
                    pointwise_residual_mode=str(pointwise_residual_mode),
                    pointwise_filtering=bool(pointwise_filtering),
                    pointwise_global_mad=bool(pointwise_global_mad),
                    sweep_wise_filtering=bool(sweep_wise_filtering),
                    sweep_metric=str(sweep_metric),
                    use_noise_mean=bool(use_noise_mean),
                    noise_normalized=bool(spool_noise_normalized),
                    sigma_source=str(sigma_source),
                )
            )
        state["legacy_cost"] = float(legacy_cost)
        state["legacy_ready"] = True
        return float(legacy_cost)

    def _ensure_bundle_rows_rescored(
        state: Dict[str, object],
    ) -> Tuple[Optional[List[dict]], Optional[dict], Optional[float]]:
        cached_noise = state.get("rescored_noise_metrics")
        if isinstance(cached_noise, dict):
            cached_rows = state.get("rows")
            cached_risk = state.get("rescored_risk_metric")
            return (
                cached_rows if isinstance(cached_rows, list) else None,
                dict(cached_noise),
                (
                    float(cached_risk)
                    if isinstance(cached_risk, (int, float)) and np.isfinite(cached_risk)
                    else None
                ),
            )

        rows: Optional[List[dict]] = None
        base_noise = state.get("base_noise_metrics")
        noise_metrics = dict(base_noise) if isinstance(base_noise, dict) else {}
        cost_fn = state.get("cost_fn")
        if (
            cost_fn is not None
            and hasattr(cost_fn, "pointwise_residual_rows")
            and callable(getattr(cost_fn, "pointwise_residual_rows", None))
        ):
            try:
                anchor_vec = np.asarray(state.get("anchor_vec"), dtype=float).ravel()
                rows = cost_fn.pointwise_residual_rows(anchor_vec)
                sigma_model_mm = noise_metrics.get("sigma_model_mm")
                if _float_or_none(sigma_model_mm) is None:
                    sigma_model_mm = noise_metrics.get("sigma_used_mm")
                rescored = _compute_tau_mad_rescore_from_rows(
                    rows,
                    cost_noise_normalized_old=(
                        state.get("detailed_total_cost")
                        if state.get("detailed_total_cost") is not None
                        else noise_metrics.get("cost_noise_normalized_old")
                    ),
                    chi2_red_old=noise_metrics.get("chi2_red"),
                    sigma_model_mm=sigma_model_mm,
                    params_count=noise_metrics.get("params"),
                )
                noise_metrics = {**noise_metrics, **rescored}
            except Exception:
                pass
        risk_metric = _trimmed_risk_metric_from_components(noise_metrics)
        state["rows"] = rows
        state["rescored_noise_metrics"] = dict(noise_metrics)
        state["rescored_risk_metric"] = risk_metric
        return rows, dict(noise_metrics), risk_metric

    def _spool_rel_std(
        transformed_dataset: dict,
        anchors_eval: np.ndarray,
        noise_metrics: Optional[dict],
    ) -> Optional[float]:
        if not isinstance(noise_metrics, dict):
            return None
        if machine_type_for_risk is None or dimensions_for_risk is None:
            return None
        anchors_arr = np.asarray(anchors_eval, dtype=float)
        if (
            anchors_arr.ndim != 2
            or anchors_arr.shape[0] != num_anchors
            or not np.all(np.isfinite(anchors_arr))
        ):
            return None
        try:
            sweep_configs = dataset_sweep_configs(transformed_dataset)
            l2_scale = l2_scale_for_machine(
                str(machine_type_for_risk),
                int(num_anchors),
                int(dimensions_for_risk),
            )
            info_obs = total_information_matrix(
                anchors_arr,
                sweep_configs,
                machine_type=str(machine_type_for_risk),
                num_anchors=int(num_anchors),
                dimensions=int(dimensions_for_risk),
                l2_scale=l2_scale,
                fd_eps_mm=1.0,
            )
            cov = _estimate_anchor_covariance(info_obs, regularization=0.0)
            cov_scaled, _cov_scale, _cov_label = _scale_covariance(cov, noise_metrics)
            ci = _confidence_intervals(cov_scaled)
            max_std = _float_or_none(ci.get("max_std_mm")) if isinstance(ci, dict) else None
            if max_std is None:
                cov_std = _covariance_diag_std(cov_scaled)
                if cov_std is not None:
                    finite = cov_std[np.isfinite(cov_std)]
                    if finite.size:
                        max_std = float(np.max(finite))
            workspace_diag = _workspace_diag_mm(
                transformed_dataset,
                anchors_arr,
                machine_type=str(machine_type_for_risk),
                num_anchors=int(num_anchors),
                dimensions=int(dimensions_for_risk),
            )
            workspace = _float_or_none(workspace_diag)
            if max_std is None or workspace is None or workspace <= 0.0:
                return None
            return float(max_std) / float(workspace)
        except Exception:
            return None

    def _eval_bundle(
        transformed_dataset: dict,
        anchors_eval: np.ndarray,
        *,
        noise_metrics_hint: Optional[dict] = None,
        rows_policy: str = "if_missing",
        need_legacy_cost: bool = False,
    ) -> Tuple[float, Optional[dict], Optional[List[dict]], Optional[float]]:
        rows_policy_norm = str(rows_policy or "if_missing").strip().lower()
        if rows_policy_norm not in ("never", "if-missing", "if_missing", "always"):
            rows_policy_norm = "if_missing"
        if rows_policy_norm == "if-missing":
            rows_policy_norm = "if_missing"

        legacy_cost = float("inf")
        rows: Optional[List[dict]] = None
        noise_metrics: Optional[dict]
        risk_metric: Optional[float]

        if isinstance(noise_metrics_hint, dict):
            noise_metrics = dict(noise_metrics_hint)
            risk_metric = _trimmed_risk_metric_from_components(noise_metrics)
            if need_legacy_cost:
                state = _get_eval_state(transformed_dataset, anchors_eval)
                legacy_cost = _ensure_bundle_legacy_cost(
                    state,
                    transformed_dataset,
                    anchors_eval,
                )
            return (
                float(legacy_cost),
                noise_metrics,
                rows,
                risk_metric,
            )

        state = _get_eval_state(transformed_dataset, anchors_eval)
        base_noise = state.get("base_noise_metrics")
        noise_metrics = dict(base_noise) if isinstance(base_noise, dict) else {}
        base_risk = state.get("base_risk_metric")
        if isinstance(base_risk, (int, float)) and np.isfinite(base_risk):
            risk_metric = float(base_risk)
        else:
            risk_metric = _trimmed_risk_metric_from_components(noise_metrics)

        use_rescored = False
        if rows_policy_norm == "always":
            use_rescored = True
        elif (
            rows_policy_norm == "if_missing"
            and _float_or_none(noise_metrics.get("chi2_red_tau_d_trimmed_direct")) is None
        ):
            use_rescored = True

        if use_rescored:
            rows_rescored, noise_rescored, risk_rescored = _ensure_bundle_rows_rescored(state)
            rows = rows_rescored
            noise_metrics = (
                dict(noise_rescored) if isinstance(noise_rescored, dict) else noise_metrics
            )
            risk_metric = risk_rescored

        if need_legacy_cost:
            legacy_cost = _ensure_bundle_legacy_cost(
                state,
                transformed_dataset,
                anchors_eval,
            )

        return (
            float(legacy_cost),
            noise_metrics if isinstance(noise_metrics, dict) else None,
            rows,
            risk_metric,
        )

    def _spool_trimmed_risk_metric(
        transformed_dataset: dict,
        anchors_eval: np.ndarray,
        *,
        noise_metrics_hint: Optional[dict] = None,
    ) -> Optional[float]:
        try:
            _legacy_cost, _noise_metrics, _rows, risk_metric = _eval_bundle(
                transformed_dataset,
                anchors_eval,
                noise_metrics_hint=noise_metrics_hint,
                rows_policy="if_missing",
                need_legacy_cost=False,
            )
            return risk_metric
        except Exception:
            return None

    def _data_cost(transformed_dataset: dict, anchors_eval: np.ndarray, blend_weight: float = _SCORE_UI_LAYERED_RISK_BLEND_WEIGHT) -> float:
        legacy_cost, _noise_metrics, _rows, risk_metric = _eval_bundle(
            transformed_dataset,
            anchors_eval,
            rows_policy="if_missing",
            need_legacy_cost=True,
        )
        if np.isfinite(legacy_cost) and risk_metric is not None and np.isfinite(risk_metric):
            return float(
                legacy_cost
                + blend_weight * max(float(risk_metric), 0.0)
            )
        return float(legacy_cost)

    objective_id = int(requested_objective_schedule[0])
    objective_name = _objective_name(objective_id)

    def _objective_cost(
        transformed_dataset: dict,
        anchors_eval: np.ndarray,
    ) -> float:
        if int(objective_id) == 0:
            score, valid_sweeps, _invalid_sweeps = _ellipse_prefit_score(transformed_dataset)
            if valid_sweeps <= 0 or not np.isfinite(score):
                return float("inf")
            return float(score)
        if int(objective_id) == 2:
            score, valid_points, _invalid_points = simulation_position_objective_score(
                transformed_dataset,
                anchors_eval,
                use_noise_mean=bool(use_noise_mean),
            )
            if valid_points > 0 and np.isfinite(score):
                return float(score)
        return float(_data_cost(transformed_dataset, anchors_eval))

    def _anchor_objective_cost(
        transformed_dataset: dict,
        anchors_eval: np.ndarray,
    ) -> float:
        if int(objective_id) == 0:
            score, valid_sweeps, _invalid_sweeps = ellipse_prediction_objective_score(
                transformed_dataset,
                anchors_eval,
                cost_fn=_cached_cost_fn(transformed_dataset),
            )
            if valid_sweeps > 0 and np.isfinite(score):
                return float(score)
        if int(objective_id) == 2:
            score, valid_points, _invalid_points = simulation_position_objective_score(
                transformed_dataset,
                anchors_eval,
                use_noise_mean=bool(use_noise_mean),
            )
            if valid_points > 0 and np.isfinite(score):
                return float(score)
        return float(_data_cost(transformed_dataset, anchors_eval))

    def _solve_anchor_proposal(
        transformed_dataset: dict,
        initial_guess: np.ndarray,
        *,
        num_restarts: int,
        max_iterations: int,
        robust_debug_enabled: bool,
    ) -> Dict[str, object]:
        if int(objective_id) != 1:
            local_restarts = max(1, int(num_restarts))
            local_iterations = max(1, int(max_iterations))
            if int(objective_id) == 0:
                local_restarts = min(local_restarts, 1)
                local_iterations = min(local_iterations, 40)
            elif int(objective_id) == 2:
                local_restarts = 1
                local_iterations = min(local_iterations, 20)
            local_result = _solve_local_anchor_objective(
                transformed_dataset,
                initial_guess,
                objective_id=int(objective_id),
                objective_name=str(objective_name),
                objective_cost=_anchor_objective_cost,
                solve_restarts=int(local_restarts),
                solve_iterations=int(local_iterations),
                solve_optimizer=str(solve_optimizer),
                low_anchor_z=low_anchor_z,
            )
            if isinstance(local_result, dict):
                return local_result
        result = calibrate_elliptical(
            transformed_dataset,
            output_path=None,
            residual_threshold=float(residual_threshold),
            num_restarts=int(num_restarts),
            max_iterations=int(max_iterations),
            method=str(solve_optimizer),
            spring_k_multiplier=float(spring_k_multiplier),
            use_flex=bool(use_flex),
            verbose=False,
            use_parallel=False,
            pointwise_residual_mode=str(pointwise_residual_mode),
            robust_debug=bool(robust_debug_enabled),
            pointwise_filtering=bool(pointwise_filtering),
            pointwise_global_mad=bool(pointwise_global_mad),
            sweep_wise_filtering=bool(sweep_wise_filtering),
            sweep_metric=str(sweep_metric),
            use_noise_mean=bool(use_noise_mean),
            sigma_source=str(sigma_source),
            generate_report=False,
            residuals_csv=None,
            initial_guess=np.asarray(initial_guess, dtype=float),
            low_anchor_z=low_anchor_z,
        )
        result_out = dict(result) if isinstance(result, dict) else {}
        result_out["objective_id"] = int(objective_id)
        result_out["objective_name"] = str(objective_name)
        result_out["solver"] = "ellipse_solver"
        return result_out

    def _extract_noise_metrics(cal_result: object) -> Optional[dict]:
        if not isinstance(cal_result, dict):
            return None
        details = cal_result.get("details")
        if not isinstance(details, dict):
            return None
        noise_metrics = details.get("noise_metrics")
        if isinstance(noise_metrics, dict):
            return noise_metrics
        return None

    def _spool_rank_score(
        transformed_dataset: dict,
        anchors_eval: np.ndarray,
        *,
        noise_metrics_hint: Optional[dict] = None,
    ) -> Tuple[float, Optional[float]]:
        try:
            _legacy_cost, noise_metrics, _rows, risk_metric = _eval_bundle(
                transformed_dataset,
                anchors_eval,
                noise_metrics_hint=noise_metrics_hint,
                rows_policy="always",
                need_legacy_cost=False,
            )
            m_layered = _layered_internal_metric_from_noise_metrics(
                noise_metrics,
                fit_structure_levels=fit_structure_set,
                use_fit_structure_penalties=False,
            )
            m_internal = _blend_internal_metric_with_risk(m_layered, risk_metric)
            rank_score = _layered_rank_score_from_internal_metric(m_internal)
            return float(rank_score), (None if m_internal is None else float(m_internal))
        except Exception:
            return float("inf"), None

    def _rank_better(
        rank_try: float,
        total_try: float,
        rank_ref: float,
        total_ref: float,
    ) -> bool:
        rank_try_finite = bool(np.isfinite(rank_try))
        rank_ref_finite = bool(np.isfinite(rank_ref))
        if rank_try_finite and rank_ref_finite:
            tol = max(1e-9, 1e-4 * max(1.0, abs(float(rank_ref))))
            if float(rank_try) + tol < float(rank_ref):
                return True
            if abs(float(rank_try) - float(rank_ref)) <= tol:
                if np.isfinite(total_try) and (not np.isfinite(total_ref) or float(total_try) + 1e-12 < float(total_ref)):
                    return True
            return False
        if rank_try_finite and (not rank_ref_finite):
            return True
        if (not rank_try_finite) and rank_ref_finite:
            return False
        if np.isfinite(total_try) and (not np.isfinite(total_ref) or float(total_try) + 1e-12 < float(total_ref)):
            return True
        return False

    def _uniform_radius_scale(new_radii_mm: np.ndarray, old_radii_mm: np.ndarray) -> Optional[float]:
        if not search_r:
            return None
        r_new = np.asarray(new_radii_mm, dtype=float).reshape(-1)
        r_old = np.asarray(old_radii_mm, dtype=float).reshape(-1)
        if r_new.size == 0 or r_old.size == 0:
            return None
        num = float(np.median(r_new))
        den = float(np.median(r_old))
        if not np.isfinite(num) or not np.isfinite(den) or num <= 0.0 or den <= 0.0:
            return None
        scale = float(num / den)
        if not np.isfinite(scale) or scale <= 0.0:
            return None
        return float(scale)

    def _radii_respect_bounds(radii_mm: np.ndarray) -> bool:
        if not search_r:
            return True
        try:
            packed = _pack_radii_opt_vector(np.asarray(radii_mm, dtype=float), find_radii_mode=mode_r)
        except Exception:
            return False
        if packed.size != lo_r.size or packed.size != hi_r.size:
            return False
        tol = 1e-9 * max(1.0, float(np.max(np.abs(packed))) if packed.size else 1.0)
        return bool(np.all(packed >= (lo_r - tol)) and np.all(packed <= (hi_r + tol)))

    def _ellipse_prefit_score(transformed_dataset: dict) -> Tuple[float, int, int]:
        sweeps = transformed_dataset.get("sweeps")
        if not isinstance(sweeps, list) or len(sweeps) == 0:
            return float("inf"), 0, 0

        rms_values: List[float] = []
        invalid_sweeps = 0
        for sweep in sweeps:
            if not isinstance(sweep, dict):
                continue
            points = sweep.get("data_points")
            if not isinstance(points, list) or len(points) < 5:
                invalid_sweeps += 1
                continue

            drive_vals: List[float] = []
            sensor_vals: List[float] = []
            for point in points:
                if not isinstance(point, dict):
                    continue
                drive_raw = point.get("l_drive", point.get("l_drive_mu"))
                sensor_raw = point.get("l_sensor", point.get("l_sensor_mu"))
                try:
                    drive = float(drive_raw)
                    sensor = float(sensor_raw)
                except (TypeError, ValueError):
                    continue
                if np.isfinite(drive) and np.isfinite(sensor):
                    drive_vals.append(drive)
                    sensor_vals.append(sensor)
            if len(drive_vals) < 5 or len(sensor_vals) < 5:
                invalid_sweeps += 1
                continue

            fit = fit_ellipse_from_sweep(
                np.asarray(drive_vals, dtype=float),
                np.asarray(sensor_vals, dtype=float),
                residual_threshold=float("inf"),
                min_points=5,
                # Prefit should stay scale-aware in length space; squaring can bias
                # the proxy objective toward lower-radius solutions.
                square_inputs=False,
            )
            rms = float(fit.residual_rms)
            if not np.isfinite(rms):
                invalid_sweeps += 1
                continue
            rms_values.append(rms)

        valid_sweeps = int(len(rms_values))
        if valid_sweeps <= 0:
            return float("inf"), 0, int(invalid_sweeps)

        arr = np.asarray(rms_values, dtype=float)
        # Fixed robust scale keeps the objective informative even with one sweep.
        scaled = arr / 1.0
        robust_sum = float(np.sum(2.0 * (np.sqrt(1.0 + np.square(scaled)) - 1.0)))
        invalid_penalty = float(invalid_sweeps) * 4.0
        return float(robust_sum + invalid_penalty), valid_sweeps, int(invalid_sweeps)

    init_result = run_initialize_pass(
        base=base,
        modeled_b=modeled_b,
        initial_radii_mm=initial_radii_mm,
        initial_buildup_factor=initial_buildup_factor,
        anchors_current=anchors_current,
        num_anchors=num_anchors,
        mode_r=mode_r,
        mode_b=mode_b,
        lo=lo,
        hi=hi,
        kinds=kinds,
        inner_iters=inner_iters,
        solve_restarts=solve_restarts,
        solve_iterations=solve_iterations,
        solve_optimizer=solve_optimizer,
        residual_threshold=residual_threshold,
        spring_k_multiplier=spring_k_multiplier,
        use_flex=use_flex,
        pointwise_residual_mode=pointwise_residual_mode,
        pointwise_filtering=pointwise_filtering,
        pointwise_global_mad=pointwise_global_mad,
        sweep_wise_filtering=sweep_wise_filtering,
        sweep_metric=sweep_metric,
        use_noise_mean=use_noise_mean,
        sigma_source=sigma_source,
        low_anchor_z=low_anchor_z,
        enable_prefit=enable_prefit,
        enable_bootstrap_anchor_refresh=enable_bootstrap_anchor_refresh,
        pack_spool_opt_vector=_pack_spool_opt_vector,
        unpack_spool_opt_vector=_unpack_spool_opt_vector,
        build_dataset_and_params=_build_dataset_and_params,
        prior_cost=_prior_cost,
        data_cost=_data_cost,
        ellipse_prefit_score=_ellipse_prefit_score,
        coordinate_descent_spool=_coordinate_descent_spool,
        spool_prefit_seed_candidates=_spool_prefit_seed_candidates,
        solve_anchor_proposal=_solve_anchor_proposal,
    )
    x_current = np.asarray(init_result["x_current"], dtype=float)
    radii_current = np.asarray(init_result["radii_current"], dtype=float)
    buildup_current = np.asarray(init_result["buildup_current"], dtype=float)
    anchors_current = np.asarray(init_result["anchors_current"], dtype=float)
    prefit_info = dict(init_result["prefit"])
    bootstrap_anchor_refresh = dict(init_result["bootstrap_anchor_refresh"])

    refinement_result = run_alternating_refinement(
        radii_current=radii_current,
        buildup_current=buildup_current,
        anchors_current=anchors_current,
        x_current=x_current,
        base=base,
        modeled_b=modeled_b,
        num_anchors=num_anchors,
        mode_r=mode_r,
        mode_b=mode_b,
        lo=lo,
        hi=hi,
        kinds=kinds,
        search_r=search_r,
        use_scale_fix_1=use_scale_fix_1,
        outer_iters=outer_iters,
        inner_iters=inner_iters,
        solve_restarts=solve_restarts,
        solve_iterations=solve_iterations,
        solve_optimizer=solve_optimizer,
        residual_threshold=residual_threshold,
        spring_k_multiplier=spring_k_multiplier,
        use_flex=use_flex,
        pointwise_residual_mode=pointwise_residual_mode,
        pointwise_filtering=pointwise_filtering,
        pointwise_global_mad=pointwise_global_mad,
        sweep_wise_filtering=sweep_wise_filtering,
        sweep_metric=sweep_metric,
        use_noise_mean=use_noise_mean,
        sigma_source=sigma_source,
        low_anchor_z=low_anchor_z,
        pack_spool_opt_vector=_pack_spool_opt_vector,
        unpack_spool_opt_vector=_unpack_spool_opt_vector,
        build_dataset_and_params=_build_dataset_and_params,
        core_objective=_objective_cost,
        core_objective_id=int(objective_id),
        core_objective_name=str(objective_name),
        data_cost=_data_cost,
        prior_cost=_prior_cost,
        spool_rank_score=_spool_rank_score,
        rank_better=_rank_better,
        spool_seed_candidates=_spool_seed_candidates,
        coordinate_descent_spool=_coordinate_descent_spool,
        extract_noise_metrics=_extract_noise_metrics,
        spool_anchor_step_gate=_spool_anchor_step_gate,
        uniform_radius_scale=_uniform_radius_scale,
        solve_anchor_proposal=_solve_anchor_proposal,
    )
    best = refinement_result["best"]
    history = list(refinement_result["history"])
    x_current = np.asarray(refinement_result["x_current"], dtype=float)
    radii_current = np.asarray(refinement_result["radii_current"], dtype=float)
    buildup_current = np.asarray(refinement_result["buildup_current"], dtype=float)
    anchors_current = np.asarray(refinement_result["anchors_current"], dtype=float)

    if best["dataset"] is None or best["spool_params"] is None:
        if x_current.size > 0:
            x_use = np.clip(x_current, lo, hi)
        else:
            x_use = x_current
        radii_fallback, buildup_fallback = _unpack_spool_opt_vector(
            x_use,
            num_anchors=num_anchors,
            find_radii_mode=mode_r,
            find_buildup_mode=mode_b,
            fixed_radii_mm=base,
            fixed_buildup_factor=modeled_b,
        )
        spool_params_fallback, transformed_fallback = _build_dataset_and_params(radii_fallback, buildup_fallback)
        fallback_cost = float(_data_cost(transformed_fallback, anchors_current))
        fallback_prior = float(_prior_cost(radii_fallback, buildup_fallback))
        fallback_total = (
            float(fallback_cost + fallback_prior)
            if np.isfinite(fallback_cost) and np.isfinite(fallback_prior)
            else float("inf")
        )
        fallback_rank_score, fallback_rank_internal = _spool_rank_score(
            transformed_fallback,
            anchors_current,
        )
        best = {
            "cost": float(fallback_cost) if np.isfinite(fallback_cost) else float("nan"),
            "total_cost": float(fallback_total) if np.isfinite(fallback_total) else float("inf"),
            "rank_score": float(fallback_rank_score) if np.isfinite(fallback_rank_score) else float("inf"),
            "rank_internal": (
                None if fallback_rank_internal is None else float(fallback_rank_internal)
            ),
            "radii_mm": np.asarray(radii_fallback, dtype=float),
            "buildup_factor": np.asarray(buildup_fallback, dtype=float),
            "anchors": np.asarray(anchors_current, dtype=float),
            "spool_params": spool_params_fallback,
            "dataset": transformed_fallback,
        }

    def _run_uniform_scale_polish_wrapped(
        *,
        radii_mm: np.ndarray,
        buildup_factor: np.ndarray,
        anchors_eval: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, Any, dict, float, Dict[str, object]]:
        return run_uniform_scale_polish(
            search_r=search_r,
            num_anchors=num_anchors,
            radii_mm=radii_mm,
            buildup_factor=buildup_factor,
            anchors_eval=anchors_eval,
            prior_cost=_prior_cost,
            data_cost=_data_cost,
            build_dataset_and_params=_build_dataset_and_params,
            radii_respect_bounds=_radii_respect_bounds,
            uniform_radius_scale=_uniform_radius_scale,
        )

    best, final_scale_polish_info = apply_final_scale_polish(
        best=best,
        use_scale_fix_2=use_scale_fix_2,
        use_scale_fix_3=use_scale_fix_3,
        prior_cost=_prior_cost,
        spool_rank_score=_spool_rank_score,
        rank_better=_rank_better,
        run_uniform_scale_polish_fn=_run_uniform_scale_polish_wrapped,
    )
    best_objective_cost = float(
        _objective_cost(
            best["dataset"],
            np.asarray(best["anchors"], dtype=float),
        )
    )

    r0_bounds_info = (
        [float(np.min(lo_r)), float(np.max(hi_r))]
        if lo_r.size > 0 and hi_r.size > 0
        else None
    )
    b_bounds_info = (
        [float(np.min(lo_b)), float(np.max(hi_b))]
        if lo_b.size > 0 and hi_b.size > 0
        else None
    )
    spool_info_hessian = None
    spool_info_rank = None
    if bool(_COMPUTE_SPOOL_INFO_MATRIX) and lo.size > 0 and hi.size > 0:
        x_best = _pack_spool_opt_vector(
            radii_mm=np.asarray(best["radii_mm"], dtype=float),
            buildup_factor=np.asarray(best["buildup_factor"], dtype=float),
            find_radii_mode=mode_r,
            find_buildup_mode=mode_b,
        )
        x_best = np.clip(np.asarray(x_best, dtype=float).reshape(-1), lo, hi)
        anchors_best = np.asarray(best["anchors"], dtype=float)

        def _objective_for_hessian(opt_vec: np.ndarray) -> float:
            if lo.size > 0:
                clipped_vec = np.clip(np.asarray(opt_vec, dtype=float).reshape(-1), lo, hi)
            else:
                clipped_vec = np.asarray(opt_vec, dtype=float).reshape(-1)
            radii_try, buildup_try = _unpack_spool_opt_vector(
                clipped_vec,
                num_anchors=num_anchors,
                find_radii_mode=mode_r,
                find_buildup_mode=mode_b,
                fixed_radii_mm=base,
                fixed_buildup_factor=modeled_b,
            )
            _, transformed_try = _build_dataset_and_params(radii_try, buildup_try)
            data_cost = _data_cost(transformed_try, anchors_best)
            if not np.isfinite(data_cost):
                return float("inf")
            return float(data_cost + _prior_cost(radii_try, buildup_try))

        hess = _numerical_hessian_symmetric(
            x_best,
            lo=lo,
            hi=hi,
            kinds=kinds,
            objective=_objective_for_hessian,
        )
        if isinstance(hess, np.ndarray) and hess.size:
            spool_info_hessian = hess
            spool_info_rank = int(np.linalg.matrix_rank(hess))

    fit_info: Dict[str, object] = {
        "mode": f"radii={mode_r},buildup={mode_b}",
        "radii_mode": mode_r,
        "buildup_mode": mode_b,
        "theta0_mode": theta0_mode_norm,
        "bounds_mm": r0_bounds_info,
        "r0_bounds_mm": r0_bounds_info,
        "b_bounds": b_bounds_info,
        "r0_prior_sigma_mm": (None if sigma_r is None else float(sigma_r)),
        "b_prior_sigma": (None if sigma_b is None else float(sigma_b)),
        "radii_pair_sigma_mm": (None if sigma_rpair is None else float(sigma_rpair)),
        "outer_iters": int(outer_iters),
        "inner_iters": int(inner_iters),
        "noise_normalized_data_term": bool(spool_noise_normalized),
        "objective_id": int(objective_id),
        "objective_name": str(objective_name),
        "optimization_objective": (
            "ellipse_prefit_score + prior"
            if int(objective_id) == 0
            else (
                "position_reconstruction_cost + prior"
                if int(objective_id) == 2
                else "legacy_data_cost + w*risk_trimmed_direct"
            )
        ),
        "anchor_step_improvement_rel_threshold": float(_SPOOL_ANCHOR_STEP_IMPROVE_REL),
        "filter_schedule_requested": [str(v) for v in requested_filter_schedule],
        "objective_schedule_requested": [int(v) for v in requested_objective_schedule],
        "scale_fix_levels": [int(v) for v in sorted(scale_fix_set)],
        "fit_structure_levels": [int(v) for v in sorted(fit_structure_set)],
        "scale_fix_1_enabled": bool(use_scale_fix_1),
        "scale_fix_2_enabled": bool(use_scale_fix_2),
        "scale_fix_3_enabled": bool(use_scale_fix_3),
        "prefit_enabled": bool(enable_prefit),
        "bootstrap_anchor_refresh_enabled": bool(enable_bootstrap_anchor_refresh),
        "prefit": dict(prefit_info),
        "bootstrap_anchor_refresh": dict(bootstrap_anchor_refresh),
        "final_scale_polish": dict(final_scale_polish_info),
        "history": history,
        "best_cost": (
            float(best["cost"])
            if isinstance(best.get("cost"), (int, float)) and np.isfinite(float(best["cost"]))
            else None
        ),
        "best_objective_cost": (
            float(best_objective_cost) if np.isfinite(best_objective_cost) else None
        ),
        "best_total_cost": (
            float(best["total_cost"])
            if isinstance(best.get("total_cost"), (int, float)) and np.isfinite(float(best["total_cost"]))
            else None
        ),
        "best_rank_score": (
            float(best["rank_score"])
            if isinstance(best.get("rank_score"), (int, float)) and np.isfinite(float(best["rank_score"]))
            else None
        ),
        "best_rank_internal": (
            float(best["rank_internal"])
            if isinstance(best.get("rank_internal"), (int, float))
            and np.isfinite(float(best["rank_internal"]))
            else None
        ),
        "best_radii_mm": [float(v) for v in np.asarray(best["radii_mm"], dtype=float).tolist()],
        "best_modeled_buildup_factor": [
            float(v) for v in np.asarray(best["buildup_factor"], dtype=float).tolist()
        ],
        "spool_info_rank": spool_info_rank,
        "spool_info_matrix": (
            None
            if spool_info_hessian is None
            else np.asarray(spool_info_hessian, dtype=float).tolist()
        ),
    }

    return (
        np.asarray(best["radii_mm"], dtype=float),
        np.asarray(best["anchors"], dtype=float),
        best["spool_params"],
        best["dataset"],
        fit_info,
    )
