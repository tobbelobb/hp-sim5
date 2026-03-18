from __future__ import annotations

from typing import Any, Callable, Dict, Optional, Sequence, Tuple

import numpy as np


def run_initialize_pass(
    *,
    base: np.ndarray,
    modeled_b: np.ndarray,
    initial_radii_mm: Optional[np.ndarray],
    initial_buildup_factor: Optional[np.ndarray],
    anchors_current: np.ndarray,
    num_anchors: int,
    mode_r: str,
    mode_b: str,
    lo: np.ndarray,
    hi: np.ndarray,
    kinds: Sequence[str],
    inner_iters: int,
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
    low_anchor_z: Optional[float],
    enable_prefit: bool,
    enable_bootstrap_anchor_refresh: bool,
    pack_spool_opt_vector: Callable[..., np.ndarray],
    unpack_spool_opt_vector: Callable[..., Tuple[np.ndarray, np.ndarray]],
    build_dataset_and_params: Callable[[np.ndarray, np.ndarray], Tuple[Any, dict]],
    prior_cost: Callable[[np.ndarray, np.ndarray], float],
    data_cost: Callable[[dict, np.ndarray], float],
    ellipse_prefit_score: Callable[[dict], Tuple[float, int, int]],
    coordinate_descent_spool: Callable[..., Tuple[np.ndarray, Dict[str, object]]],
    spool_prefit_seed_candidates: Callable[..., Sequence[np.ndarray]],
    solve_anchor_proposal: Callable[..., Dict[str, object]],
) -> Dict[str, object]:
    radii_current = np.asarray(base, dtype=float).copy()
    if initial_radii_mm is not None:
        radii_try = np.asarray(initial_radii_mm, dtype=float).reshape(-1)
        if radii_try.size == num_anchors and np.all(np.isfinite(radii_try)) and np.all(radii_try > 0.0):
            radii_current = np.asarray(radii_try, dtype=float)
    buildup_current = np.asarray(modeled_b, dtype=float).copy()
    if initial_buildup_factor is not None:
        buildup_try = np.asarray(initial_buildup_factor, dtype=float).reshape(-1)
        if buildup_try.size == num_anchors and np.all(np.isfinite(buildup_try)):
            buildup_current = np.asarray(buildup_try, dtype=float)
    x_current = pack_spool_opt_vector(
        radii_mm=radii_current,
        buildup_factor=buildup_current,
        find_radii_mode=mode_r,
        find_buildup_mode=mode_b,
    )
    if bool(enable_prefit) and x_current.size > 0:
        x_current = np.clip(x_current, lo, hi)
        radii_current, buildup_current = unpack_spool_opt_vector(
            x_current,
            num_anchors=num_anchors,
            find_radii_mode=mode_r,
            find_buildup_mode=mode_b,
            fixed_radii_mm=base,
            fixed_buildup_factor=modeled_b,
        )

    prefit_info: Dict[str, object] = {
        "enabled": False,
        "success": False,
        "message": "prefit_not_run",
        "nfev": 0,
        "nit": 0,
        "start_cost": None,
        "fitted_cost": None,
        "seed_choice": "none",
        "seed_cost": None,
        "valid_sweeps": 0,
        "invalid_sweeps": 0,
        "guarded": False,
    }

    if x_current.size > 0:
        prefit_eval_counter = {"count": 0}
        prefit_cache: Dict[Tuple[float, ...], float] = {}
        prefit_parts_cache: Dict[Tuple[float, ...], Tuple[float, float, int, int]] = {}
        prefit_seed_scores: list[tuple[float, float]] = []

        def _prefit_clip_and_key(opt_vec: np.ndarray) -> Tuple[np.ndarray, Tuple[float, ...]]:
            clipped_vec = np.clip(np.asarray(opt_vec, dtype=float).reshape(-1), lo, hi)
            key = tuple(float(v) for v in np.round(clipped_vec, decimals=9).tolist())
            return clipped_vec, key

        def _prefit_objective(opt_vec: np.ndarray) -> float:
            try:
                clipped_vec, key = _prefit_clip_and_key(opt_vec)
                cached = prefit_cache.get(key)
                if cached is not None and np.isfinite(cached):
                    return float(cached)
                prefit_eval_counter["count"] += 1
                radii_try, buildup_try = unpack_spool_opt_vector(
                    clipped_vec,
                    num_anchors=num_anchors,
                    find_radii_mode=mode_r,
                    find_buildup_mode=mode_b,
                    fixed_radii_mm=base,
                    fixed_buildup_factor=modeled_b,
                )
                _, transformed_try = build_dataset_and_params(radii_try, buildup_try)
                ellipse_cost, valid_sweeps, _ = ellipse_prefit_score(transformed_try)
                if valid_sweeps <= 0 or not np.isfinite(ellipse_cost):
                    prefit_cache[key] = 1e12
                    prefit_parts_cache[key] = (float("nan"), float("nan"), int(valid_sweeps), 0)
                    return 1e12
                prior = float(prior_cost(radii_try, buildup_try))
                score = float(ellipse_cost + prior)
                if not np.isfinite(score):
                    prefit_cache[key] = 1e12
                    prefit_parts_cache[key] = (float("nan"), float("nan"), int(valid_sweeps), 0)
                    return 1e12
                prefit_cache[key] = float(score)
                prefit_parts_cache[key] = (
                    float(ellipse_cost),
                    float(prior),
                    int(valid_sweeps),
                    0,
                )
                return float(score)
            except Exception:
                return 1e12

        def _prefit_cost_parts(opt_vec: np.ndarray) -> Tuple[float, float, float]:
            clipped_vec, key = _prefit_clip_and_key(opt_vec)
            total_cost = float(_prefit_objective(clipped_vec))
            parts = prefit_parts_cache.get(key)
            if parts is None:
                return float("nan"), float("nan"), total_cost
            return float(parts[0]), float(parts[1]), total_cost

        def _prefit_seed_landscape_is_monotonic_boundary(
            seed_scores: Sequence[Tuple[float, float]],
        ) -> bool:
            if len(kinds) != 1 or str(kinds[0]) != "r":
                return False
            if lo.size != 1 or hi.size != 1:
                return False

            by_radius: Dict[float, Tuple[float, float]] = {}
            for radius, score in seed_scores:
                if not np.isfinite(radius) or not np.isfinite(score):
                    continue
                key = float(np.round(float(radius), decimals=9))
                existing = by_radius.get(key)
                if existing is None or float(score) < float(existing[1]):
                    by_radius[key] = (float(radius), float(score))
            if len(by_radius) < 3:
                return False

            rows = [by_radius[key] for key in sorted(by_radius.keys())]
            radii = np.asarray([row[0] for row in rows], dtype=float)
            costs = np.asarray([row[1] for row in rows], dtype=float)
            if radii.size < 3 or np.any(~np.isfinite(costs)):
                return False

            diffs = np.diff(costs)
            mono_tol = max(1e-9, 1e-8 * max(1.0, float(np.max(np.abs(costs)))))
            monotonic = bool(np.all(diffs >= -mono_tol) or np.all(diffs <= mono_tol))
            if not monotonic:
                return False

            best_idx = int(np.argmin(costs))
            best_r = float(radii[best_idx])
            lo_r = float(lo[0])
            hi_r = float(hi[0])
            bound_tol = 1e-9 * max(1.0, abs(lo_r), abs(hi_r), abs(hi_r - lo_r))
            at_bound = abs(best_r - lo_r) <= bound_tol or abs(best_r - hi_r) <= bound_tol
            return bool(at_bound)

        x_prefit_start = np.clip(np.asarray(x_current, dtype=float).reshape(-1), lo, hi)
        x_prefit_seed = np.asarray(x_prefit_start, dtype=float).copy()
        prefit_start_cost = float(_prefit_objective(x_prefit_seed))
        prefit_start_data, prefit_start_prior, prefit_start_total = _prefit_cost_parts(x_prefit_start)
        if x_prefit_seed.size > 0:
            prefit_seed_scores.append((float(x_prefit_seed[0]), float(prefit_start_cost)))
        if np.isfinite(prefit_start_total) and prefit_start_total < 1e11:
            prefit_info["enabled"] = True
            seed_candidates = spool_prefit_seed_candidates(
                x_prefit_seed,
                lo,
                hi,
                kinds=kinds,
            )
            seed_choice = "current"
            seed_cost = float(prefit_start_cost)
            for idx, seed_try in enumerate(seed_candidates[1:], start=1):
                score_try = float(_prefit_objective(seed_try))
                seed_arr = np.asarray(seed_try, dtype=float).reshape(-1)
                if seed_arr.size > 0:
                    prefit_seed_scores.append((float(seed_arr[0]), float(score_try)))
                if np.isfinite(score_try) and (
                    not np.isfinite(seed_cost) or score_try + 1e-12 < seed_cost
                ):
                    x_prefit_seed = seed_arr
                    seed_cost = float(score_try)
                    seed_choice = f"seed_{idx}"

            prefit_guarded = _prefit_seed_landscape_is_monotonic_boundary(prefit_seed_scores)
            if prefit_guarded:
                prefit_opt = {
                    "success": False,
                    "message": "ellipse prefit uninformative (monotonic boundary seed landscape); skipped",
                    "nfev": int(prefit_eval_counter["count"]),
                    "nit": 0,
                }
                use_prefit = False
                x_current = np.asarray(x_prefit_start, dtype=float)
            else:
                prefit_iters = max(2, min(12, int(inner_iters)))
                x_prefit_opt, prefit_opt = coordinate_descent_spool(
                    x_prefit_seed,
                    lo=lo,
                    hi=hi,
                    kinds=kinds,
                    max_iters=int(prefit_iters),
                    objective=_prefit_objective,
                )
                x_prefit_opt = np.clip(np.asarray(x_prefit_opt, dtype=float).reshape(-1), lo, hi)
                prefit_fitted_cost = float(_prefit_objective(x_prefit_opt))
                use_prefit = (
                    np.isfinite(prefit_fitted_cost)
                    and prefit_fitted_cost + 1e-12 < float(prefit_start_total)
                )
                x_current = x_prefit_opt if use_prefit else x_prefit_seed
            prefit_fitted_data, prefit_fitted_prior, prefit_fitted_total = _prefit_cost_parts(x_current)
            radii_current, buildup_current = unpack_spool_opt_vector(
                x_current,
                num_anchors=num_anchors,
                find_radii_mode=mode_r,
                find_buildup_mode=mode_b,
                fixed_radii_mm=base,
                fixed_buildup_factor=modeled_b,
            )
            _spool_params_prefit, transformed_prefit = build_dataset_and_params(
                radii_current,
                buildup_current,
            )
            prefit_score, valid_sweeps, invalid_sweeps = ellipse_prefit_score(transformed_prefit)
            prefit_info.update(
                {
                    "success": bool(use_prefit),
                    "message": (
                        "ellipse prefit improved spool seed"
                        if use_prefit
                        else str(prefit_opt.get("message", "ellipse prefit did not improve"))
                    ),
                    "nfev": int(prefit_opt.get("nfev", prefit_eval_counter["count"])),
                    "nit": int(prefit_opt.get("nit", 0)),
                    "start_cost": float(prefit_start_total),
                    "fitted_cost": (
                        float(prefit_fitted_total)
                        if np.isfinite(prefit_fitted_total)
                        else float(prefit_start_total)
                    ),
                    "start_data_cost": (
                        float(prefit_start_data) if np.isfinite(prefit_start_data) else None
                    ),
                    "start_prior_cost": (
                        float(prefit_start_prior) if np.isfinite(prefit_start_prior) else None
                    ),
                    "start_total_cost": (
                        float(prefit_start_total) if np.isfinite(prefit_start_total) else None
                    ),
                    "fitted_data_cost": (
                        float(prefit_fitted_data) if np.isfinite(prefit_fitted_data) else None
                    ),
                    "fitted_prior_cost": (
                        float(prefit_fitted_prior) if np.isfinite(prefit_fitted_prior) else None
                    ),
                    "fitted_total_cost": (
                        float(prefit_fitted_total) if np.isfinite(prefit_fitted_total) else None
                    ),
                    "seed_choice": str(seed_choice),
                    "seed_cost": float(seed_cost) if np.isfinite(seed_cost) else None,
                    "valid_sweeps": int(valid_sweeps),
                    "invalid_sweeps": int(invalid_sweeps),
                    "guarded": bool(prefit_guarded),
                    "ellipse_cost": (
                        float(prefit_score) if np.isfinite(prefit_score) else None
                    ),
                }
            )
        else:
            prefit_info.update(
                {
                    "enabled": False,
                    "message": "ellipse prefit skipped (no valid sweep residual objective)",
                    "start_cost": (
                        float(prefit_start_total)
                        if np.isfinite(prefit_start_total)
                        else None
                    ),
                    "start_data_cost": (
                        float(prefit_start_data) if np.isfinite(prefit_start_data) else None
                    ),
                    "start_prior_cost": (
                        float(prefit_start_prior) if np.isfinite(prefit_start_prior) else None
                    ),
                    "start_total_cost": (
                        float(prefit_start_total) if np.isfinite(prefit_start_total) else None
                    ),
                }
            )

    bootstrap_anchor_refresh: Dict[str, object] = {
        "attempted": False,
        "success": False,
        "accepted": False,
        "start_cost": None,
        "candidate_cost": None,
        "accepted_cost": None,
        "accepted_alpha": 0.0,
        "objective_id": None,
        "objective_name": None,
        "solver": None,
    }
    if bool(enable_bootstrap_anchor_refresh):
        try:
            _spool_params_bootstrap, transformed_bootstrap = build_dataset_and_params(
                radii_current,
                buildup_current,
            )
            anchor_refresh_start_cost = float(data_cost(transformed_bootstrap, anchors_current))
            bootstrap_anchor_refresh["attempted"] = True
            if np.isfinite(anchor_refresh_start_cost):
                bootstrap_anchor_refresh["start_cost"] = float(anchor_refresh_start_cost)
                anchor_step_restarts = max(1, min(2, int(solve_restarts)))
                anchor_step_iterations = max(40, min(160, int(solve_iterations)))
                cal_bootstrap = solve_anchor_proposal(
                    transformed_bootstrap,
                    np.asarray(anchors_current, dtype=float),
                    num_restarts=int(anchor_step_restarts),
                    max_iterations=int(anchor_step_iterations),
                    robust_debug_enabled=False,
                )
                if isinstance(cal_bootstrap, dict):
                    bootstrap_anchor_refresh["objective_id"] = cal_bootstrap.get("objective_id")
                    bootstrap_anchor_refresh["objective_name"] = cal_bootstrap.get("objective_name")
                    bootstrap_anchor_refresh["solver"] = cal_bootstrap.get("solver")
                anchors_bootstrap_candidate = np.asarray(cal_bootstrap.get("anchors"), dtype=float)
                if (
                    anchors_bootstrap_candidate.ndim == 2
                    and anchors_bootstrap_candidate.shape == anchors_current.shape
                    and np.all(np.isfinite(anchors_bootstrap_candidate))
                ):
                    bootstrap_anchor_refresh["success"] = True
                    accepted_bootstrap_anchors = np.asarray(anchors_current, dtype=float)
                    accepted_bootstrap_cost = float(anchor_refresh_start_cost)
                    accepted_bootstrap_alpha = 0.0
                    candidate_bootstrap_cost = float("nan")
                    for alpha in (1.0, 0.5, 0.25):
                        if alpha >= 1.0 - 1e-12:
                            anchors_try = np.asarray(anchors_bootstrap_candidate, dtype=float)
                        else:
                            anchors_try = np.asarray(
                                anchors_current + (anchors_bootstrap_candidate - anchors_current) * float(alpha),
                                dtype=float,
                            )
                        cost_try = float(data_cost(transformed_bootstrap, anchors_try))
                        if not np.isfinite(cost_try):
                            continue
                        if alpha >= 1.0 - 1e-12:
                            candidate_bootstrap_cost = float(cost_try)
                        improve_tol = max(1e-9, 1e-4 * max(1.0, abs(float(anchor_refresh_start_cost))))
                        if cost_try + improve_tol < accepted_bootstrap_cost:
                            accepted_bootstrap_anchors = np.asarray(anchors_try, dtype=float)
                            accepted_bootstrap_cost = float(cost_try)
                            accepted_bootstrap_alpha = float(alpha)
                    if np.isfinite(candidate_bootstrap_cost):
                        bootstrap_anchor_refresh["candidate_cost"] = float(candidate_bootstrap_cost)
                    if accepted_bootstrap_alpha > 0.0:
                        anchors_current = np.asarray(accepted_bootstrap_anchors, dtype=float)
                        bootstrap_anchor_refresh["accepted"] = True
                        bootstrap_anchor_refresh["accepted_cost"] = float(accepted_bootstrap_cost)
                        bootstrap_anchor_refresh["accepted_alpha"] = float(accepted_bootstrap_alpha)
        except Exception:
            pass

    return {
        "x_current": np.asarray(x_current, dtype=float),
        "radii_current": np.asarray(radii_current, dtype=float),
        "buildup_current": np.asarray(buildup_current, dtype=float),
        "anchors_current": np.asarray(anchors_current, dtype=float),
        "prefit": dict(prefit_info),
        "bootstrap_anchor_refresh": dict(bootstrap_anchor_refresh),
    }
