from __future__ import annotations

from typing import Any, Callable, Dict, Optional, Sequence, Tuple

import numpy as np


def run_alternating_refinement(
    *,
    radii_current: np.ndarray,
    buildup_current: np.ndarray,
    anchors_current: np.ndarray,
    x_current: np.ndarray,
    base: np.ndarray,
    modeled_b: np.ndarray,
    num_anchors: int,
    mode_r: str,
    mode_b: str,
    lo: np.ndarray,
    hi: np.ndarray,
    kinds: Sequence[str],
    search_r: bool,
    use_scale_fix_1: bool,
    use_global_radius_anchor_probe: bool,
    outer_iters: int,
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
    pack_spool_opt_vector: Callable[..., np.ndarray],
    unpack_spool_opt_vector: Callable[..., Tuple[np.ndarray, np.ndarray]],
    build_dataset_and_params: Callable[[np.ndarray, np.ndarray], Tuple[Any, dict]],
    core_objective: Callable[[dict, np.ndarray], float],
    core_objective_id: int,
    core_objective_name: str,
    data_cost: Callable[[dict, np.ndarray], float],
    prior_cost: Callable[[np.ndarray, np.ndarray], float],
    spool_rank_score: Callable[..., Tuple[float, Optional[float]]],
    rank_better: Callable[[float, float, float, float], bool],
    spool_seed_candidates: Callable[[np.ndarray, np.ndarray, np.ndarray], Sequence[np.ndarray]],
    coordinate_descent_spool: Callable[..., Tuple[np.ndarray, Dict[str, object]]],
    extract_noise_metrics: Callable[[object], Optional[dict]],
    spool_anchor_step_gate: Callable[[float, float], Tuple[bool, float, float]],
    uniform_radius_scale: Callable[[np.ndarray, np.ndarray], Optional[float]],
    solve_anchor_proposal: Callable[..., Dict[str, object]],
) -> Dict[str, object]:
    history: list[dict[str, object]] = []
    best = {
        "cost": float("inf"),
        "total_cost": float("inf"),
        "rank_score": float("inf"),
        "rank_internal": None,
        "radii_mm": np.asarray(radii_current, dtype=float),
        "buildup_factor": np.asarray(buildup_current, dtype=float),
        "anchors": np.asarray(anchors_current, dtype=float),
        "spool_params": None,
        "dataset": None,
    }

    for outer_idx in range(outer_iters):
        spool_params_current, transformed_current = build_dataset_and_params(radii_current, buildup_current)
        current_objective_cost = float(core_objective(transformed_current, anchors_current))
        current_cost = float(data_cost(transformed_current, anchors_current))
        current_prior = float(prior_cost(radii_current, buildup_current))
        current_objective_total_cost = (
            float(current_objective_cost + current_prior)
            if np.isfinite(current_objective_cost) and np.isfinite(current_prior)
            else float("inf")
        )
        current_total_cost = (
            float(current_cost + current_prior)
            if np.isfinite(current_cost) and np.isfinite(current_prior)
            else float("inf")
        )
        current_rank_score, current_rank_internal = spool_rank_score(transformed_current, anchors_current)

        eval_counter = {"count": 0}
        objective_cache: Dict[Tuple[float, ...], float] = {}
        objective_parts_cache: Dict[Tuple[float, ...], Tuple[float, float]] = {}
        objective_dataset_cache: Dict[Tuple[float, ...], dict] = {}

        def _objective_clip_and_key(opt_vec: np.ndarray) -> Tuple[np.ndarray, Tuple[float, ...]]:
            if lo.size > 0:
                clipped_vec = np.clip(np.asarray(opt_vec, dtype=float).reshape(-1), lo, hi)
            else:
                clipped_vec = np.asarray(opt_vec, dtype=float).reshape(-1)
            key = tuple(float(v) for v in np.round(clipped_vec, decimals=9).tolist())
            return clipped_vec, key

        def _objective(opt_vec: np.ndarray) -> float:
            try:
                clipped_vec, key = _objective_clip_and_key(opt_vec)
                cached = objective_cache.get(key)
                if cached is not None and np.isfinite(cached):
                    return float(cached)
                eval_counter["count"] += 1
                radii_try, buildup_try = unpack_spool_opt_vector(
                    clipped_vec,
                    num_anchors=num_anchors,
                    find_radii_mode=mode_r,
                    find_buildup_mode=mode_b,
                    fixed_radii_mm=base,
                    fixed_buildup_factor=modeled_b,
                )
                _, transformed_try = build_dataset_and_params(radii_try, buildup_try)
                objective_cost = core_objective(transformed_try, anchors_current)
                if not np.isfinite(objective_cost):
                    objective_parts_cache[key] = (float("nan"), float("nan"))
                    return 1e12
                prior = prior_cost(radii_try, buildup_try)
                score = float(objective_cost + prior)
                if not np.isfinite(score):
                    objective_cache[key] = 1e12
                    objective_parts_cache[key] = (float("nan"), float("nan"))
                    return 1e12
                objective_cache[key] = float(score)
                objective_parts_cache[key] = (float(objective_cost), float(prior))
                objective_dataset_cache[key] = transformed_try
                return score
            except Exception:
                return 1e12

        def _objective_cost_parts(opt_vec: np.ndarray) -> Tuple[float, float, float]:
            clipped_vec, key = _objective_clip_and_key(opt_vec)
            total_cost = float(_objective(clipped_vec))
            parts = objective_parts_cache.get(key)
            if parts is None:
                return float("nan"), float("nan"), total_cost
            return float(parts[0]), float(parts[1]), total_cost

        x_seed = pack_spool_opt_vector(
            radii_mm=radii_current,
            buildup_factor=buildup_current,
            find_radii_mode=mode_r,
            find_buildup_mode=mode_b,
        )
        if x_seed.size > 0:
            x_seed = np.clip(x_seed, lo, hi)
            seed_candidates = spool_seed_candidates(x_seed, lo, hi)
            seed_choice = "current"
            seed_cost = float(_objective(x_seed))
            seed_rank_score = float("inf")
            try:
                seed_arr, seed_key = _objective_clip_and_key(x_seed)
                transformed_seed = objective_dataset_cache.get(seed_key)
                if transformed_seed is None:
                    radii_seed, buildup_seed = unpack_spool_opt_vector(
                        seed_arr,
                        num_anchors=num_anchors,
                        find_radii_mode=mode_r,
                        find_buildup_mode=mode_b,
                        fixed_radii_mm=base,
                        fixed_buildup_factor=modeled_b,
                    )
                    _spool_seed, transformed_seed = build_dataset_and_params(radii_seed, buildup_seed)
                seed_rank_score, _ = spool_rank_score(transformed_seed, anchors_current)
            except Exception:
                seed_rank_score = float("inf")
            if len(seed_candidates) > 1:
                for idx, seed_try in enumerate(seed_candidates[1:], start=1):
                    score_try = float(_objective(seed_try))
                    rank_try = float("inf")
                    try:
                        seed_arr, seed_key = _objective_clip_and_key(
                            np.asarray(seed_try, dtype=float).reshape(-1)
                        )
                        transformed_try = objective_dataset_cache.get(seed_key)
                        if transformed_try is None:
                            radii_try, buildup_try = unpack_spool_opt_vector(
                                seed_arr,
                                num_anchors=num_anchors,
                                find_radii_mode=mode_r,
                                find_buildup_mode=mode_b,
                                fixed_radii_mm=base,
                                fixed_buildup_factor=modeled_b,
                            )
                            _spool_try, transformed_try = build_dataset_and_params(radii_try, buildup_try)
                        rank_try, _ = spool_rank_score(transformed_try, anchors_current)
                    except Exception:
                        rank_try = float("inf")
                    if rank_better(rank_try, score_try, seed_rank_score, seed_cost):
                        x_seed = np.asarray(seed_try, dtype=float).reshape(-1)
                        seed_cost = float(score_try)
                        seed_rank_score = float(rank_try)
                        seed_choice = f"seed_{idx}"
            x_opt, opt_info = coordinate_descent_spool(
                x_seed,
                lo=lo,
                hi=hi,
                kinds=kinds,
                max_iters=int(inner_iters),
                objective=_objective,
            )
            opt_info["seed_choice"] = seed_choice
            opt_info["seed_cost"] = float(seed_cost) if np.isfinite(seed_cost) else None
            opt_info["seed_rank_score"] = float(seed_rank_score) if np.isfinite(seed_rank_score) else None
            if lo.size > 0:
                x_opt = np.clip(np.asarray(x_opt, dtype=float).reshape(-1), lo, hi)
        else:
            x_opt = x_seed
            fitted_cost = float(_objective(x_opt))
            opt_info = {
                "success": True,
                "message": "no spool parameters selected for fitting",
                "nfev": int(eval_counter["count"]),
                "nit": 0,
                "start_cost": float(fitted_cost),
                "fitted_cost": float(fitted_cost),
                "step_final": [],
                "seed_choice": "none",
                "seed_cost": float(fitted_cost) if np.isfinite(fitted_cost) else None,
                "seed_rank_score": None,
            }
        start_objective_cost, start_prior_cost, start_total_cost = _objective_cost_parts(x_seed)
        fitted_objective_cost, fitted_prior_cost, fitted_total_cost = _objective_cost_parts(x_opt)
        opt_info["start_cost"] = float(start_total_cost)
        opt_info["fitted_cost"] = float(fitted_total_cost)
        (
            run_anchor_step,
            anchor_step_trigger_improvement,
            anchor_step_trigger_threshold,
        ) = spool_anchor_step_gate(current_objective_total_cost, fitted_total_cost)

        radii_opt, buildup_opt = unpack_spool_opt_vector(
            x_opt,
            num_anchors=num_anchors,
            find_radii_mode=mode_r,
            find_buildup_mode=mode_b,
            fixed_radii_mm=base,
            fixed_buildup_factor=modeled_b,
        )
        spool_params_opt, transformed_opt = build_dataset_and_params(radii_opt, buildup_opt)
        spool_objective_fixed = float(core_objective(transformed_opt, anchors_current))
        spool_cost_fixed = float(data_cost(transformed_opt, anchors_current))
        spool_prior = float(prior_cost(radii_opt, buildup_opt))

        scale_fix1_ratio = None
        scale_fix1_applied = False
        anchors_step_seed = np.asarray(anchors_current, dtype=float)
        anchors_step_seed_cost = float(spool_cost_fixed)
        if use_scale_fix_1 and search_r and np.isfinite(spool_cost_fixed):
            scale_try = uniform_radius_scale(radii_opt, radii_current)
            if scale_try is not None and abs(float(scale_try) - 1.0) > 1e-12:
                anchors_scaled = np.asarray(anchors_current * float(scale_try), dtype=float)
                scaled_cost = float(data_cost(transformed_opt, anchors_scaled))
                if np.isfinite(scaled_cost):
                    improve_tol_fix1 = max(1e-9, 1e-4 * max(1.0, abs(float(spool_cost_fixed))))
                    if scaled_cost + improve_tol_fix1 < float(spool_cost_fixed):
                        anchors_step_seed = np.asarray(anchors_scaled, dtype=float)
                        anchors_step_seed_cost = float(scaled_cost)
                        scale_fix1_applied = True
                        scale_fix1_ratio = float(scale_try)

        anchors_candidate = np.asarray(anchors_step_seed, dtype=float)
        anchor_cost = float("nan")
        anchor_step_success = False
        cal_step_noise_metrics: Optional[dict] = None
        anchor_step_solver = None
        anchor_step_restarts = max(1, min(2, int(solve_restarts)))
        anchor_step_iterations = max(40, min(160, int(solve_iterations)))
        if run_anchor_step:
            try:
                cal_step = solve_anchor_proposal(
                    transformed_opt,
                    np.asarray(anchors_step_seed, dtype=float),
                    num_restarts=int(anchor_step_restarts),
                    max_iterations=int(anchor_step_iterations),
                    robust_debug_enabled=False,
                )
                if isinstance(cal_step, dict):
                    anchor_step_solver = cal_step.get("solver")
                cand_anchors = np.asarray(cal_step.get("anchors"), dtype=float)
                if cand_anchors.ndim == 2 and cand_anchors.shape == anchors_current.shape and np.all(
                    np.isfinite(cand_anchors)
                ):
                    anchors_candidate = cand_anchors
                    anchor_step_success = True
                    anchor_cost = float(data_cost(transformed_opt, anchors_candidate))
                    cal_step_noise_metrics = extract_noise_metrics(cal_step)
            except Exception:
                anchor_step_success = False

        accepted_anchors = np.asarray(anchors_step_seed, dtype=float)
        accepted_cost = float(anchors_step_seed_cost)
        accepted_total_cost = (
            float(accepted_cost + spool_prior)
            if np.isfinite(accepted_cost) and np.isfinite(spool_prior)
            else float("inf")
        )
        accepted_rank_score, accepted_rank_internal = spool_rank_score(
            transformed_opt,
            accepted_anchors,
        )
        accepted_alpha = 0.0
        if anchor_step_success:
            for alpha in (1.0, 0.5, 0.25):
                if alpha >= 1.0 - 1e-12:
                    anchors_try = np.asarray(anchors_candidate, dtype=float)
                else:
                    anchors_try = np.asarray(
                        anchors_step_seed + (anchors_candidate - anchors_step_seed) * float(alpha),
                        dtype=float,
                    )
                cost_try = float(data_cost(transformed_opt, anchors_try))
                if not np.isfinite(cost_try):
                    continue
                total_try = (
                    float(cost_try + spool_prior)
                    if np.isfinite(cost_try) and np.isfinite(spool_prior)
                    else float("inf")
                )
                if not np.isfinite(total_try):
                    continue
                noise_hint = cal_step_noise_metrics if alpha >= 1.0 - 1e-12 else None
                rank_try, rank_internal_try = spool_rank_score(
                    transformed_opt,
                    anchors_try,
                    noise_metrics_hint=noise_hint,
                )
                if rank_better(rank_try, total_try, accepted_rank_score, accepted_total_cost):
                    accepted_cost = float(cost_try)
                    accepted_total_cost = float(total_try)
                    accepted_rank_score = float(rank_try)
                    accepted_rank_internal = (
                        None if rank_internal_try is None else float(rank_internal_try)
                    )
                    accepted_anchors = np.asarray(anchors_try, dtype=float)
                    accepted_alpha = float(alpha)

        radius_anchor_probe: Dict[str, object] = {
            "attempted": False,
            "selected": False,
            "points": [],
            "best_radius": None,
        }
        probe_candidate: Optional[Dict[str, object]] = None
        if (
            bool(use_global_radius_anchor_probe)
            and bool(search_r)
            and str(mode_r) == "global"
            and x_opt.size > 0
            and lo.size > 0
            and hi.size > 0
        ):
            radius_anchor_probe["attempted"] = True
            radius_value = float(np.asarray(x_opt, dtype=float).reshape(-1)[0])
            radius_lo = float(np.asarray(lo, dtype=float).reshape(-1)[0])
            radius_hi = float(np.asarray(hi, dtype=float).reshape(-1)[0])
            radius_tol = 1e-9 * max(
                1.0,
                abs(float(radius_value)),
                abs(float(radius_lo)),
                abs(float(radius_hi)),
            )
            direction_target = None
            if abs(float(radius_value) - float(radius_lo)) <= radius_tol and radius_hi > radius_lo + radius_tol:
                direction_target = float(radius_hi)
            elif abs(float(radius_value) - float(radius_hi)) <= radius_tol and radius_lo < radius_hi - radius_tol:
                direction_target = float(radius_lo)
            if direction_target is not None:
                probe_points: list[float] = []
                for frac in (0.25, 0.5):
                    probe_radius = float(radius_value + (direction_target - radius_value) * float(frac))
                    if (
                        probe_radius <= radius_lo + radius_tol
                        or probe_radius >= radius_hi - radius_tol
                    ):
                        continue
                    if any(abs(probe_radius - prev) <= radius_tol for prev in probe_points):
                        continue
                    probe_points.append(float(probe_radius))
                radius_anchor_probe["points"] = [float(v) for v in probe_points]
                probe_restarts = 1
                probe_iterations = max(20, min(80, int(anchor_step_iterations)))
                probe_rank_score = float(accepted_rank_score)
                probe_total_cost = float(accepted_total_cost)
                for probe_radius in probe_points:
                    try:
                        x_probe = np.asarray(x_opt, dtype=float).reshape(-1).copy()
                        x_probe[0] = float(probe_radius)
                        radii_probe, buildup_probe = unpack_spool_opt_vector(
                            x_probe,
                            num_anchors=num_anchors,
                            find_radii_mode=mode_r,
                            find_buildup_mode=mode_b,
                            fixed_radii_mm=base,
                            fixed_buildup_factor=modeled_b,
                        )
                        spool_params_probe, transformed_probe = build_dataset_and_params(
                            radii_probe,
                            buildup_probe,
                        )
                        prior_probe = float(prior_cost(radii_probe, buildup_probe))
                        scale_probe = uniform_radius_scale(radii_probe, radii_current)
                        anchors_probe_seed = np.asarray(anchors_current, dtype=float)
                        if scale_probe is not None and np.isfinite(float(scale_probe)):
                            anchors_probe_seed = np.asarray(
                                anchors_current * float(scale_probe),
                                dtype=float,
                            )
                        cal_probe = solve_anchor_proposal(
                            transformed_probe,
                            anchors_probe_seed,
                            num_restarts=int(probe_restarts),
                            max_iterations=int(probe_iterations),
                            robust_debug_enabled=False,
                        )
                        anchors_probe = np.asarray(
                            cal_probe.get("anchors", anchors_probe_seed),
                            dtype=float,
                        )
                        if (
                            anchors_probe.ndim != 2
                            or anchors_probe.shape != anchors_current.shape
                            or not np.all(np.isfinite(anchors_probe))
                        ):
                            anchors_probe = np.asarray(anchors_probe_seed, dtype=float)
                        noise_probe = extract_noise_metrics(cal_probe)
                        cost_probe = float(data_cost(transformed_probe, anchors_probe))
                        total_probe = (
                            float(cost_probe + prior_probe)
                            if np.isfinite(cost_probe) and np.isfinite(prior_probe)
                            else float("inf")
                        )
                        if not np.isfinite(total_probe):
                            continue
                        rank_probe, rank_internal_probe = spool_rank_score(
                            transformed_probe,
                            anchors_probe,
                            noise_metrics_hint=noise_probe,
                        )
                        if rank_better(rank_probe, total_probe, probe_rank_score, probe_total_cost):
                            probe_rank_score = float(rank_probe)
                            probe_total_cost = float(total_probe)
                            probe_candidate = {
                                "radii": np.asarray(radii_probe, dtype=float),
                                "buildup": np.asarray(buildup_probe, dtype=float),
                                "anchors": np.asarray(anchors_probe, dtype=float),
                                "spool_params": spool_params_probe,
                                "dataset": transformed_probe,
                                "data_cost": float(cost_probe),
                                "prior_cost": float(prior_probe),
                                "rank_score": float(rank_probe),
                                "rank_internal": (
                                    None
                                    if rank_internal_probe is None
                                    else float(rank_internal_probe)
                                ),
                                "radius": float(probe_radius),
                            }
                    except Exception:
                        continue
                if probe_candidate is not None:
                    radius_anchor_probe["selected"] = True
                    radius_anchor_probe["best_radius"] = float(probe_candidate["radius"])

        radii_candidate = np.asarray(radii_opt, dtype=float)
        buildup_candidate = np.asarray(buildup_opt, dtype=float)
        anchors_candidate_final = np.asarray(accepted_anchors, dtype=float)
        spool_params_candidate = spool_params_opt
        transformed_candidate = transformed_opt
        candidate_data_cost = float(accepted_cost)
        candidate_prior_cost = float(spool_prior)
        candidate_rank_score = float("inf")
        candidate_rank_internal = None
        candidate_update_label = "spool_only"
        if accepted_alpha >= 1.0 - 1e-12:
            candidate_update_label = "anchor_full"
        elif accepted_alpha > 0.0:
            candidate_update_label = "anchor_damped"
        if probe_candidate is not None:
            radii_candidate = np.asarray(probe_candidate["radii"], dtype=float)
            buildup_candidate = np.asarray(probe_candidate["buildup"], dtype=float)
            anchors_candidate_final = np.asarray(probe_candidate["anchors"], dtype=float)
            spool_params_candidate = probe_candidate["spool_params"]
            transformed_candidate = probe_candidate["dataset"]
            candidate_data_cost = float(probe_candidate["data_cost"])
            candidate_prior_cost = float(probe_candidate["prior_cost"])
            candidate_rank_score = float(probe_candidate["rank_score"])
            candidate_rank_internal = probe_candidate["rank_internal"]
            candidate_update_label = "radius_anchor_probe"
        scale_fix3_info: Dict[str, object] = {
            "attempted": False,
            "success": False,
            "accepted": False,
            "message": "scale polish disabled",
        }
        accepted_total_cost = (
            float(candidate_data_cost + candidate_prior_cost)
            if np.isfinite(candidate_data_cost) and np.isfinite(candidate_prior_cost)
            else float("inf")
        )
        if not np.isfinite(candidate_rank_score):
            candidate_rank_score, candidate_rank_internal = spool_rank_score(
                transformed_candidate,
                anchors_candidate_final,
                noise_metrics_hint=(
                    cal_step_noise_metrics if accepted_alpha >= 1.0 - 1e-12 else None
                ),
            )

        rollback = not rank_better(
            candidate_rank_score,
            accepted_total_cost,
            current_rank_score,
            current_total_cost,
        )

        if rollback:
            radii_next = np.asarray(radii_current, dtype=float)
            buildup_next = np.asarray(buildup_current, dtype=float)
            anchors_next = np.asarray(anchors_current, dtype=float)
            spool_params_next = spool_params_current
            transformed_next = transformed_current
            model_cost = float(current_cost)
            model_prior_cost = float(current_prior)
            model_rank_score = float(current_rank_score)
            model_rank_internal = (
                None if current_rank_internal is None else float(current_rank_internal)
            )
            accepted_update = "rollback"
            accepted_alpha = 0.0
        else:
            radii_next = np.asarray(radii_candidate, dtype=float)
            buildup_next = np.asarray(buildup_candidate, dtype=float)
            anchors_next = np.asarray(anchors_candidate_final, dtype=float)
            spool_params_next = spool_params_candidate
            transformed_next = transformed_candidate
            model_cost = float(candidate_data_cost)
            model_prior_cost = float(candidate_prior_cost)
            model_rank_score = float(candidate_rank_score)
            model_rank_internal = (
                None if candidate_rank_internal is None else float(candidate_rank_internal)
            )
            accepted_update = str(candidate_update_label)

        model_total_cost = (
            float(model_cost + model_prior_cost)
            if np.isfinite(model_cost) and np.isfinite(model_prior_cost)
            else float("inf")
        )
        if rank_better(
            model_rank_score,
            model_total_cost,
            float(best.get("rank_score", float("inf"))),
            float(best.get("total_cost", float("inf"))),
        ):
            best = {
                "cost": float(model_cost),
                "total_cost": float(model_total_cost),
                "rank_score": float(model_rank_score),
                "rank_internal": (None if model_rank_internal is None else float(model_rank_internal)),
                "radii_mm": np.asarray(radii_next, dtype=float),
                "buildup_factor": np.asarray(buildup_next, dtype=float),
                "anchors": np.asarray(anchors_next, dtype=float),
                "spool_params": spool_params_next,
                "dataset": transformed_next,
            }

        start_cost = float(opt_info.get("start_cost", float("nan")))
        fitted_cost = float(opt_info.get("fitted_cost", float("nan")))
        history_total_cost = (
            float(model_cost + model_prior_cost)
            if np.isfinite(model_cost) and np.isfinite(model_prior_cost)
            else float("nan")
        )
        history.append(
            {
                "outer_iter": int(outer_idx + 1),
                "objective_id": int(core_objective_id),
                "objective_name": str(core_objective_name),
                "success": bool(opt_info.get("success", False)),
                "message": str(opt_info.get("message", "")),
                "nfev": int(opt_info.get("nfev", eval_counter["count"])),
                "nit": int(opt_info.get("nit", 0)),
                "start_cost": float(start_cost),
                "fitted_cost": float(fitted_cost),
                "start_data_cost": (
                    float(start_objective_cost) if np.isfinite(start_objective_cost) else None
                ),
                "start_prior_cost": (
                    float(start_prior_cost) if np.isfinite(start_prior_cost) else None
                ),
                "start_total_cost": float(start_cost) if np.isfinite(start_cost) else None,
                "fitted_data_cost": (
                    float(fitted_objective_cost) if np.isfinite(fitted_objective_cost) else None
                ),
                "fitted_prior_cost": (
                    float(fitted_prior_cost) if np.isfinite(fitted_prior_cost) else None
                ),
                "fitted_total_cost": float(fitted_cost) if np.isfinite(fitted_cost) else None,
                "start_objective_cost": (
                    float(start_objective_cost) if np.isfinite(start_objective_cost) else None
                ),
                "fitted_objective_cost": (
                    float(fitted_objective_cost) if np.isfinite(fitted_objective_cost) else None
                ),
                "current_objective_cost": (
                    float(current_objective_cost) if np.isfinite(current_objective_cost) else None
                ),
                "current_objective_total_cost": (
                    float(current_objective_total_cost)
                    if np.isfinite(current_objective_total_cost)
                    else None
                ),
                "current_data_cost": float(current_cost) if np.isfinite(current_cost) else None,
                "current_prior_cost": float(current_prior) if np.isfinite(current_prior) else None,
                "current_cost": float(current_cost) if np.isfinite(current_cost) else None,
                "current_total_cost": (
                    float(current_total_cost) if np.isfinite(current_total_cost) else None
                ),
                "current_rank_score": (
                    float(current_rank_score) if np.isfinite(current_rank_score) else None
                ),
                "current_rank_internal": (
                    None if current_rank_internal is None else float(current_rank_internal)
                ),
                "spool_data_cost_fixed_anchors": (
                    float(spool_cost_fixed) if np.isfinite(spool_cost_fixed) else None
                ),
                "spool_objective_cost_fixed_anchors": (
                    float(spool_objective_fixed) if np.isfinite(spool_objective_fixed) else None
                ),
                "spool_prior_cost": float(spool_prior) if np.isfinite(spool_prior) else None,
                "spool_cost_fixed_anchors": (
                    float(spool_cost_fixed) if np.isfinite(spool_cost_fixed) else None
                ),
                "spool_objective_total_cost_fixed_anchors": (
                    float(spool_objective_fixed + spool_prior)
                    if np.isfinite(spool_objective_fixed) and np.isfinite(spool_prior)
                    else None
                ),
                "spool_total_cost_fixed_anchors": (
                    float(spool_cost_fixed + spool_prior)
                    if np.isfinite(spool_cost_fixed) and np.isfinite(spool_prior)
                    else None
                ),
                "scale_fix_1_applied": bool(scale_fix1_applied),
                "scale_fix_1_ratio": (
                    float(scale_fix1_ratio) if isinstance(scale_fix1_ratio, (int, float)) else None
                ),
                "spool_data_cost_scale_seed": (
                    float(anchors_step_seed_cost) if np.isfinite(anchors_step_seed_cost) else None
                ),
                "radius_anchor_probe": dict(radius_anchor_probe),
                "anchor_step_triggered": bool(run_anchor_step),
                "anchor_step_trigger_reason": (
                    "spool_objective_improved"
                    if bool(run_anchor_step)
                    else "spool_objective_not_improved"
                ),
                "anchor_step_trigger_improvement": (
                    float(anchor_step_trigger_improvement)
                    if np.isfinite(anchor_step_trigger_improvement)
                    else None
                ),
                "anchor_step_trigger_threshold": (
                    float(anchor_step_trigger_threshold)
                    if np.isfinite(anchor_step_trigger_threshold)
                    else None
                ),
                "anchor_step_solver": (None if anchor_step_solver is None else str(anchor_step_solver)),
                "anchor_step_success": bool(anchor_step_success),
                "anchor_cost": float(anchor_cost) if np.isfinite(anchor_cost) else None,
                "cal_step_success": bool(anchor_step_success),
                "cal_cost": float(model_cost) if np.isfinite(model_cost) else None,
                "cal_data_cost": float(model_cost) if np.isfinite(model_cost) else None,
                "cal_prior_cost": (
                    float(model_prior_cost) if np.isfinite(model_prior_cost) else None
                ),
                "cal_total_cost": (
                    float(history_total_cost)
                    if np.isfinite(history_total_cost)
                    else None
                ),
                "cal_rank_score": (
                    float(model_rank_score) if np.isfinite(model_rank_score) else None
                ),
                "cal_rank_internal": (
                    None if model_rank_internal is None else float(model_rank_internal)
                ),
                "accepted_rank_score": (
                    float(accepted_rank_score) if np.isfinite(accepted_rank_score) else None
                ),
                "accepted_rank_internal": (
                    None if accepted_rank_internal is None else float(accepted_rank_internal)
                ),
                "accepted_update": str(accepted_update),
                "anchor_blend_alpha": float(accepted_alpha),
                "scale_fix_3": dict(scale_fix3_info),
                "step_final": opt_info.get("step_final"),
                "seed_choice": str(opt_info.get("seed_choice", "")),
                "seed_cost": opt_info.get("seed_cost"),
                "seed_rank_score": opt_info.get("seed_rank_score"),
                "radii_mm": [float(v) for v in np.asarray(radii_next, dtype=float).tolist()],
                "buildup_factor": [float(v) for v in np.asarray(buildup_next, dtype=float).tolist()],
            }
        )

        x_current = pack_spool_opt_vector(
            radii_mm=radii_next,
            buildup_factor=buildup_next,
            find_radii_mode=mode_r,
            find_buildup_mode=mode_b,
        )
        if x_current.size > 0:
            x_current = np.clip(x_current, lo, hi)
        radii_current = np.asarray(radii_next, dtype=float)
        buildup_current = np.asarray(buildup_next, dtype=float)
        anchors_current = np.asarray(anchors_next, dtype=float)

    return {
        "best": best,
        "history": history,
        "radii_current": np.asarray(radii_current, dtype=float),
        "buildup_current": np.asarray(buildup_current, dtype=float),
        "anchors_current": np.asarray(anchors_current, dtype=float),
        "x_current": np.asarray(x_current, dtype=float),
    }
