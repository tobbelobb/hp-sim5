from __future__ import annotations

from autocal._autocal_common import *  # noqa: F401,F403
from autocal.filter_pass import estimate_effective_radii_with_spool_model


def plan_next_ellipse_sweep(
    dataset_path: Path,
    *,
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
    residuals_csv: Optional[Path],
    generate_report: bool,
    find_radii: str,
    find_buildup_factor: str,
    base_radii: Optional[List[float]],
    buildup_factor: Optional[float],
    r0_bounds: Optional[Tuple[float, float]],
    b_bounds: Optional[Tuple[float, float]],
    r0_prior_sigma_mm: Optional[float],
    b_prior_sigma: Optional[float],
    spool_outer_iters: int,
    spool_inner_iters: int,
    theta0_mode: str,
    line_width: float,
    sigma_floor_mm: Optional[float],
    sigma_used_mm: Optional[float],
    candidate_deltas: Optional[List[float]],
    candidate_count: int,
    delta_min: Optional[float],
    delta_max: Optional[float],
    fd_eps_mm: float,
    regularization: float,
    exclude_existing: bool,
    existing_tol_mm: float,
    min_fixed_delta_spacing_mm: float,
    top_k: int,
    write_cfg: Optional[Path],
    collector_output: Optional[Path],
    collector_args: Sequence[str],
    low_anchor_z: Optional[float] = None,
    filter_schedule: Optional[Sequence[Any]] = None,
    objective_schedule: Optional[Sequence[Any]] = None,
    scale_fix: Optional[Sequence[int]] = None,
    fit_structure: Optional[Sequence[int]] = None,
) -> Dict[str, object]:
    dataset = _load_json(dataset_path)
    if low_anchor_z is not None:
        try:
            low_anchor_z = float(low_anchor_z)
        except (TypeError, ValueError):
            low_anchor_z = None
        if low_anchor_z is not None and np.isfinite(low_anchor_z):
            dataset = dict(dataset)
            dataset["_low_anchor_z"] = float(low_anchor_z)
        else:
            low_anchor_z = None
    remapped_points = _normalize_dataset_point_roles(dataset)
    machine_type = _require_machine_type(dataset, context=str(dataset_path))
    num_anchors = int(dataset.get("num_anchors", 4))
    dimensions = int(dataset.get("dimensions", 3))
    warnings: List[str] = []
    if remapped_points > 0:
        warnings.append(f"point_role_remap_applied:{int(remapped_points)}")
    find_radii_mode = _normalize_spool_find_mode(find_radii)
    find_buildup_mode = _normalize_spool_find_mode(find_buildup_factor)
    scale_fix_levels = _parse_scale_fix_levels(scale_fix)
    fit_structure_levels = _parse_fit_structure_levels(fit_structure)
    theta0_mode_norm = _normalize_theta0_mode(theta0_mode)
    search_radii = _spool_mode_enabled(find_radii_mode)
    search_buildup = _spool_mode_enabled(find_buildup_mode)
    if r0_bounds is not None and not _spool_mode_enabled(find_radii_mode):
        warnings.append("r0_bounds_ignored_without_find_radii")
    if r0_prior_sigma_mm is not None and not _spool_mode_enabled(find_radii_mode):
        warnings.append("r0_prior_sigma_mm_ignored_without_find_radii")
    if b_bounds is not None and not _spool_mode_enabled(find_buildup_mode):
        warnings.append("b_bounds_ignored_without_find_buildup_factor")
    if b_prior_sigma is not None and not _spool_mode_enabled(find_buildup_mode):
        warnings.append("b_prior_sigma_ignored_without_find_buildup_factor")

    est_buildup = _resolve_buildup_factor_seed(
        dataset,
        buildup_factor_override=buildup_factor,
    )

    length_model: Optional[Dict[str, object]] = None
    spool_params: Optional[SpoolModelParams] = None
    dataset_for_estimation = dataset
    sweep_configs_for_info = dataset_sweep_configs(dataset)
    prefer_zero_tension_angles = _arg_has_flag(
        collector_args,
        "--project-zero-tension",
        "--projectZeroTension",
    )
    _annotate_dataset_noise_model(
        dataset,
        line_width_mm=float(line_width),
        sigma_floor_mm=sigma_floor_mm,
        sigma_used_mm=sigma_used_mm,
        find_radii_mode=find_radii_mode,
        find_buildup_mode=find_buildup_mode,
        project_zero_tension=bool(prefer_zero_tension_angles),
    )

    if search_radii or search_buildup:
        validate_dataset_has_raw_angles(
            dataset,
            prefer_zero_tension_angles=bool(prefer_zero_tension_angles),
        )
        lm_params = _resolve_length_model_base_params(
            dataset,
            num_anchors=int(num_anchors),
            base_radii_override=base_radii,
        )
        base_radii_mm = np.asarray(lm_params["base_radii_mm"], dtype=float)
        spool_to_motor_gearing_factor = np.asarray(
            lm_params["spool_to_motor_gearing_factor"], dtype=float
        )
        mechanical_advantage = np.asarray(lm_params["mechanical_advantage"], dtype=float)
        lines_per_spool = np.asarray(lm_params["lines_per_spool"], dtype=float)
        modeled_buildup = _default_modeled_buildup_values(
            num_anchors=int(num_anchors),
            find_buildup_factor_mode=find_buildup_mode,
            buildup_factor=est_buildup,
        )
        anchor_initial_guess = build_anchor_initial_guess(
            dataset,
            machine_type=str(machine_type),
            base_radii_mm=base_radii_mm.tolist(),
            low_anchor_z=low_anchor_z,
        )

        seed_restarts = max(1, min(2, int(solve_restarts)))
        seed_iterations = max(60, min(200, int(solve_iterations)))
        seed_cal = calibrate_elliptical(
            dataset,
            output_path=None,
            residual_threshold=float(residual_threshold),
            num_restarts=int(seed_restarts),
            max_iterations=int(seed_iterations),
            method=str(solve_optimizer),
            spring_k_multiplier=float(spring_k_multiplier),
            use_flex=bool(use_flex),
            verbose=False,
            use_parallel=False,
            pointwise_residual_mode=str(pointwise_residual_mode),
            robust_debug=False,
            pointwise_filtering=bool(pointwise_filtering),
            pointwise_global_mad=bool(pointwise_global_mad),
            sweep_wise_filtering=bool(sweep_wise_filtering),
            sweep_metric=str(sweep_metric),
            use_noise_mean=bool(use_noise_mean),
            sigma_source=str(sigma_source),
            generate_report=False,
            residuals_csv=None,
            initial_guess=anchor_initial_guess,
            low_anchor_z=low_anchor_z,
        )
        seed_anchors = np.asarray(seed_cal["anchors"], dtype=float)
        effective_radii_mm, _fit_anchors, spool_params, dataset_for_estimation, radii_fit = (
            estimate_effective_radii_with_spool_model(
                dataset,
                seed_anchors,
                find_radii_mode=find_radii_mode,
                find_buildup_mode=find_buildup_mode,
                base_radii_mm=base_radii_mm,
                modeled_buildup_factor=modeled_buildup,
                spool_to_motor_gearing_factor=spool_to_motor_gearing_factor,
                mechanical_advantage=mechanical_advantage,
                lines_per_spool=lines_per_spool,
                r0_bounds=r0_bounds,
                b_bounds=b_bounds,
                r0_prior_sigma_mm=r0_prior_sigma_mm,
                b_prior_sigma=b_prior_sigma,
                spool_outer_iters=int(spool_outer_iters),
                spool_inner_iters=int(spool_inner_iters),
                theta0_mode=theta0_mode_norm,
                solve_restarts=int(solve_restarts),
                solve_iterations=int(solve_iterations),
                solve_optimizer=str(solve_optimizer),
                residual_threshold=float(residual_threshold),
                spring_k_multiplier=float(spring_k_multiplier),
                use_flex=bool(use_flex),
                pointwise_residual_mode=str(pointwise_residual_mode),
                pointwise_filtering=bool(pointwise_filtering),
                pointwise_global_mad=bool(pointwise_global_mad),
                sweep_wise_filtering=bool(sweep_wise_filtering),
                sweep_metric=str(sweep_metric),
                use_noise_mean=bool(use_noise_mean),
                sigma_source=str(sigma_source),
                robust_debug=bool(robust_debug),
                low_anchor_z=low_anchor_z,
                prefer_zero_tension_angles=bool(prefer_zero_tension_angles),
                scale_fix_levels=scale_fix_levels,
                fit_structure_levels=fit_structure_levels,
                filter_schedule=_parse_filter_schedule(filter_schedule, label="filter_schedule"),
                objective_schedule=_parse_objective_schedule(
                    objective_schedule,
                    label="objective_schedule",
                ),
            )
        )
        _ = _fit_anchors
        fitted_buildup = np.asarray(
            radii_fit.get("best_modeled_buildup_factor", modeled_buildup.tolist()),
            dtype=float,
        ).reshape(-1)
        if fitted_buildup.size != int(num_anchors):
            fitted_buildup = np.asarray(modeled_buildup, dtype=float).reshape(-1)
        k_summary = float(np.median(fitted_buildup)) if fitted_buildup.size else float(est_buildup)
        sweep_configs_for_info = sweep_configs_with_modeled_lengths(
            sweep_configs_for_info,
            spool_params,
        )
        final_filter_schedule_cal = (
            radii_fit.get("filter_schedule_final_calibration")
            if isinstance(radii_fit, dict)
            else None
        )
        if isinstance(final_filter_schedule_cal, dict):
            cal = final_filter_schedule_cal
        else:
            cal = calibrate_elliptical(
                dataset_for_estimation,
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
                pointwise_filtering=bool(pointwise_filtering),
                pointwise_global_mad=bool(pointwise_global_mad),
                sweep_wise_filtering=bool(sweep_wise_filtering),
                sweep_metric=str(sweep_metric),
                use_noise_mean=bool(use_noise_mean),
                sigma_source=str(sigma_source),
                generate_report=bool(generate_report),
                residuals_csv=residuals_csv,
                report_base_path=dataset_path,
                initial_guess=seed_anchors,
                low_anchor_z=low_anchor_z,
            )

        length_model = {
            "coord_planning": "L_base_mm",
            "coord_estimation": "L_model_mm",
            "find_radii_mode": str(find_radii_mode),
            "find_buildup_factor_mode": str(find_buildup_mode),
            "find_radii": _spool_mode_enabled(find_radii_mode),
            "find_buildup_factor": _spool_mode_enabled(find_buildup_mode),
            "buildup_factor_k": float(k_summary),
            "base_radii_mm": [float(v) for v in base_radii_mm.tolist()],
            "effective_radii_mm": [float(v) for v in np.asarray(effective_radii_mm, dtype=float).tolist()],
            "modeled_radii_mm": [float(v) for v in np.asarray(effective_radii_mm, dtype=float).tolist()],
            "modeled_buildup_factor": [float(v) for v in np.asarray(fitted_buildup, dtype=float).tolist()],
            "spool_to_motor_gearing_factor": [
                float(v) for v in np.asarray(spool_to_motor_gearing_factor, dtype=float).tolist()
            ],
            "mechanical_advantage": [float(v) for v in np.asarray(mechanical_advantage, dtype=float).tolist()],
            "lines_per_spool": [float(v) for v in np.asarray(lines_per_spool, dtype=float).tolist()],
            "r0_bounds_mm": (
                [float(r0_bounds[0]), float(r0_bounds[1])]
                if isinstance(r0_bounds, tuple)
                else None
            ),
            "r0_prior_sigma_mm": (
                float(r0_prior_sigma_mm)
                if isinstance(r0_prior_sigma_mm, (int, float)) and np.isfinite(r0_prior_sigma_mm)
                else None
            ),
            "b_bounds": (
                [float(b_bounds[0]), float(b_bounds[1])]
                if isinstance(b_bounds, tuple)
                else None
            ),
            "b_prior_sigma": (
                float(b_prior_sigma)
                if isinstance(b_prior_sigma, (int, float)) and np.isfinite(b_prior_sigma)
                else None
            ),
            "spool_outer_iters": int(spool_outer_iters),
            "spool_inner_iters": int(spool_inner_iters),
            "filter_schedule": [
                str(v) for v in _parse_filter_schedule(filter_schedule, label="filter_schedule")
            ],
            "objective_schedule": [
                int(v)
                for v in _parse_objective_schedule(
                    objective_schedule,
                    label="objective_schedule",
                )
            ],
            "theta0_mode": theta0_mode_norm,
            "scale_fix_levels": [int(v) for v in scale_fix_levels],
            "fit_structure_levels": [int(v) for v in fit_structure_levels],
            "radii_fit": radii_fit,
            "spool_fit": radii_fit,
        }
    else:
        anchor_initial_guess = build_anchor_initial_guess(
            dataset,
            machine_type=str(machine_type),
            base_radii_mm=None,
            low_anchor_z=low_anchor_z,
        )
        cal = calibrate_elliptical(
            dataset,
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
            pointwise_filtering=bool(pointwise_filtering),
            pointwise_global_mad=bool(pointwise_global_mad),
            sweep_wise_filtering=bool(sweep_wise_filtering),
            sweep_metric=str(sweep_metric),
            use_noise_mean=bool(use_noise_mean),
            sigma_source=str(sigma_source),
            generate_report=bool(generate_report),
            residuals_csv=residuals_csv,
            initial_guess=anchor_initial_guess,
            low_anchor_z=low_anchor_z,
        )

    anchors = np.asarray(cal["anchors"], dtype=float)
    cost = float(cal.get("cost", float("nan")))
    cost_raw = float(
        _evaluate_cost_at_anchors(
            dataset_for_estimation,
            anchors,
            residual_threshold=float(residual_threshold),
            spring_k_multiplier=float(spring_k_multiplier),
            use_flex=bool(use_flex),
            pointwise_residual_mode=str(pointwise_residual_mode),
            pointwise_filtering=bool(pointwise_filtering),
            pointwise_global_mad=bool(pointwise_global_mad),
            sweep_wise_filtering=bool(sweep_wise_filtering),
            sweep_metric=str(sweep_metric),
            use_noise_mean=bool(use_noise_mean),
            noise_normalized=False,
            sigma_source=str(sigma_source),
        )
    )

    sweeps_obs = dataset_sweep_configs(dataset)
    l2_scale = l2_scale_for_machine(machine_type, num_anchors, dimensions)
    info_obs = total_information_matrix(
        anchors,
        sweep_configs_for_info,
        machine_type=machine_type,
        num_anchors=num_anchors,
        dimensions=dimensions,
        l2_scale=l2_scale,
        fd_eps_mm=float(fd_eps_mm),
    )
    cov = _estimate_anchor_covariance(info_obs, regularization=float(regularization))
    noise_metrics = None
    if isinstance(cal, dict):
        details = cal.get("details")
        if isinstance(details, dict):
            nm = details.get("noise_metrics")
            if isinstance(nm, dict):
                noise_metrics = nm
    if isinstance(noise_metrics, dict) and bool(search_radii or search_buildup):
        cost_fn = _build_ellipse_cost_function(
            dataset_for_estimation,
            residual_threshold=float(residual_threshold),
            spring_k_multiplier=float(spring_k_multiplier),
            use_flex=bool(use_flex),
            pointwise_residual_mode=str(pointwise_residual_mode),
            pointwise_filtering=bool(pointwise_filtering),
            pointwise_global_mad=bool(pointwise_global_mad),
            sweep_wise_filtering=bool(sweep_wise_filtering),
            sweep_metric=str(sweep_metric),
            use_noise_mean=bool(use_noise_mean),
            noise_normalized=True,
            sigma_source=str(sigma_source),
        )
        rows = cost_fn.pointwise_residual_rows(np.asarray(anchors, dtype=float).ravel())
        sigma_model_mm = noise_metrics.get("sigma_model_mm")
        if _float_or_none(sigma_model_mm) is None:
            sigma_model_mm = noise_metrics.get("sigma_used_mm")
        rescored = _compute_tau_mad_rescore_from_rows(
            rows,
            cost_noise_normalized_old=cost,
            chi2_red_old=noise_metrics.get("chi2_red"),
            sigma_model_mm=sigma_model_mm,
            params_count=noise_metrics.get("params"),
        )
        noise_metrics = {**noise_metrics, **rescored}
        if isinstance(cal, dict):
            details = cal.get("details")
            if not isinstance(details, dict):
                details = {}
                cal["details"] = details
            details["noise_metrics"] = noise_metrics
    cov_scaled, cov_scale, cov_scale_label = _scale_covariance(cov, noise_metrics)
    ci = _confidence_intervals(cov_scaled)
    workspace_diag = _workspace_diag_mm(
        dataset,
        anchors,
        machine_type=machine_type,
        num_anchors=num_anchors,
        dimensions=dimensions,
    )
    rank = None
    rank_deficient = False
    if info_obs.ndim == 2 and info_obs.shape[0] == info_obs.shape[1]:
        rank = int(np.linalg.matrix_rank(info_obs))
        rank_deficient = rank < info_obs.shape[0]
    cov_scaled_std = _covariance_diag_std(cov_scaled)
    if cov_scaled_std is None or not np.all(np.isfinite(cov_scaled_std)):
        warnings.append("covariance_nonfinite")

    observed_deltas: List[float] = []
    for cfg in sweeps_obs:
        observed_deltas.extend(list(cfg.fixed_deltas_mm))

    config = dataset.get("config") if isinstance(dataset, dict) else None
    max_travel_mm = None
    if isinstance(config, dict):
        raw_max_travel = config.get("max_travel_mm")
        if isinstance(raw_max_travel, (int, float)) and np.isfinite(raw_max_travel):
            max_travel_mm = float(raw_max_travel)
    max_total_fixed_delta_mm = None
    if (
        str(machine_type) == "hangprinter_4"
        and max_travel_mm is not None
        and np.isfinite(max_travel_mm)
        and max_travel_mm > 0.0
    ):
        max_total_fixed_delta_mm = float(max_travel_mm)

    if candidate_deltas is None:
        explicit_delta_range = delta_min is not None or delta_max is not None

        default_range = None
        if not explicit_delta_range:
            default_range = _default_delta_range(
                max_travel_mm=max_travel_mm,
                observed_deltas=observed_deltas,
            )
        if default_range is not None:
            lo, hi = default_range
        elif observed_deltas:
            lo = float(np.min(observed_deltas))
            hi = float(np.max(observed_deltas))
        else:
            lo, hi = -600.0, 600.0
        if delta_min is not None:
            lo = float(delta_min)
        if delta_max is not None:
            hi = float(delta_max)
        if not np.isfinite(lo) or not np.isfinite(hi) or abs(hi - lo) < 1e-9:
            fallback = _default_delta_range(
                max_travel_mm=max_travel_mm,
                observed_deltas=observed_deltas,
            )
            if fallback is not None:
                lo, hi = fallback
            else:
                lo, hi = -600.0, 600.0
        values = np.linspace(lo, hi, max(3, int(candidate_count)))
        candidate_deltas = [float(v) for v in values.tolist()]

    candidates = generate_candidate_sweeps(
        num_anchors=num_anchors,
        dimensions=dimensions,
        fixed_delta_values_mm=candidate_deltas,
        machine_type=machine_type,
        max_total_fixed_delta_mm=max_total_fixed_delta_mm,
    )
    candidates = _filter_candidates_by_spacing(
        candidates,
        sweeps_obs,
        min_spacing_mm=float(min_fixed_delta_spacing_mm),
    )
    if bool(exclude_existing):
        existing_keys = {
            cfg.normalized_key(tol_mm=float(existing_tol_mm))
            for cfg in sweeps_obs
        }
        candidates = [
            cfg
            for cfg in candidates
            if cfg.normalized_key(tol_mm=float(existing_tol_mm)) not in existing_keys
        ]

    if spool_params is not None:
        candidates_model = sweep_configs_with_modeled_lengths(candidates, spool_params)
        id_map = {id(cfg_model): cfg_base for cfg_base, cfg_model in zip(candidates, candidates_model)}
        ranked_model = rank_candidates_d_optimal(
            anchors,
            machine_type=machine_type,
            num_anchors=num_anchors,
            dimensions=dimensions,
            l2_scale=l2_scale,
            observed=sweep_configs_for_info,
            candidates=candidates_model,
            fd_eps_mm=float(fd_eps_mm),
            regularization=float(regularization),
            exclude_existing=False,
            existing_tol_mm=float(existing_tol_mm),
            top_k=int(top_k),
        )
        ranked = [(float(score), id_map.get(id(cfg_model), cfg_model)) for score, cfg_model in ranked_model]
    else:
        ranked = rank_candidates_d_optimal(
            anchors,
            machine_type=machine_type,
            num_anchors=num_anchors,
            dimensions=dimensions,
            l2_scale=l2_scale,
            observed=sweeps_obs,
            candidates=candidates,
            fd_eps_mm=float(fd_eps_mm),
            regularization=float(regularization),
            exclude_existing=False,
            existing_tol_mm=float(existing_tol_mm),
            top_k=int(top_k),
        )

    best_cfg = ranked[0][1] if ranked else None
    cfg_path = write_cfg or dataset_path.with_suffix(".active_sweep_cfg.txt")
    if best_cfg is not None:
        _write_sweep_config_file(cfg_path, best_cfg)

    collector_args_eff, force_tuning, force_args_applied = _inject_force_args(dataset, collector_args)
    collector_args_eff, _ = _inject_spool_collection_args(
        collector_args_eff,
        find_radii_mode=find_radii_mode,
        find_buildup_mode=find_buildup_mode,
        base_radii=base_radii,
        buildup_factor=buildup_factor,
    )
    if "--return-to-origin" not in collector_args_eff and "--returnToOrigin" not in collector_args_eff:
        collector_args_eff.append("--return-to-origin")

    cmd = None
    if best_cfg is not None:
        cmd = _suggested_collect_command(
            cfg_path,
            best_cfg,
            machine_type=machine_type,
            output_file=collector_output,
            extra_args=collector_args_eff,
        )

    return {
        "dataset": dataset,
        "machine_type": machine_type,
        "num_anchors": num_anchors,
        "dimensions": dimensions,
        "calibration": cal,
        "anchors": anchors,
        "cost": cost,
        "cost_raw": cost_raw,
        "cost_noise_normalized": cost,
        "info": info_obs,
        "covariance": cov,
        "covariance_scaled": cov_scaled,
        "covariance_scale": cov_scale,
        "covariance_scale_label": cov_scale_label,
        "confidence_intervals": ci,
        "workspace_diag_mm": workspace_diag,
        "info_rank": rank,
        "info_rank_deficient": rank_deficient,
        "warnings": warnings,
        "ranked": ranked,
        "best_cfg": best_cfg,
        "cfg_path": cfg_path,
        "collect_command": cmd,
        "force_tuning": force_tuning,
        "force_args_applied": force_args_applied,
        "length_model": length_model,
        "line_width_mm": float(line_width),
        "sigma_floor_mm": (None if sigma_floor_mm is None else float(sigma_floor_mm)),
        "sigma_used_mm": (None if sigma_used_mm is None else float(sigma_used_mm)),
        "fit_structure_levels": [int(v) for v in fit_structure_levels],
        "dataset_for_estimation": dataset_for_estimation,
    }
