"""Sweep planning and the one-click fit/plan composition."""
from __future__ import annotations

from autocal._autocal_common import *  # noqa: F401,F403
from autocal.fit_stage import fit_ellipse_dataset

_PLANNING_ONLY = (
    "candidate_deltas", "candidate_count", "delta_min", "delta_max",
    "exclude_existing", "existing_tol_mm", "min_fixed_delta_spacing_mm",
    "top_k", "write_cfg", "collector_output",
)
_PLANNING_OPTIONS = _PLANNING_ONLY + ("fd_eps_mm", "regularization", "collector_args")


def plan_ellipse_sweep(
    fit: Dict[str, object], dataset_path: Path, *,
    candidate_deltas: Optional[List[float]] = None,
    candidate_count: int = 41,
    delta_min: Optional[float] = None,
    delta_max: Optional[float] = None,
    fd_eps_mm: Optional[float] = None,
    regularization: Optional[float] = None,
    exclude_existing: bool = True,
    existing_tol_mm: float = 10.0,
    min_fixed_delta_spacing_mm: float = 20.0,
    top_k: int = 10,
    write_cfg: Optional[Path] = None,
    collector_output: Optional[Path] = None,
    collector_args: Sequence[str] = (),
) -> Dict[str, object]:
    """Plan from a frozen fit; no optimizer or measurement collection is run."""
    dataset = fit["dataset"]
    anchors = np.asarray(fit["anchors"], dtype=float)
    machine_type = str(fit["machine_type"])
    num_anchors = int(fit["num_anchors"])
    dimensions = int(fit["dimensions"])
    spool_params = fit["spool_model_params"]
    sweeps_obs = dataset_sweep_configs(dataset)
    sweep_configs_for_info = fit["sweep_configs_for_info"]
    l2_scale = l2_scale_for_machine(machine_type, num_anchors, dimensions)
    info_settings = fit["information_settings"]
    fd_eps_mm = float(info_settings["fd_eps_mm"] if fd_eps_mm is None else fd_eps_mm)
    regularization = float(info_settings["regularization"] if regularization is None else regularization)
    # Exact reuse only at the fitted finite-difference setting. The matrix has
    # no regularization yet; the ranker applies that once, on a fresh array.
    info_obs = fit["info"] if fd_eps_mm == info_settings["fd_eps_mm"] else None
    collection_settings = fit["collection_settings"]
    find_radii_mode = collection_settings["find_radii_mode"]
    find_buildup_mode = collection_settings["find_buildup_mode"]
    base_radii = collection_settings["base_radii"]
    buildup_factor = collection_settings["buildup_factor"]
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
            observed_information=info_obs,
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
            observed_information=info_obs,
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

    return {**fit, "ranked": ranked, "best_cfg": best_cfg, "cfg_path": cfg_path,
            "collect_command": cmd, "force_tuning": force_tuning,
            "force_args_applied": force_args_applied}


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
    stage_artifact: Optional[Path] = None,
    initial_guess: Optional[np.ndarray] = None,
    initial_radii_mm: Optional[np.ndarray] = None,
    initial_buildup_factor: Optional[np.ndarray] = None,
) -> Dict[str, object]:
    """One-click composition of the independently callable fit and plan stages."""
    options = dict(locals())
    options.pop("dataset_path")
    options.pop("stage_artifact")
    planning_options = {name: options[name] for name in _PLANNING_OPTIONS}
    fit_options = {name: value for name, value in options.items() if name not in _PLANNING_ONLY}
    started = time.perf_counter() if stage_artifact is not None else None
    fit = fit_ellipse_dataset(dataset_path, **fit_options)
    if stage_artifact is not None:
        fit_seconds = time.perf_counter() - started
        from autocal.stage_artifacts import write_stage_artifact
        write_stage_artifact(stage_artifact, "fit", {
            "fit": fit, "dataset_path": dataset_path,
            "fit_options": fit_options, "planning_options": planning_options,
            "fit_seconds": fit_seconds,
        })
    return plan_ellipse_sweep(fit, dataset_path, **planning_options)
