"""Sweep planning and the one-click fit/plan composition."""
from __future__ import annotations

from autocal._autocal_common import *  # noqa: F401,F403
from autocal.fit_stage import fit_ellipse_dataset
from autocal.equal_strain import (
    InfeasibleForceProfileError,
    effective_strand_tension_scale,
    force_planning_parameters,
    is_equal_strain,
    profile_policy_error,
    write_sensor_force_profile,
)
from autocal.pose_fit import dataset_line_lengths, has_extra_sensor_data, dataset_stretch, has_spare_winches
from autocal.pose_planning import rank_candidates_force_aware, rank_candidates_pose_information
from autocal.workspace import exploration_grid

_PLANNING_ONLY = (
    "candidate_deltas", "candidate_count", "delta_min", "delta_max",
    "exclude_existing", "existing_tol_mm", "min_fixed_delta_spacing_mm",
    "top_k", "write_cfg", "collector_output", "equal_strain", "force_contrast",
)
_PLANNING_OPTIONS = _PLANNING_ONLY + ("fd_eps_mm", "regularization", "collector_args")
_FORCE_CONTRAST_SCALES = (1.0, 2.0)
_SENSOR_FORCE_FLAGS = (
    "--sensor-force",
    "--force-sensor",
    "--sensorForce",
    "--sensor-collection-force",
    "--sensorCollectionForce",
)


def _strip_cli_option(args: Sequence[str], option_names: Sequence[str]) -> List[str]:
    """Remove matching CLI flags along with their separate value tokens (or `--flag=value` forms)."""
    names = tuple(option_names)
    cleaned: List[str] = []
    idx = 0
    while idx < len(args):
        token = str(args[idx])
        if token in names:
            idx += 2
            continue
        if any(token.startswith(name + "=") for name in names):
            idx += 1
            continue
        cleaned.append(args[idx])
        idx += 1
    return cleaned


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
    equal_strain: bool = True,
    force_contrast: bool = False,
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
    exploration_bounds = None
    if candidate_deltas is None:
        if machine_type in ("hangprinter_3", "hangprinter_4", "hangprinter_5", "hangprinter_5_pulley_reduced") and delta_min is None and delta_max is None:
            candidate_deltas, exploration_bounds = exploration_grid(dataset, candidate_count)
        else:
            default_range = _default_delta_range(max_travel_mm=max_travel_mm, observed_deltas=observed_deltas)
            lo, hi = default_range or (-600.0, 600.0)
            lo = float(delta_min) if delta_min is not None else lo
            hi = float(delta_max) if delta_max is not None else hi
            if not np.isfinite(lo) or not np.isfinite(hi) or hi <= lo:
                raise ValueError("candidate delta range must be finite with delta_min < delta_max")
            candidate_deltas = np.linspace(lo, hi, max(3, int(candidate_count))).tolist()

    candidates = generate_candidate_sweeps(
        num_anchors=num_anchors,
        dimensions=dimensions,
        fixed_delta_values_mm=candidate_deltas,
        machine_type=machine_type,
        fixed_delta_bounds_by_anchor=exploration_bounds,
    )
    if exclude_existing:
        rejected = dataset_sweep_configs({"sweeps": dataset.get("acquisition_rejections", [])})
        rejected_keys = {cfg.normalized_key(tol_mm=existing_tol_mm) for cfg in rejected}
        candidates = [cfg for cfg in candidates if cfg.normalized_key(tol_mm=existing_tol_mm) not in rejected_keys]
        existing_keys = {cfg.normalized_key(tol_mm=existing_tol_mm) for cfg in sweeps_obs}
        candidates = [cfg for cfg in candidates
                      if cfg.normalized_key(tol_mm=existing_tol_mm) not in existing_keys]
    novel_candidates = list(candidates)
    candidates = _filter_candidates_by_spacing(
        candidates, sweeps_obs, min_spacing_mm=float(min_fixed_delta_spacing_mm),
    )
    if not candidates:
        # Relax novelty spacing if needed, but never re-admit excluded repeats.
        candidates = novel_candidates

    needs_profile = (is_equal_strain(dataset) and spool_params is None
                     and machine_type in ("hangprinter_3", "hangprinter_4", "hangprinter_5", "hangprinter_5_pulley_reduced"))

    collector_args_eff, force_tuning, force_args_applied = _inject_force_args(dataset, collector_args)
    chosen_force_scale = 1.0
    ranked_with_force: Optional[List[Tuple[float, SweepConfig, float]]] = None
    const_base_force: Optional[float] = None
    const_max_force = float("inf")
    if isinstance(force_tuning, dict) and force_tuning.get("force_mid_n") is not None:
        raw_mid = float(force_tuning["force_mid_n"])
        if np.isfinite(raw_mid) and raw_mid > 0.0:
            raw_max = force_tuning.get("force_max_n")
            if isinstance(raw_max, (int, float)) and np.isfinite(raw_max) and float(raw_max) > 0.0:
                const_max_force = float(raw_max)
            const_base_force = min(const_max_force, 5.0 * raw_mid)

    if machine_type.startswith("hangprinter_"):
        # Point-fit information. With a fitted spool model, candidates are placed by
        # their modelled lengths but keep their commanded ones.
        estimation_dataset = fit["dataset_for_estimation"]
        placed = candidates if spool_params is None else sweep_configs_with_modeled_lengths(candidates, spool_params)
        commanded = {id(model): cfg for cfg, model in zip(candidates, placed)}
        obs_stretch = dataset_stretch(estimation_dataset)
        if force_contrast and obs_stretch is None:
            from autocal.stretch_varpro import dataset_stretch_basis
            obs_stretch = dataset_stretch_basis(estimation_dataset)
        if force_contrast:
            effective_scale_fn = None
            cand_scales: Sequence[float] = _FORCE_CONTRAST_SCALES
            plan_base_sensor_force: Optional[float | np.ndarray] = None
            plan_base_extra_force: Optional[float] = None
            plan_max_extra_force: Optional[float] = None
            measure_extra = has_extra_sensor_data(dataset)
            if needs_profile:
                prof_params = force_planning_parameters(dataset, allow_force_contrast=True)
                if prof_params is not None:
                    plan_base_sensor_force = float(prof_params["strand_tension_n"]) * np.asarray(prof_params["strands"], dtype=float)
                    plan_base_extra_force = prof_params.get("extra_sensor_force_n")
                    raw_max = (force_tuning or {}).get("force_max_n")
                    if isinstance(raw_max, (int, float)) and np.isfinite(raw_max) and float(raw_max) > 0.0:
                        plan_max_extra_force = float(raw_max)
                def _profile_effective_scale(cfg_placed: SweepConfig, req_scale: float) -> Optional[float]:
                    return effective_strand_tension_scale(
                        dataset, anchors, commanded[id(cfg_placed)], req_scale, allow_force_contrast=True,
                    )
                effective_scale_fn = _profile_effective_scale
            elif const_base_force is None:
                # Without usable force_mid_n we cannot override the constant sensor force, so only
                # score the nominal 1.0x action rather than an unexecutable high-force candidate.
                cand_scales = (1.0,)
            else:
                plan_base_sensor_force = const_base_force
                plan_base_extra_force = const_base_force if measure_extra else None
                plan_max_extra_force = const_max_force if np.isfinite(const_max_force) else None
                def _const_effective_scale(_cfg_placed: SweepConfig, req_scale: float) -> Optional[float]:
                    target_f = min(const_max_force, const_base_force * float(req_scale))
                    eff = target_f / const_base_force
                    return eff if np.isfinite(eff) and eff > 0.0 else None
                effective_scale_fn = _const_effective_scale

            ranked_raw = rank_candidates_force_aware(
                anchors,
                machine_type=machine_type,
                observed_lengths_mm=dataset_line_lengths(estimation_dataset)["lengths_mm"],
                candidates=placed,
                points_per_sweep=int(np.median([len(sweep["data_points"]) for sweep in estimation_dataset["sweeps"]])),
                sigma_mm=float(fit["pose_fit"]["robust_scale_mm"]),
                regularization=float(regularization),
                top_k=int(top_k),
                measure_extra_ranges=measure_extra,
                observed_stretch=obs_stretch,
                candidate_force_scales=cand_scales,
                base_sensor_force_n=plan_base_sensor_force,
                base_extra_force_n=plan_base_extra_force,
                max_extra_force_n=plan_max_extra_force,
                effective_scale_fn=effective_scale_fn,
            )
            ranked_with_force = [(score, commanded[id(cfg)], scale) for score, cfg, scale in ranked_raw]
            ranked = [(score, cfg) for score, cfg, _ in ranked_with_force]
            if ranked_with_force:
                chosen_force_scale = float(ranked_with_force[0][2])
        else:
            ranked = rank_candidates_pose_information(
                anchors,
                machine_type=machine_type,
                observed_lengths_mm=dataset_line_lengths(estimation_dataset)["lengths_mm"],
                candidates=placed,
                points_per_sweep=int(np.median([len(sweep["data_points"]) for sweep in estimation_dataset["sweeps"]])),
                sigma_mm=float(fit["pose_fit"]["robust_scale_mm"]),
                regularization=float(regularization),
                # Keep the complete ordering until a force-feasible candidate is
                # found; the highest-information band may all be infeasible.
                top_k=len(candidates) if needs_profile else int(top_k),
                measure_extra_ranges=has_extra_sensor_data(dataset),
                observed_stretch=obs_stretch,
            )
            ranked = [(score, commanded[id(cfg)]) for score, cfg in ranked]
    elif spool_params is not None:
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

    collection_fallback = False
    best_cfg = ranked[0][1] if ranked else None
    if best_cfg is None and candidates and machine_type not in ("hangprinter_3", "hangprinter_4", "hangprinter_5", "hangprinter_5_pulley_reduced"):
        # The fit can be too degenerate to score any candidate. The generated
        # candidates still satisfy machine constraints, so keep collection
        # moving with a moderate novel candidate from the current dataset.
        target_excursion = (
            0.5 * max_travel_mm
            if max_travel_mm is not None and np.isfinite(max_travel_mm) and max_travel_mm > 0.0
            else None
        )
        if target_excursion is None:
            best_cfg = candidates[0]
        else:
            best_cfg = min(
                candidates,
                key=lambda cfg: abs(
                    sum(abs(float(delta)) for delta in cfg.fixed_deltas_mm) - target_excursion
                ),
            )
        collection_fallback = True
    cfg_path = write_cfg or dataset_path.with_suffix(".active_sweep_cfg.txt")

    if has_spare_winches(dataset) and not has_extra_sensor_data(dataset) and not needs_profile and "--no-extra-sensors" not in collector_args_eff:
        collector_args_eff.append("--no-extra-sensors")
    collector_args_eff, _ = _inject_spool_collection_args(
        collector_args_eff,
        find_radii_mode=find_radii_mode,
        find_buildup_mode=find_buildup_mode,
        base_radii=base_radii,
        buildup_factor=buildup_factor,
    )
    modeled_buildup = np.asarray((fit.get("length_model") or {}).get("modeled_buildup_factor", []), dtype=float)
    if np.any(np.isfinite(modeled_buildup) & (modeled_buildup != 0)) and "--line-layering" not in collector_args_eff:
        collector_args_eff.append("--line-layering")
    if "--return-to-origin" not in collector_args_eff and "--returnToOrigin" not in collector_args_eff:
        collector_args_eff.append("--return-to-origin")

    cmd = None
    collection_error = None
    profile_path = None
    if best_cfg is not None:
        if needs_profile:
            profile_path = cfg_path.with_suffix(".sensor_force_profile.json")
            error = (profile_policy_error(dataset, allow_force_contrast=True)
                     if force_contrast else profile_policy_error(dataset))
            if error:
                collection_error = error
            elif not equal_strain:
                collection_error = "Cannot append constant-force measurements to profiled data; use a fresh dataset for --no-equal-strain"
            else:
                # Information gain alone does not establish force feasibility,
                # especially with the redundant line now carrying a measured load.
                best_cfg = None
                iter_ranked = ranked_with_force if ranked_with_force is not None else [(s, c, 1.0) for s, c in ranked]
                for index, (score, cfg, f_scale) in enumerate(iter_ranked):
                    try:
                        written = (
                            write_sensor_force_profile(
                                profile_path, dataset, anchors, [cfg],
                                strand_tension_scale=f_scale, allow_force_contrast=True,
                            )
                            if force_contrast else
                            write_sensor_force_profile(profile_path, dataset, anchors, [cfg])
                        )
                    except InfeasibleForceProfileError:
                        continue
                    if not written:
                        collection_error = "Cannot continue profiled collection without M666 W/N/Y/U and tuned forces"
                        break
                    best_cfg = cfg
                    chosen_force_scale = float(f_scale)
                    ranked = [(s, c) for s, c, _ in iter_ranked[index:index + int(top_k)]]
                    break
        elif force_contrast and abs(chosen_force_scale - 1.0) > 1e-6:
            # Constant-force sweep with force contrast: scale the collector's sensor force safely.
            if const_base_force is not None:
                target_f = min(const_max_force, const_base_force * chosen_force_scale)
                chosen_force_scale = float(target_f / const_base_force)
                collector_args_eff = _strip_cli_option(collector_args_eff, _SENSOR_FORCE_FLAGS)
                collector_args_eff.extend(["--sensor-force", f"{target_f:.4f}"])
            else:
                chosen_force_scale = 1.0
        if best_cfg is not None and collection_error is None:
            if profile_path is not None:
                collector_args_eff.extend(["--sensor-force-profile", str(profile_path)])
            _write_sweep_config_file(cfg_path, best_cfg)
            cmd = _suggested_collect_command(
                cfg_path, best_cfg, machine_type=machine_type,
                output_file=collector_output,
                completed_sweeps=sum(bool(sweep.get("data_points")) for sweep in dataset.get("sweeps", [])),
                extra_args=collector_args_eff,
            )

    return {**fit, "ranked": ranked, "best_cfg": best_cfg, "cfg_path": cfg_path,
            "chosen_force_scale": chosen_force_scale,
            "collection_fallback": collection_fallback,
            "collect_command": cmd, "force_tuning": force_tuning,
            "collection_error": collection_error,
            "exploration_bounds_mm": exploration_bounds,
            "force_args_applied": force_args_applied}


def plan_next_ellipse_sweep(
    dataset_path: Path,
    *,
    solve_restarts: int,
    solve_iterations: int,
    solve_optimizer: str,
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
    theta0_mode: str,
    line_width: float,
    sigma_floor_mm: Optional[float],
    candidate_count: int,
    delta_min: Optional[float],
    delta_max: Optional[float],
    regularization: float,
    min_fixed_delta_spacing_mm: float,
    write_cfg: Optional[Path],
    collector_output: Optional[Path],
    collector_args: Sequence[str],
    fit_home_offsets: bool = True,
    fit_force_ripple: bool = False,
    fit_stretch_varpro: bool = True,
    ellipse_reference_fit: bool = False,
    equal_strain: bool = True,
    force_contrast: bool = False,
    filter_schedule: Optional[Sequence[Any]] = None,
    objective_schedule: Optional[Sequence[Any]] = None,
    stage_artifact: Optional[Path] = None,
    candidate_deltas: Optional[List[float]] = None,
    fd_eps_mm: float = 1.0,
    exclude_existing: bool = True,
    existing_tol_mm: float = 10.0,
    top_k: int = 10,
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
