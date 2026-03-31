from __future__ import annotations

import atexit
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from autocal._autocal_common import *  # noqa: F401,F403
from autocal.planning_pass import plan_next_ellipse_sweep


def full_auto_loop(
    *,
    work_dataset: Optional[Path],
    machine_type: str,
    max_steps: int,
    stop_cost: Optional[float],
    stop_std_mm: Optional[float],
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
    collector_args: Sequence[str],
    sim: bool,
    keep_sim_alive: bool,
    hp_sim_reset: bool,
    sweep_points: Optional[int],
    output_with_explanations: bool,
    full_auto_runs: Optional[Sequence[str]],
    full_auto_log: Optional[Path],
    patience: int,
    verbose: bool,
    low_anchor_z: Optional[float] = None,
    filter_schedule: Optional[Sequence[Any]] = None,
    objective_schedule: Optional[Sequence[Any]] = None,
    scale_fix: Optional[Sequence[int]] = None,
    fit_structure: Optional[Sequence[int]] = None,
    no_collect: bool = False,
) -> int:
    if work_dataset is not None:
        dataset_path = Path(work_dataset)
    else:
        dataset_path = Path("autocal/data/default_dataset.json")
    work_path = dataset_path
    text_log_path = _unique_path(dataset_path.with_name(f"{dataset_path.stem}.full_auto.log"))
    text_log_path.parent.mkdir(parents=True, exist_ok=True)
    text_log_path.write_text("", encoding="utf-8")
    log_handle = text_log_path.open("a", encoding="utf-8")

    def _log_line(msg: str) -> None:
        log_handle.write(str(msg) + "\n")
        log_handle.flush()

    _log_line(f"; command: {shlex.join(sys.argv)}")

    def _log_console(msg: str) -> None:
        print(msg)
        _log_line(f"Wrote to console: {msg}")

    def _log_context():
        stack = contextlib.ExitStack()
        stack.enter_context(redirect_stdout(log_handle))
        stack.enter_context(redirect_stderr(log_handle))
        return stack

    def _emit_summary_and_send(
        best_plan: Dict[str, object],
        *,
        summary_meta: Optional[Dict[str, object]] = None,
    ) -> int:
        dataset_now = _load_json(work_path)
        sweep_ids: List[str] = []
        sweeps_now = dataset_now.get("sweeps")
        if isinstance(sweeps_now, list):
            for sweep in sweeps_now:
                if isinstance(sweep, dict) and sweep.get("id"):
                    sweep_ids.append(str(sweep["id"]))
        m669 = _m669_from_plan(best_plan)
        m666 = _m666_from_plan(best_plan)
        anchors = best_plan.get("anchors")
        anchor_str = ""
        if isinstance(anchors, np.ndarray):
            anchor_str = np.array2string(anchors, precision=2, separator=", ")
        elif anchors is not None:
            anchor_str = str(np.asarray(anchors))
        summary_raw_fit_score_ui, _, summary_score_basis = _plan_score_ui(best_plan)
        meta_for_summary = summary_meta if isinstance(summary_meta, dict) else best_meta
        summary_fit_score_ui = _full_auto_display_fit_score_ui(
            score_basis=meta_for_summary.get("score_basis", summary_score_basis),
            raw_fit_score_ui=_float_or_none(
                meta_for_summary.get("raw_fit_score_ui", summary_raw_fit_score_ui)
            ),
            history_rank_score=_float_or_none(meta_for_summary.get("history_rank_score")),
        )
        summary_fit_score_for_quality = (
            float(summary_fit_score_ui) if np.isfinite(summary_fit_score_ui) else None
        )
        quality_label = _solution_quality_label(summary_fit_score_for_quality)
        _log_console("")
        _log_console("== Calibration summary ==")
        _log_console(f"Found parameters of {quality_label} quality")
        _log_console(f"Fit quality score (lower is better): {_fmt_float(summary_fit_score_for_quality)}")
        if m669:
            _log_console(f"Anchors (M669): {m669}")
        elif anchor_str:
            _log_console(f"Anchors: {anchor_str}")
        if m666:
            _log_console(f"Spools (M666): {m666}")
        if has_variants:
            best_flags = str(meta_for_summary.get("flags", "")).strip()
            best_run = str(meta_for_summary.get("run_id", "")).strip()
            label = best_flags or best_run or "default"
            _log_console(f"Selected variant/flag setup: {label}")
        _log_console(_solution_quality_message(summary_fit_score_for_quality))

        skip_sim_send = bool(sim and no_collect and not server_explicit)
        if m669:
            if skip_sim_send:
                _log_console("; --sim + --no-collect set; skipping M669 send.")
            else:
                _log_console(f"Sending {m669} to {rrf_server}")
                try:
                    reply = _send_rrf_gcode(rrf_server, m669)
                except Exception as exc:
                    _log_console(f"; failed to send M669: {exc}")
                    return _finalize(1)
                reply = reply.strip()
                if reply:
                    _log_line(f"; M669 reply: {reply}")
        else:
            _log_console("Sending M669 skipped (no command available)")
        return _finalize(0)

    find_radii_mode = _normalize_spool_find_mode(find_radii)
    find_buildup_mode = _normalize_spool_find_mode(find_buildup_factor)

    _log_console(f"Writing additional info to log: {text_log_path}")
    if dataset_path.exists():
        with _log_context():
            machine_type = _require_machine_type(
                _load_json(dataset_path),
                expected=machine_type,
                context=str(dataset_path),
                mismatch="warn",
            )

    user_no_spawn = _arg_has_flag(collector_args, "--no-spawn-rrf-simulator")
    collector_args_eff = _apply_simulation_defaults(collector_args, sim=sim)
    collector_args_eff, _ = _inject_spool_collection_args(
        collector_args_eff,
        find_radii_mode=find_radii_mode,
        find_buildup_mode=find_buildup_mode,
        base_radii=base_radii,
        buildup_factor=buildup_factor,
    )
    hp_sim_reset_eff = _effective_hp_sim_reset(
        sim=bool(sim),
        hp_sim_reset=bool(hp_sim_reset),
        find_radii_mode=find_radii_mode,
        find_buildup_mode=find_buildup_mode,
    )
    if bool(sim) and hp_sim_reset_eff and not bool(hp_sim_reset):
        _log_line("; auto-enabling --hp-sim-reset for spool search in simulation")
    sweep_points_value = None
    raw_sweep_points = _arg_value(collector_args_eff, "--sweepPoints", "--sweep-points")
    if raw_sweep_points is not None:
        try:
            parsed = int(raw_sweep_points)
        except ValueError:
            parsed = None
        if parsed is not None and parsed > 0:
            sweep_points_value = parsed
    if sweep_points is not None:
        try:
            parsed = int(sweep_points)
        except (TypeError, ValueError):
            parsed = None
        if parsed is not None and parsed > 0:
            sweep_points_value = parsed
            if raw_sweep_points is None:
                collector_args_eff.extend(["--sweepPoints", str(parsed)])
    reset_pending = bool(sim and hp_sim_reset_eff)
    rrf_server, server_explicit, port = _resolve_rrf_target(collector_args)
    sim_process: Optional[subprocess.Popen] = None
    cleanup_registered = False
    sim_config = _resolve_sim_config(
        machine_type=machine_type,
        find_radii_mode=find_radii_mode,
        find_buildup_mode=find_buildup_mode,
    )

    def _cleanup() -> None:
        if sim_process and not keep_sim_alive:
            _stop_process(sim_process)
        try:
            log_handle.close()
        except Exception:
            pass

    if sim and not no_collect and not server_explicit and not user_no_spawn:
        atexit.register(_cleanup)
        cleanup_registered = True
        target_port = port or DEFAULT_RRF_PORT
        _log_line(f"; starting rrf_simulator at http://localhost:{target_port} (config: {sim_config})")
        sim_process = _start_rrf_simulator(target_port, sim_config=sim_config)
        _wait_for_rrf_server(f"http://localhost:{target_port}")

    def _finalize(code: int) -> int:
        nonlocal cleanup_registered
        if cleanup_registered:
            try:
                atexit.unregister(_cleanup)
            except Exception:
                pass
            cleanup_registered = False
        _cleanup()
        return code

    if not dataset_path.exists():
        dataset_path.parent.mkdir(parents=True, exist_ok=True)
        bootstrap_cfg = dataset_path.with_suffix(".bootstrap_cfg.txt")
        bootstrap_sweep_count = _write_bootstrap_sweep_config(
            bootstrap_cfg,
            machine_type=str(machine_type),
        )

        def _strip_conflicts(argv: Sequence[str]) -> List[str]:
            skip_with_value = {
                "--output-file",
                "--output",
                "--out",
                "--observability-file",
                "--obs-file",
                "--machineType",
                "--machine-type",
                "--sweepMethod",
                "--sweep-method",
                "--sweep-config-file",
                "--sweep-config",
                "--sweepFile",
                "--fixed-targets",
                "--fixedTargets",
                "--fixed-target",
                "--max-travel-mm",
                "--max-travel",
            }
            out: List[str] = []
            i = 0
            while i < len(argv):
                arg = str(argv[i])
                if arg in skip_with_value:
                    i += 2
                    continue
                out.append(arg)
                i += 1
            return out

        argv_eff = _strip_conflicts(list(collector_args_eff))
        if reset_pending and not _arg_has_flag(argv_eff, "--hp-sim-reset"):
            argv_eff.append("--hp-sim-reset")
        if "--return-to-origin" not in argv_eff and "--returnToOrigin" not in argv_eff:
            argv_eff.append("--return-to-origin")

        cmd = [
            "node",
            "autocal/control/cli/collect_sweep_data.mjs",
            "--machineType",
            str(machine_type),
            "--sweep-config-file",
            str(bootstrap_cfg),
            "--output-file",
            str(dataset_path),
            *argv_eff,
        ]
        _log_line(f"; bootstrapping dataset ({bootstrap_sweep_count} sweeps, auto size-tune):")
        _log_line(";   " + " ".join(cmd))
        with _log_context():
            subprocess.run(cmd, check=True, stdout=log_handle, stderr=log_handle)
        reset_pending = False
        _log_line(f"; bootstrap dataset written to {dataset_path}")

    replay_mode = False
    replay_sweeps: List[dict] = []
    replay_index = 0
    replay_work_path: Optional[Path] = None
    if dataset_path.exists():
        dataset_full = _load_json(dataset_path)
        sweeps_full = dataset_full.get("sweeps")
        if isinstance(sweeps_full, list) and len(sweeps_full) > 3:
            replay_mode = True
            replay_work_path = dataset_path.with_suffix(".replay_tmp.json")
            work_path = replay_work_path
            initial = [s for s in sweeps_full[:3] if isinstance(s, dict)]
            replay_sweeps = [s for s in sweeps_full[3:] if isinstance(s, dict)]
            dataset_work = dict(dataset_full)
            dataset_work["timestamp"] = datetime.now().isoformat()
            dataset_work["sweeps"] = initial
            _renumber_sweeps(initial)
            _write_json(work_path, dataset_work)
            _log_line(
                f"; full-auto replay: starting from 3 sweeps, "
                f"replaying {len(replay_sweeps)} additional sweeps"
            )
            _log_line(
                f"; full-auto replay: using {work_path} "
                f"(original {dataset_path} left untouched)"
            )

    runs = _build_full_auto_runs(full_auto_runs)
    log_path = Path(full_auto_log) if full_auto_log is not None else _unique_path(
        dataset_path.with_name(f"{dataset_path.stem}.full_auto_log.jsonl")
    )
    stop_file = _full_auto_stop_path(dataset_path)
    _append_jsonl(
        log_path,
        {
            "timestamp": datetime.now().isoformat(),
            "event": "start",
            "dataset": str(dataset_path),
            "runs": runs,
            "replay_mode": replay_mode,
            "replay_remaining": len(replay_sweeps),
        },
    )
    _log_line(f"; full-auto log: {log_path}")
    _log_console(
        f"; full-auto stop: press Ctrl-C to accept best-so-far, "
        f"or create {stop_file} to request stop (cross-platform)"
    )

    base_solver = {
        "solve_restarts": int(solve_restarts),
        "solve_iterations": int(solve_iterations),
        "solve_optimizer": str(solve_optimizer),
        "optimizer_mode": _normalize_optimizer_mode(os.environ.get("AUTOCAL_OPTIMIZER_MODE", "fast")),
        "residual_threshold": float(residual_threshold),
        "spring_k_multiplier": float(spring_k_multiplier),
        "use_flex": bool(use_flex),
        "pointwise_residual_mode": str(pointwise_residual_mode),
        "pointwise_filtering": bool(pointwise_filtering),
        "pointwise_global_mad": bool(pointwise_global_mad),
        "sweep_wise_filtering": bool(sweep_wise_filtering),
        "sweep_metric": str(sweep_metric),
        "use_noise_mean": bool(use_noise_mean),
        "sigma_source": str(sigma_source),
        "robust_debug": bool(robust_debug),
        "generate_report": bool(generate_report),
        "find_radii": str(find_radii_mode),
        "find_buildup_factor": str(find_buildup_mode),
        "base_radii": (None if base_radii is None else [float(v) for v in base_radii]),
        "buildup_factor": (None if buildup_factor is None else float(buildup_factor)),
        "r0_bounds": (None if r0_bounds is None else [float(r0_bounds[0]), float(r0_bounds[1])]),
        "b_bounds": (None if b_bounds is None else [float(b_bounds[0]), float(b_bounds[1])]),
        "r0_prior_sigma_mm": (
            None
            if r0_prior_sigma_mm is None
            else float(r0_prior_sigma_mm)
        ),
        "b_prior_sigma": (None if b_prior_sigma is None else float(b_prior_sigma)),
        "spool_outer_iters": int(spool_outer_iters),
        "spool_inner_iters": int(spool_inner_iters),
        "theta0_mode": str(_normalize_theta0_mode(theta0_mode)),
        "filter_schedule": [
            str(v) for v in _parse_filter_schedule(filter_schedule, label="filter_schedule")
        ],
        "objective_schedule": [
            int(v) for v in _parse_objective_schedule(objective_schedule, label="objective_schedule")
        ],
        "scale_fix": [int(v) for v in _parse_scale_fix_levels(scale_fix)],
        "fit_structure": [int(v) for v in _parse_fit_structure_levels(fit_structure)],
        "line_width": float(line_width),
        "sigma_floor_mm": (None if sigma_floor_mm is None else float(sigma_floor_mm)),
        "sigma_used_mm": (None if sigma_used_mm is None else float(sigma_used_mm)),
        "low_anchor_z": (None if low_anchor_z is None else float(low_anchor_z)),
    }

    best_cost = float("inf")
    best_score_ui = float("inf")
    best_rank_score = float("inf")
    best_plan: Optional[Dict[str, object]] = None
    best_meta: Dict[str, object] = {}
    history_candidates: List[Dict[str, object]] = []
    history_total_iterations = 0
    no_improve = 0
    min_delta = float(DEFAULT_FULL_AUTO_MIN_DELTA)
    patience_limit = max(1, int(patience))
    has_variants = bool(full_auto_runs)
    def _select_history_summary_candidate(
        *,
        reason: str,
        fallback_plan: Optional[Dict[str, object]] = None,
        fallback_meta: Optional[Dict[str, object]] = None,
    ) -> Tuple[Optional[Dict[str, object]], Optional[Dict[str, object]]]:
        if history_candidates:
            scored: List[Tuple[Tuple[float, float, float, float, float], Dict[str, object], Dict[str, Optional[float]]]] = []
            for candidate in history_candidates:
                candidate_rank = _float_or_none(candidate.get("rank_score"))
                selection_score, selection_info = _full_auto_history_selection_score(
                    candidate_rank,
                    iteration_index=int(candidate.get("iteration", 1)),
                    coverage_adjust=_float_or_none(candidate.get("rank_coverage_adjust")),
                )
                rel_std = _float_or_none(candidate.get("rel_std"))
                cost = _float_or_none(candidate.get("cost"))
                sort_key = (
                    float(selection_score),
                    float(candidate_rank) if candidate_rank is not None else float("inf"),
                    float(rel_std) if rel_std is not None else float("inf"),
                    float(cost) if cost is not None else float("inf"),
                    -float(candidate.get("iteration", 0)),
                )
                scored.append((sort_key, candidate, selection_info))

            scored.sort(key=lambda item: item[0])
            chosen = scored[0][1]
            chosen_info = scored[0][2]
            _log_line(
                f"; history_select: reason={reason} "
                f"candidates={len(scored)} total_iterations={history_total_iterations}"
            )
            for _sort_key, candidate, info in scored[: min(5, len(scored))]:
                _log_line(
                    f"; history_select: "
                    f"iter={candidate.get('iteration')} "
                    f"run={candidate.get('run_id')} "
                    f"rank={_fmt_float(_float_or_none(candidate.get('rank_score')))} "
                    f"iteration_adjust={_fmt_float(info.get('iteration_adjust'))} "
                    f"coverage_adjust={_fmt_float(info.get('coverage_adjust'))} "
                    f"selection_score={_fmt_float(info.get('selection_score'))}"
                )
            summary_meta = dict(chosen.get("summary_meta") or {})
            summary_meta.update(
                {
                    "history_iteration_adjust": chosen_info.get("iteration_adjust"),
                    "history_coverage_adjust": chosen_info.get("coverage_adjust"),
                    "history_selection_score": chosen_info.get("selection_score"),
                }
            )
            return chosen.get("plan"), summary_meta

        if best_plan is not None:
            return best_plan, dict(best_meta)
        if fallback_plan is not None:
            return fallback_plan, dict(fallback_meta or {})
        return None, None

    def _current_history_rank_position(*, iteration: int) -> Optional[int]:
        if not history_candidates:
            return None
        scored: List[Tuple[Tuple[float, float, float, float, float], Dict[str, object]]] = []
        for candidate in history_candidates:
            candidate_rank = _float_or_none(candidate.get("rank_score"))
            selection_score, _selection_info = _full_auto_history_selection_score(
                candidate_rank,
                iteration_index=int(candidate.get("iteration", 1)),
                coverage_adjust=_float_or_none(candidate.get("rank_coverage_adjust")),
            )
            rel_std = _float_or_none(candidate.get("rel_std"))
            cost = _float_or_none(candidate.get("cost"))
            sort_key = (
                float(selection_score),
                float(candidate_rank) if candidate_rank is not None else float("inf"),
                float(rel_std) if rel_std is not None else float("inf"),
                float(cost) if cost is not None else float("inf"),
                -float(candidate.get("iteration", 0)),
            )
            scored.append((sort_key, candidate))
        scored.sort(key=lambda item: item[0])
        for idx, (_sort_key, candidate) in enumerate(scored, start=1):
            if int(candidate.get("iteration", -1)) == int(iteration):
                return idx
        return None

    def _accept_best(reason: str) -> int:
        summary_plan, summary_meta = _select_history_summary_candidate(reason=reason)
        if summary_plan is None:
            _log_console(f"; full-auto: stop requested ({reason}) but no best plan available; stopping.")
            _log_console(_solution_quality_message(None))
            return _finalize(2)
        _log_console(f"; full-auto: stop requested ({reason}); accepting best-so-far.")
        return _emit_summary_and_send(summary_plan, summary_meta=summary_meta)

    def _compute_run_metrics(plan: Dict[str, object]) -> Dict[str, object]:
        primary_cost = _plan_primary_cost(plan)
        raw_fit_score_ui, rank_score, score_basis = _plan_score_ui(plan)
        max_std_mm, rel_std, cov_ok = _plan_covariance_summary(plan)
        warnings = list(_plan_data_quality_warnings(plan))
        noise_metrics = _plan_noise_metrics(plan)
        rank_coverage_adjust, rank_coverage_info = _rank_coverage_adjustment_from_noise_metrics(
            noise_metrics
        )
        underconstrained_penalty = _plan_hits_underconstrained_penalty(plan, primary_cost)
        if underconstrained_penalty and "underconstrained_penalty" not in warnings:
            warnings.append("underconstrained_penalty")
        valid = bool(np.isfinite(primary_cost) and np.isfinite(rank_score) and cov_ok)
        return {
            "primary_cost": primary_cost,
            "raw_fit_score_ui": raw_fit_score_ui,
            "score_ui": raw_fit_score_ui,
            "rank_score": rank_score,
            "score_basis": score_basis,
            "cost_noise_normalized": plan.get("cost_noise_normalized"),
            "chi2_red": noise_metrics.get("chi2_red") if isinstance(noise_metrics, dict) else None,
            "J": noise_metrics.get("J") if isinstance(noise_metrics, dict) else None,
            "info_rank": plan.get("info_rank"),
            "info_rank_deficient": plan.get("info_rank_deficient"),
            "max_std_mm": max_std_mm,
            "rel_std": rel_std,
            "covariance_ok": cov_ok,
            "underconstrained_penalty": underconstrained_penalty,
            "rank_coverage_adjust": rank_coverage_adjust,
            "rank_effective_obs": rank_coverage_info.get("effective_obs"),
            "rank_total_obs": rank_coverage_info.get("total_obs"),
            "rank_filtered_ratio": rank_coverage_info.get("filtered_ratio"),
            "rank_obs_bonus": rank_coverage_info.get("obs_bonus"),
            "rank_filtered_penalty": rank_coverage_info.get("filtered_penalty"),
            "warnings": warnings,
            "valid": valid,
            "line_model_prefit": (
                (((plan.get("length_model") or {}).get("radii_fit") or {}).get("prefit"))
                if isinstance(plan, dict)
                else None
            ),
        }

    def _log_run_metrics(log_prefix: str, plan: Dict[str, object], metrics: Dict[str, object]) -> None:
        cost_raw = plan.get("cost_raw")
        cost_norm = plan.get("cost_noise_normalized", plan.get("cost"))
        _log_line(
            f"{log_prefix}: cost_raw={_fmt_float(cost_raw)} "
            f"cost_noise_normalized={_fmt_float(cost_norm)} J={_fmt_float(metrics.get('J'))} "
            f"chi2_red={_fmt_float(metrics.get('chi2_red'))} "
            f"raw_fit_score_ui={_fmt_float(metrics.get('raw_fit_score_ui'))} "
            f"score_basis={metrics.get('score_basis')} "
            f"rank_coverage_adjust={_fmt_float(metrics.get('rank_coverage_adjust'))} "
            f"effective_obs={_fmt_float(metrics.get('rank_effective_obs'), fmt='.0f')} "
            f"filtered_ratio={_fmt_float(metrics.get('rank_filtered_ratio'))}"
        )

    def _execute_plan_run(
        *,
        run_id: str,
        run_flags: str,
        overrides: Dict[str, object],
        settings: Dict[str, object],
        collector_output: Path,
        log_prefix: str,
        path_tag: Optional[str] = None,
        initial_guess: Optional[np.ndarray] = None,
        initial_radii_mm: Optional[np.ndarray] = None,
        initial_buildup_factor: Optional[np.ndarray] = None,
    ) -> Dict[str, object]:
        _log_line(f"{log_prefix}: flags='{run_flags}'")

        cfg_run_id = str(path_tag or run_id)
        cfg_path = _full_auto_cfg_path(work_path, cfg_run_id)
        residuals_csv_run = None
        if residuals_csv is not None:
            res_base = Path(residuals_csv)
            if len(runs) > 1 or path_tag is not None:
                residual_suffix = str(path_tag or run_id)
                residuals_csv_run = res_base.with_name(
                    f"{res_base.stem}.{residual_suffix}{res_base.suffix}"
                )
            else:
                residuals_csv_run = res_base

        with _log_context():
            plan = plan_next_ellipse_sweep(
                work_path,
                solve_restarts=int(settings["solve_restarts"]),
                solve_iterations=int(settings["solve_iterations"]),
                solve_optimizer=str(settings["solve_optimizer"]),
                residual_threshold=float(settings["residual_threshold"]),
                spring_k_multiplier=float(settings["spring_k_multiplier"]),
                use_flex=bool(settings["use_flex"]),
                pointwise_residual_mode=str(settings["pointwise_residual_mode"]),
                pointwise_filtering=bool(settings["pointwise_filtering"]),
                pointwise_global_mad=bool(settings["pointwise_global_mad"]),
                sweep_wise_filtering=bool(settings["sweep_wise_filtering"]),
                sweep_metric=str(settings["sweep_metric"]),
                use_noise_mean=bool(settings["use_noise_mean"]),
                sigma_source=str(settings["sigma_source"]),
                robust_debug=bool(settings["robust_debug"]),
                residuals_csv=residuals_csv_run,
                generate_report=bool(settings["generate_report"]),
                find_radii=str(settings.get("find_radii", "off")),
                find_buildup_factor=str(settings.get("find_buildup_factor", "off")),
                base_radii=(
                    [float(v) for v in settings["base_radii"]]
                    if isinstance(settings.get("base_radii"), list)
                    else None
                ),
                buildup_factor=(
                    float(settings["buildup_factor"])
                    if settings.get("buildup_factor") is not None
                    else None
                ),
                r0_bounds=settings.get("r0_bounds"),
                b_bounds=settings.get("b_bounds"),
                r0_prior_sigma_mm=(
                    float(settings["r0_prior_sigma_mm"])
                    if settings.get("r0_prior_sigma_mm") is not None
                    else None
                ),
                b_prior_sigma=(
                    float(settings["b_prior_sigma"])
                    if settings.get("b_prior_sigma") is not None
                    else None
                ),
                spool_outer_iters=int(settings.get("spool_outer_iters", 3)),
                spool_inner_iters=int(settings.get("spool_inner_iters", 30)),
                theta0_mode=str(settings.get("theta0_mode", "zero")),
                filter_schedule=settings.get("filter_schedule"),
                objective_schedule=settings.get("objective_schedule"),
                line_width=float(settings.get("line_width", DEFAULT_LAYER_LINE_WIDTH_MM)),
                sigma_floor_mm=(
                    None
                    if settings.get("sigma_floor_mm") is None
                    else float(settings.get("sigma_floor_mm"))
                ),
                sigma_used_mm=(
                    None
                    if settings.get("sigma_used_mm") is None
                    else float(settings.get("sigma_used_mm"))
                ),
                low_anchor_z=(
                    None
                    if settings.get("low_anchor_z") is None
                    else float(settings.get("low_anchor_z"))
                ),
                candidate_deltas=candidate_deltas,
                candidate_count=int(candidate_count),
                delta_min=delta_min,
                delta_max=delta_max,
                fd_eps_mm=float(fd_eps_mm),
                regularization=float(regularization),
                exclude_existing=bool(exclude_existing),
                existing_tol_mm=float(existing_tol_mm),
                min_fixed_delta_spacing_mm=float(min_fixed_delta_spacing_mm),
                top_k=int(top_k),
                write_cfg=cfg_path,
                collector_output=collector_output,
                collector_args=collector_args_eff,
                scale_fix=settings.get("scale_fix"),
                fit_structure=settings.get("fit_structure"),
                initial_guess=initial_guess,
                initial_radii_mm=initial_radii_mm,
                initial_buildup_factor=initial_buildup_factor,
            )

        metrics = _compute_run_metrics(plan)
        _log_run_metrics(log_prefix, plan, metrics)
        return {
            "id": run_id,
            "flags": run_flags,
            "overrides": dict(overrides),
            "settings": dict(settings),
            "plan": plan,
            "metrics": metrics,
        }

    def _warm_start_seeds_from_plan(plan: Dict[str, object]) -> Dict[str, np.ndarray]:
        def _coerce_matrix(value: object) -> Optional[np.ndarray]:
            try:
                arr = np.asarray(value, dtype=float)
            except (TypeError, ValueError):
                return None
            if arr.ndim != 2 or arr.size <= 0 or not np.all(np.isfinite(arr)):
                return None
            return arr

        def _coerce_vector(
            value: object,
            *,
            size: Optional[int] = None,
            positive: bool = False,
        ) -> Optional[np.ndarray]:
            try:
                arr = np.asarray(value, dtype=float).reshape(-1)
            except (TypeError, ValueError):
                return None
            if arr.size <= 0 or not np.all(np.isfinite(arr)):
                return None
            if arr.size == 1 and size is not None and size > 1:
                arr = np.full(size, float(arr[0]), dtype=float)
            if size is not None and arr.size != size:
                return None
            if positive and np.any(arr <= 0.0):
                return None
            return arr

        seeds: Dict[str, np.ndarray] = {}
        anchors = _coerce_matrix(plan.get("anchors"))
        if anchors is not None:
            seeds["initial_guess"] = anchors
        anchor_count = int(anchors.shape[0]) if anchors is not None else None

        length_model = plan.get("length_model")
        if not isinstance(length_model, dict):
            return seeds

        radii_seed = _coerce_vector(
            length_model.get("effective_radii_mm", length_model.get("modeled_radii_mm")),
            size=anchor_count,
            positive=True,
        )
        if radii_seed is not None:
            seeds["initial_radii_mm"] = radii_seed

        buildup_seed = _coerce_vector(
            length_model.get("modeled_buildup_factor"),
            size=anchor_count,
        )
        if buildup_seed is None and length_model.get("buildup_factor_k") is not None:
            buildup_seed = _coerce_vector(
                length_model.get("buildup_factor_k"),
                size=anchor_count,
            )
        if buildup_seed is not None:
            seeds["initial_buildup_factor"] = buildup_seed
        return seeds

    def _single_objective_schedule(value: object) -> List[int]:
        if isinstance(value, (list, tuple)) and value:
            try:
                return [int(value[-1])]
            except (TypeError, ValueError):
                pass
        return [1]

    def _build_underconstrained_recovery_attempts(
        settings: Dict[str, object]
    ) -> List[Tuple[str, Dict[str, object]]]:
        attempts: List[Tuple[str, Dict[str, object]]] = []
        current = dict(settings)
        if bool(current.get("sweep_wise_filtering")):
            current = dict(current)
            current["sweep_wise_filtering"] = False
            attempts.append(("restore_sweeps", dict(current)))
        if bool(current.get("pointwise_filtering")):
            current = dict(current)
            current["pointwise_filtering"] = False
            current["pointwise_global_mad"] = False
            attempts.append(("restore_points", dict(current)))

        out: List[Tuple[str, Dict[str, object]]] = []
        for label, attempt_settings in attempts:
            settings_eff = dict(attempt_settings)
            if (
                not bool(settings_eff.get("pointwise_filtering"))
                and not bool(settings_eff.get("sweep_wise_filtering"))
            ):
                settings_eff["filter_schedule"] = ["warmup"]
                settings_eff["objective_schedule"] = _single_objective_schedule(
                    settings_eff.get("objective_schedule")
                )
            out.append((label, settings_eff))
        return out

    def _attempt_underconstrained_recovery(
        selected: Dict[str, object],
        *,
        collector_output: Path,
    ) -> Tuple[Optional[Dict[str, object]], Optional[Dict[str, object]]]:
        settings = dict(selected.get("settings") or {})
        attempts = _build_underconstrained_recovery_attempts(settings)
        if not attempts:
            return None, None

        run_id = str(selected.get("id", "run"))
        run_flags = str(selected.get("flags", ""))
        overrides = dict(selected.get("overrides") or {})
        _log_console(
            "; selected run underconstrained; trying to restore sweeps/points from current data."
        )

        warm_start = _warm_start_seeds_from_plan(selected["plan"])
        recovery_info: Dict[str, object] = {
            "attempts_total": int(len(attempts)),
            "attempt_labels": [label for label, _settings in attempts],
            "success": False,
        }
        for idx, (label, settings_try) in enumerate(attempts, start=1):
            _log_line(
                f"; underconstrained_recovery: run={run_id} "
                f"attempt={idx}/{len(attempts)} mode={label}"
            )
            result = _execute_plan_run(
                run_id=run_id,
                run_flags=run_flags,
                overrides=overrides,
                settings=settings_try,
                collector_output=collector_output,
                log_prefix=f"; full-auto recovery {run_id} [{label}]",
                path_tag=f"{run_id}.{label}",
                initial_guess=warm_start.get("initial_guess"),
                initial_radii_mm=warm_start.get("initial_radii_mm"),
                initial_buildup_factor=warm_start.get("initial_buildup_factor"),
            )
            recovery_info.update(
                {
                    "attempt": str(label),
                    "attempt_index": int(idx),
                }
            )
            result["recovery"] = dict(recovery_info)
            if bool(result["metrics"].get("valid")) and not bool(
                result["metrics"].get("underconstrained_penalty", False)
            ):
                recovery_info["success"] = True
                result["recovery"] = dict(recovery_info)
                _log_console(
                    "; underconstrained recovery succeeded; continuing with the repaired constrained estimate."
                )
                return result, dict(recovery_info)
            warm_start = _warm_start_seeds_from_plan(result["plan"])

        _log_console(
            "; underconstrained recovery exhausted current data without re-establishing constraints."
        )
        return None, dict(recovery_info)

    def _stop_file_requested() -> bool:
        try:
            return stop_file.exists()
        except OSError:
            return False

    try:
        for step in range(1, max(1, int(max_steps)) + 1):
            if _stop_file_requested():
                return _accept_best(f"stop-file {stop_file}")
            _log_line(f"\n; === full-auto iteration {step}/{max_steps} dataset={work_path} ===")
            _log_console("")
            _log_console(f"; === full-auto iteration {step}/{max_steps} dataset={work_path} ===")
            collector_output = work_path.with_name(f"{work_path.stem}.new_{step:03d}.json")
            run_results: List[Dict[str, object]] = []

            for run in runs:
                if _stop_file_requested():
                    return _accept_best(f"stop-file {stop_file}")
                run_id = str(run.get("id", "run"))
                run_flags = str(run.get("flags", "")).strip()
                overrides = run.get("overrides") or {}
                if not isinstance(overrides, dict):
                    overrides = {}
                settings = dict(base_solver)
                settings.update(overrides)
                settings["optimizer_mode"] = _normalize_optimizer_mode(settings.get("optimizer_mode"))
                _apply_optimizer_mode_env(str(settings["optimizer_mode"]))
                if isinstance(settings.get("base_radii"), str):
                    settings["base_radii"] = _parse_csv_floats(str(settings["base_radii"]))
                settings["find_radii"] = _normalize_spool_find_mode(settings.get("find_radii"))
                settings["find_buildup_factor"] = _normalize_spool_find_mode(
                    settings.get("find_buildup_factor")
                )
                settings["theta0_mode"] = _normalize_theta0_mode(settings.get("theta0_mode"))
                settings["scale_fix"] = [int(v) for v in _parse_scale_fix_levels(settings.get("scale_fix"))]
                settings["fit_structure"] = [
                    int(v) for v in _parse_fit_structure_levels(settings.get("fit_structure"))
                ]

                raw_r0_bounds = settings.get("r0_bounds")
                if isinstance(raw_r0_bounds, str):
                    settings["r0_bounds"] = _parse_min_max_bounds(raw_r0_bounds, label="--r0-bounds")
                elif isinstance(raw_r0_bounds, (list, tuple)) and len(raw_r0_bounds) >= 2:
                    settings["r0_bounds"] = (float(raw_r0_bounds[0]), float(raw_r0_bounds[1]))
                else:
                    settings["r0_bounds"] = None

                raw_b_bounds = settings.get("b_bounds")
                if isinstance(raw_b_bounds, str):
                    settings["b_bounds"] = _parse_min_max_bounds(raw_b_bounds, label="--b-bounds")
                elif isinstance(raw_b_bounds, (list, tuple)) and len(raw_b_bounds) >= 2:
                    settings["b_bounds"] = (float(raw_b_bounds[0]), float(raw_b_bounds[1]))
                else:
                    settings["b_bounds"] = None

                if settings.get("r0_prior_sigma_mm") is not None:
                    settings["r0_prior_sigma_mm"] = float(settings["r0_prior_sigma_mm"])
                if settings.get("b_prior_sigma") is not None:
                    settings["b_prior_sigma"] = float(settings["b_prior_sigma"])
                settings["spool_outer_iters"] = int(settings.get("spool_outer_iters", 3))
                settings["spool_inner_iters"] = int(settings.get("spool_inner_iters", 30))
                settings["filter_schedule"] = [
                    str(v)
                    for v in _parse_filter_schedule(
                        settings.get("filter_schedule"),
                        label="filter_schedule",
                    )
                ]
                settings["objective_schedule"] = [
                    int(v)
                    for v in _parse_objective_schedule(
                        settings.get("objective_schedule"),
                        label="objective_schedule",
                    )
                ]
                settings["line_width"] = float(settings.get("line_width", DEFAULT_LAYER_LINE_WIDTH_MM))
                if not np.isfinite(settings["line_width"]) or settings["line_width"] < 0.0:
                    settings["line_width"] = float(DEFAULT_LAYER_LINE_WIDTH_MM)
                if settings.get("sigma_floor_mm") is not None:
                    settings["sigma_floor_mm"] = float(settings["sigma_floor_mm"])
                    if not np.isfinite(settings["sigma_floor_mm"]) or settings["sigma_floor_mm"] <= 0.0:
                        settings["sigma_floor_mm"] = None
                if settings.get("sigma_used_mm") is not None:
                    settings["sigma_used_mm"] = float(settings["sigma_used_mm"])
                    if not np.isfinite(settings["sigma_used_mm"]) or settings["sigma_used_mm"] <= 0.0:
                        settings["sigma_used_mm"] = None
                if settings.get("low_anchor_z") is not None:
                    settings["low_anchor_z"] = float(settings["low_anchor_z"])
                    if not np.isfinite(settings["low_anchor_z"]):
                        settings["low_anchor_z"] = None
                run_results.append(
                    _execute_plan_run(
                        run_id=run_id,
                        run_flags=run_flags,
                        overrides=overrides,
                        settings=settings,
                        collector_output=collector_output,
                        log_prefix=f"; full-auto run {run_id}",
                    )
                )

            valid_runs = [r for r in run_results if r["metrics"]["valid"]]
            if not valid_runs:
                _append_jsonl(
                    log_path,
                    {
                        "timestamp": datetime.now().isoformat(),
                        "event": "stop",
                        "iteration": step,
                        "dataset": str(dataset_path),
                        "reason": "no_valid_runs",
                        "runs": [
                            {
                                "id": r["id"],
                                "flags": r["flags"],
                                "metrics": r["metrics"],
                            }
                            for r in run_results
                        ],
                    },
                )
                _log_line("; full-auto: no valid calibration runs (non-finite cost or covariance).")
                _log_console("; full-auto: no valid calibration runs (non-finite cost or covariance).")
                _log_console(_solution_quality_message(best_score_ui if np.isfinite(best_score_ui) else None))
                return _finalize(2)

            def _sort_key(entry: Dict[str, object]) -> Tuple[float, float, float, float, str]:
                metrics = entry["metrics"]
                underconstrained = bool(metrics.get("underconstrained_penalty", False))
                score = float(metrics.get("rank_score", float("inf")))
                cost = float(metrics.get("primary_cost", float("inf")))
                rel = metrics.get("rel_std")
                rel_val = (
                    float(rel)
                    if isinstance(rel, (int, float)) and np.isfinite(rel)
                    else float("inf")
                )
                return (1.0 if underconstrained else 0.0), score, rel_val, cost, str(entry.get("id", ""))

            selected = sorted(valid_runs, key=_sort_key)[0]
            recovery_info = None
            if bool(selected["metrics"].get("underconstrained_penalty", False)):
                recovered, recovery_info = _attempt_underconstrained_recovery(
                    selected,
                    collector_output=collector_output,
                )
                if recovered is not None:
                    selected = recovered
            plan = selected["plan"]
            metrics = selected["metrics"]
            selected_id = str(selected.get("id", "run"))
            selected_flags = str(selected.get("flags", "")).strip()
            selected_cost = float(metrics.get("primary_cost", float("nan")))
            selected_raw_fit_score_ui = float(
                metrics.get("raw_fit_score_ui", metrics.get("score_ui", float("nan")))
            )
            selected_rank_score = float(metrics.get("rank_score", float("inf")))
            selected_score_basis = str(metrics.get("score_basis", "standard-noise"))
            selected_underconstrained = bool(metrics.get("underconstrained_penalty", False))
            selected_max_std = metrics.get("max_std_mm")
            selected_rel_std = metrics.get("rel_std")
            selected_warnings = list(metrics.get("warnings") or [])
            selected_history_rank_score, selected_history_rank_info = _full_auto_history_selection_score(
                selected_rank_score,
                iteration_index=step,
                coverage_adjust=_float_or_none(metrics.get("rank_coverage_adjust")),
            )
            selected_fit_score_ui = _full_auto_display_fit_score_ui(
                score_basis=selected_score_basis,
                raw_fit_score_ui=selected_raw_fit_score_ui,
                history_rank_score=selected_history_rank_score,
            )
            selected_summary_meta = {
                "iteration": step,
                "run_id": selected_id,
                "flags": selected_flags,
                "score_ui": selected_fit_score_ui,
                "raw_fit_score_ui": selected_raw_fit_score_ui,
                "history_rank_score": selected_history_rank_score,
                "score_basis": selected_score_basis,
                "cost": selected_cost,
                "rel_std": selected_rel_std,
                "max_std_mm": selected_max_std,
            }
            m669 = _m669_from_plan(plan)
            m666 = _m666_from_plan(plan)
            anchors = plan.get("anchors")
            anchor_str = ""
            if isinstance(anchors, np.ndarray):
                anchor_str = np.array2string(anchors, precision=2, separator=", ")
            elif anchors is not None:
                anchor_str = str(np.asarray(anchors))

            with _log_context():
                _print_ellipse_plan(
                    plan,
                    top_n=5,
                    print_command=True,
                    output_with_explanations=bool(output_with_explanations),
                )

            if write_cfg is not None and plan.get("best_cfg") is not None:
                try:
                    _write_sweep_config_file(Path(write_cfg), plan["best_cfg"])
                except Exception as exc:
                    _log_line(f"; full-auto warning: failed to write sweep config {write_cfg}: {exc}")

            improvement = None
            if np.isfinite(selected_cost):
                improvement = float(best_cost) - float(selected_cost) if np.isfinite(best_cost) else None
            score_improvement = None
            if np.isfinite(selected_fit_score_ui):
                score_improvement = (
                    float(best_score_ui) - float(selected_fit_score_ui)
                    if np.isfinite(best_score_ui)
                    else None
                )
            improved = False
            if (not selected_underconstrained) and np.isfinite(selected_rank_score) and (
                best_plan is None or selected_rank_score <= best_rank_score - min_delta
            ):
                best_score_ui = float(selected_fit_score_ui)
                best_rank_score = float(selected_rank_score)
                best_cost = float(selected_cost)
                best_plan = plan
                best_meta = dict(selected_summary_meta)
                improved = True

            score_rank: Optional[int] = None
            history_improved = False
            if (not selected_underconstrained) and np.isfinite(selected_rank_score):
                history_candidates.append(
                    {
                        "plan": plan,
                        "iteration": step,
                        "run_id": selected_id,
                        "flags": selected_flags,
                        "rank_score": selected_rank_score,
                        "history_rank_score": selected_history_rank_score,
                        "rank_coverage_adjust": metrics.get("rank_coverage_adjust"),
                        "score_ui": selected_fit_score_ui,
                        "raw_fit_score_ui": selected_raw_fit_score_ui,
                        "score_basis": selected_score_basis,
                        "cost": selected_cost,
                        "rel_std": selected_rel_std,
                        "max_std_mm": selected_max_std,
                        "summary_meta": dict(selected_summary_meta),
                    }
                )
                score_rank = _current_history_rank_position(iteration=step)
                history_improved = bool(score_rank == 1)
                if history_improved:
                    no_improve = 0
                else:
                    no_improve += 1
            history_total_iterations = max(history_total_iterations, int(step))

            stop_cost_hit = False
            stop_std_hit = False
            if best_plan is not None:
                if stop_cost is not None and np.isfinite(best_cost) and best_cost <= float(stop_cost):
                    stop_cost_hit = True
                if stop_std_mm is not None and isinstance(best_meta.get("max_std_mm"), (int, float)):
                    stop_std_hit = bool(
                        np.isfinite(float(best_meta["max_std_mm"]))
                        and float(best_meta["max_std_mm"]) <= float(stop_std_mm)
                    )

            summary_flags = f" flags='{selected_flags}'" if selected_flags else ""
            summary_rel = _fmt_float(selected_rel_std)
            summary_std = _fmt_float(selected_max_std, suffix="mm")
            summary_cost = _fmt_float(selected_cost)
            summary_fit_score_ui = _fmt_float(selected_fit_score_ui)
            selected_summary_line = (
                f"; selected run={selected_id}{summary_flags} fit_score_ui={summary_fit_score_ui} "
                f"raw_fit_score_ui={_fmt_float(selected_raw_fit_score_ui)} "
                f"score_basis={selected_score_basis} cost={summary_cost} "
                f"rel_std={summary_rel} max_std={summary_std} "
                f"rank_score={_fmt_float(selected_rank_score)} "
                f"iteration_adjust={_fmt_float(selected_history_rank_info.get('iteration_adjust'))} "
                f"coverage_adjust={_fmt_float(selected_history_rank_info.get('coverage_adjust'))} "
                f"history_rank_score={_fmt_float(selected_history_rank_score)}"
            )
            _log_line(selected_summary_line)
            if verbose:
                _log_console(selected_summary_line)
                if m669:
                    _log_console(f"Anchors (M669): {m669}")
                elif anchor_str:
                    _log_console(f"Anchors: {anchor_str}")
                if m666:
                    _log_console(f"; line_model_params (M666): {m666}")
            elif has_variants:
                _log_console(f"; selected run={selected_id}{summary_flags}")
            if not selected_underconstrained:
                if score_rank is not None:
                    rank_label = "best" if score_rank == 1 else _ordinal(score_rank) + " best"
                    _log_console(f"The {rank_label} try so far.")
                if history_improved:
                    _log_console("Patience reset.")
            if selected_underconstrained:
                _log_console("; selected run hit underconstrained sentinel; continuing to collect more sweeps.")

            threshold_accept = False
            if stop_cost is not None and stop_cost_hit:
                threshold_accept = True
            if stop_std_mm is not None and stop_std_hit:
                threshold_accept = True
            if selected_underconstrained:
                decision = "collect"
            elif threshold_accept or no_improve >= patience_limit:
                decision = "accept"
            else:
                decision = "collect"

            _append_jsonl(
                log_path,
                {
                    "timestamp": datetime.now().isoformat(),
                    "iteration": step,
                    "dataset": str(dataset_path),
                    "runs": [
                        {
                            "id": r["id"],
                            "flags": r["flags"],
                            "settings": r["settings"],
                            "metrics": r["metrics"],
                        }
                        for r in run_results
                    ],
                    "selected_run": selected_id,
                    "decision": decision,
                    "cost": selected_cost,
                    "cost_improvement": improvement,
                    "score_ui": selected_fit_score_ui,
                    "raw_fit_score_ui": selected_raw_fit_score_ui,
                    "history_rank_score": selected_history_rank_score,
                    "history_iteration_adjust": selected_history_rank_info.get("iteration_adjust"),
                    "history_coverage_adjust": selected_history_rank_info.get("coverage_adjust"),
                    "selection_rank_score": selected_history_rank_score,
                    "selection_iteration_adjust": selected_history_rank_info.get("iteration_adjust"),
                    "selection_coverage_adjust": selected_history_rank_info.get("coverage_adjust"),
                    "score_ui_improvement": score_improvement,
                    "improved_best": improved,
                    "best_cost": best_cost,
                    "best_score_ui": best_score_ui,
                    "best_score_basis": best_meta.get("score_basis"),
                    "best_run": best_meta.get("run_id"),
                    "best_iteration": best_meta.get("iteration"),
                    "no_improve_count": no_improve,
                    "patience": patience_limit,
                    "min_delta": min_delta,
                    "stop_cost_hit": stop_cost_hit,
                    "stop_std_hit": stop_std_hit,
                    "warnings": selected_warnings,
                    "recovery": recovery_info,
                },
            )

            if decision == "accept":
                summary_plan, summary_meta = _select_history_summary_candidate(reason="patience-or-threshold")
                if summary_plan is None:
                    _log_console("; full-auto: no best plan available; stopping.")
                    _log_console(_solution_quality_message(None))
                    return _finalize(2)
                return _emit_summary_and_send(
                    summary_plan,
                    summary_meta=summary_meta,
                )

            remaining = max(0, patience_limit - no_improve)

            if replay_mode:
                if replay_index >= len(replay_sweeps):
                    _log_console(
                        "; full-auto replay: no more sweeps to replay; switching to live collection."
                    )
                    _log_line(
                        "; full-auto replay: no more sweeps to replay; switching to live collection."
                    )
                    replay_mode = False
                    if work_path != dataset_path:
                        work_path = dataset_path

                if replay_mode:
                    _log_console("Replaying next sweep to try and beat it.")
                    _log_console(f"Still has patience for {remaining} more attempts.")
                    next_sweep = replay_sweeps[replay_index]
                    replay_index += 1
                    base_dataset = _load_json(work_path)
                    new_dataset = dict(base_dataset)
                    new_dataset["timestamp"] = datetime.now().isoformat()
                    new_dataset["sweeps"] = [next_sweep]
                    merged = _merge_sweep_datasets(base_dataset, new_dataset)
                    _write_json(work_path, merged)
                    sweeps = merged.get("sweeps", [])
                    count = len(sweeps) if isinstance(sweeps, list) else "?"
                    _log_line(
                        f"; replayed sweep {replay_index}/{len(replay_sweeps)} "
                        f"-> {work_path} sweeps={count}"
                    )
                    _log_console(f"Collected sweep nr {count}")
                    continue

            if no_collect:
                _log_line("; --no-collect set; stopping before live collection.")
                _log_console("; --no-collect set; stopping before live collection.")
                summary_plan, summary_meta = _select_history_summary_candidate(
                    reason="no-collect",
                    fallback_plan=plan if isinstance(plan, dict) else None,
                    fallback_meta=selected_summary_meta,
                )
                if summary_plan is None:
                    _log_console("; full-auto: no summary candidate available; stopping.")
                    _log_console(_solution_quality_message(None))
                    return _finalize(2)
                return _emit_summary_and_send(summary_plan, summary_meta=summary_meta)

            cmd = plan.get("collect_command")
            if not isinstance(cmd, list) or not cmd:
                _log_line("; No valid candidate to collect; stopping.")
                _log_console("; No valid candidate to collect; stopping.")
                _log_console(_solution_quality_message(best_score_ui if np.isfinite(best_score_ui) else None))
                return _finalize(2)

            _log_console("Collecting new sweep to try and beat it.")
            _log_console(f"Still has patience for {remaining} more attempts.")
            _log_line(f"; collecting next sweep ({selected_id})")
            _log_line(f"; running: {' '.join(str(x) for x in cmd)}")
            with _log_context():
                subprocess.run(cmd, check=True, stdout=log_handle, stderr=log_handle)
            reset_pending = False

            base_dataset = _load_json(work_path)
            new_dataset = _load_json(collector_output)
            if plan.get("force_args_applied") and isinstance(plan.get("force_tuning"), dict):
                cfg = new_dataset.get("config")
                if isinstance(cfg, dict):
                    cfg["force_tuning"] = dict(plan["force_tuning"])
                    _write_json(collector_output, new_dataset)

            merged = _merge_sweep_datasets(base_dataset, new_dataset)
            _write_json(work_path, merged)
            sweeps = merged.get("sweeps", [])
            count = len(sweeps) if isinstance(sweeps, list) else "?"
            _log_line(f"; merged {collector_output} -> {work_path} sweeps={count}")
            _log_console(f"Collected sweep nr {count}")
            try:
                if collector_output != work_path:
                    collector_output.unlink()
            except OSError:
                pass
    except KeyboardInterrupt:
        return _accept_best("Ctrl-C")

    _log_line(f"; reached max steps; dataset={work_path}")
    _log_console(f"; reached max steps; dataset={work_path}")
    _log_console(_solution_quality_message(best_score_ui if np.isfinite(best_score_ui) else None))
    return _finalize(0)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_semi_auto_parser()
    args = parser.parse_args(argv)
    _apply_optimizer_mode_env(str(args.optimizer_mode))
    spool_opts = _resolve_spool_cli_options(parser, args)
    machine_type = _normalize_machine_type(str(args.machine_type))
    full_auto_runs = list(args.full_auto_run or [])
    if bool(args.shotgun):
        try:
            full_auto_runs.extend(_load_shotgun_runs())
        except FileNotFoundError as exc:
            parser.error(str(exc))
    collector_args = _clean_collector_args(args.collector_args)
    if args.speedup is not None and not _arg_has_flag(collector_args, "--speedup"):
        speedup_arg = (
            str(int(args.speedup))
            if float(args.speedup).is_integer()
            else str(args.speedup)
        )
        collector_args.extend(["--speedup", speedup_arg])
    if bool(args.project_zero_tension) and not _arg_has_flag(collector_args, "--project-zero-tension"):
        collector_args.append("--project-zero-tension")
    if bool(args.debug_sweep_actions) and not _arg_has_flag(collector_args, "--debug-sweep-actions"):
        collector_args.append("--debug-sweep-actions")
    return full_auto_loop(
        work_dataset=args.dataset,
        machine_type=str(machine_type),
        max_steps=int(args.max_steps),
        stop_cost=args.stop_cost,
        stop_std_mm=args.stop_std_mm,
        solve_restarts=int(args.solve_restarts),
        solve_iterations=int(args.solve_iterations),
        solve_optimizer=str(args.solve_optimizer),
        residual_threshold=float(args.threshold),
        spring_k_multiplier=float(args.spring_k_multiplier),
        use_flex=bool(args.flex),
        pointwise_residual_mode=str(args.pointwise_residual),
        pointwise_filtering=bool(args.pointwise_filtering),
        pointwise_global_mad=bool(args.pointwise_global_mad),
        sweep_wise_filtering=bool(args.sweep_wise_filtering),
        sweep_metric=str(args.sweep_metric),
        use_noise_mean=bool(args.use_noise_mean),
        sigma_source=str(args.sigma_source),
        robust_debug=bool(args.robust_debug),
        residuals_csv=args.residuals_csv,
        generate_report=bool(args.report),
        find_radii=str(spool_opts["find_radii"]),
        find_buildup_factor=str(spool_opts["find_buildup_factor"]),
        base_radii=_parse_csv_floats(args.base_radii),
        buildup_factor=args.buildup_factor,
        r0_bounds=spool_opts["r0_bounds"],
        b_bounds=spool_opts["b_bounds"],
        r0_prior_sigma_mm=spool_opts["r0_prior_sigma_mm"],
        b_prior_sigma=spool_opts["b_prior_sigma"],
        spool_outer_iters=int(spool_opts["spool_outer_iters"]),
        spool_inner_iters=int(spool_opts["spool_inner_iters"]),
        theta0_mode=str(spool_opts["theta0_mode"]),
        line_width=float(spool_opts["line_width"]),
        sigma_floor_mm=(None if spool_opts.get("sigma_floor_mm") is None else float(spool_opts["sigma_floor_mm"])),
        sigma_used_mm=(None if spool_opts.get("sigma_used_mm") is None else float(spool_opts["sigma_used_mm"])),
        low_anchor_z=args.low_anchor_z,
        candidate_deltas=_parse_csv_floats(args.candidate_deltas),
        candidate_count=int(args.candidate_count),
        delta_min=args.delta_min,
        delta_max=args.delta_max,
        fd_eps_mm=float(args.fd_eps_mm),
        regularization=float(args.regularization),
        exclude_existing=not bool(args.no_exclude_existing),
        existing_tol_mm=float(args.existing_tol_mm),
        min_fixed_delta_spacing_mm=float(args.min_fixed_delta_spacing_mm),
        top_k=int(args.top_k),
        write_cfg=args.write_sweep_config,
        collector_args=collector_args,
        sim=bool(args.sim),
        keep_sim_alive=bool(args.keep_sim_alive),
        hp_sim_reset=bool(args.hp_sim_reset),
        sweep_points=args.sweep_points,
        output_with_explanations=bool(args.output_with_explanations),
        full_auto_runs=full_auto_runs,
        full_auto_log=args.full_auto_log,
        patience=int(args.patience),
        verbose=bool(args.verbose),
        filter_schedule=spool_opts.get("filter_schedule"),
        objective_schedule=spool_opts.get("objective_schedule"),
        scale_fix=spool_opts.get("scale_fix"),
        fit_structure=spool_opts.get("fit_structure"),
        no_collect=bool(args.no_collect),
    )


if __name__ == "__main__":
    raise SystemExit(main())
