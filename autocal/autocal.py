from __future__ import annotations

import os
import queue
import signal
import sys
import threading
import time
from contextlib import nullcontext, ExitStack
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from autocal._autocal_common import *  # noqa: F401,F403
from autocal.planning_pass import plan_next_ellipse_sweep, plan_ellipse_sweep
from autocal.equal_strain import write_sensor_force_profile, profile_policy_error
from autocal.rough_bootstrap import (
    MAX_EXTRA_ROUGH_SWEEPS, assess_rough_geometry, next_rough_sweep,
    bootstrap_state_error, write_bootstrap_state,
)
from autocal.history_selection import HISTORY_CONSENSUS_MAX_SPREAD_MM, evaluate_history_candidates, select_history_plan
from autocal.anchor_uncertainty import coordinate_names, newest_trusted_index, plan_anchor_uncertainty, uncertainty_label
from autocal import extended_reference
from autocal.fit_recovery import attempt_underconstrained_recovery, warm_start_seeds_from_plan
from autocal.stopping import StoppingPolicy
from autocal.dataset_roles import measurement_groups


class BrowserCollector:
    """Run sequential collection commands over one simulator bridge."""

    def __init__(self, log_line):
        self.log_line = log_line
        self.process = None

    def collect(self, cmd):
        if self.process is None:
            self.process = subprocess.Popen(
                [*cmd[:2], "--autocal-session"],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, encoding="utf-8", errors="replace",
                start_new_session=not bool(os.environ.get("HP_SIM5_AUTOCAL_JOB")),
            )
        process = self.process
        try:
            process.stdin.write(json.dumps(cmd[2:]) + "\n")
            process.stdin.flush()
        except BrokenPipeError:
            # Read the worker's diagnostics before reporting its exit.
            pass
        for line in process.stdout:
            if line == "AUTOCAL_COLLECTOR_COMPLETE\n":
                return
            self.log_line(line.rstrip("\n"))
        raise subprocess.CalledProcessError(process.wait() or 1, cmd)

    def close(self):
        if self.process is None:
            return
        process, self.process = self.process, None
        try:
            try:
                process.stdin.close()
            except BrokenPipeError:
                pass
            process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            _stop_process(process)
            process.wait()
        finally:
            with process.stdout:
                for line in process.stdout:
                    self.log_line(line.rstrip("\n"))


LOG_COLUMN_HEADER = f"{'wall':>10}  {'sim':>10}  | message"


def _format_log_entry(msg: str, *, wall_s: float, sim_s: Optional[float]) -> str:
    """Timestamp columns first, so the messages line up and can be scanned down the file."""
    sim = "-" if sim_s is None else f"{sim_s:.2f}s"
    # Keep the message verbatim: log readers match markers such as "; selected run=".
    return f"{wall_s:9.2f}s  {sim:>10}  | {msg}"


def _collector_output_lines(stream, idle_s: float = 0.25):
    """Yield a collector's output line by line. A fragment that stops arriving, such as a
    prompt waiting for Enter, is yielded once the pipe has been idle for idle_s.

    A reader thread does the blocking reads, so this works on any platform and for any
    descriptor number; select.select() supports neither on all of them."""
    fd = stream.fileno()
    chunks: "queue.Queue[Optional[bytes]]" = queue.Queue()

    def pump():
        try:
            while chunk := os.read(fd, 65536):
                chunks.put(chunk)
        finally:
            chunks.put(None)

    threading.Thread(target=pump, name="collector-output", daemon=True).start()
    pending = b""
    while True:
        try:
            chunk = chunks.get(timeout=idle_s)
        except queue.Empty:
            if pending:
                yield pending.decode("utf-8", "replace")
                pending = b""
            continue
        if chunk is None:
            break
        *lines, pending = (pending + chunk).split(b"\n")
        for line in lines:
            yield line.rstrip(b"\r").decode("utf-8", "replace")
    if pending:
        yield pending.decode("utf-8", "replace")


class _LineSink:
    """File-like stand-in for redirected stdout/stderr that logs each complete line."""

    def __init__(self, log_line):
        self.log_line = log_line
        self.pending = ""

    def write(self, text):
        *lines, self.pending = (self.pending + text).split("\n")
        for line in lines:
            self.log_line(line)
        return len(text)

    def close(self):
        if self.pending:
            self.log_line(self.pending)
            self.pending = ""


def full_auto_loop(**kwargs) -> int:
    with ExitStack() as resources:
        return _full_auto_loop(resources, **kwargs)


def _full_auto_loop(
    resources: ExitStack,
    *,
    work_dataset: Optional[Path],
    machine_type: str,
    max_steps: int,
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
    collector_args: Sequence[str],
    sim: bool,
    shape: str = "ellipse",
    keep_sim_alive: bool,
    hp_sim_reset: bool,
    sweep_points: Optional[int],
    output_with_explanations: bool,
    full_auto_runs: Optional[Sequence[str]],
    full_auto_log: Optional[Path],
    patience: int,
    stable_step_mm: float = 0.5,
    verbose: bool,
    fit_home_offsets: bool = True,
    fit_force_ripple: bool = False,
    fit: str = "point",
    equal_strain: bool = True,
    filter_schedule: Optional[Sequence[Any]] = None,
    objective_schedule: Optional[Sequence[Any]] = None,
    no_collect: bool = False,
    sparse_recovery: bool = False,
    firmware: str = "rrf",
    config: Optional[str] = None,
    rrf_config: Optional[str] = None,
    klipper_config: Optional[str] = None,
    headless_sim=None,
    stage_artifacts: Optional[Path] = None,
) -> int:
    max_steps = max(1, int(max_steps))
    if work_dataset is not None:
        dataset_path = Path(work_dataset)
    else:
        dataset_path = Path("autocal/data/default_dataset.json")
    if no_collect and not dataset_path.exists():
        print(f"; --no-collect needs an existing dataset; {dataset_path} does not exist", file=sys.stderr)
        return 2
    work_path = dataset_path
    applying_solution = False
    if error := bootstrap_state_error(dataset_path):
        print(f"; {error}")
        return 2
    if dataset_path.exists() and not no_collect:
        if error := profile_policy_error(_load_json(dataset_path)):
            print(f"; {error}")
            return 2
    log_path = Path(full_auto_log) if full_auto_log is not None else _unique_path(
        dataset_path.with_name(f"{dataset_path.stem}.full_auto_log.jsonl")
    )
    text_log_path = _unique_path(dataset_path.with_name(f"{dataset_path.stem}.full_auto.log"))
    text_log_path.parent.mkdir(parents=True, exist_ok=True)
    text_log_path.write_text("", encoding="utf-8")
    log_handle = resources.enter_context(text_log_path.open("a", encoding="utf-8"))
    log_handle.write(LOG_COLUMN_HEADER + "\n")
    console = sys.stdout
    run_started = time.monotonic()
    if extended_reference.enabled():
        log_handle = extended_reference.EventLog(log_handle, text_log_path)
        extended_reference.emit("run_start", argv=sys.argv)
        extended_reference.artifact(dataset_path)

    def _sim_seconds() -> Optional[float]:
        return None if headless_sim is None else headless_sim.simulated_seconds()

    def _write_entry(msg: str) -> str:
        line = _format_log_entry(str(msg), wall_s=time.monotonic() - run_started, sim_s=_sim_seconds())
        log_handle.write(line + "\n")
        log_handle.flush()
        return line

    def _log_line(msg: str) -> None:
        line = _write_entry(msg)
        if verbose:
            print(line, file=console, flush=True)

    def _log_event(payload: Dict[str, object]) -> None:
        sim_s = _sim_seconds()
        _append_jsonl(log_path, {
            **payload, "wall_s": round(time.monotonic() - run_started, 3),
            "sim_s": None if sim_s is None else round(sim_s, 3),
        })

    _log_line(f"command: {shlex.join(sys.argv)}")

    def _log_console(msg: str) -> None:
        line = _write_entry(msg)
        print(line if verbose else msg, file=console, flush=True)

    if extended_reference.enabled() and stage_artifacts is None:
        stage_artifacts = dataset_path.with_name(f"{dataset_path.stem}.stages")
    if stage_artifacts is not None:
        stage_artifacts = _unique_path(Path(stage_artifacts))
        stage_artifacts.mkdir(parents=True)
        _log_console(f"Writing stage artifacts to: {stage_artifacts}")

    def _log_context():
        sink = _LineSink(_log_line)
        stack = contextlib.ExitStack()
        stack.enter_context(redirect_stdout(sink))
        stack.enter_context(redirect_stderr(sink))
        stack.callback(sink.close)
        return stack

    def _emit_summary_and_send(
        best_plan: Dict[str, object],
        *,
        summary_meta: Optional[Dict[str, object]] = None,
    ) -> int:
        nonlocal applying_solution
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
        meta_for_summary = dict(summary_meta or {})
        uncertainty = _float_or_none(meta_for_summary.get("anchor_uncertainty_mm"))
        _log_event({
            "timestamp": datetime.now().isoformat(), "dataset": str(work_path),
            "event": "summary", "m669": m669, "m666": m666,
            "anchors": np.asarray(anchors, dtype=float).tolist(),
            "fitted_anchors": np.asarray(best_plan["anchors"], dtype=float).tolist(),
            "selection": meta_for_summary,
        })
        _log_console("")
        _log_console("== Calibration summary ==")
        _log_console(f"Found parameters of {uncertainty_label(uncertainty)} quality")
        _log_console(f"Anchor uncertainty (mm, lower is better): {_fmt_float(uncertainty)}")
        _log_console("; Spread of anchors refitted without one sweep at a time. "
                     "It misses bias that every sweep shares, such as unmodelled line stretch.")
        single_unit = meta_for_summary.get("single_unit_coordinates") or []
        if single_unit:
            _log_console(f"; {', '.join(single_unit)} rest on a single sweep: the refit without it lost them, "
                         "so their spread comes from the other refits and that sweep is not cross-checked.")
        if m669:
            _log_console(f"Anchors (M669): {m669}")
        elif anchor_str:
            _log_console(f"Anchors: {anchor_str}")
        if m666:
            _log_console(f"Spools (M666): {m666}")
        if full_auto_runs:
            best_flags = str(meta_for_summary.get("flags", "")).strip()
            best_run = str(meta_for_summary.get("run_id", "")).strip()
            label = best_flags or best_run or "default"
            _log_console(f"Selected variant/flag setup: {label}")

        skip_sim_send = bool(sim and no_collect and not server_explicit)
        if headless_sim is not None and not no_collect:
            applying_solution = True
            try:
                headless_sim.apply_parameters([m669, m666])
            except Exception as exc:
                _log_console(f"; failed to apply parameters: {exc}")
                return _finalize(1)
            return _finalize(0)
        if m669:
            if skip_sim_send:
                _log_console(f"; --sim + --no-collect set; skipping M669 send.")
            elif firmware == "klipper":
                _log_console("; --firmware klipper set; skipping M669 send.")
            else:
                _log_console(f"Sending {m669} to {rrf_server}")
                applying_solution = True
                try:
                    reply = provider.send_gcode(rrf_server, m669)
                except Exception as exc:
                    _log_console(f"; failed to send M669: {exc}")
                    return _finalize(1)
                reply = reply.strip()
                if reply:
                    _log_line(f"; M669 reply: {reply}")
        else:
            _log_console("Sending M669 skipped (no command available)")
        return _finalize(0)

    provider = get_firmware_provider(firmware)
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

    sim_config = _resolve_sim_config(
        machine_type=machine_type,
        find_radii_mode=find_radii_mode,
        find_buildup_mode=find_buildup_mode,
        firmware=firmware,
        config=config,
        rrf_config=rrf_config,
        klipper_config=klipper_config,
        layering=_arg_has_flag(collector_args, "--line-layering") or bool(buildup_factor),
    )

    extended_reference.artifact(REPO_ROOT / "RRF/run/vsd" / str(sim_config) if firmware == "rrf" else Path(sim_config))

    def _normalize_collector_args(argv: Sequence[str]) -> List[str]:
        normalized = list(argv)

        def _drop_flags_with_values(args: Sequence[str], *flags: str) -> List[str]:
            out: List[str] = []
            i = 0
            while i < len(args):
                arg = str(args[i])
                if arg in flags:
                    i += 2
                    continue
                if any(arg.startswith(flag + "=") for flag in flags):
                    i += 1
                    continue
                out.append(arg)
                i += 1
            return out

        normalized = _drop_flags_with_values(normalized, "--firmware")
        normalized.extend(["--firmware", str(firmware)])

        if firmware == "klipper" and sim_config is not None:
            normalized = _drop_flags_with_values(normalized, "--config")
            normalized.extend(["--config", str(sim_config)])

        return normalized

    user_no_spawn = _arg_has_flag(collector_args, "--no-spawn-rrf-simulator")
    collector_args_eff = _normalize_collector_args(collector_args)
    collector_args_eff = _apply_simulation_defaults(collector_args_eff, sim=sim)
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
    collector_speedup = None
    raw_speedup = _arg_value(collector_args_eff, "--speedup")
    if raw_speedup is not None:
        try:
            parsed_speedup = float(raw_speedup)
        except (TypeError, ValueError):
            parsed_speedup = None
        if parsed_speedup is not None and np.isfinite(parsed_speedup) and parsed_speedup > 0:
            collector_speedup = float(parsed_speedup)
    reset_pending = bool(sim and hp_sim_reset_eff)
    rrf_server, server_explicit, port = _resolve_rrf_target(collector_args)
    if sim and not no_collect and not server_explicit and not user_no_spawn:
        target_port = port or DEFAULT_RRF_PORT
        _log_line(f"; starting {firmware} simulator at http://localhost:{target_port} (config: {sim_config})")
        sim_process = provider.start_simulator(
            target_port,
            sim_config=sim_config,
            speedup=collector_speedup,
        )
        if not keep_sim_alive:
            resources.callback(_stop_process, sim_process)
        provider.wait_for_ready(f"http://localhost:{target_port}")

    browser_collector = None
    if sim and not no_collect and headless_sim is None and not _arg_has_flag(
        collector_args_eff, "--no-ws", "--headless-url"
    ):
        raw_ws_port = _arg_value(collector_args_eff, "--ws-port")
        try:
            ws_port = int(raw_ws_port) if raw_ws_port is not None else 8790
        except ValueError:
            ws_port = 0
        if ws_port > 0:
            _log_console(f"; --sim: collection waits for a browser connected to ws://localhost:{ws_port}")
            _log_console(
                f";        open http://localhost:5173/hp-sim5/hp-sim-3d/"
                f"?gcode_ws=ws://localhost:{ws_port} and keep the simulator page open"
            )
        if not _arg_has_flag(collector_args_eff, "--step-gcode", "--step-gcodes", "--interactive-gcode"):
            browser_collector = BrowserCollector(_log_line)
            resources.callback(browser_collector.close)

    def _finalize(code: int) -> int:
        extended_reference.artifact(work_path)
        extended_reference.artifact(text_log_path)
        extended_reference.artifact(log_path)
        extended_reference.emit("run_complete", returncode=code)
        return code

    def _run_collection_command(cmd):
        extended_reference.emit("collection_start", command=cmd)
        for index, value in enumerate(cmd[:-1]):
            if value in ("--config", "--sweep-config", "--sweep-config-file"):
                extended_reference.artifact(cmd[index + 1])
        if browser_collector is not None:
            code = 1
            try:
                browser_collector.collect(cmd)
                code = 0
            except subprocess.CalledProcessError as error:
                code = error.returncode
                raise
            except (KeyboardInterrupt, SystemExit) as error:
                code = 130 if isinstance(error, KeyboardInterrupt) else error.code
                browser_collector.close()
                raise
            finally:
                extended_reference.emit("collection_exit", returncode=code)
                extended_reference.artifact(work_path)
            return
        # Plain collection stays in the terminal's foreground group so Ctrl-C reaches it directly, as before.
        owns_session = (headless_sim is not None or extended_reference.enabled()) and not os.environ.get('HP_SIM5_AUTOCAL_JOB')
        process = subprocess.Popen(
            cmd, stdin=None if headless_sim is None else subprocess.DEVNULL,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            start_new_session=owns_session,
        )
        if headless_sim is not None:
            headless_sim.track_process('collector', process)
        try:
            for line in _collector_output_lines(process.stdout):
                _log_line(line)
            code = process.wait()
            extended_reference.emit("collection_exit", returncode=code)
            extended_reference.artifact(work_path)
            if code:
                raise subprocess.CalledProcessError(code, cmd)
        finally:
            _stop_process(process)

    ellipse_reference_fit = fit == "ellipse"
    if shape == "ellipsoid":
        from autocal.ellipsoid_autocal import run_ellipsoid_autocal
        owned = {"--output-file", "--output", "--out", "--machineType", "--machine-type", "--sweep-config-file",
                 "--sweep-config", "--fixed-targets", "--fixedTargets", "--max-travel-mm", "--sweepPoints",
                 "--sweep-points", "--completed-sweeps"}
        mesh_args = [arg for index, arg in enumerate(collector_args_eff)
                     if arg not in owned and (index == 0 or collector_args_eff[index - 1] not in owned)]
        if reset_pending and "--hp-sim-reset" not in mesh_args:
            mesh_args.append("--hp-sim-reset")

        def _collect_mesh(cmd):
            with _log_context():
                _run_collection_command(cmd)

        try:
            accepted = run_ellipsoid_autocal(
                dataset_path, machine_type=str(machine_type), collector_args=mesh_args,
                run_collection=_collect_mesh, log_line=_log_line, log_console=_log_console,
                append_log=lambda row: _append_jsonl(log_path, row), no_collect=bool(no_collect), use_noise_mean=bool(use_noise_mean),
                stable_step_mm=float(stable_step_mm), stop_file=_full_auto_stop_path(dataset_path),
                model=fit)
        except (subprocess.CalledProcessError, KeyboardInterrupt) as error:
            _log_console(f"; ellipsoid autocal stopped: {error!r}; data so far: {dataset_path}")
            return _finalize(130 if isinstance(error, KeyboardInterrupt) else 2)
        if accepted is None:
            return _finalize(2)
        if headless_sim is not None:
            headless_sim.manifest["stop_reason"] = accepted["reason"]
        return _emit_summary_and_send(accepted["plan"], summary_meta=accepted["meta"])

    checked_hp5_seed = None
    collected_bootstrap = not dataset_path.exists()
    if collected_bootstrap:
        if no_collect:
            _log_console(f"; --no-collect set but {dataset_path} is missing; not collecting.")
            return _finalize(2)
        dataset_path.parent.mkdir(parents=True, exist_ok=True)
        checked_bootstrap = (equal_strain and str(machine_type) in ("hangprinter_3", "hangprinter_4", "hangprinter_5", "hangprinter_5_pulley_reduced")
                             and not (_spool_mode_enabled(find_radii_mode) or _spool_mode_enabled(find_buildup_mode)))
        if checked_bootstrap:
            write_bootstrap_state(dataset_path, "collecting_rough")
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
        try:
            with _log_context():
                _run_collection_command(cmd)
        except (subprocess.CalledProcessError, KeyboardInterrupt, SystemExit) as error:
            if checked_bootstrap:
                write_bootstrap_state(dataset_path, "failed", reason=str(error))
            if isinstance(error, SystemExit):
                raise
            _log_console(f"; bootstrap collection stopped: {error}; partial data retained")
            if headless_sim is not None:
                headless_sim.manifest["stop_reason"] = "bootstrap-interrupted" if isinstance(error, KeyboardInterrupt) else "bootstrap-collection-failed"
            return _finalize(130 if isinstance(error, KeyboardInterrupt) else 2)
        reset_pending = False
        _log_line(f"; bootstrap dataset written to {dataset_path}")

        if checked_bootstrap:
            rough = _load_json(dataset_path)
            checks = []
            rough_path = dataset_path.with_suffix(".rough_bootstrap.json")
            check_path = dataset_path.with_suffix(".rough_bootstrap_checks.json")
            # Tune once; every subsequent collection shares the encoder origin and forces.
            force_args, _, _ = _inject_force_args(rough, [arg for arg in argv_eff if arg != "--hp-sim-reset"])

            def _collect_bootstrap_sweep(cfg, tag, *, profile=None):
                cfg_path = dataset_path.with_suffix(f".{tag}.cfg.txt")
                output_path = dataset_path.with_suffix(f".{tag}.json")
                _write_sweep_config_file(cfg_path, cfg)
                cmd = ["node", "autocal/control/cli/collect_sweep_data.mjs",
                       "--machineType", str(machine_type), "--sweep-config-file", str(cfg_path),
                       "--fixed-targets", _fixed_targets_spec(cfg), "--output-file", str(output_path),
                       *force_args]
                if profile is not None:
                    cmd.extend(["--sensor-force-profile", str(profile)])
                _log_line(";   " + " ".join(cmd))
                with _log_context():
                    _run_collection_command(cmd)
                return _load_json(output_path)

            try:
                for extra in range(MAX_EXTRA_ROUGH_SWEEPS + 1):
                    write_bootstrap_state(dataset_path, "checking_rough", extra_sweeps=extra)
                    _write_json(rough_path, rough)
                    rough_anchors, check = assess_rough_geometry(
                        rough, use_noise_mean=bool(use_noise_mean),
                    )
                    checks.append(check)
                    check_path.write_text(json.dumps({"checks": checks}, indent=2) + "\n", encoding="utf-8")
                    _log_console(f"; rough bootstrap: {check}")
                    if check["ready"]:
                        break
                    if extra == MAX_EXTRA_ROUGH_SWEEPS:
                        raise ValueError(f"rough geometry is still unreliable after {len(rough['sweeps'])} sweeps")
                    cfg = next_rough_sweep(rough, rough_anchors)
                    added = _collect_bootstrap_sweep(cfg, f"rough_extra_{extra + 1}")
                    rough = _merge_sweep_datasets(rough, added)
                    _write_json(dataset_path, rough)
                # Never mix rough constant-force observations with refined observations.
                # Each sweep has its own fixed targets and matching force table.
                refined = {**rough, "sweeps": []}
                for index, cfg in enumerate(dataset_sweep_configs(rough), start=1):
                    write_bootstrap_state(dataset_path, "remeasuring", sweep=index)
                    profile = dataset_path.with_suffix(f".bootstrap_force_{index}.json")
                    if not write_sensor_force_profile(profile, rough, rough_anchors, [cfg]):
                        raise ValueError("missing M666 W/N/Y/U or tuned forces for equal-strain bootstrap")
                    measured = _collect_bootstrap_sweep(cfg, f"equal_strain_{index}", profile=profile)
                    refined = _merge_sweep_datasets(refined, measured)
                    _write_json(dataset_path.with_suffix(".refining_bootstrap.json"), refined)
                _write_json(dataset_path, refined)
                if machine_type in ("hangprinter_5", "hangprinter_5_pulley_reduced"):
                    checked_hp5_seed = rough_anchors
                write_bootstrap_state(dataset_path, "complete")
            except (ValueError, np.linalg.LinAlgError, subprocess.CalledProcessError, KeyboardInterrupt, SystemExit) as error:
                write_bootstrap_state(dataset_path, "failed", reason=str(error))
                _log_console(f"; checked bootstrap stopped: {error}; rough data: {rough_path}; checks: {check_path}")
                if isinstance(error, SystemExit):
                    raise
                if headless_sim is not None:
                    headless_sim.manifest["stop_reason"] = "bootstrap-interrupted" if isinstance(error, KeyboardInterrupt) else "unreliable-rough-bootstrap"
                return _finalize(130 if isinstance(error, KeyboardInterrupt) else 2)

    replay_mode = False
    # Replay units: one sweep, or every record of one sphere mesh.
    replay_sweeps: List[List[dict]] = []
    replay_index = 0
    replay_work_path: Optional[Path] = None
    if dataset_path.exists():
        dataset_full = _load_json(dataset_path)
        sweeps_full = dataset_full.get("sweeps")
        units = ([[sweeps_full[index] for index in group] for group in measurement_groups(dataset_full)]
                 if isinstance(sweeps_full, list) else [])
        if not collected_bootstrap and len(units) > 3:
            replay_mode = True
            replay_work_path = dataset_path.with_suffix(".replay_tmp.json")
            work_path = replay_work_path
            initial = [s for unit in units[:3] for s in unit if isinstance(s, dict)]
            replay_sweeps = [[s for s in unit if isinstance(s, dict)] for unit in units[3:]]
            dataset_work = dict(dataset_full)
            dataset_work["timestamp"] = datetime.now().isoformat()
            dataset_work["sweeps"] = initial
            # Recorded rejections happened at unknown later points of the original
            # run; replay plans without them rather than seeing the future.
            dataset_work.pop("acquisition_rejections", None)
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
    stop_file = _full_auto_stop_path(dataset_path)
    _log_event(
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
        "theta0_mode": str(_normalize_theta0_mode(theta0_mode)),
        "filter_schedule": [
            str(v) for v in _parse_filter_schedule(filter_schedule, label="filter_schedule")
        ],
        "objective_schedule": [
            int(v) for v in _parse_objective_schedule(objective_schedule, label="objective_schedule")
        ],
        "line_width": float(line_width),
        "sigma_floor_mm": (None if sigma_floor_mm is None else float(sigma_floor_mm)),
        "fit_home_offsets": bool(fit_home_offsets),
        "fit_force_ripple": bool(fit_force_ripple),
        "ellipse_reference_fit": bool(ellipse_reference_fit),
    }

    history_candidates: List[Dict[str, object]] = []
    stopping = StoppingPolicy(patience=patience, stop_file=stop_file, stable_step_mm=float(stable_step_mm), machine_type=str(machine_type))
    stopping.announce(_log_console)
    has_variants = bool(full_auto_runs)

    spool_fit = _spool_mode_enabled(find_radii_mode) or _spool_mode_enabled(find_buildup_mode)

    def _heldout_candidate() -> Tuple[Optional[Dict[str, object]], Optional[Dict[str, object]]]:
        scored = evaluate_history_candidates(
            history_candidates, use_noise_mean=use_noise_mean,
            pointwise_residual_mode=pointwise_residual_mode, sigma_source=sigma_source, log_line=_log_line,
        )
        plan, meta, ranked = select_history_plan(scored)
        if not ranked:
            _log_console("; full-auto: all held-out predictions failed; stopping without applying calibration.")
            return None, None
        for _key, candidate, info in ranked[:5]:
            _log_line(f"; history_select: iter={candidate.get('iteration')} run={candidate.get('run_id')} "
                      f"anchor_comparison={_fmt_float(info.get('anchor_comparison_score'))} "
                      f"heldout_prediction={_fmt_float(info.get('prediction_score'))} "
                      f"heldout_sweeps={_fmt_float(info.get('prediction_sweeps'), fmt='.0f')}")
        members = meta["history_consensus_members"]
        _log_line(f"; history_select: consensus selected_iter={meta.get('iteration')} members={members} "
                  f"spread_mm={_fmt_float(meta['history_consensus_spread_mm'])} "
                  f"max_spread_mm={HISTORY_CONSENSUS_MAX_SPREAD_MM:g} "
                  f"applied={'consensus' if len(members) > 1 else 'best-ranked'}")
        return plan, meta

    def _newest_candidate(
        fallback_plan: Optional[Dict[str, object]] = None,
        fallback_meta: Optional[Dict[str, object]] = None,
    ) -> Tuple[Optional[Dict[str, object]], Optional[Dict[str, object]]]:
        # Apply the fit to all admissible data. Across 69 distinct recorded headless
        # runs the newest fit was within 0.8 mm of the best candidate on average and no
        # worse than fit-cost, held-out or consensus selection. A newest fit whose
        # jackknife failed or diverged is skipped; the replayed 23_april_2026 dataset
        # ends with diverged fits.
        if history_candidates and spool_fit:
            return _heldout_candidate()
        if history_candidates:
            index = newest_trusted_index([c.get("anchor_uncertainty_mm") for c in history_candidates])
            if index is None:
                _log_console("; full-auto: no fit has a usable anchor uncertainty.")
                return None, None
            newest = history_candidates[index]
            if newest is not history_candidates[-1]:
                _log_console(f"; full-auto: newest fit failed or diverged (anchor uncertainty "
                             f"{_fmt_float(history_candidates[-1].get('anchor_uncertainty_mm'), suffix=' mm')}); "
                             f"applying iteration {newest['iteration']}.")
            return newest["plan"], dict(newest["summary_meta"])
        if fallback_plan is not None:
            return fallback_plan, dict(fallback_meta or {})
        return None, None

    def _accept_best(reason: str) -> int:
        if headless_sim is not None:
            headless_sim.manifest["stop_reason"] = reason
        summary_plan, summary_meta = _newest_candidate()
        if summary_plan is None:
            _log_console(f"; full-auto: stop requested ({reason}) but no admissible fit available; stopping.")
            return _finalize(2)
        _log_console(f"; full-auto: stop requested ({reason}); accepting the newest fit.")
        return _emit_summary_and_send(summary_plan, summary_meta=summary_meta)

    def _compute_run_metrics(plan: Dict[str, object]) -> Dict[str, object]:
        primary_cost = _plan_primary_cost(plan)
        max_std_mm, rel_std, cov_ok = _plan_covariance_summary(plan)
        warnings = list(_plan_data_quality_warnings(plan))
        noise_metrics = _plan_noise_metrics(plan)
        underconstrained_penalty = _plan_hits_underconstrained_penalty(plan, primary_cost)
        if underconstrained_penalty and "underconstrained_penalty" not in warnings:
            warnings.append("underconstrained_penalty")
        valid = bool(np.isfinite(primary_cost) and cov_ok)
        pose_fit_metrics: Dict[str, object] = {}
        pose_fit = plan.get("pose_fit")
        if isinstance(pose_fit, dict):
            pose_anchors = np.asarray(pose_fit["anchors"], dtype=float)
            pose_fit_metrics = {
                "pose_fit_anchors": pose_anchors.tolist(),
                "pose_fit_rms_mm": float(pose_fit["rms_mm"]),
                "pose_fit_max_diff_mm": float(np.max(np.abs(pose_anchors - np.asarray(plan["anchors"], dtype=float)))),
                "home_offsets_mm": np.asarray(plan["home_offsets_mm"], dtype=float).tolist(),
                **({"force_ripple": plan["force_ripple"]} if plan.get("force_ripple") is not None else {}),
            }
        return {
            **pose_fit_metrics,
            "primary_cost": primary_cost,
            "cost_noise_normalized": plan.get("cost_noise_normalized"),
            "chi2_red": noise_metrics.get("chi2_red") if isinstance(noise_metrics, dict) else None,
            "J": noise_metrics.get("J") if isinstance(noise_metrics, dict) else None,
            "info_rank": plan.get("info_rank"),
            "info_rank_deficient": plan.get("info_rank_deficient"),
            "max_std_mm": max_std_mm,
            "rel_std": rel_std,
            "covariance_ok": cov_ok,
            "underconstrained_penalty": underconstrained_penalty,
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
            f"chi2_red={_fmt_float(metrics.get('chi2_red'))}"
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
        dataset_path_override: Optional[Path] = None,
        initial_guess: Optional[np.ndarray] = None,
        initial_radii_mm: Optional[np.ndarray] = None,
        initial_buildup_factor: Optional[np.ndarray] = None,
    ) -> Dict[str, object]:
        _log_line(f"{log_prefix}: flags='{run_flags}'")

        cfg_run_id = str(path_tag or run_id)
        dataset_for_run = Path(dataset_path_override or work_path)
        cfg_path = _full_auto_cfg_path(dataset_for_run, cfg_run_id)
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
                dataset_for_run,
                solve_restarts=int(settings["solve_restarts"]),
                solve_iterations=int(settings["solve_iterations"]),
                solve_optimizer=str(settings["solve_optimizer"]),
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
                theta0_mode=str(settings.get("theta0_mode", "zero")),
                filter_schedule=settings.get("filter_schedule"),
                objective_schedule=settings.get("objective_schedule"),
                line_width=float(settings.get("line_width", DEFAULT_LAYER_LINE_WIDTH_MM)),
                sigma_floor_mm=(
                    None
                    if settings.get("sigma_floor_mm") is None
                    else float(settings.get("sigma_floor_mm"))
                ),
                fit_home_offsets=bool(settings["fit_home_offsets"]),
                fit_force_ripple=bool(settings["fit_force_ripple"]),
                ellipse_reference_fit=bool(settings["ellipse_reference_fit"]),
                equal_strain=bool(equal_strain),
                candidate_count=int(candidate_count),
                delta_min=delta_min,
                delta_max=delta_max,
                regularization=float(regularization),
                min_fixed_delta_spacing_mm=float(min_fixed_delta_spacing_mm),
                write_cfg=cfg_path,
                collector_output=collector_output,
                collector_args=collector_args_eff,
                stage_artifact=(
                    stage_artifacts / f"fit-{step:03d}-{cfg_run_id}.json"
                    if stage_artifacts is not None else None
                ),
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
            "optimizer_runs": (plan.get("calibration") or {}).get("optimizer_runs", []),
        }

    rejected_probe_count = 0
    try:
        for step in range(1, max_steps + 1):
            if step + rejected_probe_count > max_steps:
                break
            if stopping.stop_requested():
                return _accept_best(f"stop-file {stop_file}")
            _log_console("")
            _log_console(f"== iteration {step}/{max_steps} - {work_path.name} ==")
            collector_output = work_path.with_name(f"{work_path.stem}.new_{step:03d}.json")
            run_results: List[Dict[str, object]] = []

            def _resolve_run_settings(
                run: Dict[str, object]
            ) -> Tuple[str, str, Dict[str, object], Dict[str, object]]:
                run_id = str(run.get("id", "run"))
                run_flags = str(run.get("flags", "")).strip()
                overrides = run.get("overrides") or {}
                if not isinstance(overrides, dict):
                    overrides = {}
                settings = dict(base_solver)
                settings.update(overrides)
                if (
                    str(base_solver["solve_optimizer"]) == "lbfgsb+trf-fallback"
                    and str(settings.get("solve_optimizer")) == "lbfgsb"
                ):
                    settings["solve_optimizer"] = "lbfgsb+trf-fallback"
                settings["optimizer_mode"] = _normalize_optimizer_mode(settings.get("optimizer_mode"))
                _apply_optimizer_mode_env(str(settings["optimizer_mode"]))
                if isinstance(settings.get("base_radii"), str):
                    settings["base_radii"] = _parse_csv_floats(str(settings["base_radii"]))
                settings["find_radii"] = _normalize_spool_find_mode(settings.get("find_radii"))
                settings["find_buildup_factor"] = _normalize_spool_find_mode(
                    settings.get("find_buildup_factor")
                )
                settings["theta0_mode"] = _normalize_theta0_mode(settings.get("theta0_mode"))

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
                return run_id, run_flags, overrides, settings

            def _execute_run_variant(
                run: Dict[str, object],
                *,
                warm_seeds: Optional[Dict[str, np.ndarray]] = None,
            ) -> Dict[str, object]:
                run_id, run_flags, overrides, settings = _resolve_run_settings(run)
                if warm_seeds is None:
                    return _execute_plan_run(
                        run_id=run_id,
                        run_flags=run_flags,
                        overrides=overrides,
                        settings=settings,
                        collector_output=collector_output,
                        log_prefix=f"; full-auto run {run_id}",
                        initial_guess=checked_hp5_seed if step == 1 else None,
                    )
                warm_id = f"{run_id}.warm"
                return _execute_plan_run(
                    run_id=warm_id,
                    run_flags=run_flags,
                    overrides=overrides,
                    settings=settings,
                    collector_output=collector_output,
                    log_prefix=f"; full-auto run {warm_id}",
                    path_tag=warm_id,
                    initial_guess=warm_seeds.get("initial_guess"),
                    initial_radii_mm=warm_seeds.get("initial_radii_mm"),
                    initial_buildup_factor=warm_seeds.get("initial_buildup_factor"),
                )

            for run in runs:
                if stopping.stop_requested():
                    return _accept_best(f"stop-file {stop_file}")
                run_results.append(_execute_run_variant(run))

            # History-seeded refit: a cold multi-start occasionally lands on the
            # cost-cap sentinel, so keep a warm-started candidate in the pool.
            warm_source: Optional[Dict[str, object]] = None
            if history_candidates:
                candidate_plan = history_candidates[-1].get("plan")
                if isinstance(candidate_plan, dict):
                    warm_source = candidate_plan
            if warm_source is not None and not stopping.stop_requested():
                warm_seeds = warm_start_seeds_from_plan(warm_source)
                if warm_seeds.get("initial_guess") is not None:
                    run_results.append(_execute_run_variant(runs[0], warm_seeds=warm_seeds))

            valid_runs = [r for r in run_results if r["metrics"]["valid"]]
            if not valid_runs:
                _log_event(
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
                                "optimizer_runs": r["optimizer_runs"],
                            }
                            for r in run_results
                        ],
                    },
                )
                _log_console("; full-auto: no valid calibration runs (non-finite cost or covariance).")
                if history_candidates:
                    return _accept_best("no-valid-runs")
                return _finalize(2)

            def _sort_key(entry: Dict[str, object]) -> Tuple[float, float, str]:
                # Cold and warm starts of one run spec share an objective, so their
                # costs compare. --shotgun specs that change the sigma source or
                # filtering do not share one; main ranked those by cost as well.
                metrics = entry["metrics"]
                underconstrained = bool(metrics.get("underconstrained_penalty", False))
                cost = float(metrics.get("primary_cost", float("inf")))
                return (1.0 if underconstrained else 0.0), cost, str(entry.get("id", ""))

            selected = sorted(valid_runs, key=_sort_key)[0]
            recovery_info = None
            if bool(selected["metrics"].get("underconstrained_penalty", False)):
                recovered, recovery_info = attempt_underconstrained_recovery(
                    selected,
                    collector_output=collector_output,
                    work_path=work_path,
                    sparse_recovery=sparse_recovery,
                    execute_plan_run=_execute_plan_run,
                    log_console=_log_console,
                    log_line=_log_line,
                )
                if recovered is not None:
                    selected = recovered
            plan = selected["plan"]
            metrics = selected["metrics"]
            selected_id = str(selected.get("id", "run"))
            selected_flags = str(selected.get("flags", "")).strip()
            selected_cost = float(metrics.get("primary_cost", float("nan")))
            selected_underconstrained = bool(metrics.get("underconstrained_penalty", False))
            selected_max_std = metrics.get("max_std_mm")
            selected_rel_std = metrics.get("rel_std")
            selected_warnings = list(metrics.get("warnings") or [])
            history_admissible = (not selected_underconstrained) and np.isfinite(selected_cost)
            uncertainty_started = time.perf_counter()
            selected_uncertainty, single_unit = plan_anchor_uncertainty(plan) if history_admissible else (None, [])
            uncertainty_seconds = time.perf_counter() - uncertainty_started
            selected_summary_meta = {
                "iteration": step,
                "run_id": selected_id,
                "flags": selected_flags,
                "anchor_uncertainty_mm": selected_uncertainty,
                "single_unit_coordinates": coordinate_names(single_unit, len(np.asarray(plan.get("anchors")))) if plan.get("anchors") is not None else [],
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

            if history_admissible:
                training_dataset = plan.get("dataset")
                training_sweeps = (
                    training_dataset.get("sweeps", [])
                    if isinstance(training_dataset, dict)
                    else []
                )
                history_candidates.append(
                    {
                        "plan": plan,
                        "iteration": step,
                        "run_id": selected_id,
                        "flags": selected_flags,
                        "anchor_uncertainty_mm": selected_uncertainty,
                        "cost": selected_cost,
                        "rel_std": selected_rel_std,
                        "max_std_mm": selected_max_std,
                        "training_sweep_ids": [
                            str(sweep.get("id", ""))
                            for sweep in training_sweeps
                            if isinstance(sweep, dict)
                        ],
                        "summary_meta": dict(selected_summary_meta),
                        "settings": dict(selected["settings"]),
                    }
                )
            stop = stopping.update(
                history_candidates, iteration=step, admissible=bool(history_admissible),
                underconstrained=selected_underconstrained, log_line=_log_line,
            )

            summary_flags = f" flags='{selected_flags}'" if selected_flags else ""
            summary_rel = _fmt_float(selected_rel_std)
            summary_std = _fmt_float(selected_max_std, suffix="mm")
            summary_cost = _fmt_float(selected_cost)
            selected_summary_line = (
                f"; selected run={selected_id}{summary_flags} "
                f"anchor_uncertainty_mm={_fmt_float(selected_uncertainty)} cost={summary_cost} "
                f"rel_std={summary_rel} max_std={summary_std}"
            )
            _log_line(selected_summary_line)
            if verbose:
                if m669:
                    _log_console(f"Anchors (M669): {m669}")
                elif anchor_str:
                    _log_console(f"Anchors: {anchor_str}")
                if m666:
                    _log_console(f"; line_model_params (M666): {m666}")
            elif has_variants:
                _log_console(f"; selected run={selected_id}{summary_flags}")
            if not selected_underconstrained:
                if stop.rank is not None:
                    rank_label = "lowest" if stop.rank == 1 else _ordinal(stop.rank) + " lowest"
                    _log_console(f"Anchor uncertainty {_fmt_float(selected_uncertainty, suffix=' mm')}, "
                                 f"the {rank_label} so far.")
                if stop.history_improved:
                    _log_console("Patience reset.")
            if selected_underconstrained:
                _log_console("; selected run hit underconstrained sentinel; continuing to collect more sweeps.")

            if stop.decision == "accept" and stop.good_fit_hit:
                _log_console("; full-auto: stopping after two good HP5 fits "
                             "(range RMS <=0.05 mm, estimated anchor std <=1 mm, agreement <=2 mm).")
            elif stop.decision == "accept" and stop.stable_hit:
                _log_console(
                    "; full-auto: stopping; the estimate has stopped moving "
                    f"(last step {_fmt_float(stop.step_mm, suffix='mm')}, "
                    f"limit {_fmt_float(stopping.stable_step_mm, suffix='mm')})."
                )

            _log_event(
                {
                    "timestamp": datetime.now().isoformat(),
                    "iteration": step,
                    "dataset": str(dataset_path),
                    # Collection units: a sphere mesh counts once, like a sweep.
                    "num_sweeps": len(measurement_groups(plan.get("dataset") or {})),
                    "runs": [
                        {
                            "id": r["id"],
                            "flags": r["flags"],
                            "settings": r["settings"],
                            "metrics": r["metrics"],
                            "optimizer_runs": r["optimizer_runs"],
                        }
                        for r in run_results
                    ],
                    "selected_run": selected_id,
                    "decision": stop.decision,
                    "cost": selected_cost,
                    "anchor_uncertainty_mm": selected_uncertainty,
                    "single_unit_coordinates": selected_summary_meta["single_unit_coordinates"],
                    "uncertainty_compute_seconds": uncertainty_seconds,
                    "uncertainty_rank": stop.rank,
                    "no_improve_count": stopping.no_improve,
                    "patience": stopping.patience,
                    "stable_hit": stop.stable_hit,
                    "good_fit_hit": stop.good_fit_hit,
                    "anchor_step_mm": stop.step_mm,
                    "stable_streak": stopping.stable_streak,
                    "warnings": selected_warnings,
                    "recovery": recovery_info,
                },
            )

            if stop.decision == "accept":
                if stopping.stop_requested():
                    return _accept_best("stop-file")
                stop_reason = stop.reason
                if headless_sim is not None:
                    headless_sim.manifest["stop_reason"] = stop_reason
                summary_plan, summary_meta = _newest_candidate()
                if summary_plan is None:
                    _log_console("; full-auto: no admissible fit available; stopping.")
                    return _finalize(2)
                return _emit_summary_and_send(
                    summary_plan,
                    summary_meta=summary_meta,
                )

            remaining = stopping.remaining

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
                    new_dataset["sweeps"] = list(next_sweep)
                    new_dataset.pop("acquisition_rejections", None)
                    merged = _merge_sweep_datasets(base_dataset, new_dataset)
                    _write_json(work_path, merged)
                    sweeps = merged.get("sweeps", [])
                    count = len(sweeps) if isinstance(sweeps, list) else "?"
                    _log_line(
                        f"; replayed sweep {replay_index}/{len(replay_sweeps)} "
                        f"-> {work_path} sweeps={count}"
                    )
                    _log_console(f"Collected sweep nr {len(measurement_groups(merged))}")
                    continue

            if no_collect:
                _log_console("; --no-collect set; stopping before live collection.")
                summary_plan, summary_meta = _newest_candidate(
                    fallback_plan=plan if isinstance(plan, dict) else None,
                    fallback_meta=selected_summary_meta,
                )
                if summary_plan is None:
                    _log_console("; full-auto: no summary candidate available; stopping.")
                    return _finalize(2)
                return _emit_summary_and_send(summary_plan, summary_meta=summary_meta)

            while True:
                if stopping.stop_requested():
                    return _accept_best("stop-file")
                cmd = plan.get("collect_command")
                if not isinstance(cmd, list) or not cmd:
                    if plan.get("collection_error"):
                        _log_console("; " + str(plan["collection_error"]))
                        return _finalize(2)
                    if _load_json(work_path).get("acquisition_rejections"):
                        return _accept_best("workspace-exhausted")
                    _log_console("; No valid candidate to collect; stopping.")
                    return _finalize(2)

                _log_console("Collecting new sweep to try and beat it.")
                _log_console(f"Still has patience for {remaining} more attempts.")
                _log_line(f"; collecting next sweep ({selected_id})")
                _log_line(f"; running: {' '.join(str(x) for x in cmd)}")
                try:
                    with _log_context():
                        _run_collection_command(cmd)
                except subprocess.CalledProcessError as error:
                    _log_console(f"; collection failed with exit {error.returncode}; stopping without applying calibration")
                    return _finalize(2)
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
                if stopping.stop_requested():
                    return _accept_best("stop-file")
                if new_dataset.get("sweeps"):
                    _log_console(f"Collected sweep nr {len(measurement_groups(merged))}")
                    if collector_output != work_path:
                        collector_output.unlink(missing_ok=True)
                    break
                if not new_dataset.get("acquisition_rejections"):
                    _log_console("; Collector returned neither measurements nor boundary evidence; stopping")
                    return _finalize(2)
                rejected_probe_count += 1
                _log_console("; boundary probe rejected; retaining evidence and planning another sweep without refitting")
                if step + rejected_probe_count >= max_steps:
                    return _accept_best("max-steps")
                collector_output = _unique_path(work_path.with_name(f"{work_path.stem}.new_{step:03d}.json"))
                frozen = {**plan, "dataset": merged}
                plan = plan_ellipse_sweep(
                    frozen, work_path, candidate_count=candidate_count, delta_min=delta_min,
                    delta_max=delta_max, regularization=regularization,
                    min_fixed_delta_spacing_mm=min_fixed_delta_spacing_mm,
                    write_cfg=plan.get("cfg_path"), collector_output=collector_output,
                    collector_args=collector_args_eff, equal_strain=equal_strain,
                )
    except KeyboardInterrupt:
        if applying_solution:
            _log_console("; application interrupted; firmware outcome is unknown, stopping without retry")
            if headless_sim is not None:
                headless_sim.manifest["stop_reason"] = "apply-interrupted"
            return _finalize(130)
        return _accept_best("Ctrl-C")

    if headless_sim is not None:
        headless_sim.manifest["stop_reason"] = "max-steps"
    _log_console(f"; reached max steps; dataset={work_path}")
    summary_plan, summary_meta = _newest_candidate()
    if summary_plan is not None:
        return _emit_summary_and_send(summary_plan, summary_meta=summary_meta)
    return _finalize(0)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_semi_auto_parser()
    parser.add_argument("--extended-recording", "--recording", "--record", action="store_true",
                        help='Record simulated collection beside the dataset.')
    parser.add_argument("--extended-reference-ws", metavar="URL",
                        help="Record timestamped logs and collector events in an --extended-reference flight recorder")
    parser.add_argument("--extended-reference", action="store_true",
                        help="Record headless physics and all autocal events in one RRD; starts an owned recorder unless a WebSocket URL is supplied")
    parser.add_argument("--recording-mode", choices=['none', 'events', 'sampled', 'full'],
                        help='Headless evidence: events, sampled physics, full physics, or none.')
    parser.add_argument("--headless-sim", action="store_true",
                        help='Collect in the headless production simulator; implies --sim')
    args = parser.parse_args(argv)
    try:
        args.fit = resolve_fit(args.shape, args.fit)
    except ValueError as error:
        parser.error(str(error))
    if not np.isfinite(args.stable_step_mm) or args.stable_step_mm < 0:
        parser.error("--stable-step-mm must be a finite nonnegative distance")
    if args.project_zero_tension and not args.no_collect:
        parser.error("--project-zero-tension collection was retired; use equal-strain collection with home-offset fitting. Historical projected datasets can still be replayed with --no-collect.")
    if args.extended_recording:
        if args.no_collect or not (args.sim or args.headless_sim):
            parser.error("--extended-recording requires simulated collection (--sim or --headless-sim)")
        if args.headless_sim:
            args.extended_reference = True
    if args.recording_mode is not None:
        if not args.headless_sim or args.no_collect:
            parser.error("--recording-mode requires --headless-sim collection")
        if args.extended_reference or args.extended_reference_ws or extended_reference.enabled():
            parser.error("--recording-mode cannot be combined with extended-reference flags or AUTOCAL_REFERENCE_WS")
    if args.extended_reference_ws:
        os.environ['AUTOCAL_REFERENCE_WS'] = args.extended_reference_ws
    if args.extended_reference and not args.headless_sim:
        parser.error("--extended-reference requires --headless-sim; use --extended-reference-ws for browser collection")
    if args.extended_reference and args.no_collect and not extended_reference.enabled():
        parser.error("--extended-reference requires collection; use --extended-reference-ws to record offline fitting events")
    from autocal.headless_sim import HeadlessSimulation
    if args.headless_sim:
        if args.firmware != "rrf":
            parser.error("--headless-sim requires --firmware rrf")
        try:
            catalogued_machine(args.machine_type)
        except ValueError as error:
            parser.error(f"No headless scene/configuration for {args.machine_type}: {error}")
        args.sim = True
        if args.dataset is None:
            args.dataset = Path("autocal/data/default_dataset.json")
    _apply_optimizer_mode_env(str(args.optimizer_mode))
    if args.dataset is not None and (error := bootstrap_state_error(args.dataset)):
        parser.error(error)
    if args.dataset is not None and args.dataset.exists() and not args.no_collect:
        if error := profile_policy_error(_load_json(args.dataset)):
            parser.error(error)
    spool_opts = _resolve_spool_cli_options(parser, args)
    machine_type = _normalize_machine_type(str(args.machine_type))
    full_auto_runs = list(args.full_auto_run or [])
    if bool(args.shotgun):
        try:
            full_auto_runs.extend(_load_shotgun_runs())
        except FileNotFoundError as exc:
            parser.error(str(exc))
    collector_args = _clean_collector_args(args.collector_args)
    if args.solver_iterations is None and args.sim:
        args.solver_iterations = autocal_solver_iterations(args.machine_type)
    if args.solver_iterations is not None and not _arg_has_flag(collector_args, "--solver-iterations"):
        collector_args.extend(["--solver-iterations", str(args.solver_iterations)])
    if args.physics_substeps is not None and not _arg_has_flag(collector_args, "--physics-substeps"):
        collector_args.extend(["--physics-substeps", str(args.physics_substeps)])
    if args.open_loop_motors and "--open-loop-motors" not in collector_args:
        collector_args.append("--open-loop-motors")
    if args.speedup is not None and not _arg_has_flag(collector_args, "--speedup"):
        speedup_arg = (
            str(int(args.speedup))
            if float(args.speedup).is_integer()
            else str(args.speedup)
        )
        collector_args.extend(["--speedup", speedup_arg])
    if bool(args.project_zero_tension) and not _arg_has_flag(
        collector_args, "--project-zero-tension", "--projectZeroTension"
    ):
        collector_args.append("--project-zero-tension")
    if bool(args.debug_sweep_actions) and not _arg_has_flag(collector_args, "--debug-sweep-actions"):
        collector_args.append("--debug-sweep-actions")
    if args.no_collect and not (args.dataset or Path("autocal/data/default_dataset.json")).exists():
        parser.error("--no-collect needs an existing dataset; pass --dataset to one that exists")
    if args.dataset is not None:
        args.dataset.parent.mkdir(parents=True, exist_ok=True)
    context = HeadlessSimulation(args, collector_args) if args.headless_sim and not args.no_collect else nullcontext()
    with ExitStack() as stack:
        if args.extended_recording and not args.headless_sim and not extended_reference.enabled():
            from autocal.browser_recording import browser_recording
            stack.enter_context(browser_recording(args.dataset))
        headless = stack.enter_context(context)
        if headless is not None:
            print(f"; headless simulator manifest: {headless.directory / 'manifest.json'}")
        result = _run_full_auto(args, spool_opts, machine_type, full_auto_runs, collector_args, headless)
        if headless is not None and result:
            headless.manifest["exit_code"] = result
        return result


def _run_full_auto(args, spool_opts, machine_type, full_auto_runs, collector_args, headless):
    solve_optimizer = str(args.solve_optimizer)
    if solve_optimizer == "lbfgsb" and bool(args.trf_solver_fallback):
        solve_optimizer = "lbfgsb+trf-fallback"
    return full_auto_loop(
        work_dataset=args.dataset,
        machine_type=str(machine_type),
        max_steps=int(args.max_steps),
        solve_restarts=int(args.solve_restarts),
        solve_iterations=int(args.solve_iterations),
        solve_optimizer=solve_optimizer,
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
        theta0_mode=str(spool_opts["theta0_mode"]),
        line_width=float(spool_opts["line_width"]),
        sigma_floor_mm=(None if spool_opts.get("sigma_floor_mm") is None else float(spool_opts["sigma_floor_mm"])),
        fit_home_offsets=bool(args.fit_home_offsets),
        fit_force_ripple=bool(args.fit_force_ripple),
        fit=args.fit,
        equal_strain=bool(args.equal_strain),
        candidate_count=int(args.candidate_count),
        delta_min=args.delta_min,
        delta_max=args.delta_max,
        regularization=float(args.regularization),
        min_fixed_delta_spacing_mm=float(args.min_fixed_delta_spacing_mm),
        shape=str(args.shape),
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
        stable_step_mm=float(args.stable_step_mm),
        verbose=bool(args.verbose),
        filter_schedule=spool_opts.get("filter_schedule"),
        objective_schedule=spool_opts.get("objective_schedule"),
        no_collect=bool(args.no_collect),
        sparse_recovery=bool(args.sparse_recovery),
        firmware=str(args.firmware),
        config=args.config,
        rrf_config=args.rrf_config,
        klipper_config=args.klipper_config,
        headless_sim=headless,
        stage_artifacts=args.stage_artifacts,
    )


if __name__ == "__main__":
    def interrupt(signum, frame):
        raise SystemExit(128 + signum)

    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupt)
    try:
        exit_code = main()
    except KeyboardInterrupt:
        print("; interrupted; completed measurements and partial-point journals retained", file=sys.stderr)
        exit_code = 130
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        print(f"; autocal failed: {exc}", file=sys.stderr)
        exit_code = 1
    raise SystemExit(exit_code)
