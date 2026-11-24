"""Local execution runner and parallel CPU scheduler for hp-sim5 experiments."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor, as_completed
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from typing import Any, Sequence

from .firmware import stop_process_tree
from .interfaces import ArtifactStore, ExperimentResult, ExperimentSpec
from .provenance import git_provenance
from .storage import (
    ArtifactUploadError,
    DEFAULT_EXCLUDE_GLOBS,
    DriveMountArtifactStore,
    GoogleDriveArtifactStore,
    LOCAL_STORE_DIR,
    ScratchWorkspace,
    UPLOAD_PENDING_DIR,
    retry_upload_command,
    utc_now_iso,
    write_upload_pending,
)


def _artifact_exclude_globs(spec: ExperimentSpec, status: str) -> tuple[str, ...]:
    """Skip multi-tens-of-MB headless event logs on successful non-recording runs."""
    globs = list(DEFAULT_EXCLUDE_GLOBS)
    if (
        status == "ok"
        and spec.recording_mode == "none"
        and not spec.metadata.get("keep_events_log")
    ):
        globs.append("*.headless*/events.jsonl")
    return tuple(globs)


def _default_python(repo_root: Path) -> str:
    venv_py = repo_root / ".venv/bin/python"
    if venv_py.is_file():
        return str(venv_py)
    return sys.executable


def _build_env(spec: ExperimentSpec, scratch: Path) -> dict[str, str]:
    keep = ("PATH", "HOME", "LANG", "LC_ALL", "LD_LIBRARY_PATH", "TMPDIR", "NODE_PATH")
    env = {k: os.environ[k] for k in keep if k in os.environ}
    threads = str(max(1, int(spec.threads)))
    env.update(
        PYTHONUNBUFFERED="1",
        MPLCONFIGDIR=str(scratch / ".matplotlib"),
        OMP_NUM_THREADS=threads,
        OPENBLAS_NUM_THREADS=threads,
        MKL_NUM_THREADS=threads,
        NUMEXPR_NUM_THREADS=threads,
        XLA_FLAGS=f"--xla_cpu_multi_thread_eigen={'true' if spec.threads > 1 else 'false'} "
                  f"intra_op_parallelism_threads={threads}",
        HP_SIM5_AUTOCAL_JOB="1",
        HP_SIM5_PHYSICS_SOLVER=spec.solver,
        HP_SIM5_SCRATCH_DIR=str(scratch),
    )
    if spec.max_physics_steps is not None:
        env["HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS"] = str(int(spec.max_physics_steps))
    env.update({str(k): str(v) for k, v in spec.env.items()})
    return env


def _build_command(repo_root: Path, spec: ExperimentSpec, scratch: Path, python_exe: str) -> list[str]:
    if spec.kind == "autocal":
        dataset = scratch / "sweeps.json"
        cmd = [
            python_exe,
            "autocal/autocal.py",
            "--headless-sim",
            "--machine-type",
            spec.machine,
            "--dataset",
            str(dataset),
            "--recording-mode",
            spec.recording_mode,
            *spec.args,
        ]
        if spec.collector_args:
            cmd.extend(["--collector-args", *spec.collector_args])
        return cmd

    if spec.kind == "sweep_collection":
        # Runs one headless collection script using HeadlessSimulation + collect_sweep_data.mjs.
        helper = "\n".join([
            "import json, subprocess, sys",
            "from argparse import Namespace",
            "from pathlib import Path",
            "sys.path.insert(0, '.')",
            "from autocal.headless_sim import HeadlessSimulation",
            "cfg = json.loads(sys.argv[1])",
            "scratch = Path(cfg['scratch'])",
            "ds = scratch / 'sweeps.json'",
            "c_args = list(cfg['collector_args'])",
            "if cfg.get('sweep_config_text'):",
            "    cfg_file = scratch / 'sweep_config.txt'",
            "    cfg_file.write_text(cfg['sweep_config_text'], encoding='utf-8')",
            "    c_args = ['--sweep-config-file', str(cfg_file), *c_args]",
            "args = Namespace(",
            "    machine_type=cfg['machine'], find_radii='off', find_buildup_factor='off',",
            "    rrf_config=None, config=None, dataset=ds, keep_sim_alive=False,",
            "    recording_mode=cfg['recording_mode'], extended_reference=False, buildup_factor=cfg.get('buildup_factor'),",
            ")",
            "with HeadlessSimulation(args, c_args) as _sim:",
            "    cmd = ['node', 'autocal/control/cli/collect_sweep_data.mjs', '--machineType', cfg['machine'],",
            "           '--output-file', str(ds), '--sim', '--firmware', 'rrf', *c_args]",
            "    rc = subprocess.run(cmd).returncode",
            "sys.exit(rc)",
        ])
        payload = json.dumps({
            "scratch": str(scratch),
            "machine": spec.machine,
            "recording_mode": spec.recording_mode,
            "sweep_config_text": spec.metadata.get("sweep_config_text"),
            "buildup_factor": spec.metadata.get("buildup_factor"),
            "collector_args": [*spec.args, *spec.collector_args],
        })
        return [python_exe, "-c", helper, payload]

    if spec.kind == "command":
        if not spec.command:
            raise ValueError("ExperimentSpec(kind='command') requires a non-empty command list")
        mapping = {
            "scratch": str(scratch),
            "repo_root": str(repo_root),
            "python": python_exe,
            "experiment_id": spec.experiment_id,
            "machine": spec.machine,
        }
        return [token.format(**mapping) for token in spec.command]

    raise ValueError(f"Unsupported ExperimentSpec kind: {spec.kind!r}")


def _extract_autocal_metrics(repo_root: Path, scratch: Path) -> tuple[float | None, int | None, dict[str, Any]]:
    simulated_s: float | None = None
    total_steps: int | None = None
    metrics: dict[str, Any] = {}
    headless_manifest_path = scratch / "sweeps.headless/manifest.json"
    if headless_manifest_path.is_file():
        hm = json.loads(headless_manifest_path.read_text(encoding="utf-8"))
        simulated_s = hm.get("simulated_s")
        total_steps = hm.get("total_steps")
        for key in ("stop_reason", "normal_completion", "physics_engine", "service_wall_s", "epoch"):
            if key in hm:
                metrics[key] = hm[key]
    dataset = scratch / "sweeps.json"
    log_jsonl = scratch / "sweeps.full_auto_log.jsonl"
    if dataset.is_file() and headless_manifest_path.is_file() and log_jsonl.is_file():
        try:
            if str(repo_root) not in sys.path:
                sys.path.insert(0, str(repo_root))
            from autocal.tools.autocal_benchmark import benchmark_metrics

            bm = benchmark_metrics(dataset, repo_root)
            metrics.update({k: v for k, v in bm.items() if v is not None})
        except Exception as err:
            metrics["scoring_note"] = str(err)
    elif dataset.is_file():
        try:
            data = json.loads(dataset.read_text(encoding="utf-8"))
            sweeps = data.get("sweeps", [])
            metrics["sweeps"] = len(sweeps)
            metrics["points"] = sum(len(s.get("data_points", [])) for s in sweeps)
        except Exception:
            pass
    return simulated_s, total_steps, metrics


def run_experiment_spec(
    repo_root: Path | str,
    spec: ExperimentSpec,
    *,
    scratch_base: Path | None = None,
    store: ArtifactStore | None = None,
    keep_scratch: bool = False,
    python_exe: str | None = None,
    backend_label: str = "local-cpu",
    raise_on_upload_error: bool = True,
) -> ExperimentResult:
    """Execute one ExperimentSpec on local scratch, then sync artifacts to store if given.

    If persisting to `store` fails, the local scratch directory is always preserved
    with an `upload_pending.json` file so the experiment is never lost.
    """
    root = Path(repo_root).resolve()
    py = python_exe or _default_python(root)
    started = time.monotonic()
    provenance = git_provenance(root)
    if scratch_base is None and isinstance(store, (GoogleDriveArtifactStore, DriveMountArtifactStore)):
        scratch_base = root / LOCAL_STORE_DIR / UPLOAD_PENDING_DIR

    with ScratchWorkspace(spec.experiment_id, base_dir=scratch_base, keep=keep_scratch) as ws:
        assert ws.path is not None
        scratch = ws.path
        env = _build_env(spec, scratch)
        cmd = _build_command(root, spec, scratch, py)
        (scratch / "spec.json").write_text(json.dumps(spec.to_dict(), indent=2, sort_keys=True) + "\n", encoding="utf-8")

        status = "failed"
        exit_code = 1
        error_msg: str | None = None
        process: subprocess.Popen | None = None
        stdout_path = scratch / "stdout.log"
        stderr_path = scratch / "stderr.log"

        try:
            with stdout_path.open("w", encoding="utf-8") as out_f, stderr_path.open("w", encoding="utf-8") as err_f:
                process = subprocess.Popen(
                    cmd,
                    cwd=root,
                    env=env,
                    stdin=subprocess.DEVNULL,
                    stdout=out_f,
                    stderr=err_f,
                    start_new_session=True,
                )
                try:
                    exit_code = process.wait(timeout=spec.timeout_s)
                    status = "ok" if exit_code == 0 else "failed"
                except subprocess.TimeoutExpired:
                    status = "timeout"
                    exit_code = 124
                    error_msg = f"Experiment timed out after {spec.timeout_s:g}s"
                    stop_process_tree(process, timeout_s=10.0)
        except KeyboardInterrupt:
            status = "cancelled"
            exit_code = 130
            error_msg = "Interrupted"
            stop_process_tree(process, timeout_s=5.0)
        except Exception as err:
            status = "failed"
            exit_code = 1
            error_msg = f"{type(err).__name__}: {err}"
            stop_process_tree(process, timeout_s=5.0)
        finally:
            stop_process_tree(process, timeout_s=5.0)

        wall_s = time.monotonic() - started
        simulated_s, total_steps, metrics = _extract_autocal_metrics(root, scratch)
        if status == "failed" and not error_msg:
            tail = (stderr_path.read_text(encoding="utf-8", errors="replace")[-1200:] or
                    stdout_path.read_text(encoding="utf-8", errors="replace")[-1200:]).strip()
            error_msg = tail or f"Process exited with code {exit_code}"

        result = ExperimentResult(
            experiment_id=spec.experiment_id,
            kind=spec.kind,
            status=status,
            exit_code=exit_code,
            backend=backend_label,
            wall_s=round(wall_s, 3),
            simulated_s=simulated_s,
            total_steps=total_steps,
            metrics=metrics,
            scratch_dir=str(scratch) if keep_scratch else None,
            error=error_msg,
        )
        provenance["finished_at"] = utc_now_iso()
        result_path = scratch / "result.json"
        result_path.write_text(
            json.dumps({**result.to_dict(), **provenance}, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

        if store is not None:
            meta = {
                **spec.metadata,
                **provenance,
                "kind": spec.kind,
                "machine": spec.machine,
                "status": status,
                "backend": backend_label,
                "solver": spec.solver,
                "recording_mode": spec.recording_mode,
                "tags": list(spec.tags),
                "wall_s": result.wall_s,
                "simulated_s": simulated_s,
                "total_steps": total_steps,
                "metrics": metrics,
            }
            exclude_globs = _artifact_exclude_globs(spec, status)
            try:
                manifest = store.put_directory(
                    spec.experiment_id,
                    scratch,
                    metadata=meta,
                    exclude_globs=exclude_globs,
                )
            except Exception as err:
                ws.preserve()
                err_text = f"{type(err).__name__}: {err}"
                write_upload_pending(
                    scratch,
                    experiment_id=spec.experiment_id,
                    store_uri=store.uri,
                    metadata=meta,
                    error=err_text,
                )
                retry_cmd = retry_upload_command(spec.experiment_id, scratch, store.uri)
                result.scratch_dir = str(scratch)
                result.artifact_uri = None
                result.metrics["experiment_status"] = status
                result.metrics["upload_error"] = err_text
                result.error = f"{result.error}\nArtifact upload failed: {err_text}" if result.error else f"Artifact upload failed: {err_text}"
                if not raise_on_upload_error:
                    result.status = "upload_failed"
                result_path.write_text(
                    json.dumps({**result.to_dict(), **provenance}, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8",
                )
                print(
                    f"[hp_sim5_sim] WARNING: Artifact upload failed for {spec.experiment_id} -> {store.uri}: {err_text}\n"
                    f"[hp_sim5_sim] Preserved local scratch at: {scratch}\n"
                    f"[hp_sim5_sim] Retry upload with: {retry_cmd}",
                    file=sys.stderr,
                    flush=True,
                )
                if raise_on_upload_error:
                    raise ArtifactUploadError(
                        f"Artifact upload failed for {spec.experiment_id} -> {store.uri}: {err_text}. "
                        f"Preserved scratch at {scratch}. Retry with: {retry_cmd}",
                        experiment_id=spec.experiment_id,
                        scratch_dir=scratch,
                        store_uri=store.uri,
                        result=result,
                    ) from err
                return result

            result.manifest = manifest
            result.artifact_uri = f"{store.uri}/experiments/{spec.experiment_id}"

        return result


class LocalScheduler:
    """Schedules single or parallel CPU experiments on the current host."""

    def __init__(
        self,
        repo_root: Path | str,
        *,
        scratch_base: Path | None = None,
        python_exe: str | None = None,
        backend_label: str = "local-cpu",
    ) -> None:
        self.repo_root = Path(repo_root).resolve()
        self.scratch_base = Path(scratch_base).resolve() if scratch_base else None
        self.python_exe = python_exe
        self._backend_label = backend_label

    @property
    def backend_name(self) -> str:
        return self._backend_label

    def run(
        self,
        spec: ExperimentSpec,
        *,
        store: ArtifactStore | None = None,
        keep_scratch: bool = False,
        raise_on_upload_error: bool = True,
    ) -> ExperimentResult:
        return run_experiment_spec(
            self.repo_root,
            spec,
            scratch_base=self.scratch_base,
            store=store,
            keep_scratch=keep_scratch,
            python_exe=self.python_exe,
            backend_label=self._backend_label,
            raise_on_upload_error=raise_on_upload_error,
        )

    def run_batch(
        self,
        specs: Sequence[ExperimentSpec],
        *,
        max_workers: int | None = None,
        store: ArtifactStore | None = None,
        keep_scratch: bool = False,
        raise_on_upload_error: bool = True,
    ) -> list[ExperimentResult]:
        spec_list = list(specs)
        if not spec_list:
            return []
        if store is not None and hasattr(store, "ensure_experiments_folder"):
            try:
                store.ensure_experiments_folder()  # type: ignore[attr-defined]
            except Exception:
                # Let per-experiment upload handling preserve scratch if Drive is unreachable.
                pass
        if max_workers is None:
            cpus = os.cpu_count() or 2
            max_threads = max(1, max(int(s.threads) for s in spec_list))
            # With RRF idle-spin fixed, one headless sim uses ~0.95 cores.
            max_workers = max(1, min(len(spec_list), cpus // max_threads))
        else:
            max_workers = max(1, min(len(spec_list), int(max_workers)))

        if max_workers == 1:
            return [
                self.run(s, store=store, keep_scratch=keep_scratch, raise_on_upload_error=raise_on_upload_error)
                for s in spec_list
            ]

        results_by_index: dict[int, ExperimentResult] = {}
        first_error: Exception | None = None
        with ThreadPoolExecutor(max_workers=max_workers) as pool:
            futures = {
                pool.submit(
                    self.run,
                    spec,
                    store=store,
                    keep_scratch=keep_scratch,
                    raise_on_upload_error=raise_on_upload_error,
                ): idx
                for idx, spec in enumerate(spec_list)
            }
            for fut in as_completed(futures):
                idx = futures[fut]
                try:
                    results_by_index[idx] = fut.result()
                except Exception as err:
                    if first_error is None:
                        first_error = err
        if first_error is not None:
            raise first_error
        return [results_by_index[i] for i in range(len(spec_list))]
