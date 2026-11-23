#!/usr/bin/env python3
"""Unified CLI for running hp-sim5 headless experiments locally or on Google Colab,
with persistent artifact storage locally or in Google Drive.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/python"))
sys.path.insert(0, str(ROOT))

from hp_sim5_sim import (  # noqa: E402
    ArtifactUploadError,
    ColabConfig,
    ColabScheduler,
    ColabShutdownError,
    ExperimentSpec,
    LocalScheduler,
    resolve_colab_cli,
    store_from_uri,
)
from hp_sim5_sim.storage import DriveAuthError, GoogleDriveArtifactStore, UPLOAD_PENDING_NAME  # noqa: E402


def _resolve_store(uri: str | None):
    effective = uri or os.environ.get("HP_SIM5_ARTIFACT_STORE")
    return store_from_uri(effective) if effective else None


def cmd_run(args: argparse.Namespace) -> int:
    spec = ExperimentSpec(
        kind=args.kind,
        machine=args.machine,
        experiment_id=args.id or ExperimentSpec(kind=args.kind).experiment_id,
        args=list(args.exp_args or []),
        collector_args=list(args.collector_args or []),
        command=list(args.command or []),
        recording_mode=args.recording_mode,
        solver=args.solver,
        timeout_s=args.timeout,
        max_physics_steps=args.max_physics_steps,
        threads=args.threads,
        tags=list(args.tag or []),
    )
    store = _resolve_store(args.store)
    if args.backend == "local":
        scheduler = LocalScheduler(ROOT)
    else:
        cfg = ColabConfig(
            gpu=args.gpu,
            tpu=args.tpu,
            high_mem=args.high_mem,
            session_name=args.session,
            keep_session=args.keep_session,
            remote_store_uri=args.remote_store,
            direct_drive_upload=args.direct_drive,
        )
        scheduler = ColabScheduler(ROOT, config=cfg)

    result = scheduler.run(spec, store=store, keep_scratch=args.keep_scratch)
    print(json.dumps(result.to_dict(), indent=2, sort_keys=True))
    return 0 if result.status == "ok" else (result.exit_code or 1)


def cmd_batch(args: argparse.Namespace) -> int:
    raw = json.loads(Path(args.manifest).read_text(encoding="utf-8"))
    items = raw["experiments"] if isinstance(raw, dict) else raw
    specs = [ExperimentSpec.from_dict(item) for item in items]
    store = _resolve_store(args.store)

    if args.backend == "local":
        scheduler = LocalScheduler(ROOT)
    else:
        cfg = ColabConfig(
            gpu=args.gpu,
            tpu=args.tpu,
            high_mem=args.high_mem,
            session_name=args.session,
            keep_session=args.keep_session,
            remote_workers=args.workers,
            remote_store_uri=args.remote_store,
            direct_drive_upload=args.direct_drive,
        )
        scheduler = ColabScheduler(ROOT, config=cfg)

    results = scheduler.run_batch(specs, max_workers=args.workers, store=store, keep_scratch=args.keep_scratch)
    payload = [r.to_dict() for r in results]
    if args.output:
        Path(args.output).write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if all(r.status == "ok" for r in results) else 1


def cmd_artifacts(args: argparse.Namespace) -> int:
    store = _resolve_store(args.store)
    if store is None:
        raise SystemExit("Pass --store URI or set HP_SIM5_ARTIFACT_STORE")

    if args.action == "list":
        rows = store.list_experiments(tags=args.tag or (), kind=args.kind, limit=args.limit)
        print(json.dumps(rows, indent=2, sort_keys=True))
        return 0
    if args.action == "manifest":
        print(json.dumps(store.get_manifest(args.experiment_id), indent=2, sort_keys=True))
        return 0
    if args.action == "push":
        source_dir = Path(args.source)
        meta: dict[str, Any] = {}
        pending_file = source_dir / UPLOAD_PENDING_NAME
        if pending_file.is_file():
            try:
                pending_data = json.loads(pending_file.read_text(encoding="utf-8"))
                if isinstance(pending_data.get("metadata"), dict):
                    meta.update(pending_data["metadata"])
            except ValueError:
                pass
        elif (source_dir / "result.json").is_file():
            try:
                res_data = json.loads((source_dir / "result.json").read_text(encoding="utf-8"))
                for key in ("kind", "status", "backend", "wall_s", "simulated_s", "total_steps", "metrics"):
                    if res_data.get(key) is not None:
                        meta[key] = res_data[key]
            except ValueError:
                pass
        if args.tag:
            merged_tags = list(meta.get("tags") or [])
            for tag in args.tag:
                if tag not in merged_tags:
                    merged_tags.append(tag)
            meta["tags"] = merged_tags
        else:
            meta.setdefault("tags", [])
        if args.kind:
            meta["kind"] = args.kind
        if args.machine:
            meta["machine"] = args.machine
        manifest = store.put_directory(args.experiment_id, source_dir, metadata=meta)
        pending_file.unlink(missing_ok=True)
        print(json.dumps(manifest, indent=2, sort_keys=True))
        return 0
    if args.action == "get":
        dest = store.get_experiment(
            args.experiment_id,
            Path(args.dest),
            rel_paths=args.file or None,
        )
        print(json.dumps({"experiment_id": args.experiment_id, "dest": str(dest.resolve())}, indent=2))
        return 0
    raise SystemExit(f"Unknown artifacts action: {args.action}")


def cmd_doctor(_args: argparse.Namespace) -> int:
    report: dict[str, Any] = {
        "repo_root": str(ROOT),
        "cpus": os.cpu_count(),
        "python": sys.executable,
        "node": shutil.which("node"),
        "rrf_simulator": (ROOT / "RRF/build/rrf_simulator").is_file(),
        "native_physics": (ROOT / "src/wasm/cable_solver/target/native/physics.node").is_file(),
        "wasm_physics": (ROOT / "public/wasm/cable_solver.wasm").is_file(),
    }
    try:
        colab_cmd = resolve_colab_cli()
        report["colab_cli"] = colab_cmd
        p = subprocess.run([*colab_cmd, "sessions"], capture_output=True, text=True, timeout=20)
        report["colab_auth_ok"] = p.returncode == 0
        if p.returncode != 0:
            report["colab_error"] = (p.stderr or p.stdout).strip()[-400:]
    except Exception as err:
        report["colab_cli"] = None
        report["colab_auth_ok"] = False
        report["colab_error"] = str(err)

    try:
        drive_store = GoogleDriveArtifactStore()
        _, _, body = drive_store._request(
            "GET",
            f"{drive_store.api_base}/about?fields=user(emailAddress),storageQuota(limit,usage)",
        )
        report["drive_auth_ok"] = True
        report["drive_about"] = json.loads(body.decode("utf-8"))
    except DriveAuthError as err:
        report["drive_auth_ok"] = False
        report["drive_error"] = str(err)
    except Exception as err:
        report["drive_auth_ok"] = False
        report["drive_error"] = f"{type(err).__name__}: {err}"

    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="subcommand", required=True)

    def add_compute_flags(p: argparse.ArgumentParser) -> None:
        p.add_argument("--backend", choices=("local", "colab"), default="local")
        p.add_argument("--store", help="Artifact store URI (local://..., gdrive://..., drivemount://...)")
        p.add_argument("--remote-store", help="Optional store URI writable directly from Colab (e.g. drivemount://...)")
        p.add_argument(
            "--direct-drive",
            action=argparse.BooleanOptionalAction,
            default=False,
            help="For Colab + gdrive://, upload directly from Colab using a short-lived access token (default: stage via this host)",
        )
        p.add_argument("--gpu", help="Colab GPU type (T4, L4, A100, H100)")
        p.add_argument("--tpu", help="Colab TPU type (v5e1, v6e1)")
        p.add_argument("--high-mem", action=argparse.BooleanOptionalAction, default=True, help="Request high-RAM Colab VM")
        p.add_argument("--session", help="Explicit Colab session name")
        p.add_argument("--keep-session", action="store_true", help="Do not stop Colab VM after completion")
        p.add_argument("--keep-scratch", action="store_true", help="Keep local scratch directory")

    run_p = sub.add_parser("run", help="Run one headless experiment locally or on Colab")
    add_compute_flags(run_p)
    run_p.add_argument("--kind", choices=("autocal", "sweep_collection", "command"), default="autocal")
    run_p.add_argument("--machine", default="hangprinter_3")
    run_p.add_argument("--id", help="Explicit experiment ID")
    run_p.add_argument("--recording-mode", choices=("none", "events", "sampled", "full"), default="none")
    run_p.add_argument("--solver", choices=("native", "wasm"), default="native")
    run_p.add_argument("--timeout", type=float, default=3600.0)
    run_p.add_argument("--max-physics-steps", type=int)
    run_p.add_argument("--threads", type=int, default=1)
    run_p.add_argument("--tag", action="append", default=[])
    run_p.add_argument("--exp-args", nargs="*", default=[], help="Extra arguments for autocal/sweep_collection")
    run_p.add_argument("--collector-args", nargs="*", default=[], help="Extra --collector-args for autocal")
    run_p.add_argument("--command", nargs="*", default=[], help="Explicit argv for --kind command")
    run_p.set_defaults(func=cmd_run)

    batch_p = sub.add_parser("batch", help="Run a JSON manifest of experiments in parallel locally or on Colab")
    add_compute_flags(batch_p)
    batch_p.add_argument("manifest", help="Path to JSON list or {experiments: [...]} file")
    batch_p.add_argument("--workers", type=int, help="Parallel worker count")
    batch_p.add_argument("--output", help="Optional path to write JSON results")
    batch_p.set_defaults(func=cmd_batch)

    art_p = sub.add_parser("artifacts", help="Manage artifacts in local or Google Drive storage")
    art_p.add_argument("--store", help="Artifact store URI")
    art_sub = art_p.add_subparsers(dest="action", required=True)
    ls_p = art_sub.add_parser("list")
    ls_p.add_argument("--kind")
    ls_p.add_argument("--tag", action="append", default=[])
    ls_p.add_argument("--limit", type=int, default=50)
    man_p = art_sub.add_parser("manifest")
    man_p.add_argument("experiment_id")
    push_p = art_sub.add_parser("push")
    push_p.add_argument("experiment_id")
    push_p.add_argument("source")
    push_p.add_argument("--kind")
    push_p.add_argument("--machine")
    push_p.add_argument("--tag", action="append", default=[])
    get_p = art_sub.add_parser("get")
    get_p.add_argument("experiment_id")
    get_p.add_argument("dest")
    get_p.add_argument("--file", action="append", default=[], help="Optional relative file(s) to fetch")
    art_p.set_defaults(func=cmd_artifacts)

    doc_p = sub.add_parser("doctor", help="Inspect local tools, Colab auth, and Google Drive auth")
    doc_p.set_defaults(func=cmd_doctor)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return int(args.func(args))
    except (ArtifactUploadError, ColabShutdownError) as err:
        print(f"ERROR: {err}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
