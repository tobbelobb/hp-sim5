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
from urllib.parse import urlparse

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/python"))
sys.path.insert(0, str(ROOT))

from hp_sim5_sim import (  # noqa: E402
    ArtifactUploadError,
    ColabConfig,
    ColabScheduler,
    ColabShutdownError,
    ExperimentSpec,
    LocalArtifactStore,
    LocalScheduler,
    resolve_colab_cli,
    store_from_uri,
)
from hp_sim5_sim.experiment_lookup import directory_metadata, local_runs  # noqa: E402
from hp_sim5_sim.storage import (  # noqa: E402
    DEFAULT_DRIVE_URI,
    DriveAuthError,
    GoogleDriveArtifactStore,
    LOCAL_STORE_DIR,
    MANIFEST_NAME,
    UPLOAD_PENDING_NAME,
    default_adc_path,
)

DEFAULT_LOCAL_STORE = ROOT / LOCAL_STORE_DIR


def _resolve_store(uri: str | None):
    effective = uri or os.environ.get("HP_SIM5_ARTIFACT_STORE")
    return store_from_uri(effective) if effective else None


def _compute_target(args: argparse.Namespace):
    if args.discard and (args.store or args.remote_store or args.keep_scratch or args.direct_drive):
        raise ValueError("--discard conflicts with explicit artifact storage and --keep-scratch")
    store = None if args.discard else _resolve_store(args.store)
    if args.direct_drive and not (
        args.backend == "colab" and isinstance(store, GoogleDriveArtifactStore) and not args.remote_store
    ):
        raise ValueError("--direct-drive requires --backend colab and --store gdrive://...")
    if args.backend == "local":
        if store is None and not (args.discard or args.keep_scratch):
            store = LocalArtifactStore(DEFAULT_LOCAL_STORE)
        return LocalScheduler(ROOT), store
    cfg = ColabConfig(
        gpu=args.gpu,
        tpu=args.tpu,
        high_mem=args.high_mem,
        session_name=args.session,
        keep_session=args.keep_session,
        remote_workers=getattr(args, "workers", None),
        remote_store_uri=args.remote_store,
        direct_drive_upload=args.direct_drive,
        persist_by_default=not args.discard,
    )
    return ColabScheduler(ROOT, config=cfg), store


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
        metadata={"summary": args.summary} if args.summary else {},
    )
    scheduler, store = _compute_target(args)
    result = scheduler.run(spec, store=store, keep_scratch=args.keep_scratch)
    print(json.dumps(result.to_dict(), indent=2, sort_keys=True))
    return 0 if result.status == "ok" else (result.exit_code or 1)


def cmd_batch(args: argparse.Namespace) -> int:
    raw = json.loads(Path(args.manifest).read_text(encoding="utf-8"))
    items = raw["experiments"] if isinstance(raw, dict) else raw
    specs = [ExperimentSpec.from_dict(item) for item in items]
    scheduler, store = _compute_target(args)
    results = scheduler.run_batch(specs, max_workers=args.workers, store=store, keep_scratch=args.keep_scratch)
    payload = [r.to_dict() for r in results]
    if args.output:
        Path(args.output).write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if all(r.status == "ok" for r in results) else 1


def cmd_artifacts(args: argparse.Namespace) -> int:
    store = _resolve_store(args.store)
    if store is None:
        store = LocalArtifactStore(DEFAULT_LOCAL_STORE)
        if args.action in ("manifest", "get") and not store.exists(args.experiment_id):
            previous = LocalArtifactStore(store.experiments_dir)
            if previous.exists(args.experiment_id):
                store = previous

    if args.action == "list":
        rows = store.list_experiments(tags=args.tag or (), kind=args.kind, limit=args.limit)
        print(json.dumps(rows, indent=2, sort_keys=True))
        return 0
    if args.action == "manifest":
        print(json.dumps(store.get_manifest(args.experiment_id), indent=2, sort_keys=True))
        return 0
    if args.action == "push":
        source_dir = Path(args.source)
        meta = directory_metadata(source_dir)
        pending_file = source_dir / UPLOAD_PENDING_NAME
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
        if args.summary is not None:
            meta["summary"] = args.summary
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


def cmd_find(args: argparse.Namespace) -> int:
    stores = [LocalArtifactStore(DEFAULT_LOCAL_STORE)]
    previous = LocalArtifactStore(stores[0].experiments_dir)
    if previous.experiments_dir.is_dir():
        stores.append(previous)
    error = False
    seen_stores = {store.uri for store in stores}
    configured_store = args.store or (None if args.local_only else os.environ.get("HP_SIM5_ARTIFACT_STORE"))
    for uri in filter(None, [configured_store, *args.also_store]):
        if args.local_only and urlparse(uri).scheme in ("gdrive", "drivemount"):
            raise ValueError("--local-only conflicts with an explicit Drive store")
        try:
            store = store_from_uri(uri)
        except (OSError, ValueError, RuntimeError) as err:
            print(f"ERROR: Could not open {uri}: {err}", file=sys.stderr)
            error = True
            continue
        if store.uri not in seen_stores:
            stores.append(store)
            seen_stores.add(store.uri)

    implicit_drive = None
    if not (args.local_only or configured_store or DEFAULT_DRIVE_URI in seen_stores) and default_adc_path():
        try:
            implicit_drive = GoogleDriveArtifactStore(timeout_s=5.0, max_retries=2)
            if implicit_drive.uri not in seen_stores:
                stores.append(implicit_drive)
        except DriveAuthError as err:
            print(f"WARNING: Could not open {DEFAULT_DRIVE_URI}: {err}", file=sys.stderr)

    rows = []
    canonical = stores[0].experiments_dir.resolve()
    native_runs = {path.parent.name for path in canonical.glob("*/manifest.json")}
    for store in stores:
        try:
            for row in store.list_experiments(kind=args.kind, tags=args.tag, limit=None):
                rows.append({**row, "store_uri": store.uri,
                             "artifact_uri": f"{store.uri}/experiments/{row['experiment_id']}"})
                if store is stores[0] and row.get("kind") == "recorded_experiment":
                    native_runs.add(row["experiment_id"])
        except (OSError, ValueError, RuntimeError) as err:
            level = "WARNING" if store is implicit_drive else "ERROR"
            print(f"{level}: Could not search {store.uri}: {err}", file=sys.stderr)
            error |= store is not implicit_drive

    roots = [ROOT / "output", ROOT / "research/output"]
    if stores[0].experiments_dir.is_symlink():
        roots.append(canonical)
    for path in args.path:
        if not path.is_dir():
            print(f"ERROR: Run tree not found: {path}", file=sys.stderr)
            error = True
        else:
            roots.append(path)
    if session_dir := os.environ.get("HP_SIM5_SESSION_DIR"):
        roots.append(Path(session_dir))
    # Scan loose runs in the same directory as packaged runs; skip packages already listed.
    skip = {
        manifest.parent.resolve()
        for store in stores if isinstance(store, LocalArtifactStore)
        for manifest in store.experiments_dir.glob(f"*/{MANIFEST_NAME}")
    }
    seen_paths = set()
    for root in dict.fromkeys(path.resolve() for path in roots):
        for row in local_runs(root, skip=skip):
            metadata_path = Path(row["metadata_path"])
            if (row.get("kind") == "recorded_experiment" and row["experiment_id"] in native_runs
                    and metadata_path.parent.parent.resolve() != canonical):
                continue
            if row["metadata_path"] not in seen_paths:
                rows.append(row)
                seen_paths.add(row["metadata_path"])

    terms = [term.casefold() for term in args.terms]
    matches = []
    for row in rows:
        if any(value and row.get(key) != value
               for key, value in (("kind", args.kind), ("machine", args.machine), ("status", args.status))):
            continue
        if not set(args.tag).issubset(row.get("tags") or []):
            continue
        if args.revision and not (row.get("git_revision") or "").startswith(args.revision.lower()):
            continue
        searchable = " ".join(str(value) for value in row.values() if value is not None).casefold()
        if all(term in searchable for term in terms):
            matches.append(row)
    matches.sort(key=lambda row: row.get("finished_at") or row.get("stored_at") or row.get("modified_at", ""),
                 reverse=True)
    print(json.dumps(matches[:args.limit], indent=2, sort_keys=True))
    return int(error)


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
        p.add_argument("--discard", action="store_true", help="Skip persistence for throwaway runs")

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
    run_p.add_argument("--summary", help="Short note on the question or purpose of the experiment")
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
    find_p = art_sub.add_parser("find", help="Search saved and loose runs without downloading artifacts")
    find_p.add_argument("terms", nargs="*", help="Case-insensitive terms (all must match)")
    find_p.add_argument("--also-store", action="append", default=[], help="Additional store URI (repeatable)")
    find_p.add_argument("--local-only", action="store_true", help="Search local files and explicit local stores only")
    find_p.add_argument("--path", action="append", default=[], type=Path, help="Additional local run tree")
    find_p.add_argument("--kind")
    find_p.add_argument("--machine")
    find_p.add_argument("--status")
    find_p.add_argument("--revision", help="Git revision or prefix")
    find_p.add_argument("--tag", action="append", default=[])
    find_p.add_argument("--limit", type=int, default=20)
    find_p.set_defaults(func=cmd_find)
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
    push_p.add_argument("--summary", help="Short note on the question, result, or limitations")
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
    except (ArtifactUploadError, ColabShutdownError, DriveAuthError, ValueError, FileNotFoundError) as err:
        print(f"ERROR: {err}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
