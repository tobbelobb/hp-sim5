#!/usr/bin/env python3
"""Generate a complete dated set of full-auto regression reference logs."""

from __future__ import annotations

import argparse
import concurrent.futures
from datetime import date
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import subprocess
import sys
import uuid

ROOT = Path(__file__).resolve().parents[2]
LOCAL_PYTHON = ROOT / ".venv" / "bin" / "python"
if LOCAL_PYTHON.is_file() and LOCAL_PYTHON.resolve() != Path(sys.executable).resolve():
    os.execv(str(LOCAL_PYTHON), [str(LOCAL_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]])

sys.path.insert(0, str(ROOT))

from autocal.tools.regress_calibration_logs import (  # noqa: E402
    DATASETS,
    parse_generated_log_path,
    prepare_isolated_dataset_copy,
    run_autocal,
)

MONTH_NAMES = (
    "january", "february", "march", "april", "may", "june",
    "july", "august", "september", "october", "november", "december",
)


def _date_label(run_date: date, suffix: int | None = None) -> str:
    suffix_text = f"_{suffix}" if suffix is not None else ""
    return f"{MONTH_NAMES[run_date.month - 1]}_{run_date.day}{suffix_text}_{run_date.year}"


def _dependency_versions() -> dict[str, str]:
    versions = {}
    for package in ("jax", "jaxlib", "numpy", "scipy"):
        try:
            versions[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            continue
    return versions


def _run_dataset(dataset_spec, repo_root: Path, data_dir: Path, scratch_root: Path, sparse_recovery: bool):
    source = data_dir / f"{dataset_spec.name}.json"
    if not source.is_file():
        raise FileNotFoundError(f"dataset not found: {source}")

    isolated_dataset = prepare_isolated_dataset_copy(dataset_spec.name, source, scratch_root)
    jsonl_path = isolated_dataset.with_name(f"{isolated_dataset.stem}.full_auto_log.jsonl")
    returncode, output = run_autocal(
        repo_root=repo_root,
        dataset_path=isolated_dataset,
        dataset_spec=dataset_spec,
        full_auto_log=jsonl_path,
        sparse_recovery=sparse_recovery,
    )
    if returncode != 0:
        raise RuntimeError(f"autocal exited with {returncode}\n{output.rstrip()}")
    generated_log = parse_generated_log_path(output)
    if not generated_log:
        raise RuntimeError("autocal output did not contain a generated log path")
    generated_log_path = Path(generated_log)
    if not generated_log_path.is_absolute():
        generated_log_path = repo_root / generated_log_path
    if not generated_log_path.is_file():
        raise FileNotFoundError(f"generated log not found: {generated_log_path}")

    return generated_log_path.read_text(encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=ROOT)
    parser.add_argument("--data-dir", type=Path, default=Path("autocal/data/references"))
    parser.add_argument("--ref-dir", type=Path, default=Path("autocal/data/references"))
    parser.add_argument("--date", type=date.fromisoformat, default=date.today(), help="Run date (YYYY-MM-DD).")
    parser.add_argument("--only", choices=sorted({dataset.machine_type for dataset in DATASETS}))
    parser.add_argument("--sparse-recovery", action="store_true")
    args = parser.parse_args()

    repo_root = args.repo_root.resolve()
    data_dir = args.data_dir if args.data_dir.is_absolute() else repo_root / args.data_dir
    ref_dir = args.ref_dir if args.ref_dir.is_absolute() else repo_root / args.ref_dir
    data_dir = data_dir.resolve()
    ref_dir = ref_dir.resolve()
    if not (repo_root / "autocal" / "autocal.py").is_file():
        print(f"ERROR: repo_root={repo_root} does not contain autocal/autocal.py", file=sys.stderr)
        return 1

    datasets = [d for d in DATASETS if args.only is None or d.machine_type == args.only]
    # A filtered run still needs a collision-free label for the whole selected set.
    suffix = None
    while True:
        label = _date_label(args.date, suffix)
        if not any(
            (ref_dir / f"{dataset.name}.full_auto_reference_run_{label}.log").exists()
            for dataset in datasets
        ):
            break
        suffix = 2 if suffix is None else suffix + 1

    scratch_root = repo_root / "autocal" / "data" / ".regress_parallel_runs"
    scratch_root.mkdir(parents=True, exist_ok=True)
    ref_dir.mkdir(parents=True, exist_ok=True)

    outputs = {}
    failures = {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=len(datasets)) as pool:
        futures = {
            pool.submit(_run_dataset, dataset, repo_root, data_dir, scratch_root, args.sparse_recovery): dataset
            for dataset in datasets
        }
        for future in concurrent.futures.as_completed(futures):
            dataset = futures[future]
            try:
                outputs[dataset.name] = future.result()
                print(f"Completed {dataset.name}")
            except Exception as exc:
                failures[dataset.name] = str(exc)
                print(f"FAILED {dataset.name}: {exc}", file=sys.stderr)

    if failures:
        print("No reference logs were published because one or more runs failed.", file=sys.stderr)
        return 1

    try:
        revision = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=repo_root, check=True,
            capture_output=True, text=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        revision = "unknown"

    staged = []
    published = []
    try:
        for dataset in datasets:
            source = data_dir / f"{dataset.name}.json"
            output = outputs[dataset.name]
            provenance = {
                "candidate_revision": revision,
                "command": f"{Path(sys.executable).name} autocal/tools/generate_reference_runs.py",
                "dataset": dataset.name,
                "dataset_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
                "dependencies": _dependency_versions(),
                "interpreter": sys.executable,
                "python": sys.version,
                "thread_environment": {
                    name: os.environ.get(name, "1 (runner default)")
                    for name in ("MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS", "OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS")
                },
            }
            target = ref_dir / f"{dataset.name}.full_auto_reference_run_{label}.log"
            temp = target.with_name(f".{target.name}.{uuid.uuid4().hex}.tmp")
            temp.write_text(
                f"; reference_provenance: {json.dumps(provenance, sort_keys=True)}\n" + output.rstrip() + "\n",
                encoding="utf-8",
            )
            staged.append((temp, target))

        for temp, target in staged:
            if target.exists():
                raise FileExistsError(f"reference appeared during generation: {target}")
            temp.replace(target)
            published.append(target)
            print(f"Wrote {target}")
    except Exception as exc:
        for target in published:
            target.unlink(missing_ok=True)
        for temp, _target in staged:
            temp.unlink(missing_ok=True)
        print(f"ERROR: could not publish reference logs: {exc}", file=sys.stderr)
        return 1

    print(f"Generated {len(datasets)} reference logs with date label {label}.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
