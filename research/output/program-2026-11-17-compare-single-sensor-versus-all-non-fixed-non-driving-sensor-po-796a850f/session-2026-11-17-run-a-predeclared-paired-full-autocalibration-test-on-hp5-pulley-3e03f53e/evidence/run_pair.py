"""Run one arm of the predeclared paired full autocal experiment.

Baseline runs the unchanged production command. Treatment runs the same command
through evidence/run_treatment.py with the experiment collector. Both use the
existing contract: headless-sim, recording none, patience 20, default
stable-step 0.5 mm / two sweeps, 6,000,000 physics steps, 1000 s external limit.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
SESSION = HERE.parent
REPO = next(p for p in SESSION.parents if (p / "autocal/autocal.py").exists())
PY = REPO / ".venv/bin/python"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run_arm(arm: str) -> int:
    dataset = SESSION / "runs" / arm / "sweeps.json"
    run_dir = dataset.parent
    run_dir.mkdir(parents=True, exist_ok=False)
    log_path = run_dir / "run.log"
    common = [
        "--machine-type", "hangprinter_5_pulley_reduced",
        "--dataset", str(dataset),
        "--headless-sim",
        "--recording-mode", "none",
        "--patience", "20",
        "--verbose",
    ]
    env = dict(os.environ)
    env["HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS"] = "6000000"
    if arm == "baseline":
        command = [str(PY), "autocal/autocal.py", *common]
        env.pop("HP5_EXTRA_RANGE_COLLECTOR", None)
    elif arm == "treatment":
        command = [str(PY), str(HERE / "run_treatment.py"), *common]
        env["HP5_EXTRA_RANGE_COLLECTOR"] = str(HERE / "extra_range_collector/collect_sweep_data.mjs")
    else:
        raise SystemExit(f"unknown arm {arm}")
    record = {
        "arm": arm,
        "started_unix": time.time(),
        "command": command,
        "cwd": str(REPO),
        "dataset": str(dataset),
        "env": {
            "HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS": env["HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS"],
            "HP5_EXTRA_RANGE_COLLECTOR": env.get("HP5_EXTRA_RANGE_COLLECTOR"),
            "OMP_NUM_THREADS": env.get("OMP_NUM_THREADS"),
            "OPENBLAS_NUM_THREADS": env.get("OPENBLAS_NUM_THREADS"),
        },
        "external_limit_s": 1000,
        "contract": {
            "headless_sim": True,
            "recording_mode": "none",
            "patience": 20,
            "stable_step_mm": 0.5,
            "max_physics_steps": 6000000,
            "max_steps_default": 20,
        },
    }
    started = time.monotonic()
    with log_path.open("w") as log:
        completed = subprocess.run(
            ["timeout", "-s", "INT", "1000", *command],
            cwd=REPO, env=env, stdout=log, stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
        )
    wall = time.monotonic() - started
    record.update(returncode=completed.returncode, wall_s=wall, finished_unix=time.time())
    manifest_path = dataset.with_suffix(".headless") / "manifest.json"
    clock_path = dataset.with_suffix(".headless") / "clock.json"
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        record["manifest"] = {
            key: manifest.get(key) for key in (
                "status", "stop_reason", "normal_completion", "wall_s", "simulated_s",
                "total_steps", "collector_time_s", "service_wall_s", "applied_parameters",
                "firmware_final", "source_sha256", "autocal_source_sha256", "git_revision",
                "scene_sha256", "firmware_config_sha256", "rrf_sha256",
                "physics_artifact", "baked_scene_sha256", "physics_returncode",
                "owned_services", "services", "kept_alive", "error",
            )
        }
        record["manifest_sha256"] = sha256(manifest_path)
    if clock_path.exists():
        record["clock"] = json.loads(clock_path.read_text())
    if dataset.exists():
        data = json.loads(dataset.read_text())
        record["dataset"] = {
            "path": str(dataset),
            "sha256": sha256(dataset),
            "sweeps": len(data.get("sweeps", [])),
            "points": sum(len(s.get("data_points", [])) for s in data.get("sweeps", [])),
            "acquisition_rejections": len(data.get("acquisition_rejections", [])),
        }
        validity = dataset.with_suffix(".extra_range_validity.json")
        if validity.exists():
            record["extra_range_validity"] = {
                "path": str(validity),
                "sha256": sha256(validity),
                "summary": {
                    key: json.loads(validity.read_text())["gate"].get(key)
                    for key in ("accepted", "rejected", "no_policy")
                },
            }
    (run_dir / "run.json").write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps({key: record.get(key) for key in ("arm", "returncode", "wall_s")}, indent=2))
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm", required=True, choices=["baseline", "treatment"])
    args = parser.parse_args()
    return run_arm(args.arm)


if __name__ == "__main__":
    raise SystemExit(main())
