"""Collect one short, isolated HP5 pulley-reduced sweep dataset in a private headless world.

Nothing here touches the live browser/autocal run: HeadlessSimulation allocates
free ports, its own dataset directory, and its own RRF/physics processes.

Example:
  .venv/bin/python evidence/collect_run.py --run-id flo-0051-n3 \
      --force-low 0.0516 --force-mid 1.0318 --force-max 2.8313 \
      --sensor-force 4.9914 --fixed-targets -340,0 --points 8 \
      --configs 0,4:1:3 2,4:3:1 1,4:0:2
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
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent
SESSION = HERE.parent
REPO = next(p for p in SESSION.parents if (p / "autocal/headless_sim.py").exists())
sys.path.insert(0, str(REPO))

from autocal.headless_sim import HeadlessSimulation  # noqa: E402

MACHINE = "hangprinter_5_pulley_reduced"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_config(spec: str):
    fixed, drive, sensor = spec.split(":")
    return [int(v) for v in fixed.split(",")], int(drive), int(sensor)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--force-low", type=float, required=True,
                        help="Compliant/idle force for every non-fixed, non-driving line (N)")
    parser.add_argument("--force-mid", type=float, required=True)
    parser.add_argument("--force-max", type=float, required=True)
    parser.add_argument("--sensor-force", type=float, required=True)
    parser.add_argument("--fixed-targets", required=True, help="Comma separated fixed length deltas (mm)")
    parser.add_argument("--points", type=int, default=8)
    parser.add_argument("--configs", nargs="+", required=True, help="fixed:drive:sensor per sweep")
    parser.add_argument("--solver-iterations", type=int, default=20)
    parser.add_argument("--speedup", type=float, default=1.0)
    parser.add_argument("--settle-ms", type=int, default=200)
    parser.add_argument("--recording-mode", default="none", choices=["none", "events", "sampled", "full"])
    parser.add_argument("--collector", default="autocal/control/cli/collect_sweep_data.mjs",
                        help="Collector CLI to run (relative to the repo or absolute)")
    parser.add_argument("--redundant-force-mode", default=None,
                        choices=[None, "sensor", "passive"],
                        help="How the redundant free line was torque-controlled "
                             "(default: sensor for the prototype, passive otherwise)")
    parser.add_argument("--keep-sim-alive", action="store_true")
    args = parser.parse_args()

    run = SESSION / "runs" / args.run_id
    run.mkdir(parents=True, exist_ok=False)
    dataset = run / "sweeps.json"
    configs = [parse_config(spec) for spec in args.configs]
    cfg_file = run / "sweeps.cfg.txt"
    cfg_file.write_text("".join(f"[{','.join(map(str, fixed))}] {drive} {sensor}\n"
                                for fixed, drive, sensor in configs))

    sim_args = SimpleNamespace(
        machine_type=MACHINE,
        find_radii="off",
        find_buildup_factor="off",
        config=None,
        rrf_config=None,
        buildup_factor=0,
        recording_mode=args.recording_mode,
        extended_reference=False,
        dataset=str(dataset),
        keep_sim_alive=bool(args.keep_sim_alive),
    )
    collector = Path(args.collector)
    if not collector.is_absolute():
        collector = REPO / collector
    collector_args = [
        "--machineType", MACHINE,
        "--sweep-config-file", str(cfg_file),
        "--output-file", str(dataset),
        "--solver-iterations", str(args.solver_iterations),
        "--speedup", str(args.speedup),
        "--firmware", "rrf",
        "--sim",
        "--return-to-origin",
        "--sweepPoints", str(args.points),
        "--settle-ms", str(args.settle_ms),
        "--fixed-targets", args.fixed_targets,
        "--force-low", str(args.force_low),
        "--force-mid", str(args.force_mid),
        "--force-max", str(args.force_max),
        "--sensor-collection-force", str(args.sensor_force),
        "--no-auto-tune-force",
    ]

    invocation = dict(
        run_id=args.run_id, machine=MACHINE, argv=sys.argv,
        redundant_force_mode=(args.redundant_force_mode
                              or ("sensor" if "prototype" in str(collector) else "passive")),
        force_low_n=args.force_low, force_mid_n=args.force_mid, force_max_n=args.force_max,
        sensor_force_n=args.sensor_force, fixed_targets_mm=args.fixed_targets,
        points_per_sweep=args.points, configs=args.configs,
        solver_iterations=args.solver_iterations, recording_mode=args.recording_mode,
        settle_ms=args.settle_ms,
        collector=str(collector),
        collector_sha256=sha256(collector),
        script_sha256=sha256(Path(__file__)),
        started_unix=time.time(),
    )
    (run / "invocation.json").write_text(json.dumps(invocation, indent=2) + "\n")

    started = time.monotonic()
    status = 0
    env_before = dict(os.environ)
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
    try:
        with HeadlessSimulation(sim_args, collector_args) as sim:
            command = ["node", str(collector), *collector_args]
            (run / "collector-command.json").write_text(json.dumps(command, indent=2) + "\n")
            with (run / "collector.log").open("w") as log:
                result = subprocess.run(command, cwd=REPO, stdout=log, stderr=subprocess.STDOUT,
                                        stdin=subprocess.DEVNULL)
            status = result.returncode
            sim.manifest["collector_returncode"] = status
            sim.manifest["dataset"] = str(dataset)
    finally:
        os.environ.clear()
        os.environ.update(env_before)
    elapsed = time.monotonic() - started
    summary = dict(run_id=args.run_id, collector_returncode=status, wall_s=elapsed)
    clock = run / "sweeps.headless" / "clock.json"
    if clock.exists():
        state = json.loads(clock.read_text())
        summary.update(collector_time_s=state.get("collector_time_s"),
                       simulated_s=state.get("simulated_s"), service_wall_s=state.get("wall_s"))
    if dataset.exists():
        data = json.loads(dataset.read_text())
        summary.update(sweeps=len(data.get("sweeps", [])),
                       points=sum(len(s.get("data_points", [])) for s in data.get("sweeps", [])),
                       acquisition_rejections=len(data.get("acquisition_rejections", [])),
                       dataset_sha256=sha256(dataset))
    (run / "run-summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return 0 if status == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
