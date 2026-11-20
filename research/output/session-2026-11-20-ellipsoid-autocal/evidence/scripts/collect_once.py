"""Run one collector call in a fresh headless sim: collect_once.py MACHINE OUTDIR CFG [collector args...]"""
import subprocess, sys
from pathlib import Path
from types import SimpleNamespace
from autocal.headless_sim import HeadlessSimulation
machine, out, cfg, *extra = sys.argv[1:]
out = Path(out); out.mkdir(parents=True, exist_ok=True)
args = SimpleNamespace(machine_type=machine, find_radii=None, find_buildup_factor=None, rrf_config=None, config=None,
                       dataset=str(out / "sweeps.json"), recording_mode="none", keep_sim_alive=False)
collector_args = ["--solver-iterations", "20", "--firmware", "rrf", "--sim", *extra]
with HeadlessSimulation(args, collector_args) as sim:
    cmd = ["node", "autocal/control/cli/collect_sweep_data.mjs", "--machineType", machine, "--sweep-config-file", cfg,
           "--output-file", str(out / "sweeps.json"), "--sweepPoints", "5", "--return-to-origin", *collector_args]
    print(" ".join(cmd), flush=True)
    code = subprocess.call(cmd)
    print("exit", code, "simulated_s", sim.simulated_seconds())
