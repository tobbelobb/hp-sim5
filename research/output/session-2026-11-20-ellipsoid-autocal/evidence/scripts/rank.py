"""Rank variants: per machine, error x time (twice the time costs as much as twice the error).
error = geometric mean of the clean error and the mean error of each disturbance scenario."""
import json, glob, sys
from pathlib import Path
import numpy as np
S = Path(sys.argv[1])
TIME = {"sweeps": {"hangprinter_3": 5516, "hangprinter_4": 4897, "hangprinter_5": 3762, "hangprinter_5_pulley_reduced": 4672}}
rows = [json.loads(p.read_text()) for p in sorted(S.glob("robust/*.json")) if p.stat().st_size]
def sim_time(label, machine):
    if label in TIME: return TIME[label][machine]
    return json.loads(Path(f"output/{label}-{machine}/sweeps.headless/manifest.json").read_text())["simulated_s"]
scen = ["mixed", "noise0.05", "noise0.2", "mixed+noise0.2"]
table = {}
print(f"{'variant':10s} {'machine':30s} {'time s':>7s} {'clean':>6s} " + " ".join(f"{s:>14s}" for s in scen) + f" {'err gm':>7s} {'clean*t':>8s} {'gm*t':>8s}")
for r in rows:
    t = sim_time(r["label"], r["machine"])
    errs = [r["clean"]] + [float(np.mean(r[s])) for s in scen]
    gm = float(np.exp(np.mean(np.log(errs))))
    table.setdefault(r["label"], {})[r["machine"]] = (r["clean"] * t, gm * t)
    print(f"{r['label']:10s} {r['machine']:30s} {t:7.0f} {r['clean']:6.2f} " + " ".join(f"{e:14.2f}" for e in errs[1:]) + f" {gm:7.2f} {r['clean']*t:8.0f} {gm*t:8.0f}")
print()
for label, machines in table.items():
    if len(machines) == 4:
        c = np.exp(np.mean(np.log([v[0] for v in machines.values()]))); d = np.exp(np.mean(np.log([v[1] for v in machines.values()])))
        print(f"{label:10s} geometric mean over machines: clean err*time {c:7.0f}, disturbed-inclusive err*time {d:7.0f}")
