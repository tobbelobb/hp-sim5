"""Paired table: base (338ded23) vs home (home-strain default) per run name.

usage: paired.py RUNS_ROOT PREFIX     e.g. paired.py evidence/runs hp3-
"""
import json, sys
from pathlib import Path
import numpy as np

root, prefix = Path(sys.argv[1]), sys.argv[2]


def load(arm, name):
    path = root / arm / name / "result.json"
    if not path.exists():
        return None
    d = json.loads(path.read_text())
    if d.get("max_abs_frame_error_mm") is None:
        return dict(stop=d.get("stop_reason"), err=np.nan, top_z=np.nan, low_z=np.nan, sim=d.get("simulated_s"), wall=d.get("wall_s"), worst=None)
    fe = d["frame_error_mm"]
    fe = np.array(list(fe.values()) if isinstance(fe, dict) else fe, float)
    return dict(stop=d["stop_reason"], err=d["max_abs_frame_error_mm"], worst=d["worst_frame_coordinate"],
                top_z=fe[-1, 2], low_z=fe[:-1, 2].mean(), sim=d["simulated_s"], wall=d["wall_s"])


names = sorted({p.name for arm in ("base", "home") for p in (root / arm).glob(prefix + "*") if p.is_dir()},
               key=lambda n: (not n.endswith("nominal"), len(n), n))
print("| Run | Frame max, base → home (mm) | Δ | Worst coord, base → home | Top z, base → home | Low z mean, base → home | Sim s, base → home | Stop, base → home |")
print("|---|---|---|---|---|---|---|---|")
deltas, seeds = [], {"base": [], "home": []}
for n in names:
    b, h = load("base", n), load("home", n)
    if b is None or h is None:
        print(f"| {n} | missing {'base' if b is None else 'home'} |"); continue
    delta = h["err"] - b["err"]
    if not n.endswith("nominal"):
        deltas.append(delta); seeds["base"].append(b["err"]); seeds["home"].append(h["err"])
    print(f"| {n} | {b['err']:.2f} → {h['err']:.2f} | {delta:+.2f} | {b['worst']} → {h['worst']} | {b['top_z']:+.2f} → {h['top_z']:+.2f} | "
          f"{b['low_z']:+.2f} → {h['low_z']:+.2f} | {b['sim']:.0f} → {h['sim']:.0f} | {b['stop']} → {h['stop']} |")
if len(deltas) > 1:
    d = np.array(deltas)
    print(f"\nRandom seeds n={len(d)}: base {np.mean(seeds['base']):.2f} ± {np.std(seeds['base'], ddof=1):.2f}, "
          f"home {np.mean(seeds['home']):.2f} ± {np.std(seeds['home'], ddof=1):.2f} mm; paired Δ {d.mean():+.2f} ± {d.std(ddof=1) / np.sqrt(len(d)):.2f} (se) mm; "
          f"home better in {int(np.sum(d < 0))}/{len(d)}")
