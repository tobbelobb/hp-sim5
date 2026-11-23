"""Write results/tables.md from the E1, E3 and live JSON results."""
import json, glob
from pathlib import Path
import numpy as np
R = Path(__file__).resolve().parents[1] / "results"
out = []
out.append("## E1: equal-z default fit, anchor A raised by h (delta-resynthesised benchmark sweeps)\n")
out.append("| machine | h mm | max err mm | after rotation mm | D.z err mm | rms mm | jackknife mm |")
out.append("|---|---|---|---|---|---|---|")
for m in ("hangprinter_3", "hangprinter_4", "hangprinter_5", "hangprinter_5_pulley_reduced"):
    for r in json.loads((R / f"e1_{m}.json").read_text())["rows"]:
        out.append(f"| {m.replace('hangprinter_', 'HP')} | {r['h']:+.0f} | {r['err']:.2f} | {r['err_rot']:.2f} | {r['dz_D']:+.2f} | {r['rms']:.3f} | {r['jackknife']:.2f} |")
out.append("\n## E3: equal-z vs free-warp refit, 4 low anchors\n")
out.append("| dataset | h mm | seed | F | equal-z err | free err | free after rotation | warp (A+C-B-D)/2 mm |")
out.append("|---|---|---|---|---|---|---|---|")
for f in sorted(R.glob("e3_*.json")):
    for r in json.loads(f.read_text())["rows"]:
        z = r["free_low_z"]; w = (z[0] + z[2] - z[1] - z[3]) / 2
        out.append(f"| {f.stem[3:].replace('hangprinter_', 'HP')} | {r['h']:+.0f} | {r['seed']} | {r['F']:.1f} | {r['eq_err']:.2f} | {r['free_err']:.2f} | {r['free_err_rot']:.2f} | {w:+.2f} |")
out.append("\n## E4: live headless runs (applied firmware anchors)\n")
out.append("| run | machine | shifts mm | sweeps | sim s | applied err | after rotation | refit D.z err | rms | jackknife |")
out.append("|---|---|---|---|---|---|---|---|---|---|")
for f in sorted((R.parent / "runs").glob("*/summary.json")):
    x = json.loads(f.read_text())
    out.append(f"| {x['name']} | {x.get('machine', 'hangprinter_3').replace('hangprinter_', 'HP')} | {x['shifts_mm']} | {x['sweeps']} | {x['simulated_s']:.0f} | {x['applied_err']:.2f} | {x['applied_err_rot']:.2f} | {x['refit_dz_D']:+.2f} | {x['rms_mm']:.3f} | {x['jackknife_mm']:.2f} |")
(R / "tables.md").write_text("\n".join(out) + "\n")
print("\n".join(out))
