"""Write results/live_new.md: old vs new code, same live cases."""
import json
from pathlib import Path
runs = Path(__file__).resolve().parents[1] / "runs"
lines = ["| case | machine | old code: applied err / stop / sweeps / sim s / jackknife | new code: applied err / stop / sweeps / sim s / jackknife | new refit D.z err |",
         "|---|---|---|---|---|"]
def cell(path):
    if not path.exists():
        return "—", None
    x = json.loads(path.read_text())
    return f"{x['applied_err']:.2f} / {x['stop_reason']} / {x['sweeps']} / {x['simulated_s']:.0f} / {x['jackknife_mm']:.2f}", x
for new in sorted(runs.glob("new_*/summary.json")):
    case = new.parent.name[4:]
    old_text, _ = cell(runs / case / "summary.json")
    if old_text == "—" and (runs / case / "sweeps.headless" / "manifest.json").exists():
        m = json.loads((runs / case / "sweeps.headless" / "manifest.json").read_text())
        old_text = f"failed: {m.get('stop_reason')}"
    new_text, x = cell(new)
    lines.append(f"| {case} | {x.get('machine', 'hangprinter_3').replace('hangprinter_', 'HP')} | {old_text} | {new_text} | {x['refit_dz_D']:+.2f} |")
(runs.parent / "results" / "live_new.md").write_text("\n".join(lines) + "\n")
print("\n".join(lines))
