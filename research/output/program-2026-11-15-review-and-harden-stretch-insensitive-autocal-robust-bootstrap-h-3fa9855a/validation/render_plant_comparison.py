"""Plot retained full-run accuracy measurements; missing trial cells stay empty."""
import json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
root = Path(__file__).resolve().parent
rows = json.loads((root / "plant-comparison.json").read_text())
terminal = root.parent / "session-2026-11-15-test-one-bounded-terminal-validation-sweep-that-compares-frozen-43d8bbc3/evidence"
for row, case in zip(rows, ["hp3", "hp3-soft", None, "hp4", None]):
    path = terminal / str(case) / "summary.json"
    row["terminal_mm"] = json.loads(path.read_text())["anchors"]["max_abs_coordinate_mm"] if case and path.exists() else None
fig, ax = plt.subplots(figsize=(8.6, 4.6), layout="constrained")
y = np.arange(len(rows))
for key, shift, color, label in [("earlier_mm", -.23, "#9da6b3", "Earlier design"), ("checked_mm", 0, "#2563b9", "Checked rough pass"), ("terminal_mm", .23, "#d58028", "Terminal pilot")]:
    for index, row in enumerate(rows):
        if row[key] is None: continue
        value = row[key]
        ax.barh(index + shift, value, height=.21, color=color, label=label if index == 0 else None)
        ax.text(value+.12, index+shift, f"{value:.2f}", va="center", fontsize=9)
ax.axvline(1., color="#af3030", linestyle="--", linewidth=1, label="1 mm goal")
ax.set(yticks=y, yticklabels=[row["case"] for row in rows], xlim=(0, 11.2), xlabel="Worst anchor coordinate error (mm)")
ax.invert_yaxis(); ax.spines[["top", "right"]].set_visible(False)
ax.grid(axis="x", alpha=.15); ax.set_axisbelow(True)
ax.legend(loc="lower right", ncols=2, fontsize=8, frameon=False)
ax.set_title("Full simulations: accuracy remains uneven", loc="left", fontsize=13, pad=14)
fig.savefig(root / "plant-comparison.png", dpi=170)
(root / "plant-comparison-plotted.json").write_text(json.dumps(rows, indent=2)+"\n")
