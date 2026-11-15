"""Reproduce the same-data fitter chart from the retained CLI comparison."""
import json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

root = Path(__file__).resolve().parent
rows = json.loads((root / "joint-cli-comparison/results/summary.json").read_text())
keys = ["errors", "pose_fit_errors", "joint_fit_errors"]
labels = ["Ellipse fitter", "Nested point fitter", "Independent joint fit"]
fig, ax = plt.subplots(figsize=(8.2, 3.7), layout="constrained")
y = np.arange(3)
for row, offset, color, label in zip(rows, [-.18, .18], ["#9da6b3", "#2563b9"], ["Offsets off", "Offsets fitted"]):
    values = [row[key]["max_abs_coordinate_mm"] for key in keys]
    ax.barh(y + offset, values, height=.32, color=color, label=label)
    for pos, value in zip(y + offset, values):
        ax.text(value + .15, pos, f"{value:.2f}", va="center", fontsize=10)
ax.axvline(1., color="#af3030", linestyle="--", linewidth=1., label="1 mm goal")
ax.set(yticks=y, yticklabels=labels, xlabel="Worst anchor coordinate error (mm)", xlim=(0, 16.5))
ax.invert_yaxis()
ax.spines[["top", "right"]].set_visible(False)
ax.grid(axis="x", alpha=.15)
ax.set_axisbelow(True)
ax.legend(loc="lower right", frameon=False, ncols=3, fontsize=9)
ax.set_title("Same HP3 observations: fitting the home offset helps", loc="left", fontsize=13, pad=14)
fig.savefig(root / "fit-comparison.png", dpi=170)
