"""Plot recorded-data accuracy and repeated full-auto compute timings."""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
points = json.loads((HERE / "E03-recorded-comparison.json").read_text())
timings = json.loads((HERE / "E06-isolated-summary.json").read_text())
fig, (accuracy, speed) = plt.subplots(1, 2, figsize=(11, 4.6), layout="constrained")
colors = ["#277DA1", "#E09F3E", "#B24764"]
x = np.arange(len(points))
for i, (key, label) in enumerate(zip(
        ["full", "uniform", "informed"],
        ["Full data", "Uniform half", "Informative half"])):
    values = [row["models"][key]["anchor_rmse_mm"] for row in points]
    accuracy.bar(x + (i - 1) * .24, values, width=.23, color=colors[i], label=label)
accuracy.set_xticks(x, ["Bad points", "Ten points", "Seventh HP3", "April HP3"])
accuracy.set_yscale("log")
accuracy.set_ylabel("Returned anchor RMSE (mm, log scale)")
accuracy.set_title("Halving points is not reliably accurate")
accuracy.legend(frameon=False, fontsize=9)
accuracy.grid(axis="y", alpha=.2, which="both")
accuracy.set_axisbelow(True)
for i, row in enumerate(timings):
    for j, (key, label) in enumerate([("baseline", "Baseline"), ("cache", "Exact cache")]):
        values = row[key + "_seconds"]
        at = i + (j - .5) * .30
        speed.bar(at, row[key + "_median"], width=.27, color=colors[j],
                  label=label if i == 0 else None, alpha=.8)
        speed.scatter(at + np.linspace(-.04, .04, len(values)), values,
                      color="#252525", s=14, zorder=3)
    speed.text(i, max(row["baseline_seconds"]) + 2,
               f'{row["median_saving_percent"]:.1f}% faster', ha="center", fontsize=10)
speed.set_xticks(range(len(timings)), ["Bad points", "Seventh HP3"])
speed.set_ylabel("Complete recorded replay wall time (s)")
speed.set_title("Exact reuse preserves every fit and stop")
speed.set_ylim(0, max(max(r["baseline_seconds"]) for r in timings) + 7)
speed.legend(frameon=False, fontsize=9)
speed.grid(axis="y", alpha=.2)
speed.set_axisbelow(True)
fig.suptitle("Recorded browser simulation; three timing repetitions; no hardware evidence", fontsize=10)
fig.savefig(HERE / "results.png", dpi=180)
fig.savefig(HERE / "results.pdf")
