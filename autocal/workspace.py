"""Grow acquisition from completed fixed-target probes, without a scene model."""
from __future__ import annotations

import numpy as np


def exploration_grid(dataset: dict, count: int):
    """One measured excursion beyond each anchor's observed fixed-target range.

    Size tuning supplies an initial step, not a permanent workspace boundary.
    Subsequent successful sweeps extend the next range; failed/empty sweeps do
    not. Predicted geometry and force feasibility are checked by the planner.
    """
    values = [[0.0] for _ in range(int(dataset["num_anchors"]))]
    for sweep in dataset.get("sweeps", []):
        if not sweep.get("data_points"):
            continue
        for anchor, delta in zip(sweep["fixed_anchors"], sweep["fixed_lengths"]):
            if np.isfinite(delta):
                values[int(anchor)].append(float(delta))
    step = (dataset.get("config") or {}).get("max_travel_mm")
    if not isinstance(step, (int, float)) or not np.isfinite(step) or step <= 0:
        step = max(abs(value) for axis in values for value in axis)
    if step <= 0:
        step = max((abs(float(point[key])) for sweep in dataset.get("sweeps", [])
                    for point in sweep.get("data_points", []) for key in ("l_drive", "l_sensor")
                    if point.get(key) is not None and np.isfinite(point[key])), default=0.0)
    if step <= 0:
        raise ValueError("Workspace exploration needs a positive tuned excursion or a completed fixed-target probe")
    bounds = {axis: (min(observed) - step, max(observed) + step)
              for axis, observed in enumerate(values)}
    edges = [edge for pair in bounds.values() for edge in pair]
    grid = np.unique(np.r_[np.linspace(min(edges), max(edges), max(3, int(count))),
                           0.0, edges])
    return grid.tolist(), bounds
