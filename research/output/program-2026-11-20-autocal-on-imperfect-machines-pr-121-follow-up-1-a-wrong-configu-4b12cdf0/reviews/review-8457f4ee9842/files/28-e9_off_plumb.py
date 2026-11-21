"""E9: anchor error from a top line that is off plumb, through the stretch model's gravity.

In the convention frame (top anchor above home) the fit's stretch correction puts the
effector weight along -z, i.e. along the top line. If the real top anchor sits `offset` mm
horizontally off plumb at height H, real gravity is tilted by theta = offset / H in that
frame. Equivalent test: refit the same recorded data with the model's gravity tilted by
theta, and measure how far the fitted anchors move (and their error vs truth).
Only datasets whose fit uses the stretch model are affected (HP5/HP5r; HP4 equal strain).
"""
import json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
from lowz_common import TRUTH, load, fit_default, anchor_error
import autocal.pose_fit as pf

state = {"g": np.array([0.0, 0.0, 1.0])}

def tilted(anchors, positions, absolute, measured, stretch):
    directions = anchors[None, :, :] - positions[:, None, :]
    directions /= np.maximum(np.linalg.norm(directions, axis=2), 1e-9)[:, :, None]
    known = np.where(measured, np.nan, stretch["passive"])
    known = np.where(np.isfinite(stretch["known"]), stretch["known"], known)
    known[np.arange(len(positions)), stretch["sensors"]] = stretch["forces"]
    selectors = np.eye(anchors.shape[0])[None, :, :] * np.isfinite(known)[:, :, None]
    matrix = np.concatenate([directions.transpose(0, 2, 1), selectors], axis=1)
    load = np.tile(stretch["weight"] * state["g"], (len(positions), 1))  # only change: gravity direction
    transpose = matrix.transpose(0, 2, 1)
    normal = transpose @ matrix + 1e-12 * np.eye(matrix.shape[-1])
    rhs = transpose @ np.concatenate([load, np.nan_to_num(known)], axis=1)[:, :, None]
    tensions = np.linalg.solve(normal, rhs)[:, :, 0]
    return np.column_stack([stretch["model"].corrected_distance_mm(absolute[:, axis], tensions[:, axis], axis=axis)
                            for axis in range(anchors.shape[0])])

pf.stretch_corrected_lengths = tilted

cases = [(f"benchmark {m}", load(f"benchmark_923c2baa_{m}"), TRUTH[m]) for m in ("hangprinter_5", "hangprinter_5_pulley_reduced")]
run = Path(__file__).resolve().parents[1] / "runs" / "new_hp5_base_0"
cases.append(("live new_hp5_base_0", json.loads((run / "sweeps.json").read_text()),
              np.asarray(json.loads((run / "input.json").read_text())["truth"])))
rows = []
for label, data, truth in cases:
    assert pf.dataset_stretch(data) is not None, label
    height = float(truth[-1, 2])
    state["g"] = np.array([0.0, 0.0, 1.0]); ref = fit_default(data)[0]["anchors"]
    for offset in (10.0, 25.0, 50.0):
        for azimuth in (0.0, 90.0):
            theta = offset / height
            phi = np.radians(azimuth)
            state["g"] = np.array([np.sin(theta) * np.cos(phi), np.sin(theta) * np.sin(phi), np.cos(theta)])
            a = fit_default(data)[0]["anchors"]
            rows.append(dict(case=label, top_offset_mm=offset, azimuth_deg=azimuth, theta_mrad=1e3 * theta,
                             anchor_shift_mm=anchor_error(a, ref), err_ref=anchor_error(ref, truth), err=anchor_error(a, truth)))
            print(json.dumps({k: (round(v, 4) if isinstance(v, float) else v) for k, v in rows[-1].items()}), flush=True)
Path(sys.argv[1]).write_text(json.dumps(rows, indent=1))
