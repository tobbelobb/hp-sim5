"""Corrupt one sweep of a reference dataset: its held low anchor moves by d mm (outward, radial)
for that sweep's ranges only (an anchor or line slip that one sweep sees). Report err, J0, J1.

usage: one_bad_sweep.py out.json d_mm name...
"""
import copy, json, sys
from pathlib import Path
import numpy as np

WT = Path("/home/torbjorn/.t3/worktrees/hp-sim5/t3-244bb580-382b3b0103efa274")
sys.path.insert(0, str(WT))
sys.path.insert(0, str(next(WT.glob("research/output/program-2026-11-20-*/session-2026-11-20-uneven*/scripts"))))
from lowz_common import TRUTH, load, fit_default, anchor_error, resynthesise, point_positions
from autocal._autocal_common import _normalize_dataset_point_roles
from autocal.dataset_roles import measurement_groups
from autocal.pose_fit import HOME_OFFSET_PRIOR_SIGMA_MM, fit_pose_least_squares


def spreads(data, anchors):
    sweeps = data["sweeps"]; folds = []
    for left in measurement_groups(data):
        sub = dict(data, sweeps=[s for i, s in enumerate(sweeps) if i not in left])
        folds.append(fit_pose_least_squares(sub, anchors, fit_home_offsets=True,
                                            home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM)["anchors"])
    f = np.stack(folds); n = len(f)
    j0 = np.sqrt((n - 1) / n * np.sum((f - f.mean(0)) ** 2, axis=0)).max()
    j1 = (np.sqrt(n - 1) * 1.4826 * np.median(np.abs(f - np.median(f, axis=0)), axis=0)).max()
    dev = np.abs(f - f.mean(0)).max(axis=(1, 2))
    return float(j0), float(j1), dev.round(2).tolist()


out, d_mm, names = Path(sys.argv[1]), float(sys.argv[2]), sys.argv[3:]
rows = []
for name in names:
    base = load(name); _normalize_dataset_point_roles(base)
    truth = TRUTH[base["machine_type"]]
    pos = point_positions(base, truth)
    for s, sweep in enumerate(base["sweeps"]):
        low = [int(a) for a in sweep["fixed_anchors"] if int(a) != len(truth) - 1]
        moved = truth.copy()
        for a in low:
            radial = moved[a, :2] / np.linalg.norm(moved[a, :2])
            moved[a, :2] += d_mm * radial
        whole = resynthesise(base, truth, moved, positions=pos)
        data = copy.deepcopy(base); data["sweeps"][s] = whole["sweeps"][s]
        fit = fit_default(data)[0]
        j0, j1, dev = spreads(data, fit["anchors"])
        holders = {a: sum(a in [int(x) for x in sw["fixed_anchors"]] for sw in base["sweeps"]) for a in low}
        rows.append(dict(case=name, sweep=s, moved=low, holders=holders, d_mm=d_mm,
                         err=anchor_error(fit["anchors"], truth), J0=j0, J1=j1, fold_dev=dev))
        print(json.dumps(rows[-1]), flush=True)
out.write_text(json.dumps(rows, indent=1))
