"""E10: which anchor coordinates drive the sweep jackknife (per-coordinate std, per-fold anchors)."""
import json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
from lowz_common import fit_default, anchor_error
from autocal.dataset_roles import measurement_groups
from autocal.pose_fit import HOME_OFFSET_PRIOR_SIGMA_MM, fit_pose_least_squares
rows = []
for run in map(Path, sys.argv[2:]):
    data = json.loads((run / "sweeps.json").read_text()); truth = np.asarray(json.loads((run / "input.json").read_text())["truth"])
    full = fit_default(data)[0]["anchors"]
    sweeps = data["sweeps"]; folds = []
    for left in measurement_groups(data):
        sub = dict(data, sweeps=[s for i, s in enumerate(sweeps) if i not in left])
        fit = fit_pose_least_squares(sub, full, fit_home_offsets=True, home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM)
        folds.append(fit["anchors"])
        rows.append(dict(run=run.name, left_out=sorted(left), fold_err=anchor_error(fit["anchors"], truth),
                         fold_minus_full=(fit["anchors"] - full).round(2).tolist(), skipped=fit.get("home_offsets_skipped")))
    folds = np.stack(folds); n = len(folds)
    std = np.sqrt((n - 1) / n * np.sum((folds - folds.mean(0)) ** 2, axis=0))
    rows.append(dict(run=run.name, full_err=anchor_error(full, truth), std=std.round(2).tolist(), max_std=float(std.max())))
    for r in rows[-n - 1:]:
        print(json.dumps(r), flush=True)
Path(sys.argv[1]).write_text(json.dumps(rows, indent=1))
