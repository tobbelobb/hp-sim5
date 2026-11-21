"""E11: current sweep jackknife vs a robust (median) spread of the same folds, against true error.

Folds are exactly anchor_uncertainty.sweep_jackknife_mm's (leave one measurement group out,
home offsets on, start at the full fit). Per coordinate:
  J0 = sqrt((n-1)/n * sum (theta_i - mean)^2)               (current)
  J1 = sqrt(n-1) * 1.4826 * median |theta_i - median theta|  (robust; equals J0 for normal folds)
Both report the max over coordinates. Inputs: live run dirs (truth from input.json) and
reference dataset names (truth = benchmark truth). The fit is the sweep autocal's two-stage fit.
"""
import json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
from lowz_common import TRUTH, load, fit_default, anchor_error
from autocal.dataset_roles import measurement_groups
from autocal.pose_fit import HOME_OFFSET_PRIOR_SIGMA_MM, fit_pose_least_squares

def estimators(data, anchors):
    sweeps = data["sweeps"]; folds = []
    for left in measurement_groups(data):
        sub = dict(data, sweeps=[s for i, s in enumerate(sweeps) if i not in left])
        folds.append(fit_pose_least_squares(sub, anchors, fit_home_offsets=True,
                                            home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM)["anchors"])
    f = np.stack(folds); n = len(f)
    j0 = np.sqrt((n - 1) / n * np.sum((f - f.mean(0)) ** 2, axis=0))
    j1 = np.sqrt(n - 1) * 1.4826 * np.median(np.abs(f - np.median(f, axis=0)), axis=0)
    return float(j0.max()), float(j1.max()), n

out = Path(sys.argv[1]); rows = []
for item in sys.argv[2:]:
    item, _, prefix = item.partition(":")  # "name:k" keeps only the first k measurement groups
    path = Path(item)
    if path.is_dir():
        data = json.loads((path / "sweeps.json").read_text()); truth = np.asarray(json.loads((path / "input.json").read_text())["truth"])
        label = path.name
    else:
        data = load(item); truth = TRUTH[data["machine_type"]]; label = item
    import copy
    from autocal._autocal_common import _normalize_dataset_point_roles
    data = copy.deepcopy(data); _normalize_dataset_point_roles(data)
    if prefix:
        keep = sorted(i for group in measurement_groups(data)[:int(prefix)] for i in group)
        data["sweeps"] = [data["sweeps"][i] for i in keep]; label += f":{prefix}"
    fit = fit_default(data)[0]
    j0, j1, n = estimators(data, fit["anchors"])
    rows.append(dict(case=label, machine=data["machine_type"], units=n, err=anchor_error(fit["anchors"], truth), J0=j0, J1=j1))
    print(json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in rows[-1].items()}), flush=True)
out.write_text(json.dumps(rows, indent=1))
