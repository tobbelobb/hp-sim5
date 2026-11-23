"""E12: store leave-one-unit-out folds and fold information ratios for three test sets.

  live OUT RUN_DIR...            live runs (truth from input.json)
  inject OUT D_MM NAME...        reviewer 8457f4ee9842's single-sweep injection: the held low
                                 anchor of one sweep moves radially by D_MM for that sweep only
  corpus OUT FOLDS_JSONL I K     ratios for the reviewer's corpus folds (worker I of K); folds
                                 and anchors are taken from the reviewer's file unchanged
The fit is the sweep autocal's two-stage point fit (lowz_common.fit_default).
"""
import copy, json, re, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
from lowz_common import TRUTH, load, fit_default, anchor_error, resynthesise, point_positions
from structural import fold_ratios
from autocal._autocal_common import _normalize_dataset_point_roles
from autocal.dataset_roles import measurement_groups
from autocal.pose_fit import HOME_OFFSET_PRIOR_SIGMA_MM, fit_pose_least_squares

MAIN = Path("/home/torbjorn/repos/hp-sim5")


def folds_of(data, anchors):
    sweeps = data["sweeps"]; out = []
    for left in measurement_groups(data):
        sub = dict(data, sweeps=[s for i, s in enumerate(sweeps) if i not in left])
        fit = fit_pose_least_squares(sub, anchors, fit_home_offsets=True, home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM)
        if not fit["success"] or not np.all(np.isfinite(fit["anchors"])):
            return None
        out.append(fit["anchors"])
    return np.stack(out)


def record(label, data, truth, **extra):
    fit = fit_default(data)[0]; a = fit["anchors"]
    folds = folds_of(data, a); ratios, full = fold_ratios(data, a)
    row = dict(case=label, machine=data["machine_type"], err=anchor_error(a, truth), anchors=a.tolist(),
               folds=None if folds is None else folds.tolist(), ratios=ratios.tolist(), **extra)
    print(json.dumps({k: v for k, v in row.items() if k not in ("anchors", "folds", "ratios")}), flush=True)
    return row


mode, out = sys.argv[1], Path(sys.argv[2]); rows = []
if mode == "live":
    for run in map(Path, sys.argv[3:]):
        data = json.loads((run / "sweeps.json").read_text()); _normalize_dataset_point_roles(data)
        rows.append(record(run.name, data, np.asarray(json.loads((run / "input.json").read_text())["truth"])))
elif mode == "inject":
    d_mm = float(sys.argv[3])
    for name in sys.argv[4:]:
        base = load(name); _normalize_dataset_point_roles(base); truth = TRUTH[base["machine_type"]]
        pos = point_positions(base, truth)
        for s, sweep in enumerate(base["sweeps"]):
            low = [int(a) for a in sweep["fixed_anchors"] if int(a) != len(truth) - 1]
            moved = truth.copy()
            for a in low:
                moved[a, :2] += d_mm * moved[a, :2] / np.linalg.norm(moved[a, :2])
            whole = resynthesise(base, truth, moved, positions=pos)
            data = copy.deepcopy(base); data["sweeps"][s] = whole["sweeps"][s]
            holders = {a: sum(a in [int(x) for x in sw["fixed_anchors"]] for sw in base["sweeps"]) for a in low}
            rows.append(record(f"{name}#{s}", data, truth, sweep=s, d_mm=d_mm, holders=holders))
elif mode == "corpus":
    worker, count = int(sys.argv[4]), int(sys.argv[5])
    sys.path.insert(0, str(MAIN / "research/output/program-2026-11-20-autocal-on-imperfect-machines-pr-121-follow-up-1-a-wrong-configu-4b12cdf0/reviews/review-8457f4ee9842/review-evidence"))
    for ri, line in enumerate(open(sys.argv[3])):
        if ri % count != worker:
            continue
        run = json.loads(line); f = MAIN / run["f"]; base = f.name.replace(".full_auto_log.jsonl", "")
        data = json.loads((f.parent / f"{base}.json").read_text()); _normalize_dataset_point_roles(data)
        groups = measurement_groups(data)
        for it in run["iters"]:
            if it.get("folds") is None:
                continue
            keep = sorted(i for g in groups[:it["sweeps"]] for i in g)
            sub = dict(copy.deepcopy(data), sweeps=[data["sweeps"][i] for i in keep])
            try:
                it["ratios"] = fold_ratios(sub, np.asarray(it["anchors"]))[0].tolist()
            except Exception as error:  # singular prefixes: leave unflagged (classical behaviour)
                it["ratios_error"] = repr(error)[:120]
        rows.append(run); print(ri, run["f"], flush=True)
out.write_text(json.dumps(rows))
