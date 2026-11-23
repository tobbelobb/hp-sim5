"""E1: anchor error of the default equal-height point fit when one low anchor is off.

Recorded benchmark sweeps are delta-resynthesised for a machine whose anchor `--anchor`
sits h mm higher. Reported per h: max anchor error vs scene truth, the same after the best
rotation about home (intrinsic shape error), the prediction of a pure tilt (level_frame),
fit rms and optionally the sweep jackknife.
"""
import argparse, json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
from lowz_common import TRUTH, load, fit_default, resynthesise, level_frame, rotation_aligned, anchor_error, tilt_deg, point_positions

p = argparse.ArgumentParser()
p.add_argument("--machine", default="hangprinter_3")
p.add_argument("--anchor", type=int, default=0)
p.add_argument("--heights", default="-50,-20,-10,-5,-2,0,2,5,10,20,50")
p.add_argument("--noise-mm", type=float, default=0.0)
p.add_argument("--seeds", type=int, default=1)
p.add_argument("--jackknife", action="store_true")
p.add_argument("--out")
args = p.parse_args()

truth = TRUTH[args.machine]
data = load(f"benchmark_923c2baa_{args.machine}")
positions = point_positions(data, truth)
rows = []
for h in [float(v) for v in args.heights.split(",")]:
    new = truth.copy(); new[args.anchor, 2] += h
    for seed in range(args.seeds):
        ds = resynthesise(data, truth, new, positions, noise_mm=args.noise_mm, seed=seed)
        fit, shifted = fit_default(ds)
        a = fit["anchors"]
        row = dict(h=h, seed=seed, err=anchor_error(a, new), err_rot=anchor_error(rotation_aligned(a, new), new),
                   err_vs_level=anchor_error(a, level_frame(new)), dz_D=float(a[-1, 2] - new[-1, 2]),
                   per_anchor=np.linalg.norm(a - new, axis=1).round(2).tolist(),
                   tilt_true_deg=tilt_deg(new), rms=fit["rms_mm"], scale=fit["robust_scale_mm"],
                   rejected=len(fit["rejected_points"]))
        if args.jackknife:
            from autocal.anchor_uncertainty import sweep_jackknife_mm
            row["jackknife"] = sweep_jackknife_mm(ds, a)
        rows.append(row)
        print(json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in row.items()}), flush=True)
if args.out:
    Path(args.out).write_text(json.dumps(dict(args=vars(args), rows=rows), indent=1))
