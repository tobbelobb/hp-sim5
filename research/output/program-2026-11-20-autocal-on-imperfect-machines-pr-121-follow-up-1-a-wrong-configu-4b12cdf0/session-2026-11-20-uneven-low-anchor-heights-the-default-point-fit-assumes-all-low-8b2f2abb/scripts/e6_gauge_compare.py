"""E6: default point fit, old equal-z gauge vs new vertical-top gauge, on the same datasets.

Run once with the old code (PYTHONPATH=<checkout at b249a1a5>) and once with the new
(PYTHONPATH=. on this branch); `--label` names the output. Error is against scene truth
in the convention frame (top anchor above home, A.x = 0; the benchmark truth already is).
Cases: anchor `--anchor` at each height, clean and with Gaussian range noise seeds.
"""
import argparse, json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
from lowz_common import TRUTH, load, fit_default, resynthesise, anchor_error, point_positions
import autocal.theoretical_ellipse as te

p = argparse.ArgumentParser()
p.add_argument("--label", required=True)
p.add_argument("--machine", default="hangprinter_3")
p.add_argument("--anchor", type=int, default=0)
p.add_argument("--heights", default="-20,-10,-2,0,2,10,20")
p.add_argument("--noise-mm", type=float, default=0.0)
p.add_argument("--seeds", type=int, default=1)
p.add_argument("--jackknife", action="store_true")
p.add_argument("--out", required=True)
args = p.parse_args()
truth = TRUTH[args.machine]
assert np.allclose(truth[-1, :2], 0) and truth[0, 0] == 0
data = load(f"benchmark_923c2baa_{args.machine}")
positions = point_positions(data, truth)
rows = []
for h in [float(v) for v in args.heights.split(",")]:
    new = truth.copy(); new[args.anchor, 2] += h
    for seed in range(args.seeds):
        ds = resynthesise(data, truth, new, positions, noise_mm=args.noise_mm, seed=seed)
        fit, _ = fit_default(ds)
        a = fit["anchors"]
        row = dict(h=h, seed=seed, err=anchor_error(a, new), dz_D=float(a[-1, 2] - new[-1, 2]),
                   low_z_err=(a[:-1, 2] - new[:-1, 2]).round(3).tolist(),
                   per_anchor=np.linalg.norm(a - new, axis=1).round(3).tolist(),
                   rms=fit["rms_mm"], scale=fit["robust_scale_mm"], rejected=len(fit["rejected_points"]),
                   offsets_skipped=fit.get("home_offsets_skipped"))
        if args.jackknife:
            from autocal.anchor_uncertainty import sweep_jackknife_mm
            row["jackknife"] = sweep_jackknife_mm(ds, a)
        rows.append(row)
        print(json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in row.items()}), flush=True)
Path(args.out).write_text(json.dumps(dict(args=vars(args), code=te.__file__, rows=rows), indent=1))
