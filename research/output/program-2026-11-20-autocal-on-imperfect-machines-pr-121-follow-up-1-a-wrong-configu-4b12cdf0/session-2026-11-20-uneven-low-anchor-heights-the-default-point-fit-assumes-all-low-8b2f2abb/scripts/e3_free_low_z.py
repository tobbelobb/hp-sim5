"""E3: per-anchor low z on machines with 4 low anchors, and an equal-z vs free-z model test.

The free model adds the low-z deviations that no rotation about home can produce: the
part of (z_i) orthogonal to {1, x_i, y_i}. HP5 has one such mode (a warp). Tilt stays
fixed by gauge: the fitted low anchors' best plane is horizontal ("level frame").
Both models are refitted from the equal-z default fit's home-offset-shifted dataset with
a plain least-squares (clean data). Model test: F = (C_eq - C_free)/k / (C_free/(N-p)).
"""
import argparse, json, sys
from pathlib import Path
import numpy as np
from scipy.stats import f as fdist
sys.path.insert(0, str(Path(__file__).parent))
from lowz_common import TRUTH, load, fit_default, resynthesise, rotation_aligned, anchor_error, point_positions, level_frame_ls, refit


p = argparse.ArgumentParser()
p.add_argument("--machine", default="hangprinter_5")
p.add_argument("--anchor", type=int, default=0)
p.add_argument("--heights", default="-20,-5,-2,-1,0,1,2,5,20")
p.add_argument("--noise-mm", type=float, default=0.0)
p.add_argument("--seeds", type=int, default=1)
p.add_argument("--out")
args = p.parse_args()
truth = TRUTH[args.machine]; data = load(f"benchmark_923c2baa_{args.machine}"); positions = point_positions(data, truth)
rows = []
for h in [float(v) for v in args.heights.split(",")]:
    new = truth.copy(); new[args.anchor, 2] += h
    for seed in range(args.seeds):
        ds = resynthesise(data, truth, new, positions, noise_mm=args.noise_mm, seed=seed)
        fit, shifted = fit_default(ds)
        eq, c_eq, N, _, _ = refit(shifted, fit["anchors"], False)
        fr, c_fr, _, pfree, k = refit(shifted, fit["anchors"], True)
        F = (c_eq - c_fr) / k / (c_fr / (N - pfree))
        row = dict(h=h, seed=seed, default_err=anchor_error(fit["anchors"], new),
                   eq_err=anchor_error(eq, new), free_err=anchor_error(fr, new),
                   free_err_rot=anchor_error(rotation_aligned(fr, new), new),
                   free_err_vs_level=anchor_error(fr, level_frame_ls(new)),
                   eq_rms=float(np.sqrt(c_eq / N)), free_rms=float(np.sqrt(c_fr / N)), F=float(F),
                   p_value=float(fdist.sf(F, k, N - pfree)),
                   free_low_z=fr[:-1, 2].round(2).tolist())
        rows.append(row)
        print(json.dumps({kk: (round(v, 4) if isinstance(v, float) else v) for kk, v in row.items()}), flush=True)
if args.out:
    Path(args.out).write_text(json.dumps(dict(args=vars(args), rows=rows), indent=1))
