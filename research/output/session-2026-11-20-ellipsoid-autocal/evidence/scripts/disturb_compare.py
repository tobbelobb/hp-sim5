"""Point vs ellipsoid fit on an ellipsoid-shape dataset, clean and disturbed.
usage: disturb_compare.py MACHINE DATASET [seeds]"""
import json, sys
import numpy as np
from autocal.dataset_roles import normalize_dataset_point_roles
from autocal.ellipsoid_autocal import fit, SEED_SCALES
from autocal.anchor_seed import build_anchor_initial_guess
from autocal.tools.disturb_dataset import apply_events, scenario_events
from autocal.tests.test_disturbed_robustness import TRUTH
machine, path = sys.argv[1], sys.argv[2]
seeds = int(sys.argv[3]) if len(sys.argv) > 3 else 3
base = json.load(open(path)); normalize_dataset_point_roles(base)
truth = np.asarray(TRUTH[machine], float)
def errors(dataset):
    seed = build_anchor_initial_guess(dataset, machine_type=machine)
    starts = [seed * s for s in SEED_SCALES]
    out = {}
    for model in ("ellipsoid", "point"):
        r = fit(dataset, starts, None, True, model)
        out[model] = float(np.max(np.abs(r["anchors"] - truth)))
    return out
print(machine, "clean", {k: round(v, 2) for k, v in errors(base).items()}, flush=True)
for scenario, kw in (("mixed", {}), ("noise", {"noise_mm": 0.2}), ("noise", {"noise_mm": 0.5}), ("mixed", {"noise_mm": 0.2})):
    rows = [errors(apply_events(base, truth, scenario_events(base, scenario, seed=s, **kw))) for s in range(seeds)]
    print(machine, scenario, kw, {m: f"mean {np.mean([r[m] for r in rows]):.2f} max {np.max([r[m] for r in rows]):.2f}" for m in rows[0]}, flush=True)
