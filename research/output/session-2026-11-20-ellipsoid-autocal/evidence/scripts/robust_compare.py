"""Clean vs disturbed anchor error of one dataset under one fitter.
usage: robust_compare.py LABEL MACHINE DATASET FITTER [seeds]
FITTER: sweep (the sweep autocal's two-stage point fit), ellipsoid, point (ellipsoid_autocal.fit)."""
import copy, json, sys
import numpy as np
from autocal.dataset_roles import normalize_dataset_point_roles
from autocal.anchor_seed import build_anchor_initial_guess
from autocal.tools.disturb_dataset import apply_events, scenario_events
from autocal.tests.test_disturbed_robustness import TRUTH, _fit as sweep_fit
from autocal.ellipsoid_autocal import fit, SEED_SCALES
label, machine, path, fitter = sys.argv[1:5]
seeds = int(sys.argv[5]) if len(sys.argv) > 5 else 3
base = json.load(open(path)); base.setdefault("machine_type", machine); normalize_dataset_point_roles(base)
truth = np.asarray(TRUTH[machine], float)
def err(dataset):
    dataset = copy.deepcopy(dataset)
    if fitter == "sweep":
        anchors = sweep_fit(dataset)["anchors"]
    else:
        seed = build_anchor_initial_guess(dataset, machine_type=machine)
        anchors = fit(dataset, [seed * s for s in SEED_SCALES], None, True, fitter)["anchors"]
    return float(np.max(np.abs(np.asarray(anchors) - truth)))
out = {"label": label, "machine": machine, "fitter": fitter, "clean": err(base)}
for name, scenario, kw in (("mixed", "mixed", {}), ("noise0.05", "noise", {"noise_mm": 0.05}),
                           ("noise0.2", "noise", {"noise_mm": 0.2}), ("mixed+noise0.2", "mixed", {"noise_mm": 0.2})):
    out[name] = [err(apply_events(base, truth, scenario_events(base, scenario, seed=s, **kw))) for s in range(seeds)]
print(json.dumps(out), flush=True)
