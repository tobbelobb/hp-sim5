import json, sys
import numpy as np
from autocal.dataset_roles import normalize_dataset_point_roles, mesh_metadata
from autocal.ellipsoid_autocal import fit, SEED_SCALES
from autocal.anchor_seed import build_anchor_initial_guess
from autocal.tests.test_disturbed_robustness import TRUTH
machine, path = sys.argv[1], sys.argv[2]
d = json.load(open(path)); normalize_dataset_point_roles(d)
truth = np.asarray(TRUTH[machine], float)
def h(s): return mesh_metadata(s)["held_length_mm"]
for lo in (-2000, -1400, -1000, -500):
    sub = dict(d, sweeps=[s for s in d["sweeps"] if h(s) > lo])
    seed = build_anchor_initial_guess(sub, machine_type=machine)
    r = fit(sub, [seed * s for s in SEED_SCALES], None, True, "ellipsoid")
    print(machine, f"held > {lo}", len(sub["sweeps"]), "rows", "err", round(float(np.max(np.abs(r["anchors"] - truth))), 2), flush=True)
