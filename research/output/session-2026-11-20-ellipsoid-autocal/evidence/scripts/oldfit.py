"""Fit a dataset with whichever ellipsoid_autocal.fit is on sys.path (old or new commit)."""
import json, sys, inspect
import numpy as np
from autocal.dataset_roles import normalize_dataset_point_roles
from autocal.ellipsoid_autocal import fit, SEED_SCALES
from autocal.anchor_seed import build_anchor_initial_guess
sys.path.insert(0, "/home/torbjorn/repos/hp-sim5")
from autocal.tests.test_disturbed_robustness import TRUTH
machine, path = sys.argv[1:3]
d = json.load(open(path)); normalize_dataset_point_roles(d)
seed = build_anchor_initial_guess(d, machine_type=machine)
r = fit(d, [seed * s for s in SEED_SCALES], None, True)
e = np.asarray(r["anchors"]) - np.asarray(TRUTH[machine], float)
print(machine, path.split("/")[-2], "err", round(float(np.max(np.abs(e))), 2), "rms", round(r["rms_mm"], 4), np.round(e, 2).tolist())
