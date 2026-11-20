"""Scale the stretch correction (all lines, or one line) and refit: does a stiffness/force mismatch explain the misfit?
usage: stretch_scale.py MACHINE DATASET"""
import json, sys
import numpy as np
import autocal.pose_fit as pf, autocal.ellipse_fit as ef
from autocal.dataset_roles import normalize_dataset_point_roles
from autocal.ellipsoid_autocal import fit, SEED_SCALES, held_anchor
from autocal.anchor_seed import build_anchor_initial_guess
from autocal.tests.test_disturbed_robustness import TRUTH
machine, path = sys.argv[1:3]
d = json.load(open(path)); normalize_dataset_point_roles(d)
truth = np.asarray(TRUTH[machine], float)
original = pf.dataset_stretch
class Scaled:
    def __init__(self, model, scale): self.model, self.scale = model, scale
    def corrected_distance_mm(self, absolute, tensions, axis):
        corrected = self.model.corrected_distance_mm(absolute, tensions, axis=axis)
        return absolute + self.scale[axis] * (corrected - absolute)
def run(label, scale):
    def scaled(dataset):
        s = original(dataset); s["model"] = Scaled(s["model"], scale); return s
    ef.dataset_stretch = scaled
    seed = build_anchor_initial_guess(d, machine_type=machine)
    r = fit(d, [seed * s for s in SEED_SCALES] + [truth], None, True, "ellipsoid")
    e = r["anchors"] - truth
    print(f"{label:28s} err {np.max(np.abs(e)):.2f} rms {r['rms_mm']:.4f} held-anchor error {np.round(e[held_anchor(machine)], 2).tolist()}", flush=True)
n = int(d["num_anchors"]); held = held_anchor(machine)
for k in (0.9, 0.95, 1.0, 1.05, 1.1):
    run(f"all lines x{k}", np.full(n, k))
for k in (0.9, 1.1):
    s = np.ones(n); s[held] = k; run(f"held line x{k}", s)
    s = np.full(n, k); s[held] = 1.0; run(f"free lines x{k}", s)
