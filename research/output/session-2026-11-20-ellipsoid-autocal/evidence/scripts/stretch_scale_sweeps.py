"""Same probe as stretch_scale.py, on the HP5 sweep benchmark with the sweep autocal's point fit."""
import json, sys
import numpy as np
import autocal.pose_fit as pf
from autocal.tests.test_disturbed_robustness import TRUTH, _fit
machine, path = sys.argv[1:3]
d = json.load(open(path)); truth = np.asarray(TRUTH[machine], float)
original = pf.dataset_stretch
class Scaled:
    def __init__(self, model, scale): self.model, self.scale = model, scale
    def corrected_distance_mm(self, absolute, tensions, axis):
        c = self.model.corrected_distance_mm(absolute, tensions, axis=axis); return absolute + self.scale[axis] * (c - absolute)
n = int(d["num_anchors"])
for label, scale in [(f"all lines x{k}", np.full(n, k)) for k in (0.9, 1.0, 1.1)] + [("top line x1.1", np.r_[np.ones(n - 1), 1.1]), ("free lines x1.1", np.r_[np.full(n - 1, 1.1), 1.0])]:
    def scaled(dataset, scale=scale):
        s = original(dataset)
        if s is not None: s["model"] = Scaled(s["model"], scale)
        return s
    pf.dataset_stretch = scaled
    r = _fit(d); e = np.asarray(r["anchors"]) - truth
    print(f"{label:20s} err {np.max(np.abs(e)):.2f} worst anchor error {np.round(e[np.argmax(np.max(np.abs(e), axis=1))], 2).tolist()}", flush=True)
