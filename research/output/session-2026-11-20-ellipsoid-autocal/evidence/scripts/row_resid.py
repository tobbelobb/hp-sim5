import json, sys
import numpy as np
from autocal.dataset_roles import normalize_dataset_point_roles, mesh_metadata
from autocal.pose_fit import dataset_line_lengths, dataset_stretch, consistency_residuals
from autocal.tests.test_disturbed_robustness import TRUTH
machine, path = sys.argv[1], sys.argv[2]
d = json.load(open(path)); normalize_dataset_point_roles(d)
truth = np.asarray(TRUTH[machine], float)
L = dataset_line_lengths(d)["lengths_mm"]
offsets = np.zeros(truth.shape[0]); 
if len(sys.argv) > 3: offsets[-1] = float(sys.argv[3])
res = consistency_residuals(truth, L, offsets, stretch=dataset_stretch(d))
res = np.atleast_2d(res); per_point = np.sqrt(np.nanmean(res.reshape(L.shape[0], -1) ** 2, axis=1))
start = 0
for i, s in enumerate(d["sweeps"]):
    n = len(s["data_points"]); r = per_point[start:start + n]; start += n
    m = mesh_metadata(s)
    print(f"{i:2d} {m['id']} held {m['held_length_mm']:8.1f} fixed {s['fixed_anchors']} {[round(x,1) for x in s['fixed_lengths']]} drive {s['drive_anchor']} rms {np.sqrt(np.mean(r**2)):.3f} max {np.max(r):.3f}")
