# Effector tilt, tip displacement caused by tilt, and servo error stats from logo_qc runs.
import json, sys
import numpy as np
from scipy.spatial.transform import Rotation as R
for f in sys.argv[1:]:
    d = json.load(open(f)); s = d['sub']
    q = np.array(s['quat']); ph = np.array(s['phase'])
    rot = R.from_quat(q)
    tip = rot.apply([0, 0, -0.1]) - np.array([0, 0, -0.1])
    ang = np.degrees(rot.magnitude())
    th = np.linalg.norm(tip[:, :2], axis=1) * 1e3
    print(f"{d['label']:12s} tilt deg mean {ang.mean():.3f} max {ang.max():.3f} | tilt-induced tip shift mm mean {th.mean():.3f} max {th.max():.3f} mean vec {np.round(tip.mean(0)*1e3,3)}"
          f" | servo err rad p99 {np.round(np.percentile(abs(ph),99,axis=0),4)} axes {d['axes']}")
