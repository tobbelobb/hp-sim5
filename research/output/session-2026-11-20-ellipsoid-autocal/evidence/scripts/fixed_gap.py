"""Max |fixed_lengths - target| per dataset: the double-counted held-length gap."""
import json, sys
import numpy as np
for path in sys.argv[1:]:
    d = json.load(open(path)); mpd = d["config"]["mm_per_degree"]; gaps = []
    for s in d["sweeps"]:
        p = s["data_points"][0] if s["data_points"] else None
        if not p or not p.get("fixed_target_angles_deg"): continue
        for a, L in zip(s["fixed_anchors"], s["fixed_lengths"]):
            t = p["fixed_target_angles_deg"][a]
            if t is not None: gaps.append(L - t * mpd[a])
    g = np.abs(gaps)
    print(f"{path:75s} sweeps {len(d['sweeps']):3d} max {g.max():.3f} mm mean {g.mean():.3f} mm  >0.02mm: {(g > 0.02).sum()}/{len(g)}")
