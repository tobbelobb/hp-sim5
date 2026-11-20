"""Write a copy of a dataset whose fixed_lengths are the targets its fixed error angles refer to."""
import json, sys
src, dst = sys.argv[1:3]
d = json.load(open(src)); mpd = d["config"]["mm_per_degree"]
for s in d["sweeps"]:
    p = s["data_points"][0] if s["data_points"] else None
    if p and p.get("fixed_target_angles_deg"):
        s["fixed_lengths"] = [p["fixed_target_angles_deg"][a] * mpd[a] if p["fixed_target_angles_deg"][a] is not None else L
                              for a, L in zip(s["fixed_anchors"], s["fixed_lengths"])]
json.dump(d, open(dst, "w"))
