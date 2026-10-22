import collections
import json
from pathlib import Path
import numpy as np
from autocal.stage_artifacts import read_stage_artifact
from autocal.history_selection import rank_history_candidates

out = Path(__file__).parent
root = out.parents[2] / "extended-autocal/no-buildup-reference-003"
events = [json.loads(l) for l in (out / "E01-events.jsonl").read_text().splitlines()]
sends = sorted((e for e in events if e["kind"] == "gcode_send"), key=lambda e: e["wall_time_ms"])
costs = collections.defaultdict(lambda: dict(count=0, wall_s=0., sim_s=0.))
for a, b in zip(sends, sends[1:]):
    if a["session"] != b["session"]:
        continue
    src = a["payload"]["source"]
    key = f"{src['file']}:{src['line']}"
    x = costs[key]
    x["count"] += 1
    x["wall_s"] += (b["wall_time_ms"]-a["wall_time_ms"])/1000
    if a.get("sim_time_s") is not None and b.get("sim_time_s") is not None:
        x["sim_s"] += b["sim_time_s"]-a["sim_time_s"]
collections_wall = []
beg = None
for e in sorted(events, key=lambda e:e["wall_time_ms"]):
    if e["source"]=="python" and e["kind"]=="collection_start":
        beg=e["wall_time_ms"]
    if e["source"]=="python" and e["kind"]=="collection_exit":
        collections_wall.append((e["wall_time_ms"]-beg)/1000)
history = read_stage_artifact(root/"sweeps.stages/history-007.json")["payload"]
truth=np.array([[0,-1900,-280],[1645.45,950,-280],[-1645.45,950,-280],[0,0,1900.]])
rows=[]
for c in history["candidates"]:
    a=np.array(c["plan"]["anchors"])
    err=np.linalg.norm(a-truth,axis=1)
    rows.append(dict(iteration=c["iteration"],points=sum(len(s["data_points"]) for s in c["plan"]["dataset"]["sweeps"]),
                     score=c["rank_score"],history_score=c["history_rank_score"],
                     anchor_rmse_mm=float(np.sqrt(np.mean(err**2))),anchor_sum_mm=float(err.sum()),
                     max_anchor_mm=float(err.max()),D_height_mm=float(a[3,2])))
chosen=rank_history_candidates(history["evaluated"])[0][1]["iteration"]
summary=dict(collection_wall_s=collections_wall, command_intervals=sorted(
    [dict(source=k,**v) for k,v in costs.items()],key=lambda x:-x["sim_s"]),
    geometry=rows,chosen_iteration=chosen)
(out/"E01-profile-and-geometry.json").write_text(json.dumps(summary,indent=2))
print(json.dumps(summary,indent=2))
