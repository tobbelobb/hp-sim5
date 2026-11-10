"""E02: equal-prefix known-radius fits; thinning is retrospective, not collection."""
import contextlib
import json
import time
from pathlib import Path
import numpy as np
from autocal.fit_stage import fit_ellipse_dataset
from autocal.stage_artifacts import read_stage_artifact

out=Path(__file__).parent
root=out.parents[2]/"extended-autocal/no-buildup-reference-003"
truth=np.array([[0,-1900,-280],[1645.45,950,-280],[-1645.45,950,-280],[0,0,1900]])
rows=[]
for iteration in (1,2,4,7):
    art=read_stage_artifact(root/f"sweeps.stages/fit-{iteration:03d}-default.json")["payload"]
    for mode in ("full","six","five","one_direction"):
        d=json.loads(json.dumps(art["fit"]["dataset"]))
        for sweep in d["sweeps"]:
            pts=sweep["data_points"]
            if mode!="full":
                groups=[pts[:10],pts[10:]]
                if mode=="one_direction":keep=groups[0]
                else:
                    n=6 if mode=="six" else 5
                    keep=[g[i] for g in groups for i in np.linspace(0,len(g)-1,n).round().astype(int)]
                sweep["data_points"]=keep
        p=out/f"E02-{iteration}-{mode}.json"
        p.write_text(json.dumps(d))
        options=dict(art["fit_options"],robust_debug=False,collector_args=[])
        started=time.perf_counter()
        with (out/f"E02-{iteration}-{mode}.log").open("w") as log,contextlib.redirect_stdout(log):
            result=fit_ellipse_dataset(p,**options)
        err=np.linalg.norm(np.array(result["anchors"])-truth,axis=1)
        row=dict(iteration=iteration,mode=mode,points=sum(len(s["data_points"]) for s in d["sweeps"]),
                 cost=result["cost"],anchor_rmse_mm=float(np.sqrt(np.mean(err**2))),
                 max_anchor_mm=float(err.max()),info_rank=result["info_rank"],
                 elapsed_s=time.perf_counter()-started,anchors=np.array(result["anchors"]).tolist())
        rows.append(row)
        (out/"E02-thinning.json").write_text(json.dumps(rows,indent=2))
        print(json.dumps({k:v for k,v in row.items() if k!="anchors"}),flush=True)
