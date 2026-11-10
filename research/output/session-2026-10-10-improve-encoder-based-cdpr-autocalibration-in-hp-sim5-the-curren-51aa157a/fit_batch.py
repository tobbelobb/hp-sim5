import contextlib
import json
import sys
import time
from pathlib import Path
import numpy as np
from autocal._autocal_common import _merge_sweep_datasets
from autocal.fit_stage import fit_ellipse_dataset
from autocal.stage_artifacts import read_stage_artifact
from autocal.predictive_validation import score_future_sweeps

out=Path(__file__).parent
root=out.parents[2]/"extended-autocal/no-buildup-reference-003"
truth=np.array([[0,-1900,-280],[1645.45,950,-280],[-1645.45,950,-280],[0,0,1900]])
options=read_stage_artifact(root/"sweeps.stages/fit-004-default.json")["payload"]["fit_options"]
options.update(robust_debug=False,collector_args=[])
variant=sys.argv[1]
directory=out/variant
first=json.loads((directory/"sweeps-0.json").read_text())
second=json.loads((directory/"sweeps-1.json").read_text())
merged=_merge_sweep_datasets(first,second)
merged_path=directory/"merged.json"
merged_path.write_text(json.dumps(merged))
summary_path=directory/"fit-summary.json"
rows=json.loads(summary_path.read_text()) if summary_path.exists() else []
for row in rows:
    previous=row.pop("heldout_second_three",None)
    if previous is not None:
        row["second_three_prediction"]=previous
        row["second_three_heldout"]=row["mode"]=="first_three"
for mode,p in (("first_three",directory/"sweeps-0.json"),("second_three",directory/"sweeps-1.json"),("all_six",merged_path)):
    if any(row["mode"]==mode for row in rows):
        continue
    started=time.perf_counter()
    opt=dict(options)
    attempts=[]
    for recovery in ("default","restore_sweeps","restore_points"):
        with (directory/f"fit-{mode}-{recovery}.log").open("w") as log,contextlib.redirect_stdout(log):
            result=fit_ellipse_dataset(p,**opt)
        err=np.linalg.norm(np.array(result["anchors"])-truth,axis=1)
        attempts.append(dict(recovery=recovery,cost=float(result["cost"]),anchor_rmse_mm=float(np.sqrt(np.mean(err**2)))))
        if result["cost"] < 100.:break
        opt.update(sweep_wise_filtering=False,initial_guess=result["anchors"])
        if recovery=="restore_sweeps":opt["pointwise_filtering"]=False
    training=first if mode=="first_three" else second if mode=="second_three" else merged
    frozen=score_future_sweeps(merged,np.array(result["anchors"]),
        [s["id"] for s in training["sweeps"]],use_flex=False)
    rows.append(dict(mode=mode,attempts=attempts,anchors=np.array(result["anchors"]).tolist(),
        anchor_rmse_mm=float(np.sqrt(np.mean(err**2))),max_anchor_mm=float(err.max()),
        cost=float(result["cost"]),heldout_prediction=frozen,elapsed_s=time.perf_counter()-started))
summary_path.write_text(json.dumps(rows,indent=2))
print(json.dumps(rows,indent=2))
