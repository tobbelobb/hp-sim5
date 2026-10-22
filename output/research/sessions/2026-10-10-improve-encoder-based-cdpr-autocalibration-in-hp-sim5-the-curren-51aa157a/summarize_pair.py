import json
from pathlib import Path
import numpy as np
from autocal.ellipse_cost import EllipseCostFunction

out=Path(__file__).parent
truth=np.array([[0,-1900,-280],[1645.45,950,-280],[-1645.45,950,-280],[0,0,1900]])
dirs=[out/"E07-no-layering-baseline",out/"E08-no-layering-reuse"]
metrics=[]
datasets=[]
for d in dirs:
    manifest=json.loads((d/"manifest.json").read_text())
    all_data=[]
    rows=[]
    for i in range(2):
        data=json.loads((d/f"sweeps-{i}.json").read_text())
        pts=[p for s in data["sweeps"] for p in s["data_points"]]
        all_data.append(data)
        cost=EllipseCostFunction(data,use_flex=False,pointwise_filtering=False,sweep_wise_filtering=False)
        residuals=cost.pointwise_residual_rows(truth.reshape(-1))
        rows.append(dict(batch=i,points=len(pts),predictions=len(residuals),
            truth_residual_rms_mm=float(np.sqrt(np.mean([r["residual_mm"]**2 for r in residuals]))),
            maximum_sigma_deg=max(max(p["sigma"]) for p in pts),
            max_fixed_drift_deg=max(abs(v) for p in pts for v in p["fixed_error_angles_deg"] if v is not None),
            noise_sample_counts=sorted(set(p["sample_count"] for p in pts)),
            max_collection_attempt=max(p["collection_attempt"] for p in pts)))
    datasets.append(all_data)
    metrics.append(dict(variant=d.name,manifest=manifest,batches=rows,
                        fits=json.loads((d/"fit-summary.json").read_text())))
diffs=[]
for i in range(2):
    aa=[p for s in datasets[0][i]["sweeps"] for p in s["data_points"]]
    bb=[p for s in datasets[1][i]["sweeps"] for p in s["data_points"]]
    diffs.append(dict(batch=i,max_raw_angle_difference_deg=max(abs(x-y) for a,b in zip(aa,bb)
        for x,y in zip(a["raw_angles_deg"],b["raw_angles_deg"])),
        max_mean_length_difference_mm=max(abs(a[k]-b[k]) for a,b in zip(aa,bb) for k in ("l_drive_mu","l_sensor_mu"))))
a,b=[x["manifest"]["simulated_s"] for x in metrics]
summary=dict(metrics=metrics,point_differences=diffs,time_saved_s=a-b,time_saved_percent=(a-b)/a*100)
(out/"E08-paired-comparison.json").write_text(json.dumps(summary,indent=2))
print(json.dumps({k:v for k,v in summary.items() if k!="metrics"},indent=2))
for m in metrics:print(m["variant"],m["batches"],[(x["mode"],x["anchor_rmse_mm"]) for x in m["fits"]])
