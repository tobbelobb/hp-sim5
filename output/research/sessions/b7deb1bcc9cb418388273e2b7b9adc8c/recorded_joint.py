"""Offline joint anchor/radius prototype; no collector or production routing."""
import copy
import json
import sys
import time
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT))
from autocal.calibrate import build_anchor_initial_guess
from autocal.dataset_roles import normalize_dataset_point_roles
from autocal.theoretical_ellipse import anchors_matrix_to_opt_vec, anchor_opt_vec_to_matrix, get_anchor_opt_bounds
from autocal.tools import regress_calibration_logs as regression
from audit_predictions import measure
from synthetic_trials import residual_fn

SESSION=Path(__file__).parent
results=[]
for spec in regression.DATASETS:
    path=ROOT/'autocal/data/references'/f'{spec.name}.json'
    dataset=json.loads(path.read_text());normalize_dataset_point_roles(dataset)
    n=int(dataset['num_anchors']);dims=int(dataset['dimensions']);machine=spec.machine_type
    holdout=copy.deepcopy(dataset);holdout['sweeps']=dataset['sweeps'][-2:]
    initial_data=copy.deepcopy(dataset);initial_data['sweeps']=dataset['sweeps'][:3]
    initial=build_anchor_initial_guess(initial_data,machine_type=machine,base_radii_mm=[30.]*n)
    lo,hi=get_anchor_opt_bounds(machine,n,dims)
    lower_r,upper_r=(39.,40.) if dims==3 else (20.,55.)
    x0=np.r_[anchors_matrix_to_opt_vec(initial,machine),39.5 if dims==3 else 30.]
    lo=np.r_[lo,lower_r];hi=np.r_[hi,upper_r];x0=np.clip(x0,lo+1e-6,hi-1e-6)
    candidates=[]
    start=time.perf_counter()
    for train_count in range(3,len(dataset['sweeps'])-1):
        training=copy.deepcopy(dataset);training['sweeps']=dataset['sweeps'][:train_count]
        fun=residual_fn(training,.636619,flex=False)
        iteration_start=time.perf_counter()
        fit=least_squares(fun,x0,bounds=(lo,hi),loss='soft_l1',f_scale=.5,
            x_scale=np.r_[np.full(x0.size-1,1000.),40.],max_nfev=100)
        x0=fit.x
        anchors=anchor_opt_vec_to_matrix(x0[:-1],machine,n,dims)
        radii=[x0[-1]]*n
        physical=regression.error_to_true(anchors.tolist(),radii,spec.true_anchors,spec.true_radii)
        metrics=measure(holdout,anchors,radii,flex=False)
        row=dict(training_sweeps=train_count,anchor_error_sum_mm=physical[1],radius_error_sum_mm=physical[2]/(2*np.pi),
            total_error_mm=physical[0],elapsed_seconds=time.perf_counter()-iteration_start,nfev=fit.nfev,
            train_rmse_base_mm=float(np.sqrt(np.mean(fun(x0)**2))),anchors=anchors.tolist(),radii=radii,**metrics)
        candidates.append(row)
    best=min(candidates,key=lambda x:x['sensor_rmse_base_mm'])
    last=candidates[-1]
    print(spec.name,'last',round(last['total_error_mm'],3),'val',round(best['total_error_mm'],3),
        'sensor',round(best['sensor_rmse_base_mm'],3),'sec',round(time.perf_counter()-start,3),flush=True)
    results.append(dict(dataset=spec.name,input=str(path),holdout_ids=[s['id'] for s in holdout['sweeps']],
        candidates=candidates,validation_selected=best,last=last,elapsed_seconds=time.perf_counter()-start))
    (SESSION/'recorded_joint_results.json').write_text(json.dumps(results,indent=2))
