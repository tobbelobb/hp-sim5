"""Deterministic synthetic sweep stress and bounded-update pilot experiments."""
import contextlib
import copy
import hashlib
import io
import json
import sys
import time
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))
from autocal import _autocal_common as common
from autocal.ellipse_cost import EllipseCostFunction
from autocal.spool_model import WinchSpoolModel, build_spool_model_params, dataset_with_modeled_lengths
from autocal.theoretical_ellipse import (compute_constraint_circle_2d, anchors_matrix_to_opt_vec,
    anchor_opt_vec_to_matrix, get_anchor_opt_bounds)
from audit_predictions import measure

SESSION = Path(__file__).parent
TRUE = np.array([[0.,-1900.],[1645.44826719,950.],[-1645.44826719,950.]])
RADIUS = 39.184


def generate(case, seed):
    rng=np.random.default_rng(seed)
    b=0. if case=='zero_buildup' else 0.636619
    spring=2000. if case=='elasticity' else 20000.
    dataset=dict(version='2.0',machine_type='slideprinter',num_anchors=3,dimensions=2,
        config=dict(angles_unit='deg',lengths_unit='mm',m666=dict(R=[30.]*3,Q=0.,U=[1.]*3,O=[1.]*3,
            L=[20.]*3,H=[20.]*3,S=spring,Y=[528.3]*3)),sweeps=[])
    model=WinchSpoolModel.from_firmware(base_radius=RADIUS,buildup_factor=b,
        spool_to_motor_gearing_factor=1.,mechanical_advantage=1.,lines_per_spool=1.)
    base=WinchSpoolModel.from_firmware(base_radius=30.,buildup_factor=0.,
        spool_to_motor_gearing_factor=1.,mechanical_advantage=1.,lines_per_spool=1.)
    for block,delta in enumerate((0.,-300.,350.,-150.)):
        for fixed in range(3):
            free=[i for i in range(3) if i!=fixed]
            circle=compute_constraint_circle_2d(TRUE,fixed,np.linalg.norm(TRUE[fixed])+delta)
            center_angle=np.arctan2(-circle.center[1],-circle.center[0])
            points=[]
            for idx,angle in enumerate(center_angle+np.linspace(-.25,.25,16)):
                pos=circle.center+circle.radius*np.array([np.cos(angle),np.sin(angle)])
                geometric=np.linalg.norm(TRUE-pos,axis=1)
                material=geometric.copy()
                if case=='elasticity':
                    # Fixed geometric constraint, known drive/sensor tension.
                    for axis,force in zip(free,(5.,2.)):
                        material[axis]=geometric[axis]*(1-force/spring)-force*528.3/spring
                delta_lengths=material-np.linalg.norm(TRUE,axis=1)
                raw=np.array([model.linepos_mm_to_theta_deg(x) for x in delta_lengths])
                noise=.4 if case=='noise' else .02
                raw+=rng.normal(0,noise*model.degrees_per_unit_times_r/RADIUS,3)
                if case=='outliers' and idx in (3,12):raw[free[1]]+=20.*model.degrees_per_unit_times_r/RADIUS
                base_delta=np.array([base.theta_deg_to_linepos_mm(x) for x in raw])
                points.append(dict(l_drive=float(base_delta[free[0]]),l_sensor=float(base_delta[free[1]]),
                    raw_angles_deg=raw.tolist(),assumed_tension_drive_n=5. if case=='elasticity' else 0.,
                    assumed_tension_sensor_n=2. if case=='elasticity' else 0.))
            fixed_base=base.theta_deg_to_linepos_mm(model.linepos_mm_to_theta_deg(delta))
            dataset['sweeps'].append(dict(id=f'sweep_{block}_{fixed}',fixed_anchors=[fixed],
                fixed_lengths=[float(fixed_base)],drive_anchor=free[0],sensor_anchor=free[1],data_points=points))
    return dataset,b


def residual_fn(dataset,b,flex=True):
    n=int(dataset['num_anchors']);dims=int(dataset['dimensions']);machine=str(dataset['machine_type'])
    lp=common._resolve_length_model_base_params(dataset,num_anchors=n,base_radii_override=None)
    def residual(x):
        anchors=anchor_opt_vec_to_matrix(x[:-1],machine,n,dims)
        params=build_spool_model_params(dataset,modeled_radii_mm=[x[-1]]*n,
            modeled_buildup_factor=[b]*n,**lp)
        modeled=dataset_with_modeled_lengths(dataset,params)
        fn=EllipseCostFunction(modeled,use_flex=flex,pointwise_filtering=False,sweep_wise_filtering=False,
            noise_normalized=False,use_noise_mean=False)
        rows=fn.pointwise_residual_rows(anchors.ravel())
        expected=sum(len(s['data_points']) for s in dataset['sweeps'])
        if len(rows)!=expected:return np.full(expected,1e4)
        # Fixed encoder scale prevents reducing residuals by shrinking radii.
        return np.array([row['residual_mm_signed'] for row in rows])*30./x[-1]
    return residual


def pilot(case,seed):
    dataset,b=generate(case,seed)
    name=f'{case}_seed_{seed}'
    out=SESSION/'synthetic'/name;out.mkdir(parents=True,exist_ok=True)
    (out/'input.json').write_text(json.dumps(dataset,indent=2))
    holdout=copy.deepcopy(dataset);holdout['sweeps']=dataset['sweeps'][-3:]
    # Entire fixed length configurations stay out of both fitting and validation.
    pool=dataset['sweeps'][:9]
    initial=TRUE* (0.65 if case=='poor_initial' else .94)
    x0=np.r_[anchors_matrix_to_opt_vec(initial,'slideprinter'),30. if case=='poor_initial' else 38.]
    lo,hi=get_anchor_opt_bounds('slideprinter',3,2);lo=np.r_[lo,20.];hi=np.r_[hi,55.]
    results=[]
    for mode in ('unbounded','bounded_only','validate_only','bounded_validate'):
        x=x0.copy();trace=[];fit_sweeps=pool[:3];start=time.perf_counter();movements=3*16
        for round_idx in range(3):
            fitting=copy.deepcopy(dataset);fitting['sweeps']=fit_sweeps
            fun=residual_fn(fitting,b)
            if mode in ('bounded_only','bounded_validate'):
                cap=np.r_[np.full(x.size-1,150.),3.]
                lower=np.maximum(lo,x-cap);upper=np.minimum(hi,x+cap)
            else:lower,upper=lo,hi
            fit=least_squares(fun,x,bounds=(lower,upper),loss='soft_l1',f_scale=.5,
                x_scale=np.r_[np.full(x.size-1,1000.),40.],max_nfev=50)
            proposal=fit.x
            validation=copy.deepcopy(dataset);validation['sweeps']=[pool[3+round_idx*2]]
            vf=residual_fn(validation,b)
            before=float(np.sqrt(np.mean(vf(x)**2)));after=float(np.sqrt(np.mean(vf(proposal)**2)))
            accepted=mode in ('unbounded','bounded_only') or after<before
            trace.append(dict(round=round_idx,prediction_before_mm=before,prediction_after_mm=after,
                proposal=proposal.tolist(),accepted=accepted,nfev=fit.nfev,validation_id=validation['sweeps'][0]['id']))
            if accepted:x=proposal
            fit_sweeps=fit_sweeps+pool[3+round_idx*2:5+round_idx*2]
            movements+=32
        anchors=anchor_opt_vec_to_matrix(x[:-1],'slideprinter',3,2)
        metrics=measure(holdout,anchors,[x[-1]]*3,buildup=b)
        row=dict(case=case,seed=seed,mode=mode,anchor_error_sum_mm=float(np.linalg.norm(anchors-TRUE,axis=1).sum()),
            radius_error_sum_mm=3*abs(x[-1]-RADIUS),elapsed_seconds=time.perf_counter()-start,
            movements=movements,anchors=anchors.tolist(),radii=[x[-1]]*3,trace=trace,**metrics)
        (out/f'{mode}.json').write_text(json.dumps(row,indent=2));results.append(row)
        print(case,seed,mode,round(row['anchor_error_sum_mm'],3),round(row['radius_error_sum_mm'],4),
            round(row['sensor_rmse_base_mm'],4),round(row['elapsed_seconds'],2),flush=True)
    return results


if __name__=='__main__':
    results=[]
    for case in ('clean','noise','outliers','elasticity','zero_buildup','poor_initial'):
        for seed in (7,19):
            results.extend(pilot(case,seed))
            (SESSION/'synthetic_results.json').write_text(json.dumps(results,indent=2))
