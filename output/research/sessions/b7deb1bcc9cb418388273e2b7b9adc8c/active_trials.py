"""Isolate informative movement selection in the offline predict/update pilot."""
import copy
import json
import time
import numpy as np
from scipy.optimize import least_squares
from synthetic_trials import generate, residual_fn, TRUE, RADIUS, SESSION
from audit_predictions import measure
from autocal.active_learning import dataset_sweep_configs, rank_candidates_d_optimal, l2_scale_for_machine
from autocal.theoretical_ellipse import anchors_matrix_to_opt_vec, anchor_opt_vec_to_matrix, get_anchor_opt_bounds
from autocal import _autocal_common as common
from autocal.spool_model import build_spool_model_params, sweep_configs_with_modeled_lengths

rows=[]
for case in ('noise','outliers','poor_initial'):
    for seed in (7,19):
        data,b=generate(case,seed)
        holdout=copy.deepcopy(data);holdout['sweeps']=data['sweeps'][-3:]
        for selection in ('fixed','d_optimal'):
            start=time.perf_counter()
            remaining=copy.deepcopy(data['sweeps'][3:9]);train=copy.deepcopy(data['sweeps'][:3]);trace=[]
            initial=TRUE*(.65 if case=='poor_initial' else .94)
            x=np.r_[anchors_matrix_to_opt_vec(initial,'slideprinter'),30. if case=='poor_initial' else 38.]
            lo,hi=get_anchor_opt_bounds('slideprinter',3,2);lo=np.r_[lo,20.];hi=np.r_[hi,55.]
            for iteration in range(3):
                training=copy.deepcopy(data);training['sweeps']=train
                anchors=anchor_opt_vec_to_matrix(x[:-1],'slideprinter',3,2)
                if selection=='d_optimal':
                    pool=copy.deepcopy(data);pool['sweeps']=remaining
                    lp=common._resolve_length_model_base_params(training,num_anchors=3,base_radii_override=None)
                    params=build_spool_model_params(training,modeled_radii_mm=[x[-1]]*3,
                        modeled_buildup_factor=[b]*3,**lp)
                    configs=sweep_configs_with_modeled_lengths(dataset_sweep_configs(pool),params)
                    ranked=rank_candidates_d_optimal(anchors,machine_type='slideprinter',num_anchors=3,dimensions=2,
                        l2_scale=l2_scale_for_machine('slideprinter',3,2),
                        observed=sweep_configs_with_modeled_lengths(dataset_sweep_configs(training),params),
                        candidates=configs,exclude_existing=True,top_k=0)
                    keys=[cfg.normalized_key() for _,cfg in ranked]
                    order={s['id']:keys.index(cfg.normalized_key()) for s,cfg in zip(remaining,configs)}
                    remaining.sort(key=lambda s:order[s['id']])
                informative=copy.deepcopy(data);informative['sweeps']=[remaining[0]]
                informative_residual=float(np.sqrt(np.mean(residual_fn(informative,b)(x)**2)))
                # Observe the informative sweep, fit it, then validate on another
                # entire configuration that has not entered this proposal fit.
                training['sweeps']=train+[remaining[0]]
                validation=copy.deepcopy(data);validation['sweeps']=[remaining[1]]
                vf=residual_fn(validation,b);before=float(np.sqrt(np.mean(vf(x)**2)))
                cap=np.r_[np.full(x.size-1,150.),3.]
                lower=np.maximum(lo,x-cap);upper=np.minimum(hi,x+cap)
                fit=least_squares(residual_fn(training,b),x,bounds=(lower,upper),loss='soft_l1',f_scale=.5,
                    x_scale=np.r_[np.full(x.size-1,1000.),40.],max_nfev=50)
                after=float(np.sqrt(np.mean(vf(fit.x)**2)));accepted=after<before
                if accepted:x=fit.x
                trace.append(dict(iteration=iteration,informative_sweep=remaining[0]['id'],
                    informative_encoder_residual_mm=informative_residual,validation_sweep=remaining[1]['id'],prediction_before_mm=before,
                    prediction_after_mm=after,accepted=accepted,nfev=fit.nfev))
                train+=remaining[:2];remaining=remaining[2:]
            anchors=anchor_opt_vec_to_matrix(x[:-1],'slideprinter',3,2)
            row=dict(case=case,seed=seed,selection=selection,elapsed_seconds=time.perf_counter()-start,movements=144,
                anchor_error_sum_mm=float(np.linalg.norm(anchors-TRUE,axis=1).sum()),radius_error_sum_mm=3*abs(x[-1]-RADIUS),
                trace=trace,**measure(holdout,anchors,[x[-1]]*3,buildup=b))
            rows.append(row)
            print(case,seed,selection,round(row['anchor_error_sum_mm'],3),round(row['sensor_rmse_base_mm'],4),flush=True)
            (SESSION/'active_results.json').write_text(json.dumps(rows,indent=2))
