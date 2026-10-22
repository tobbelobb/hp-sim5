import copy, hashlib, json, sys, time
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path.cwd()))
from autocal.tools.regress_calibration_logs import DATASETS, parse_log_file, resolve_reference_log_path, history_ranking_audit
from autocal._autocal_common import _resolve_length_model_base_params
from autocal.dataset_roles import normalize_dataset_point_roles
from autocal.spool_model import build_spool_model_params, dataset_with_modeled_lengths
from autocal.ellipse_cost import EllipseCostFunction
from autocal.predictive_validation import score_future_sweeps
from autocal.theoretical_ellipse import anchors_matrix_to_opt_vec
OUT=Path(__file__).resolve().parent

def model(d,r,k=.636619):
 p=_resolve_length_model_base_params(d,num_anchors=d['num_anchors'],base_radii_override=[30.]*d['num_anchors'])
 return dataset_with_modeled_lengths(d,build_spool_model_params(d,**p,modeled_radii_mm=r,modeled_buildup_factor=[k]*d['num_anchors']))

def metrics(d,a):
 cost=EllipseCostFunction(d,pointwise_filtering=False,sweep_wise_filtering=False,use_flex=False,robust_loss=True)
 cost._min_sweeps_after_trim=1
 vec=anchors_matrix_to_opt_vec(a,d['machine_type'])
 rows=cost.pointwise_residual_rows(vec)
 n=sum(len(s['data_points']) for s in d['sweeps'])
 if len(rows)!=n or not rows:return {'objective':float(cost.evaluate(vec)), 'coverage':len(rows)/max(n,1),'rms_mm':None,'robust_mm':None}
 residual=np.array([r['residual_mm'] for r in rows])
 # Fixed 1 mm pseudo-Huber, equal-weight sweeps: no candidate-dependent noise or filtering.
 values=[np.mean(2*(np.hypot(1.,np.array([r['residual_mm'] for r in rows if r['sweep_id']==s['id']]))-1)) for s in d['sweeps']]
 return {'objective':float(cost.evaluate(vec)),'coverage':1.,'rms_mm':float(np.sqrt(np.mean(residual**2))), 'robust_mm':float(np.sqrt(np.mean(values))), 'p95_mm':float(np.percentile(residual,95))}

def error(spec,a,r):
 truth=np.array(spec.true_anchors)[:,:a.shape[1]]
 return {'anchor_rmse_mm':float(np.sqrt(np.mean(np.sum((a-truth)**2,axis=1)))),'radius_rmse_mm':float(np.sqrt(np.mean((r-spec.true_radii)**2)))}

def audit():
 results=[]
 for spec in DATASETS:
  source=Path('autocal/data/references')/f'{spec.name}.json'
  d=json.loads(source.read_text());normalize_dataset_point_roles(d)
  ref=resolve_reference_log_path(spec.name,source.parent,Path('autocal/data'),())
  parsed=parse_log_file(ref); items=[i for i in parsed.iterations if i.anchors and i.radii]
  observed=dict(d,sweeps=d['sweeps'][:len(items)+2])
  latest=model(observed,items[-1].radii)
  candidates=[]
  for idx,i in enumerate(items):
   a=np.array(i.anchors)[:,:d['dimensions']];r=np.array(i.radii)
   own=model(observed,r)
   fold=dict(own,sweeps=own['sweeps'][idx+3:]);common=dict(own,sweeps=own['sweeps'][-1:])
   c={'iteration':idx+1,'anchors':a.tolist(),'radii':r.tolist(),**error(spec,a,r),'rank':i.history_rank_score,'own_future':metrics(fold,a) if fold['sweeps'] else None,'common':metrics(common,a) if idx<len(items)-1 else None}
   try:c['latest_future']=score_future_sweeps(latest,a,[s['id'] for s in observed['sweeps'][:idx+3]])
   except Exception as e:c['failure']=repr(e)
   candidates.append(c)
  selections={}
  for key,sub in [('latest_future',None),('own_future','objective'),('own_future','robust_mm'),('common','objective'),('common','robust_mm')]:
   valid=[c for c in candidates if c.get(key) is not None and (sub is None or c[key][sub] is not None)]
   if valid:
    chosen=min(valid,key=lambda c:c[key][0] if sub is None else c[key][sub])
    selections[f'{key}_{sub}']={k:chosen[k] for k in ('iteration','anchor_rmse_mm','radius_rmse_mm')}
  baseline=error(spec,np.array(parsed.summary.anchors)[:,:d['dimensions']],np.array(parsed.summary.radii))
  result={'dataset':spec.name,'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'reference':str(ref),'audit':history_ranking_audit(parsed.iterations,spec),'baseline_summary':baseline,'observed_sweeps':len(observed['sweeps']),'candidates':candidates,'selections':selections}
  results.append(result)
  print(json.dumps({k:v for k,v in result.items() if k!='candidates'}),flush=True)
 (OUT/'E01-audit.json').write_text(json.dumps(results,indent=2)+'\n')
if __name__=='__main__':audit()

def sensor_metrics(raw,anchors,radii,buildup=.636619):
 """Sensor-only two-branch prediction error; disclose missing/invalid predictions.

 Without a measured effector pose the circle has two sensor solutions for a
 drive observation. The nearest branch is a lower bound, not a uniquely
 predicted sensor path. Evaluate encoder angles in deg and lengths in mm.
 """
 from autocal.theoretical_ellipse import compute_constraint_circle_2d,compute_constraint_circle_3d,squared_length_coefficients
 from autocal.spool_model import WinchSpoolModel
 data=model(raw,radii,buildup);anchors=np.asarray(anchors);norms=np.linalg.norm(anchors,axis=1)
 base=_resolve_length_model_base_params(raw,num_anchors=raw['num_anchors'],base_radii_override=[30.]*raw['num_anchors'])
 spools=[WinchSpoolModel.from_firmware(base_radius=radii[i],buildup_factor=buildup,spool_to_motor_gearing_factor=base['spool_to_motor_gearing_factor'][i],mechanical_advantage=base['mechanical_advantage'][i],lines_per_spool=base['lines_per_spool'][i]) for i in range(len(anchors))]
 mm_errors=[];deg_errors=[];invalid=0;expected=0
 for sweep,original in zip(data['sweeps'],raw['sweeps']):
  ids=sweep['fixed_anchors'];lengths=[norms[i]+l for i,l in zip(ids,sweep['fixed_lengths'])]
  circle=compute_constraint_circle_3d(anchors,ids,lengths) if anchors.shape[1]==3 else compute_constraint_circle_2d(anchors,ids[0],lengths[0])
  for point,observation in zip(sweep['data_points'],original['data_points']):
   expected+=1
   if circle is None:invalid+=1;continue
   drive,sensor=sweep['drive_anchor'],sweep['sensor_anchor']
   kd,md,nd=squared_length_coefficients(circle,anchors[drive]);ks,ms,ns=squared_length_coefficients(circle,anchors[sensor]);amplitude=np.hypot(md,nd)
   if amplitude<1e-12:invalid+=1;continue
   q=((norms[drive]+point['l_drive'])**2-kd)/amplitude
   if not np.isfinite(q) or abs(q)>1+1e-9:invalid+=1;continue
   phases=np.arctan2(nd,md)+np.array([-1,1])*np.arccos(np.clip(q,-1,1))
   predicted_sq=ks+ms*np.cos(phases)+ns*np.sin(phases)
   if np.any(predicted_sq<0):invalid+=1;continue
   predictions=np.sqrt(predicted_sq)-norms[sensor]
   chosen=int(np.argmin(abs(predictions-point['l_sensor'])))
   predicted=predictions[chosen];spool=spools[sensor]
   if not spool.use_constant_model and spool.k1+predicted*spool.k2<0:invalid+=1;continue
   mm_errors.append(float(predicted-point['l_sensor']))
   deg_errors.append(float(spool.linepos_mm_to_theta_deg(predicted)-observation['raw_angles_deg'][sensor]))
 return {'branch_policy':'nearest of two circle branches (lower bound)','expected_points':expected,'predicted_points':len(mm_errors),'invalid_predictions':invalid,'sensor_rmse_mm':float(np.sqrt(np.mean(np.square(mm_errors)))) if mm_errors else None,'encoder_rmse_deg':float(np.sqrt(np.mean(np.square(deg_errors)))) if deg_errors else None}
