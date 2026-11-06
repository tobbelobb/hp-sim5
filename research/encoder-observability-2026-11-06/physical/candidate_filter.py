"""Development-only encoder residual correction; see frozen protocol."""
import copy,json,re
import numpy as np
from scipy.optimize import least_squares
from autocal.ellipse_cost import EllipseCostFunction
from autocal.theoretical_ellipse import anchors_matrix_to_opt_vec,anchor_opt_vec_to_matrix

def filter_command(dataset,command):
 def cost(data):return EllipseCostFunction(data,pointwise_filtering=False,sweep_wise_filtering=False)
 if len(dataset.get('sweeps',[]))<5:return command,{'action':'fallback','reason':'too few sweeps'}
 a=[]
 for axis in 'ABCD':
  m=re.search(r'\b'+axis+r'([-+\d.eE]+):([-+\d.eE]+):([-+\d.eE]+)',command)
  if not m:return command,{'action':'fallback','reason':'invalid stock command'}
  a.append(list(map(float,m.groups())))
 a=np.array(a);allcost=cost(dataset);x0=anchors_matrix_to_opt_vec(a,'hangprinter_3',allcost.low_anchor_z);choices=[]
 for i,sweep in enumerate(dataset['sweeps']):
  train={**dataset,'sweeps':[s for j,s in enumerate(dataset['sweeps']) if j!=i]};ct=cost(train)
  fit=least_squares(ct.anchor_residual_vector,x0,max_nfev=150,xtol=1e-10,ftol=1e-10,gtol=1e-10)
  if not fit.success or not np.all(np.isfinite(fit.x)):return command,{'action':'fallback','reason':'leave-out fit failed','index':i}
  rows=allcost.pointwise_residual_rows(fit.x);vals=[]
  for s in dataset['sweeps']:
   residuals=[v['residual_mm'] for v in rows if v['sweep_id']==s['id']]
   if not residuals:return command,{'action':'fallback','reason':'missing residuals','index':i}
   vals.append(float(np.sqrt(np.mean(np.square(residuals)))))
  other=float(np.median([v for j,v in enumerate(vals) if j!=i]));ratio=vals[i]/max(other,.001)
  choices.append({'index':i,'ratio':ratio,'heldout_rms_mm':vals[i],'training_median_rms_mm':other,'fit_x':fit.x,'rows':rows})
 best=max(choices,key=lambda v:v['ratio']);diag={'candidates':[{k:v for k,v in c.items() if k not in ('fit_x','rows')} for c in choices]};
 if best['ratio']<=10 or best['heldout_rms_mm']<=.3:return command,{**diag,'action':'fallback','reason':'threshold not met'}
 i=best['index'];sid=dataset['sweeps'][i]['id'];rows=best['rows'];other=[r['residual_mm'] for r in rows if r['sweep_id']!=sid];threshold=max(.3,10*float(np.median(other)));removed=[r['point_idx'] for r in rows if r['sweep_id']==sid and r['residual_mm']>threshold]
 diag.update(sweep_index=i,sweep_id=sid,point_threshold_mm=threshold,removed_indices=removed)
 if not removed:return command,{**diag,'action':'fallback','reason':'no offending points'}
 d=copy.deepcopy(dataset);points=d['sweeps'][i]['data_points'];d['sweeps'][i]['data_points']=[v for j,v in enumerate(points) if j not in removed]
 if len(d['sweeps'][i]['data_points'])<10:return command,{**diag,'action':'fallback','reason':'too few retained points'}
 cc=cost(d);fit=least_squares(cc.anchor_residual_vector,x0,max_nfev=150,xtol=1e-10,ftol=1e-10,gtol=1e-10);new=anchor_opt_vec_to_matrix(fit.x,'hangprinter_3',4,3,cc.low_anchor_z)
 if not fit.success or not np.all(np.isfinite(new)) or np.max(np.linalg.norm(new-a,axis=1))>50:return command,{**diag,'action':'fallback','reason':'refit failed or exceeds 50 mm'}
 result='M669 '+' '.join(axis+':'.join(f'{v:.2f}' for v in new[i]) for i,axis in enumerate('ABCD'))
 return result,{**diag,'action':'replace','stock_command':command,'replacement_command':result,'anchors_unrounded':new.tolist(),'fit_nfev':fit.nfev}
