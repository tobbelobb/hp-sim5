import copy,json,sys,time
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path.cwd()))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from informative_points import select
from autocal.theoretical_ellipse import compute_constraint_circle_3d,anchors_matrix_to_opt_vec,anchor_opt_vec_to_matrix
from autocal.ellipse_solver import solve_anchors
from autocal.ellipse_cost import EllipseCostFunction
OUT=Path(__file__).resolve().parent
TRUE=np.array([[0.,-1900.,-280.],[1645.45,950.,-280.],[-1645.45,950.,-280.],[0.,0.,1900.]])
def generate(seed,condition):
 rng=np.random.default_rng(seed);sweeps=[]
 for delta in (0.,250.,-250.,100.):
  for fixed in range(3):
   ids=[fixed,3];lens=np.linalg.norm(TRUE[ids],axis=1)+[delta,0.]
   circle=compute_constraint_circle_3d(TRUE,ids,lens.tolist())
   drive,sensor=[i for i in range(3) if i!=fixed]
   phi=np.linspace(.3,2.8,24) if condition!='short_arc' else np.linspace(.3,.6,24)
   pos=circle.center+circle.radius*(np.cos(phi)[:,None]*circle.u+np.sin(phi)[:,None]*circle.v)
   obs=np.linalg.norm(pos[:,None,:]-TRUE[None,:,:],axis=2)-np.linalg.norm(TRUE,axis=1)
   obs+=rng.normal(0.,.02 if condition=='clean' else .5,obs.shape)
   if condition=='outlier':obs[rng.choice(24,2,replace=False),sensor]+=20.
   points=[dict(l_drive=float(v[drive]),l_sensor=float(v[sensor]),raw_angles_deg=(v/(39.1845*np.pi/180)).tolist()) for v in obs]
   sweeps.append(dict(id=f'sweep_{len(sweeps)+1:03d}',fixed_anchors=ids,fixed_lengths=(lens-np.linalg.norm(TRUE[ids],axis=1)).tolist(),drive_anchor=drive,sensor_anchor=sensor,data_points=points))
 return dict(version='2',machine_type='hangprinter_4',num_anchors=4,dimensions=3,sweeps=sweeps)
def metrics(d,a):
 c=EllipseCostFunction(d,use_flex=False,pointwise_filtering=False,sweep_wise_filtering=False);c._min_sweeps_after_trim=1
 rows=c.pointwise_residual_rows(anchors_matrix_to_opt_vec(a,'hangprinter_4'))
 return dict(expected=sum(len(s['data_points']) for s in d['sweeps']),valid=len(rows),rms_mm=float(np.sqrt(np.mean([r['residual_mm_signed']**2 for r in rows]))) if rows else None)
results=[]
for condition in ('clean','outlier','short_arc'):
 for seed in (0,1):
  d=generate(seed,condition);(OUT/f'E03-synthetic-{condition}-{seed}.json').write_text(json.dumps(d))
  train=dict(d,sweeps=d['sweeps'][:9]);test=dict(d,sweeps=d['sweeps'][9:]);x=anchors_matrix_to_opt_vec(TRUE,'hangprinter_4')+np.random.default_rng(seed+100).normal(0.,60.,9);init=anchor_opt_vec_to_matrix(x,'hangprinter_4',4,3)
  informed,design=select(train,init)
  uniform=copy.deepcopy(train)
  for s in uniform['sweeps']:s['data_points']=[s['data_points'][i] for i in np.linspace(0,23,12).round().astype(int)]
  for variant,data in (('full',train),('uniform',uniform),('informed',informed)):
   t=time.perf_counter();fit=solve_anchors(data,initial_guess=init,num_restarts=2,max_iterations=400,use_parallel=False,use_flex=False,robust_loss=True,pointwise_filtering=False,sweep_wise_filtering=False,noise_normalized=False)
   a=np.asarray(fit['anchors']);detail=fit['details']
   result=dict(condition=condition,seed=seed,variant=variant,points=sum(len(s['data_points']) for s in data['sweeps']),seconds=time.perf_counter()-t,anchor_rmse_mm=float(np.sqrt(np.mean(np.sum((a-TRUE)**2,axis=1)))),cost=float(fit['cost']),optimizer_success=bool(fit['success']),invalid_sweeps=detail.num_invalid_sweeps,heldout=metrics(test,a),anchors=a.tolist(),design=design if variant=='informed' else None)
   results.append(result);print(json.dumps(result),flush=True);(OUT/'E03-synthetic-results.json').write_text(json.dumps(results,indent=2))
