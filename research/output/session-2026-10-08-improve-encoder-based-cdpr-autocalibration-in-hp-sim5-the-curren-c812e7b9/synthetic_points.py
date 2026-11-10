import copy,json,sys,time
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path.cwd()))
from autocal.theoretical_ellipse import compute_constraint_circle_3d,anchors_matrix_to_opt_vec,anchor_opt_vec_to_matrix
from autocal.ellipse_solver import solve_anchors
from autocal.ellipse_cost import EllipseCostFunction
OUT=Path(__file__).resolve().parent
TRUE=np.array([[0.,-1900.,-280.],[1645.45,950.,-280.],[-1645.45,950.,-280.],[0.,0.,1900.]])

def generate(seed,noise,bad):
 rng=np.random.default_rng(seed);sweeps=[]
 for delta in (0.,250.,-250.,100.):
  for fixed in range(3):
   ids=[fixed,3];lens=np.linalg.norm(TRUE[ids],axis=1)+[delta,0.]
   circle=compute_constraint_circle_3d(TRUE,ids,lens.tolist())
   drive,sensor=[i for i in range(3) if i!=fixed]
   phi=np.linspace(.3,2.8,24)
   pos=circle.center+circle.radius*(np.cos(phi)[:,None]*circle.u+np.sin(phi)[:,None]*circle.v)
   observed=np.linalg.norm(pos[:,None,:]-TRUE[None,:,:],axis=2)-np.linalg.norm(TRUE,axis=1)
   observed+=rng.normal(0.,noise,observed.shape)
   if bad:observed[5,sensor]+=20.
   points=[{'l_drive':float(row[drive]),'l_sensor':float(row[sensor]),'raw_angles_deg':(row/(39.1845*np.pi/180)).tolist()} for row in observed]
   sweeps.append({'id':f'sweep_{len(sweeps)+1:03d}','fixed_anchors':ids,'fixed_lengths':(lens-np.linalg.norm(TRUE[ids],axis=1)).tolist(),'drive_anchor':drive,'sensor_anchor':sensor,'data_points':points})
 return {'version':'2','machine_type':'hangprinter_4','num_anchors':4,'dimensions':3,'sweeps':sweeps}

def metrics(d,a):
 cost=EllipseCostFunction(d,pointwise_filtering=False,sweep_wise_filtering=False,use_flex=False);cost._min_sweeps_after_trim=1
 rows=cost.pointwise_residual_rows(anchors_matrix_to_opt_vec(a,'hangprinter_4'))
 residual=[r['residual_mm_signed'] for r in rows]
 return {'expected':sum(len(s['data_points']) for s in d['sweeps']),'valid':len(rows),'rms_mm':float(np.sqrt(np.mean(np.square(residual)))) if rows else None}

results=[]
for condition,noise,bad in [('clean',.02,False),('noise_outlier',.5,True)]:
 for seed in (0,1):
  d=generate(seed,noise,bad);(OUT/f'E05-{condition}-{seed}.json').write_text(json.dumps(d))
  test=dict(d,sweeps=d['sweeps'][9:]);init=anchor_opt_vec_to_matrix(anchors_matrix_to_opt_vec(TRUE,'hangprinter_4')+np.random.default_rng(seed+100).normal(0.,60.,9),'hangprinter_4',4,3)
  for count in (24,12,6):
   train=copy.deepcopy(dict(d,sweeps=d['sweeps'][:9]))
   for s in train['sweeps']:s['data_points']=[s['data_points'][i] for i in np.linspace(0,23,count).round().astype(int)]
   t=time.perf_counter();r=solve_anchors(train,initial_guess=init,max_iterations=400,num_restarts=2,use_parallel=False,use_flex=False,robust_loss=True,pointwise_filtering=False,sweep_wise_filtering=False,noise_normalized=False)
   a=np.asarray(r['anchors']);detail=r['details']
   result={'condition':condition,'seed':seed,'points_per_sweep':count,'train_points':9*count,'test_points':72,'seconds':time.perf_counter()-t,'anchor_rmse_mm':float(np.sqrt(np.mean(np.sum((a-TRUE)**2,axis=1)))),'cost':float(r['cost']),'optimizer_success':bool(r['success']),'valid_train_sweeps':detail.num_valid_sweeps,'invalid_train_sweeps':detail.num_invalid_sweeps,'heldout':metrics(test,a),'anchors':a.tolist()}
   results.append(result);print(json.dumps(result),flush=True);(OUT/'E05-synthetic-points.json').write_text(json.dumps(results,indent=2))
