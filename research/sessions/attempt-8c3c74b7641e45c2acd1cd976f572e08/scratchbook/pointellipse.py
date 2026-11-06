import json,copy,sys,re,numpy as np
from pathlib import Path
from scipy.optimize import least_squares
from autocal.ellipse_cost import EllipseCostFunction
from autocal.theoretical_ellipse import anchors_matrix_to_opt_vec,anchor_opt_vec_to_matrix
p=Path(sys.argv[1]);d=json.load(open(p/'sweeps.json'));m=json.load(open(p/'sweeps.headless/manifest.json'));cmd=[s for s in m['applied_parameters'] if s.startswith('M669')][-1]; a=np.array([[float(x) for x in re.search(r'\b'+ax+r'([-+\d.eE]+):([-+\d.eE]+):([-+\d.eE]+)',cmd).groups()] for ax in 'ABCD'])
sweeps=[];diff=[]
for s in d['sweeps']:
 for j,pt in enumerate(s['data_points']):
  fixed=np.array(pt['mu'])[s['fixed_anchors']]*np.pi/6
  diff.extend(fixed-np.array(s['fixed_lengths']))
  t=copy.copy(s);t['id']=str(s['id'])+'_'+str(j);t['fixed_lengths']=fixed.tolist();t['data_points']=[pt];sweeps.append(t)
d['sweeps']=sweeps;c=EllipseCostFunction(d,pointwise_filtering=False,sweep_wise_filtering=False);x=anchors_matrix_to_opt_vec(a,'hangprinter_3',c.low_anchor_z)
def res(v):
 rows=c.pointwise_residual_rows(v)
 if len(rows)!=len(sweeps):raise ValueError(f'only {len(rows)} rows of {len(sweeps)}')
 return np.array([r['residual_mm_signed'] for r in rows])
print('fixed-diff abs median,p90,p99,max',np.percentile(np.abs(diff),[50,90,99,100]).tolist(),file=sys.stderr,flush=True)
f=least_squares(res,x,loss='soft_l1',f_scale=.03,max_nfev=150,xtol=1e-9,ftol=1e-9,gtol=1e-9)
print(json.dumps(dict(anchors=anchor_opt_vec_to_matrix(f.x,'hangprinter_3',4,3,c.low_anchor_z).tolist(),rms=float(np.sqrt(np.mean(f.fun**2))),success=bool(f.success),nfev=f.nfev,fixed_abs_percentiles=np.percentile(np.abs(diff),[50,90,99,100]).tolist()),indent=2))
