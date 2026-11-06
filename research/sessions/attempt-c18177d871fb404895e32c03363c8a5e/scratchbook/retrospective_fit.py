import json,re,sys,numpy as np
from pathlib import Path
from scipy.optimize import least_squares
from autocal.ellipse_cost import EllipseCostFunction
from autocal.theoretical_ellipse import anchors_matrix_to_opt_vec,anchor_opt_vec_to_matrix
p=Path(sys.argv[1]);d=json.loads((p/'sweeps.json').read_text());m=json.loads((p/'sweeps.headless/manifest.json').read_text());cmd=next(s for s in m['applied_parameters'] if s.startswith('M669')); a=[]
for axis in 'ABCD':
 match=re.search(r'\b'+axis+r'([-+\d.eE]+):([-+\d.eE]+):([-+\d.eE]+)',cmd);a.append(list(map(float,match.groups())))
a=np.array(a); cost=lambda sweeps: EllipseCostFunction({**d,'sweeps':sweeps},pointwise_filtering=False,sweep_wise_filtering=False);c=cost(d['sweeps']);x=anchors_matrix_to_opt_vec(a,'hangprinter_3',c.low_anchor_z)
results=[]
for holdout in [-1,*range(len(d['sweeps']))]:
 train=[s for i,s in enumerate(d['sweeps']) if i!=holdout]; ct=cost(train);fit=least_squares(ct.anchor_residual_vector,x,max_nfev=150,xtol=1e-10,ftol=1e-10,gtol=1e-10)
 full=cost(d['sweeps']); r0=full.anchor_residual_vector(x);r=full.anchor_residual_vector(fit.x)
 results.append({'holdout_index':holdout,'anchor_matrix':anchor_opt_vec_to_matrix(fit.x,'hangprinter_3',4,3,ct.low_anchor_z).tolist(),'full_rms_before':float(np.sqrt(np.mean(r0*r0))),'full_rms_after':float(np.sqrt(np.mean(r*r))),'fit_success':bool(fit.success),'nfev':fit.nfev})
print(json.dumps({'source':str(p),'starting_anchors':a.tolist(),'results':results},indent=2))
