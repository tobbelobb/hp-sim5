import json,re,sys,numpy as np
from scipy.optimize import least_squares
from autocal.ellipse_cost import EllipseCostFunction
from autocal.theoretical_ellipse import anchors_matrix_to_opt_vec,anchor_opt_vec_to_matrix
from pathlib import Path
p=Path(sys.argv[1]);d=json.load(open(p/'sweeps.json'));m=json.load(open(p/'sweeps.headless/manifest.json'));cmd=next(x for x in m['applied_parameters'] if x.startswith('M669'));a=[]
for axis in 'ABCD':
 ma=re.search(r'\b'+axis+r'([-+\d.eE]+):([-+\d.eE]+):([-+\d.eE]+)',cmd);a.append([float(v) for v in ma.groups()])
a=np.array(a);c=EllipseCostFunction(d,pointwise_filtering=False,sweep_wise_filtering=False);x=anchors_matrix_to_opt_vec(a,'hangprinter_3',c.low_anchor_z)
def res(v):return np.array([r['residual_mm_signed'] for r in c.pointwise_residual_rows(v)])
out=[]
for scale in [.03,.1,.3]:
 fit=least_squares(res,x,loss='soft_l1',f_scale=scale,max_nfev=150,xtol=1e-9,ftol=1e-9,gtol=1e-9)
 out.append({'scale_mm':scale,'anchors':anchor_opt_vec_to_matrix(fit.x,'hangprinter_3',4,3,c.low_anchor_z).tolist(),'success':bool(fit.success),'nfev':fit.nfev,'median_abs_mm':float(np.median(np.abs(fit.fun)))})
print(json.dumps({'source':str(p),'stock':a.tolist(),'fits':out},indent=2))
