import json,re,numpy as np
from scipy.optimize import least_squares
from autocal.ellipse_cost import EllipseCostFunction
from autocal.theoretical_ellipse import anchors_matrix_to_opt_vec,anchor_opt_vec_to_matrix
from pathlib import Path
p=Path('research/sessions/attempt-c18177d871fb404895e32c03363c8a5e/runs/filtered7301');d=json.load(open(p/'sweeps.json')); diag=json.load(open(p/'point-filter.json'));i=diag['sweep_index'];removed=diag['removed_indices'];d['sweeps'][i]['data_points']=[v for j,v in enumerate(d['sweeps'][i]['data_points']) if j not in removed];a=np.array(diag['anchors_unrounded']);out=[]
for omit in [None,7]:
 dd={**d,'sweeps':[s for j,s in enumerate(d['sweeps']) if j!=omit]};c=EllipseCostFunction(dd,pointwise_filtering=False,sweep_wise_filtering=False);x=anchors_matrix_to_opt_vec(a,'hangprinter_3',c.low_anchor_z);r=least_squares(c.anchor_residual_vector,x,max_nfev=150,xtol=1e-10,ftol=1e-10,gtol=1e-10);out.append({'omitted_sweep_index':omit,'anchors':anchor_opt_vec_to_matrix(r.x,'hangprinter_3',4,3,c.low_anchor_z).tolist(),'success':bool(r.success),'nfev':r.nfev})
print(json.dumps(out,indent=2))
