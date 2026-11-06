"""Development-selected soft-L1 encoder estimator; see frozen protocol."""
import re
import numpy as np
from scipy.optimize import least_squares
from autocal.ellipse_cost import EllipseCostFunction
from autocal.theoretical_ellipse import anchors_matrix_to_opt_vec,anchor_opt_vec_to_matrix

def filter_command(dataset,command):
 anchors=[]
 for axis in 'ABCD':
  m=re.search(r'\b'+axis+r'([-+\d.eE]+):([-+\d.eE]+):([-+\d.eE]+)',command)
  if not m:return command,{'action':'fallback','reason':'invalid stock command'}
  anchors.append(list(map(float,m.groups())))
 anchors=np.array(anchors);cost=EllipseCostFunction(dataset,pointwise_filtering=False,sweep_wise_filtering=False);x0=anchors_matrix_to_opt_vec(anchors,'hangprinter_3',cost.low_anchor_z)
 count=sum(len(s.get('data_points',[])) for s in dataset.get('sweeps',[]))
 if len(dataset.get('sweeps',[]))<3 or count<30:return command,{'action':'fallback','reason':'insufficient observations'}
 def residual(x):
  rows=cost.pointwise_residual_rows(x)
  if len(rows)!=count:raise ValueError('missing or changing point residuals')
  a=np.array([r['residual_mm_signed'] for r in rows])
  if not np.all(np.isfinite(a)):raise ValueError('nonfinite residual')
  return a
 fit=least_squares(residual,x0,loss='soft_l1',f_scale=.03,max_nfev=150,xtol=1e-9,ftol=1e-9,gtol=1e-9)
 matrix=anchor_opt_vec_to_matrix(fit.x,'hangprinter_3',4,3,cost.low_anchor_z)
 if not fit.success or not np.all(np.isfinite(matrix)) or np.max(np.linalg.norm(matrix-anchors,axis=1))>50:return command,{'action':'fallback','reason':'fit failed or exceeds 50 mm'}
 cmd='M669 '+' '.join(axis+':'.join(f'{v:.2f}' for v in matrix[i]) for i,axis in enumerate('ABCD'))
 return cmd,{'action':'replace','scale_mm':.03,'stock_command':command,'replacement_command':cmd,'anchors_unrounded':matrix.tolist(),'nfev':fit.nfev,'num_points':count,'median_abs_residual_mm':float(np.median(np.abs(fit.fun)))}
