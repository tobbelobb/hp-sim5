import json,numpy as np
from pathlib import Path
from autocal.ellipse_cost import EllipseCostFunction
from autocal.theoretical_ellipse import anchors_matrix_to_opt_vec
root=Path('research/sessions/attempt-4f9da7c284d04b85810ceb12d0acbdfb')
d=json.loads((root/'runs/fresh-baseline/sweeps.json').read_text()); a=np.array(json.loads((root/'evidence/measurements.json').read_text())['fresh-baseline']['applied']['anchors']); cost=EllipseCostFunction(d,pointwise_filtering=False,sweep_wise_filtering=False);x=anchors_matrix_to_opt_vec(a,'hangprinter_3',cost.low_anchor_z)
f=cost.anchor_residual_vector
h=.01;J=np.column_stack([(f(x+np.eye(len(x))[i]*h)-f(x-np.eye(len(x))[i]*h))/(2*h) for i in range(len(x))]);u,s,v=np.linalg.svd(J,full_matrices=False)
print(json.dumps({'source':str(root/'runs/fresh-baseline/sweeps.json'),'sweeps':len(d['sweeps']),'sweep_points':[len(s['data_points']) for s in d['sweeps']],'parameters':x.tolist(),'low_anchor_z':cost.low_anchor_z,'unfiltered':True,'jacobian_shape':list(J.shape),'singular_values':s.tolist(),'weakest_right_singular_vector':v[-1].tolist(),'scaling_direction_cosine':float(abs(v[-1]@x)/np.linalg.norm(x)),'valid_sweeps':cost.evaluate_detailed(x).num_valid_sweeps,'step_mm':h},indent=2))
