import json,itertools,numpy as np
from pathlib import Path
from autocal.ellipse_cost import EllipseCostFunction
from autocal.theoretical_ellipse import anchors_matrix_to_opt_vec
root=Path('research/sessions/attempt-4f9da7c284d04b85810ceb12d0acbdfb');d=json.loads((root/'runs/fresh-baseline/sweeps.json').read_text()); a=np.array(json.loads((root/'evidence/measurements.json').read_text())['fresh-baseline']['applied']['anchors']); h=.01
out=[]
for n in [4,6,8]:
 for inds in itertools.combinations(range(8),n):
  dd={**d,'sweeps':[d['sweeps'][i] for i in inds]}; c=EllipseCostFunction(dd,pointwise_filtering=False,sweep_wise_filtering=False); x=anchors_matrix_to_opt_vec(a,'hangprinter_3',c.low_anchor_z); f=c.anchor_residual_vector; J=np.column_stack([(f(x+np.eye(len(x))[i]*h)-f(x-np.eye(len(x))[i]*h))/(2*h) for i in range(len(x))]);s=np.linalg.svd(J,compute_uv=False)
  # production normalizes by number of sweeps: undo this for independent information accumulation
  out.append({'indices':inds,'minimum_singular_per_point_normalized':s[-1], 'information_scaled_minimum':s[-1]*np.sqrt(n),'condition':s[0]/s[-1] if s[-1]>0 else None,'valid':c.evaluate_detailed(x).num_valid_sweeps})
print(json.dumps({'sweeps':[{k:s.get(k) for k in ['id','fixed_anchors','fixed_lengths','drive_anchor','sensor_anchor']} for s in d['sweeps']],'combinations':out},indent=2))
