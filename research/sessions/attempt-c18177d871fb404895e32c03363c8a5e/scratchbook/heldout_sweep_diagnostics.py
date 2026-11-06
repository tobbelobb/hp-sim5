import json,sys,numpy as np
from pathlib import Path
from autocal.ellipse_cost import EllipseCostFunction
from autocal.theoretical_ellipse import anchors_matrix_to_opt_vec
p=Path(sys.argv[1]);d=json.loads((p/'sweeps.json').read_text());fits=json.loads(Path(sys.argv[2]).read_text());c=EllipseCostFunction(d,pointwise_filtering=False,sweep_wise_filtering=False)
for fit in fits['results']:
 a=np.array(fit['anchor_matrix']);x=anchors_matrix_to_opt_vec(a,'hangprinter_3',c.low_anchor_z);rows=c.pointwise_residual_rows(x);summ=[]
 for s in d['sweeps']:
  r=[e['residual_mm_signed'] for e in rows if e['sweep_id']==s['id']];summ.append({'id':s['id'],'median_signed':float(np.median(r)),'rms':float(np.sqrt(np.mean(np.square(r))))})
 fit['per_sweep']=summ
print(json.dumps(fits,indent=2))
