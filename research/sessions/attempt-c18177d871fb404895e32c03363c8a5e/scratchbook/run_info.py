import json,sys,re,numpy as np
from pathlib import Path
from autocal.ellipse_cost import EllipseCostFunction
from autocal.theoretical_ellipse import anchors_matrix_to_opt_vec
p=Path(sys.argv[1]); d=json.loads((p/'sweeps.json').read_text());m=json.loads((p/'sweeps.headless/manifest.json').read_text());cmd=next(s for s in m['applied_parameters'] if s.startswith('M669'));a=[]
for axis in 'ABCD':
 x=re.search(r'\b'+axis+r'([-+\d.eE]+):([-+\d.eE]+):([-+\d.eE]+)',cmd);assert x;a.append(list(map(float,x.groups())))
a=np.array(a);h=.01;out=[]
for n in range(4,len(d['sweeps'])+1):
 dd={**d,'sweeps':d['sweeps'][:n]}; c=EllipseCostFunction(dd,pointwise_filtering=False,sweep_wise_filtering=False);x=anchors_matrix_to_opt_vec(a,'hangprinter_3',c.low_anchor_z); f=c.anchor_residual_vector; E=np.eye(len(x))*h; J=np.column_stack([(f(x+e)-f(x-e))/(2*h) for e in E]);s=np.linalg.svd(J,compute_uv=False);out.append({'sweeps':n,'valid':c.evaluate_detailed(x).num_valid_sweeps,'min_singular_count_scaled':float(s[-1]*np.sqrt(n)),'condition':float(s[0]/s[-1])})
print(json.dumps({'applied_command':cmd,'sweep_count':len(d['sweeps']),'cumulative':out},indent=2))
