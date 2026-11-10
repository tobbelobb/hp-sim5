import json,re,numpy as np,sys
from scipy.optimize import least_squares
from pathlib import Path
p=Path(sys.argv[1]);d=json.load(open(p/'sweeps.json'));m=json.load(open(p/'sweeps.headless/manifest.json'));cmd=next(x for x in m['applied_parameters'] if x.startswith('M669'));a=[]
for axis in 'ABCD':
 m=re.search(r'\b'+axis+r'([-+\d.eE]+):([-+\d.eE]+):([-+\d.eE]+)',cmd);a.append([float(v) for v in m.groups()])
a=np.array(a);out=[];scale=np.array(d['config']['mm_per_degree'])
for s in d['sweeps']:
 pts=s['data_points'];n=len(pts)//2;sol=[]
 for pnt in [pts[0],pts[n]]:
  length=np.linalg.norm(a,axis=1)+np.array(pnt['mu'])*scale
  f=lambda p:np.linalg.norm(a-p,axis=1)-length
  candidates=[least_squares(f,start,max_nfev=300) for start in [np.zeros(3),[100,100,100],[500,-500,500],[-500,500,-500]]];best=min(candidates,key=lambda r:np.linalg.norm(r.fun));sol.append({'position_mm':best.x.tolist(),'range_residual_rms_mm':float(np.sqrt(np.mean(best.fun**2)))})
 out.append({'sweep':s['id'],'roles_positions':sol,'separation_mm':float(np.linalg.norm(np.array(sol[0]['position_mm'])-sol[1]['position_mm']))})
print(json.dumps(out,indent=2))
