import json,re,sys,numpy as np
from pathlib import Path
from autocal.ellipse_cost import EllipseCostFunction
from autocal.theoretical_ellipse import anchors_matrix_to_opt_vec
p=Path(sys.argv[1]);d=json.loads((p/'sweeps.json').read_text());m=json.loads((p/'sweeps.headless/manifest.json').read_text());cmd=next(x for x in m['applied_parameters'] if x.startswith('M669'));a=[]
for axis in 'ABCD':
 z=re.search(r'\b'+axis+r'([-+\d.eE]+):([-+\d.eE]+):([-+\d.eE]+)',cmd);a.append(list(map(float,z.groups())))
c=EllipseCostFunction(d,pointwise_filtering=False,sweep_wise_filtering=False);x=anchors_matrix_to_opt_vec(np.array(a),'hangprinter_3',c.low_anchor_z);rows=c.pointwise_residual_rows(x);out=[]
for s in d['sweeps']:
 rs=[r for r in rows if r['sweep_id']==s['id']]; pts=s['data_points']; rr=np.array([r['residual_mm_signed'] for r in rs]);sig=[v for p in pts for v in p.get('sigma',[]) if v is not None and np.isfinite(v)]; halves=[]
 for step in [0,1]:
  vals=[r['residual_mm_signed'] for r in rs if (r['point_idx']<len(pts)//2)==(step==0)];halves.append({'median_mm':float(np.median(vals)),'rms_mm':float(np.sqrt(np.mean(np.square(vals)))),'max_abs_mm':float(np.max(np.abs(vals)))})
 out.append({'id':s['id'],'fixed':s['fixed_anchors'],'drive':s['drive_anchor'],'sensor':s['sensor_anchor'],'count':len(pts),'rms_mm':float(np.sqrt(np.mean(rr*rr))),'first_half':halves[0],'second_half':halves[1],'point_residuals_mm':rr.tolist(),'median_sigma_deg':float(np.median(sig)) if sig else None})
print(json.dumps({'command':cmd,'sweeps':out},indent=2))
