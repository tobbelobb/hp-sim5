import json,re,sys,copy,numpy as np
from pathlib import Path
from scipy.optimize import least_squares
from autocal.ellipse_cost import EllipseCostFunction
from autocal.theoretical_ellipse import anchors_matrix_to_opt_vec,anchor_opt_vec_to_matrix
p=Path(sys.argv[1]); diag=json.loads(Path(sys.argv[2]).read_text()); d=json.loads((p/'sweeps.json').read_text());m=json.loads((p/'sweeps.headless/manifest.json').read_text());cmd=next(s for s in m['applied_parameters'] if s.startswith('M669')); a=[]
for axis in 'ABCD':
 match=re.search(r'\b'+axis+r'([-+\d.eE]+):([-+\d.eE]+):([-+\d.eE]+)',cmd);a.append(list(map(float,match.groups())))
a=np.array(a);cost=lambda ds:EllipseCostFunction(ds,pointwise_filtering=False,sweep_wise_filtering=False)
choices=[]
for r in diag['results']:
 i=r['holdout_index'];
 if i<0:continue
 vals=[s['rms'] for s in r['per_sweep']];other=np.median([v for j,v in enumerate(vals) if j!=i]);choices.append((vals[i]/max(other,.001),vals[i],i,r))
ratio,rms,i,r=max(choices)
if ratio<10 or rms<.3:print(json.dumps({'accepted':False,'ratio':ratio,'rms':rms}));sys.exit(0)
# Freeze numerical rejection as abs residual > max(0.3 mm, 10x median abs residual in other sweeps)
c=cost(d);x=anchors_matrix_to_opt_vec(np.array(r['anchor_matrix']),'hangprinter_3',c.low_anchor_z);rows=c.pointwise_residual_rows(x);sid=d['sweeps'][i]['id'];rest=[row['residual_mm'] for row in rows if row['sweep_id']!=sid];threshold=max(.3,10*np.median(rest));remove=[row['point_idx'] for row in rows if row['sweep_id']==sid and row['residual_mm']>threshold]; dd=copy.deepcopy(d);dd['sweeps'][i]['data_points']=[v for j,v in enumerate(dd['sweeps'][i]['data_points']) if j not in remove];assert len(dd['sweeps'][i]['data_points'])>=10
cc=cost(dd); x0=anchors_matrix_to_opt_vec(a,'hangprinter_3',cc.low_anchor_z);fit=least_squares(cc.anchor_residual_vector,x0,max_nfev=150,xtol=1e-10,ftol=1e-10,gtol=1e-10)
print(json.dumps({'accepted':True,'sweep_index':i,'sweep_id':sid,'ratio':ratio,'heldout_rms_mm':rms,'point_threshold_mm':threshold,'removed_indices':remove,'anchor_matrix':anchor_opt_vec_to_matrix(fit.x,'hangprinter_3',4,3,cc.low_anchor_z).tolist(),'fit_success':bool(fit.success),'nfev':fit.nfev},indent=2))
