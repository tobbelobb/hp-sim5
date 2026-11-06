import copy,re,numpy as np
from scipy.optimize import least_squares
from autocal.ellipse_cost import EllipseCostFunction
from autocal.theoretical_ellipse import anchors_matrix_to_opt_vec,anchor_opt_vec_to_matrix

def filter_command(dataset,command):
 anchors=[]
 for axis in 'ABCD':
  m=re.search(r'\b'+axis+r'([-+\d.eE]+):([-+\d.eE]+):([-+\d.eE]+)',command)
  if not m:return command,{'action':'fallback','reason':'invalid stock command'}
  anchors.append(list(map(float,m.groups())))
 anchors=np.array(anchors); sweeps=[]
 for s in dataset.get('sweeps',[]):
  for j,p in enumerate(s.get('data_points',[])):
   mu=p.get('mu'); axes=s['fixed_anchors']
   if not isinstance(mu,list) or len(mu)!=4 or not np.isfinite(mu).all():return command,{'action':'fallback','reason':'missing point encoder means'}
   t=copy.copy(s);t['id']=str(s['id'])+'_'+str(j);t['fixed_lengths']=(np.array(mu)[axes]*np.pi/6).tolist();t['data_points']=[p];sweeps.append(t)
 if len(sweeps)<30:return command,{'action':'fallback','reason':'insufficient points'}
 d=copy.copy(dataset);d['sweeps']=sweeps;c=EllipseCostFunction(d,pointwise_filtering=False,sweep_wise_filtering=False);x=anchors_matrix_to_opt_vec(anchors,'hangprinter_3',c.low_anchor_z)
 def residual(v):
  rows=c.pointwise_residual_rows(v)
  if len(rows)!=len(sweeps):raise ValueError('invalid point geometry')
  r=np.array([r['residual_mm_signed'] for r in rows]);
  if not np.isfinite(r).all():raise ValueError('nonfinite residual')
  return r
 fit=least_squares(residual,x,loss='soft_l1',f_scale=.03,max_nfev=150,xtol=1e-9,ftol=1e-9,gtol=1e-9);a=anchor_opt_vec_to_matrix(fit.x,'hangprinter_3',4,3,c.low_anchor_z)
 if not fit.success or not np.isfinite(a).all() or np.max(np.linalg.norm(a-anchors,axis=1))>50:return command,{'action':'fallback','reason':'fit failed or exceeds 50 mm'}
 cmd='M669 '+' '.join(axis+':'.join(f'{v:.2f}' for v in a[i]) for i,axis in enumerate('ABCD'))
 return cmd,{'action':'replace','stock_command':command,'replacement_command':cmd,'anchors_unrounded':a.tolist(),'nfev':fit.nfev,'npoints':len(sweeps),'scale_mm':.03}
