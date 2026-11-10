import json,numpy as np
from scipy.optimize import least_squares
from autocal.theoretical_ellipse import anchors_matrix_to_opt_vec,anchor_opt_vec_to_matrix
p='research/sessions/attempt-c18177d871fb404895e32c03363c8a5e/runs/stock7301/sweeps.json'; d=json.load(open(p)); A=np.array([[0,-2000,-280],[1725.448267,950,-280],[-1715.448267,950,-280],[0,0,1980.]])
base=np.linalg.norm(A,axis=1); mat=2*(A[1:]-A[0]); batches=[]
for s in d['sweeps']:
 r=base+np.array([p['mu'] for p in s['data_points']])*np.pi/6
 b=(np.sum(A[1:]**2,axis=1)-np.sum(A[0]**2))-(r[:,1:]**2-r[:,0,None]**2)
 pos=np.linalg.solve(mat,b.T).T
 # Force exact geometric ranges at recorded approximate positions, making an internally consistent synthetic case.
 exact=np.linalg.norm(pos[:,None,:]-A[None,:,:],axis=2)-base
 batches.append((s['fixed_anchors'],exact))
x=anchors_matrix_to_opt_vec(A,'hangprinter_3',-280)+np.array([2.,-2.,1.,1.,-1.,2.,1.,-1.])
for mode in ['per_point','per_sweep']:
 arr=[]
 for axes,ex in batches:
  v=ex.copy()
  if mode=='per_sweep': v[:,axes]=np.mean(v[:,axes],axis=0)
  arr.extend(v)
 delta=np.array(arr)
 def res(v):
  a=anchor_opt_vec_to_matrix(v,'hangprinter_3',4,3,-280);r=np.linalg.norm(a,axis=1)+delta
  b=(np.sum(a[1:]**2,axis=1)-np.sum(a[0]**2))-(r[:,1:]**2-r[:,0,None]**2)
  pos=np.linalg.solve(2*(a[1:]-a[0]),b.T).T
  return np.linalg.norm(pos-a[0],axis=1)-r[:,0]
 f=least_squares(res,x,max_nfev=200,xtol=1e-11,ftol=1e-11,gtol=1e-11); fit=anchor_opt_vec_to_matrix(f.x,'hangprinter_3',4,3,-280)
 print(json.dumps(dict(mode=mode,errors=np.linalg.norm(fit-A,axis=1).tolist(),rms=float(np.sqrt(np.mean(f.fun**2))),success=bool(f.success))))
print(json.dumps(dict(max_fixed_variation_mm=max(float(np.ptp(ex[:,axes],axis=0).max()) for axes,ex in batches))))
