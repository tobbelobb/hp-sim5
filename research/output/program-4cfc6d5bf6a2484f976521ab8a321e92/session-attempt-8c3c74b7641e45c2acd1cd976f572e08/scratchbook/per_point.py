import sys,json,re,numpy as np
from pathlib import Path
from scipy.optimize import least_squares
from autocal.theoretical_ellipse import anchors_matrix_to_opt_vec,anchor_opt_vec_to_matrix
p=Path(sys.argv[1]); d=json.loads((p/'sweeps.json').read_text()); m=json.loads((p/'sweeps.headless/manifest.json').read_text()); cmd=[s for s in m['applied_parameters'] if s.startswith('M669')][-1]
a=np.array([[float(x) for x in re.search(r'\b'+axis+r'([-+\d.eE]+):([-+\d.eE]+):([-+\d.eE]+)',cmd).groups()] for axis in 'ABCD']); x=anchors_matrix_to_opt_vec(a,'hangprinter_3',a[0,2]); angles=np.array([pt['mu'] for s in d['sweeps'] for pt in s['data_points']]); delta=angles*np.pi/6
# linear trilateration differences solve position, then signed range discrepancy.
def res(v):
 A=anchor_opt_vec_to_matrix(v,'hangprinter_3',4,3,a[0,2]); r=np.linalg.norm(A,axis=1)[None,:]+delta
 mat=2*(A[1:]-A[0]); b=(np.sum(A[1:]**2,axis=1)-np.sum(A[0]**2))[None,:]-(r[:,1:]**2-r[:,0,None]**2)
 pos=np.linalg.solve(mat,b.T).T
 return np.linalg.norm(pos-A[0],axis=1)-r[:,0]
out=[]
for scale in [0.03,0.3,1000]:
 f=least_squares(res,x,loss='soft_l1',f_scale=scale,max_nfev=500,xtol=1e-10,ftol=1e-10,gtol=1e-10)
 out.append(dict(scale=scale,anchors=anchor_opt_vec_to_matrix(f.x,'hangprinter_3',4,3,a[0,2]).tolist(),rms=float(np.sqrt(np.mean(f.fun**2))),success=bool(f.success),nfev=f.nfev))
print(json.dumps(dict(run=str(p),stock=a.tolist(),fits=out),indent=2))
