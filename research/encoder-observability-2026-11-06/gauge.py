import numpy as np, json
from scipy.linalg import null_space
from scipy.optimize import least_squares
rng=np.random.default_rng(917)
# Gauge-fixed tetrahedron, mm. Each point has four range differences to reference point.
a=np.array([[0,0,0],[3000,0,0],[1000,2600,0],[1300,1200,2800.]],float)
p0=np.array([1400,1000,700.])
def unpack(q):
 a=np.zeros((4,3));a[1,0]=q[0];a[2,:2]=q[1:3];a[3,:]=q[3:6]; return a
def pack(a):return np.r_[a[1,0],a[2,:2],a[3,:]]
def ranges(a,p):return np.linalg.norm(p[:,None,:]-a[None,:,:],axis=2)
def obs(a,p):return ranges(a,p)-ranges(a,p0[None,:])
def jac(a,p):
 # Reference position must also be unknown, include it among positions.
 points=np.vstack([p0,p]); n=len(p); ds=points[:,None,:]-a[None,:,:]; u=ds/np.linalg.norm(ds,axis=2)[:,:,None]
 J=np.zeros((4*n,6+3*(n+1)))
 for i in range(n):
  for k in range(4):
   row=4*i+k
   # anchor gradients d(r_i-r_0)/da = -u_i+u_0
   for c, (ak, ax) in enumerate([(1,0),(2,0),(2,1),(3,0),(3,1),(3,2)]):
    if k==ak:J[row,c]=-u[i+1,k,ax]+u[0,k,ax]
   J[row,6:9]=-u[0,k];J[row,9+3*i:12+3*i]=u[i+1,k]
 return J

def summary(p):
 J=jac(a,p); U,s,V=np.linalg.svd(J,full_matrices=False); G=J[:,:6]; P=J[:,6:]; Q=null_space(P.T); reduced=Q.T@G; sr=np.linalg.svd(reduced,compute_uv=False)
 scale=np.r_[pack(a),p0,p.ravel()];return {'points':len(p),'shape':list(J.shape),'full_tail':s[-8:].tolist(),'reduced_singular':sr.tolist(),'scale_residual_norm':float(np.linalg.norm(J@scale)), 'scale_norm':float(np.linalg.norm(scale)),'rank':int(np.linalg.matrix_rank(J))}
p= p0+rng.uniform([-800,-700,-350],[800,700,700],(80,3)); narrow=p0+rng.uniform([-200,-200,-40],[200,200,40],(80,3))
# Unknown common radius scale: encoder differences y=range difference / r; null direction includes common radius.
J=jac(a,p); y=obs(a,p).ravel(); Ju=np.column_stack([J,-y/30]); xu=np.r_[pack(a),p0,p.ravel(),30.]
# Exact finite scaling in unknown radius case, fixed encoder differences.
finite=np.max(np.abs(obs(1.01*a,1.01*p)/30.3 - obs(a,p)/30)) if False else None
# Reference scales as well.
def obsref(a,p,pref):return ranges(a,p)-ranges(a,pref[None,:])
finite=float(np.max(np.abs(obsref(a*1.01,p*1.01,p0*1.01)/30.3-obsref(a,p,p0)/30)))
# Known-radius ideal model fit, noise-free, with independent all-position unknowns.
y=obs(a,p); z0=np.r_[pack(a)*[1.01,.99,1.01,1.005,.995,1.01],p0+np.array([10,-10,10]),(p+rng.normal(0,10,p.shape)).ravel()]
def f(z):return (obsref(unpack(z[:6]),z[9:].reshape(-1,3),z[6:9])-y).ravel()
fit=least_squares(f,z0,jac='2-point',max_nfev=80,xtol=1e-11,ftol=1e-11,gtol=1e-11)
out={'seed':917,'wide':summary(p),'narrow':summary(narrow),'unknown_radius':{'smallest_singular':float(np.linalg.svd(Ju,compute_uv=False)[-1]),'scale_null_residual':float(np.linalg.norm(Ju@xu)),'finite_1pct_encoder_max_diff_rad':finite},'known_radii_fit':{'success':bool(fit.success),'max_residual_mm':float(np.max(np.abs(fit.fun))),'anchor_errors_mm':np.linalg.norm(unpack(fit.x[:6])-a,axis=1).tolist(),'nfev':fit.nfev}}
print(json.dumps(out,indent=2))
# Diagnostic noise and axis-dependent proportional length bias; extra length is independently assumed exact.
experiments=[]
for label, pts in [('wide',p),('narrow',narrow)]:
 clean=obsref(a,pts,p0); base=np.r_[pack(a),p0,pts.ravel()]
 for kind in ['noise','axis_bias']:
  for constrained in [False,True]:
   delta=rng.normal(0,.03,clean.shape) if kind=='noise' else clean*np.array([.0001,-.0001,.0002,-.0002])
   measured=clean+delta
   def residual(z):
    aa=unpack(z[:6]); rr=(obsref(aa,z[9:].reshape(-1,3),z[6:9])-measured).ravel()
    return np.r_[rr,(aa[1,0]-3000)*100] if constrained else rr
   sol=least_squares(residual,base,max_nfev=30,xtol=1e-10,ftol=1e-10,gtol=1e-10)
   experiments.append({'layout':label,'kind':kind,'extra_exact_anchor_distance':constrained,'anchor_errors_mm':np.linalg.norm(unpack(sol.x[:6])-a,axis=1).tolist(),'rms_encoder_residual_mm':float(np.sqrt(np.mean(residual(sol.x)[:clean.size]**2))),'nfev':sol.nfev})
print(json.dumps({'perturbation_experiments':experiments},indent=2))
