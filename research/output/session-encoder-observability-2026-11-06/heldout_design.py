import numpy as np,json
from scipy.optimize import least_squares
from scipy.linalg import null_space
A=np.array([[0,0,0],[3000,0,0],[1000,2600,0],[1300,1200,2800.]],float); ref=np.array([1400,1000,700.]); aidx=[(1,0),(2,0),(2,1),(3,0),(3,1),(3,2)]
def pack(a):return np.array([a[i,j] for i,j in aidx])
def unpack(x):
 a=np.zeros((4,3))
 for v,(i,j) in zip(x,aidx):a[i,j]=v
 return a
def ranges(a,p):return np.linalg.norm(p[:,None,:]-a[None,:,:],axis=2)
def obs(a,p,r):return ranges(a,p)-ranges(a,r[None,:])
def solve(p,y):
 # Fixed coarse perturbation starts near the generating geometry; this is a local-solver test.
 x=np.r_[pack(A)+[12,-10,8,6,-7,9],ref+[10,-10,8],(p+np.array([10,-10,8])).ravel()]
 def f(z):return (obs(unpack(z[:6]),z[9:].reshape(-1,3),z[6:9])-y).ravel()
 r=least_squares(f,x,xtol=1e-9,ftol=1e-9,gtol=1e-9,max_nfev=35)
 return np.linalg.norm(unpack(r.x[:6])-A,axis=1).tolist(),float(np.sqrt(np.mean(r.fun*r.fun))),r.nfev
rows=[]
for seed in range(101,111):
 rng=np.random.default_rng(seed); narrow=ref+rng.uniform([-200,-200,-40],[200,200,40],(80,3));wide=ref+rng.uniform([-800,-700,-350],[800,700,700],(40,3)); noise=rng.normal(0,.03,(80,4))
 for mode in ['narrow','mixed']:
  p=narrow if mode=='narrow' else np.vstack((narrow[:40],wide)); y=obs(A,p,ref)+noise
  err,rms,n=solve(p,y); rows.append(dict(seed=seed,design=mode,anchor_errors_mm=err,max_mm=max(err),rms_mm=rms,nfev=n))
print(json.dumps({'rows':rows,'median_max_mm':{mode:float(np.median([r['max_mm'] for r in rows if r['design']==mode])) for mode in ['narrow','mixed']}},indent=2))
