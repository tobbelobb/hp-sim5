import sys,numpy as np
from scipy.optimize import least_squares
np.set_printoptions(precision=2,suppress=True,linewidth=250)
sys.path.insert(0,sys.argv[0].rsplit('/',1)[0]); from elastic import *
def unpack(x): return np.array([[0,x[0],x[1]],[x[2],x[3],x[1]],[x[4],x[5],x[1]],[x[6],x[7],x[8]]])
def pack(A): return np.array([A[0,1],A[0,2],A[1,0],A[1,1],A[2,0],A[2,1],A[3,0],A[3,1],A[3,2]])
X0=pack(TRUE)
def resid(x,D,P0):
    """Signed distance of each 4-length measurement from the rigid model's 3-dof manifold. D: (N,4) length changes."""
    A=unpack(x); La=np.linalg.norm(A,axis=1); out=np.empty(len(D))
    for k,(dl,p) in enumerate(zip(D,P0)):
        l=La+dl; p=p.copy()
        for _ in range(12):
            v=p-A; nn=np.linalg.norm(v,axis=1); f=nn-l; J=v/nn[:,None]
            step=np.linalg.lstsq(J,f,rcond=None)[0]; p-=step
            if np.abs(step).max()<1e-9: break
        v=p-A; nn=np.linalg.norm(v,axis=1); f=nn-l; J=v/nn[:,None]
        n=np.linalg.svd(J.T)[2][-1]; n*=np.sign(n[3]); out[k]=n@f
    return out
def jac(D,P0,x=X0,h=1e-3): return np.array([(resid(x+e,D,P0)-resid(x-e,D,P0))/(2*h) for e in np.eye(9)*h]).T
def measure(poses,F,EA,policy='min',sensors=None):
    D=[];T=[]
    for k,p in enumerate(poses):
        s=sensors[k] if sensors is not None else min_tension_line(p,F)[0]
        e,t,_=encoders(p,s,F[k] if np.ndim(F) else F,EA); D.append(e); T.append(t)
    return np.array(D),np.array(T)
def bias(poses,D):
    r=least_squares(resid,X0,args=(D,poses),x_scale=100,xtol=1e-13,ftol=1e-13)
    return unpack(r.x)-TRUE,np.sqrt((r.fun**2).mean())
def report(label,poses,F=2.23,EA=12000.,sensors=None,quiet=False):
    poses=np.asarray(poses,float)
    D,T=measure(poses,F,EA,sensors=sensors)
    e,rms=bias(poses,D)
    Dr,_=measure(poses,F,1e15,sensors=sensors); J=jac(Dr,poses)
    sv=np.linalg.svd(J/np.sqrt(len(poses)),compute_uv=False)
    if not quiet: print(f'{label:44s} N={len(poses):4d} Tmax={T.max():5.1f}N  err max={np.abs(e).max():6.2f} rms={np.sqrt((e**2).mean()):5.2f} | A.y {e[0,1]:+.2f} zlow {e[0,2]:+.2f} B {e[1,0]:+.2f},{e[1,1]:+.2f} D.z {e[3,2]:+.2f} | fit res {rms:.3f} | noise gain max {1/sv[-1]:6.1f}')
    return e,sv
def feasible(p,F=2.23,Tmax=15.):
    b=min_tension_line(p,F); return b is not None and b[1].max()<=Tmax
def grid(zs,step,F=2.23,Tmax=15.):
    P=[]
    for z in zs:
        for x in np.arange(-1500,1501,step):
            for y in np.arange(-1500,1501,step):
                if feasible(np.array([x,y,z]),F,Tmax): P.append([x,y,z])
    return np.array(P,float)
def dsphere(step,rmax,F=2.23):
    P=[]
    for x in np.arange(-rmax,rmax+1,step):
        for y in np.arange(-rmax,rmax+1,step):
            if x*x+y*y<=rmax*rmax:
                p=np.array([x,y,1900-np.sqrt(1900**2-x*x-y*y)])
                if feasible(p,F): P.append(p)
    return np.array(P)
if __name__=='__main__':
    # existing bootstrap: reproduce from recorded poses
    import json,pickle
    dirn=sys.argv[1]; d=json.load(open(dirn+'/sweeps.json')); out,_=pickle.load(open(dirn+'/rrd.pkl','rb'))
    t,pos=out[('/world/machines/default/bodies/Effector_16','translation')]
    P=[];Sn=[]
    for s in d['sweeps'][:3]:
        for pi,p in enumerate(s['data_points']):
            i=min(np.searchsorted(t,p['timestamp_ms']/1000),len(t)-1); P.append(pos[i]*1000-np.array([0,0,100.]))
            Sn.append(s['sensor_anchor'] if pi<10 else s['drive_anchor'])
    for EA in (20000,12000): report(f'existing bootstrap poses, EA={EA}',P,EA=EA,sensors=Sn)
    report('same poses, lowest-tension line as sensor',P)
    report('D-sphere disc r<=700 (current family)',dsphere(175,700))
    for zs in ([0],[0,300],[0,300,600],[0,300,600,900],[0,400,800,1200],[0,1200],[600],[0,300,600,900,1200,1400]):
        report(f'volume grid z={zs}',grid(zs,350))
