"""Sweep-form design study on the static elastic model, fitted with the production pose fit."""
import sys, numpy as np
sys.path.insert(0, '/home/torbjorn/repos/hp-sim5')
from autocal.pose_fit import fit_pose_least_squares, HOME_OFFSET_PRIOR_SIGMA_MM
TRUE=np.array([[0,-1900,-280],[1645.44826719,950,-280],[-1645.44826719,950,-280],[0,0,1900.]])
Y=np.array([528.3,528.3,528.3,398.9]); NL=np.array([2,2,2,3.]); MG=0.081*9.82
L0=np.linalg.norm(TRUE,axis=1)
def statics(p,A=TRUE,mg=MG):
    u=A-p; L=np.linalg.norm(u,axis=1); U=(u/L[:,None]).T
    Tp=np.linalg.lstsq(U,np.array([0,0,mg]),rcond=None)[0]; n=np.linalg.svd(U)[2][-1]; n=n*np.sign(n[3])
    return Tp,n,L
def tens(p,s,F,A=TRUE,mg=MG):
    Tp,n,L=statics(p,A,mg); return Tp+(F-Tp[s])/n[s]*n
def uniform_T(p,tau,A=TRUE,mg=MG,Yv=Y):
    Tp,n,L=statics(p,A,mg); g=Yv+L
    lam=(tau*(n*g).sum()-(n*g*Tp/NL).sum())/(n*n*g/NL).sum()
    return Tp+lam*n
def enc(p,T,EA):
    EA=np.broadcast_to(np.asarray(EA,float),(4,)); L=np.linalg.norm(TRUE-p,axis=1)
    return (Y+L)/(1+T/(NL*EA))-(Y+L0)/(1+np.array([0,0,0,MG])/(NL*EA))
def circle(low,dlow,dD,A=TRUE,n=721):
    c1,c2=A[low],A[3]; r1=np.linalg.norm(c1)+dlow; r2=np.linalg.norm(c2)+dD
    d=np.linalg.norm(c2-c1); ex=(c2-c1)/d; a=(d*d+r1*r1-r2*r2)/(2*d); h2=r1*r1-a*a
    if h2<=0: return np.zeros((0,3))
    c=c1+a*ex; e1=np.cross(ex,[0,0,1.]); e1/=np.linalg.norm(e1); e2=np.cross(ex,e1)
    t=np.linspace(-np.pi,np.pi,n,endpoint=False)
    return c+np.sqrt(h2)*(np.cos(t)[:,None]*e1+np.sin(t)[:,None]*e2)
def sweep_poses(low,dlow,dD,F=2.0,Tmin=0.3,npts=20,Aplan=TRUE):
    """Poses a sweep would visit: the arc where the lines stay taut with the sensor at F."""
    free=[i for i in range(3) if i!=low]
    P=circle(low,dlow,dD,Aplan); ok=[]
    for p in P:
        if p[2]<-200: ok.append(False); continue
        ok.append(all(tens(p,s,F,Aplan).min()>=Tmin for s in free))
    ok=np.array(ok)
    if ok.sum()<npts: return []
    idx=np.flatnonzero(ok)
    # contiguous arc (may wrap)
    idx=np.sort(idx); gaps=np.flatnonzero(np.diff(idx)>1)
    if len(gaps): idx=np.concatenate([idx[gaps[0]+1:],idx[:gaps[0]+1]])
    sel=idx[np.linspace(0,len(idx)-1,npts).round().astype(int)]
    return [(P[i],free[k>=npts//2]) for k,i in enumerate(sel)]   # (pose, sensor line) : second half swaps roles
def dataset(sweeps,policy,val,EA,Aplan=TRUE,mgplan=MG,noise=0.0,rng=None,ferr=0.0):
    out=[]; Ts=[]
    for (low,dlow,dD) in sweeps:
        for p_plan,s in sweep_poses(low,dlow,dD,Aplan=Aplan):
            # the machine realises the planned fixed lengths; pose = true circle point nearest (approx: plan with truth anchors when Aplan is TRUE)
            p=p_plan
            if policy=='equal':
                F=uniform_T(p,val,Aplan,mgplan)[s]
                if F<0.3: continue
            else: F=val
            if rng is not None: F=F+rng.uniform(-ferr,ferr)
            T=tens(p,s,F)
            if T.min()<0.2: continue
            d=enc(p,T,EA)
            if rng is not None: d=d+rng.normal(0,noise,4)
            drive=[i for i in range(3) if i not in (low,s)][0]
            out.append({"fixed_anchors":[low,3],"fixed_lengths":[d[low],d[3]],"drive_anchor":drive,"sensor_anchor":s,
                        "data_points":[{"l_drive":d[drive],"l_sensor":d[s]}]}); Ts.append(T)
    return {"machine_type":"hangprinter_3","num_anchors":4,"dimensions":3,"config":{},"sweeps":out},np.array(Ts)
def shape(e): q=e.mean(0); return np.abs(e-q).max(),q
def report(label,sweeps,policy,val,EA=12000.,**kw):
    ds,Ts=dataset(sweeps,policy,val,EA,**kw)
    seed=TRUE+np.array([[0,30,10],[-20,15,10],[20,-15,10],[0,0,30.]])
    res=[]
    for off in (False,True):
        r=fit_pose_least_squares(ds,seed,fit_home_offsets=off,home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM,extra_initial_anchors=[TRUE])
        e=r['anchors']-TRUE; s,q=shape(e); res.append((np.abs(e).max(),s,q[2],r['rms_mm'],r['home_offsets_mm'][3],e[3,2]))
    a,b=res
    print(f'{label:52s} N={len(ds["sweeps"]):3d} T {Ts.min():.1f}..{Ts.max():4.1f} | rigid raw {a[0]:5.2f} shape {a[1]:5.2f} Dz {a[5]:+6.2f} rms {a[3]:.3f} | +offset raw {b[0]:5.2f} shape {b[1]:5.2f} Dz {b[5]:+6.2f} kappa {b[4]:+.3f} rms {b[3]:.4f}')
    return res
BOOT=[(0,390,0),(1,390,0),(2,390,0)]
def levels(dDs,dlows=(390,)): return [(low,dl,dD) for dD in dDs for dl in dlows for low in range(3)]
if __name__=='__main__':
    for p,s in sweep_poses(0,390,0)[::5]: print(np.round(p),s)
    for dD in (0,-300,-600,-900): 
        sp=sweep_poses(0,390,dD); print(dD,len(sp), np.round(sp[0][0]) if sp else None, np.round(sp[-1][0]) if sp else None)
    print('EA 12 kN')
    report('bootstrap (D=0), const 2.0',BOOT,'const',2.0)
    report('D=0 x dlow 200,390,580 const 2.0',levels([0],(200,390,580)),'const',2.0)
    for dDs in ([0,-300],[0,-300,-600],[0,-400,-800],[0,-300,-600,-900]):
        report(f'D {dDs} const 2.0',levels(dDs),'const',2.0)
        report(f'D {dDs} equal 1.0',levels(dDs),'equal',1.0)
        report(f'D {dDs} equal 1.5',levels(dDs),'equal',1.5)
