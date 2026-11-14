import sys,json,pickle,numpy as np
from scipy.optimize import least_squares
sys.path.insert(0,sys.argv[0].rsplit('/',1)[0]); from elastic import *; from design import *
def resid_k(x,D,P0):
    D2=D.copy(); D2[:,3]+=x[9]; return resid(x[:9],D2,P0)
def bias_k(poses,D):
    r=least_squares(resid_k,np.r_[X0,0.],args=(D,poses),x_scale=np.r_[np.full(9,100.),1.],xtol=1e-13,ftol=1e-13)
    return unpack(r.x[:9])-TRUE,np.sqrt((r.fun**2).mean()),r.x[9]
def rep(label,poses,F=2.23,EA=12000.,sensors=None):
    poses=np.asarray(poses,float); D,T=measure(poses,F,EA,sensors=sensors)
    e,rms=bias(poses,D); ek,rmsk,kap=bias_k(poses,D)
    Dr,_=measure(poses,F,1e15,sensors=sensors)
    Jk=np.array([(resid_k(np.r_[X0,0.]+h,Dr,poses)-resid_k(np.r_[X0,0.]-h,Dr,poses))/2e-3 for h in np.eye(10)*1e-3]).T
    C=np.linalg.inv(Jk.T@Jk); sd=np.sqrt(np.diag(C)[:9].max())   # worst anchor-coordinate std per mm of independent noise per point
    f=lambda e:f'max {np.abs(e).max():6.2f} (A.y {e[0,1]:+.2f} zlow {e[0,2]:+.2f} B.x {e[1,0]:+.2f} D.z {e[3,2]:+.2f})'
    print(f'{label:40s} N={len(poses):3d} T {T.min():.1f}..{T.max():4.1f}N | rigid: {f(e)} res {rms:.3f} | +offset: {f(ek)} res {rmsk:.3f} kappa {kap:+.3f} | noise amp {sd:5.1f}')
    return ek
if __name__=='__main__':
    dirn=sys.argv[1]; d=json.load(open(dirn+'/sweeps.json')); out,_=pickle.load(open(dirn+'/rrd.pkl','rb'))
    t,pos=out[('/world/machines/default/bodies/Effector_16','translation')]
    P=[];Sn=[]
    for s in d['sweeps'][:3]:
        for pi,p in enumerate(s['data_points']):
            i=min(np.searchsorted(t,p['timestamp_ms']/1000),len(t)-1); P.append(pos[i]*1000-np.array([0,0,100.]))
            Sn.append(s['sensor_anchor'] if pi<10 else s['drive_anchor'])
    rep('existing bootstrap poses',P,sensors=Sn)
    rep('D-sphere disc r<=700',dsphere(175,700))
    for zs in ([0,300],[0,300,600,900],[0,400,800,1200]):
        rep(f'volume grid z={zs}',grid(zs,350))
        rep(f'volume grid z={zs} Tmax 6N',grid(zs,250,Tmax=6.))
