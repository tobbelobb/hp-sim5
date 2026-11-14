import sys,numpy as np
sys.path.insert(0,sys.argv[0].rsplit('/',1)[0]); from elastic import *; from design import *; from design2 import *
import elastic
def uniform_T(p,tau,A=TRUE,mg=MG,Yv=Y):
    """Tensions whose n-weighted mean per-strand strain is pose independent: sum n_i g_i (T_i/N_i - tau) = 0."""
    u=A-p; L=np.linalg.norm(u,axis=1); U=(u/L[:,None]).T
    Tp=np.linalg.lstsq(U,np.array([0,0,mg]),rcond=None)[0]; n=np.linalg.svd(U)[2][-1]; n*=np.sign(n[3])
    g=Yv+L; lam=(tau*(n*g).sum()-(n*g*Tp/NL).sum())/(n*n*g/NL).sum()
    return Tp+lam*n
def enc_T(p,T,EA):
    EA=np.broadcast_to(np.asarray(EA,float),(4,)); L=np.linalg.norm(TRUE-p,axis=1)
    return (Y+L)/(1+T/(NL*EA))-(Y+L0)/(1+np.array([0,0,0,MG])/(NL*EA))
def run(label,poses,tau,EA=12000.,Aplan=TRUE,mgplan=MG,Tmin=0.8,Yplan=Y):
    P=[];D=[];Ts=[]
    for p in poses:
        Tplan=uniform_T(p,tau,Aplan,mgplan,Yplan)
        if Tplan.min()<Tmin: continue
        s=int(np.argmin(Tplan))                       # torque-control the least loaded line at its planned force
        T,_,_=tensions(p,s,Tplan[s])                  # true tensions that result
        P.append(p);D.append(enc_T(p,T,EA));Ts.append(T)
    P=np.array(P);D=np.array(D);Ts=np.array(Ts)
    e,rms=bias(P,D); ek,rmsk,kap=bias_k(P,D)
    f=lambda e:f'max {np.abs(e).max():5.2f} (A.y {e[0,1]:+.2f} zlow {e[0,2]:+.2f} B.x {e[1,0]:+.2f} D.z {e[3,2]:+.2f})'
    print(f'{label:46s} N={len(P):3d} T {Ts.min():.1f}..{Ts.max():4.1f}N | rigid: {f(e)} | +offset: {f(ek)} res {rmsk:.4f}')
    return P
if __name__=='__main__':
    G=grid([0,300,600,900],250,Tmax=8.); G2=grid([0,400,800,1200],250,Tmax=8.)
    for tau in (1.0,1.5,2.5):
        run(f'grid z=0..900, equal-strain force, tau={tau}',G,tau)
    run('grid z=0..1200, tau=1.5',G2,1.5)
    run('same, EA=20000',G2,1.5,EA=20000.)
    run('same, EA=8500',G2,1.5,EA=8500.)
    rng=np.random.default_rng(1)
    run('EA per line 12k x [0.7,1.3,1.0,1.0]',G2,1.5,EA=12000*np.array([.7,1.3,1,1]))
    run('EA per line 12k x [1,1,1,0.6] (D soft)',G2,1.5,EA=12000*np.array([1,1,1,.6]))
    run('EA per line 12k x [1,1,1,1.6] (D stiff)',G2,1.5,EA=12000*np.array([1,1,1,1.6]))
    run('planned with anchors off by ~20mm',G2,1.5,Aplan=TRUE+rng.normal(0,20,(4,3)))
    run('planned with mover mass 25% too high',G2,1.5,mgplan=MG*1.25)
    run('planned with guy lengths = 0',G2,1.5,Yplan=Y*0)
    print('-- same poses, constant sensor force on least loaded line (no force planning)')
    rep('grid z=0..1200 constant F=2.23',G2)
    rep('grid z=0..1200 constant F=1.0',G2,F=1.0)
