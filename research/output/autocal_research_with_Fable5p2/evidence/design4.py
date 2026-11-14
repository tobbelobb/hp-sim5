import sys,numpy as np
sys.path.insert(0,sys.argv[0].rsplit('/',1)[0]); from elastic import *; from design import *; from design2 import *; from design3 import *
def shape(e):
    q=e.mean(0); s=e-q; return np.abs(s).max(),q
def run2(label,poses,tau=None,F=None,EA=12000.,Aplan=TRUE,mgplan=MG,Tmin=0.8):
    P=[];D=[];Ts=[]
    for p in poses:
        if tau is not None:
            Tplan=uniform_T(p,tau,Aplan,mgplan)
            if Tplan.min()<Tmin: continue
            s=int(np.argmin(Tplan)); T,_,_=tensions(p,s,Tplan[s])
        else:
            s,T=min_tension_line(p,F)
        P.append(p);D.append(enc_T(p,T,EA));Ts.append(T)
    P=np.array(P);D=np.array(D);Ts=np.array(Ts)
    e,_=bias(P,D); ek,rk,_=bias_k(P,D); (a,q),(b,qk)=shape(e),shape(ek)
    print(f'{label:44s} N={len(P):3d} T {Ts.min():.1f}..{Ts.max():4.1f}N | rigid: raw {np.abs(e).max():5.2f} shape {a:5.2f} | +offset: raw {np.abs(ek).max():5.2f} shape {b:5.2f} home shift z {qk[2]:+.2f} res {rk:.3f}')
if __name__=='__main__':
    G2=grid([0,400,800,1200],250,Tmax=8.); DS=dsphere(175,700)
    print('EA = 12 kN unless stated. "shape" = worst anchor coordinate error after removing a common translation of all anchors')
    run2('D-sphere poses (current family), F=2.23',DS,F=2.23)
    run2('D-sphere poses, equal-strain tau=1.5',DS,tau=1.5)
    run2('volume poses, constant F=2.23',G2,F=2.23)
    run2('volume poses, constant F=1.0',G2,F=1.0)
    run2('volume poses, equal-strain tau=1.5',G2,tau=1.5)
    run2('volume poses, equal-strain tau=1.0',G2,tau=1.0)
    run2('  ... EA=8.5k',G2,tau=1.5,EA=8500.)
    run2('  ... EA=20k',G2,tau=1.5,EA=20000.)
    run2('  ... EA x [0.7,1.3,1,1]',G2,tau=1.5,EA=12000*np.array([.7,1.3,1,1]))
    run2('  ... EA x [1,1,1,0.6]',G2,tau=1.5,EA=12000*np.array([1,1,1,.6]))
    rng=np.random.default_rng(1)
    run2('  ... planned with anchors off ~20 mm',G2,tau=1.5,Aplan=TRUE+rng.normal(0,20,(4,3)))
    run2('  ... planned with anchors off ~100 mm',G2,tau=1.5,Aplan=TRUE+rng.normal(0,100,(4,3)))
    run2('  ... planned with mass 25% high',G2,tau=1.5,mgplan=MG*1.25)
    for zs in ([0,300],[0,300,600],[0,600],[0,400,800,1200]):
        run2(f'equal-strain tau=1.5, heights {zs}',grid(zs,250,Tmax=8.),tau=1.5)
