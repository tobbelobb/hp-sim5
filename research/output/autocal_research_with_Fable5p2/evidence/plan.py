"""Make a measurement plan (poses + torque line + force) and analyse the simulator's answer."""
import sys,json,subprocess,numpy as np
sys.path.insert(0,sys.argv[0].rsplit('/',1)[0]); from elastic import *; from design import *; from design2 import *; from design3 import *; from design4 import shape
AXN='ABCD'
def order(P):                                       # serpentine: layer by layer, nearest neighbour
    P=[np.array(p) for p in P]; out=[P.pop(int(np.argmin([np.linalg.norm(p) for p in P])))]
    while P:
        i=int(np.argmin([np.linalg.norm(p-out[-1])+3*abs(p[2]-out[-1][2]) for p in P])); out.append(P.pop(i))
    return out
def make(poses,policy,val,Aplan=TRUE,mgplan=MG,Tmin=0.8):
    plan=[]
    for p in order(poses):
        if policy=='equal':
            Tp=uniform_T(p,val,Aplan,mgplan)
            if Tp.min()<Tmin: continue
            s=int(np.argmin(Tp)); F=float(Tp[s])
        else:
            s,_=min_tension_line(p,val,Aplan,mgplan); F=val
        dl=np.linalg.norm(Aplan-p,axis=1)-np.linalg.norm(Aplan,axis=1)      # rigid lengths from the planning anchors
        plan.append(dict(pos={AXN[i]:float(dl[i]) for i in range(4) if i!=s},torque={AXN[s]:F},target=[float(v) for v in p]))
    return plan
def analyse(res,label):
    pts=res['points']; D=np.array([q['enc_mm'] for q in pts]); P=np.array([q['pos_mm'] for q in pts]); T=np.array([q['T'] for q in pts])
    geom=np.linalg.norm(TRUE[None]-P[:,None],axis=2)-L0
    Fc=np.array([q['forceCmd'] for q in pts]); Fr=np.array([q['T'][AXN.index(q['torqueAxis'])] for q in pts])
    e,r=bias(P,D); ek,rk,kap=bias_k(P,D); eg,_=bias(P,geom)
    (a,_),(b,qk)=shape(e),shape(ek)
    print(f'{label:36s} N={len(P):3d} T {T.min():.2f}..{T.max():.2f}N  enc-geom up to {np.abs(D-geom).max():.3f}mm  realised-commanded sensor force {np.min(Fr-Fc):+.2f}..{np.max(Fr-Fc):+.2f}N')
    print(f'{"":36s} rigid fit: raw {np.abs(e).max():5.2f} shape {a:5.2f} res {r:.3f} | +offset: raw {np.abs(ek).max():5.2f} shape {b:5.2f} home shift z {qk[2]:+.2f} res {rk:.4f} | true-geometry fit {np.abs(eg).max():.3f}')
    return ek
if __name__=='__main__':
    S=sys.argv[0].rsplit('/',1)[0]; name=sys.argv[1]; policy=sys.argv[2]; val=float(sys.argv[3]); iters=sys.argv[4] if len(sys.argv)>4 else '20'
    G2=grid([0,400,800,1200],250,Tmax=8.)
    plan=make(G2,policy,val); json.dump(plan,open(f'{S}/{name}.plan.json','w'))
    subprocess.run(['node',f'{S}/direct.mjs',f'{S}/{name}.plan.json',f'{S}/{name}.out.json',iters],check=True,stderr=subprocess.DEVNULL)
    analyse(json.load(open(f'{S}/{name}.out.json')),f'{name}: {policy} {val} iters {iters}')
