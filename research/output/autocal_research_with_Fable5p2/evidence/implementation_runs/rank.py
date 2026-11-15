import sys, numpy as np
sys.path.insert(0,'.'); from sw import *
from autocal.pose_fit import consistency_residuals
from autocal.theoretical_ellipse import anchor_opt_vec_to_matrix, anchors_matrix_to_opt_vec
MT='hangprinter_3'
def inside_poses(low,dlow,dD,A,npts=12):
    P=circle(low,dlow,dD,A,n=360); ok=[]
    for p in P:
        _,n,_=statics(p,A); ok.append(n.min()>0.02 and p[2]>A[:3,2].max())
    idx=np.flatnonzero(ok)
    if len(idx)<npts: return np.zeros((0,3))
    gaps=np.flatnonzero(np.diff(idx)>1)
    if len(gaps): idx=np.concatenate([idx[gaps[0]+1:],idx[:gaps[0]+1]])
    return P[idx[np.linspace(0,len(idx)-1,npts).round().astype(int)]]
def info(P,A,with_offset=True,eps=0.5):
    if len(P)==0: return None
    x0=anchors_matrix_to_opt_vec(A,MT,None); lengths=np.linalg.norm(A[None]-P[:,None],axis=2)-np.linalg.norm(A,axis=1)
    def r(x,k):
        off=np.zeros(4); off[3]=k
        return consistency_residuals(anchor_opt_vec_to_matrix(x,MT,4,3,None),lengths,off).ravel()
    cols=[]
    for i in range(x0.size):
        d=np.zeros(x0.size); d[i]=eps; cols.append((r(x0+d,0)-r(x0-d,0))/(2*eps))
    if with_offset: cols.append((r(x0,eps)-r(x0,-eps))/(2*eps))
    J=np.array(cols).T; return J.T@J
def greedy(A,start,cands,steps,with_offset=True,reg=1e-9):
    I=sum(info(inside_poses(*c,A),A,with_offset) for c in start); chosen=[]
    infos={c:info(inside_poses(*c,A),A,with_offset) for c in cands}
    infos={c:v for c,v in infos.items() if v is not None}
    for _ in range(steps):
        base=np.linalg.slogdet(I+reg*np.eye(len(I)))[1]
        sc=sorted(((np.linalg.slogdet(I+v+reg*np.eye(len(I)))[1]-base,c) for c,v in infos.items() if c not in chosen and c not in start),reverse=True)
        chosen.append(sc[0][1]); I=I+infos[sc[0][1]]
        ev=np.linalg.eigvalsh(I); print('  pick',sc[0][1],f'gain {sc[0][0]:.2f}  next best',[(c,round(g,2)) for g,c in sc[1:4]],f'min eig {ev[0]:.2e}')
    return chosen
if __name__=='__main__':
    cands=[(low,dl,dD) for low in range(3) for dl in (0,100,200,300,390,500,600) for dD in (0,-150,-300,-450,-600,-750,-900) if abs(dl)+abs(dD)<=1000]
    Aplan=TRUE+np.array([[0,30,10],[-20,15,10],[20,-15,10],[0,0,-30.]])
    for wo in (True,False):
        print('with offset' if wo else 'anchors only')
        ch=greedy(Aplan,BOOT,cands,6,wo)
        for k in (2,3,6):
            report(f' boot+{k} const 2.0',BOOT+ch[:k],'const',2.0)
            report(f' boot+{k} equal 1.0',BOOT+ch[:k],'equal',1.0)
