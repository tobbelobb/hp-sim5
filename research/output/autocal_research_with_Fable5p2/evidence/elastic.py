"""Static elastic point-effector CDPR model: pose + torque-controlled line + force -> encoder length changes."""
import numpy as np
TRUE=np.array([[0,-1900,-280],[1645.44826719,950,-280],[-1645.44826719,950,-280],[0,0,1900.]])
Y=np.array([528.3,528.3,528.3,398.9]); NL=np.array([2,2,2,3.]); MG=0.081*9.82
L0=np.linalg.norm(TRUE,axis=1)
def tensions(p,s,F,A=TRUE,mg=MG):
    """Line tensions at pose p when line s is torque controlled at F. Returns T (4,), n (unit null vector, n>0 inside)."""
    u=A-p; L=np.linalg.norm(u,axis=1); U=(u/L[:,None]).T          # 3x4
    Tp=np.linalg.lstsq(U,np.array([0,0,mg]),rcond=None)[0]        # min-norm, orthogonal to n
    n=np.linalg.svd(U)[2][-1]; n=n*np.sign(n[3])
    lam=(F-Tp[s])/n[s]
    return Tp+lam*n,n,L
def encoders(p,s,F,EA,A=TRUE,mg=MG,Yv=Y):
    """EA: scalar or 4-vector, N per single line. Reference: home pose, ABC at zero tension, D carries mg."""
    EA=np.broadcast_to(np.asarray(EA,float),(4,))
    T,n,L=tensions(p,s,F,A,mg)
    Tref=np.array([0,0,0,mg]); La=np.linalg.norm(A,axis=1)
    U=(Yv+L)/(1+T/(NL*EA)); Uref=(Yv+La)/(1+Tref/(NL*EA))
    return U-Uref,T,n
def min_tension_line(p,F,A=TRUE,mg=MG):
    """Torque-control the line that ends up with the lowest tension, so every line carries at least F."""
    best=None
    for s in range(4):
        T,n,_=tensions(p,s,F,A,mg)
        if T.min()>=F-1e-9 and (best is None or T.max()<best[1].max()): best=(s,T)
    return best
