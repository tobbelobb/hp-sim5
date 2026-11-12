"""Spectral radius of the unsaturated servo+line step map (finite-difference Jacobian)."""
import numpy as np, sys
def step(s, p):
    th, w, x, v, I, ep = s
    T,J,dt,r,k,k2,m,kp,kd,ki = p
    e=-th; cmd=kp*e+I+kd*(e-ep)/dt; I=I+ki*e*dt; K=kp+kd/dt
    w=w+dt*cmd/(J+K*dt*dt)
    ths=th+dt*w; xs=x+dt*v
    Jeff=J+K*dt*dt; a1=1/(k*dt*dt); a2=1/(k2*dt*dt)
    # exact solve of the 2 linear rows (converged GS)
    l1=l2=0.0
    for _ in range(200):
        d=-(xs-r*ths+a1*l1)/(1/m+r*r/Jeff+a1); l1+=d; xs+=d/m; ths-=r/Jeff*d
        d2=-(xs+a2*l2)/(1/m+a2); l2+=d2; xs+=d2/m
    return np.array([ths,(ths-th)/dt,xs,(xs-x)/dt,I,e])
def rho(T=1.0,J=5e-5,npp=50,dt=2e-3,r=0.03,k=13000.,k2=40000.,m=0.05,gains=None):
    kp=T*npp; kd=2*np.sqrt(kp*J); ki=kp
    if gains: kp,kd,ki=gains(T,npp,J,dt)
    p=(T,J,dt,r,k,k2,m,kp,kd,ki); s0=np.zeros(6); A=np.zeros((6,6)); h=1e-7
    for i in range(6):
        e=np.zeros(6); e[i]=h; A[:,i]=(step(s0+e,p)-step(s0-e,p))/(2*h)
    ev=np.linalg.eigvals(A); return max(abs(ev)), ev
if __name__=="__main__":
    for dt in [2e-3,1e-3,5e-4]:
        print(dt, " ".join(f"T{T}:{rho(T=T,dt=dt)[0]:.4f}" for T in [0.25,0.5,1,2,4]))
    print(np.round(rho(T=1.0)[1],4))
