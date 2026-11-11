# Firmware-intended anchor forces (min-norm in [Tmin,Tmax], gravity) vs simulated spool loads in a run whose servos track.
import json,sys,numpy as np
from scipy.optimize import minimize
anch=np.array([[0,-1900,-280],[1645.44826719,950,-280],[-1645.44826719,950,-280],[0,0,1900]])/1000
MA=np.array([2,2,2,4]); Tmin,Tmax=float(sys.argv[2]),float(sys.argv[3]); g=np.array([0,0,-9.81*0.081])
def planned(p):
    U=(anch-p); U/=np.linalg.norm(U,axis=1)[:,None]
    r=minimize(lambda T:T@T,np.full(4,20.),constraints=[{'type':'eq','fun':lambda T:U.T@T+g}],bounds=[(Tmin,Tmax)]*4,method='SLSQP')
    return r.x
d=json.load(open(sys.argv[1])); P=np.array(d['pos']); t=np.array(d['t']); T=np.array(d['tension'])
spool=np.c_[T[:,0]+T[:,1],T[:,2]+T[:,3],T[:,4]+T[:,5],T[:,6]+T[:,7]+T[:,8]]   # sim line pull on each spool
print('pos(mm)         planned F_ABCD (N)        planned spool load F/MA      sim spool load     ratio sim/planned')
for X,Y in ((0,0),(200,-200),(500,-500),(-500,-500),(-500,500),(500,500),(600,-600)):
    k=np.where((np.hypot(P[:,0]*1000-X,P[:,1]*1000-Y)<3)&(t>1.5))[0]
    i=k[len(k)//2]; F=planned(P[i]); pl=F/MA
    print(f'({X:4d},{Y:4d})  {np.round(F,1)}  {np.round(pl,1)}  {np.round(spool[i],1)}  {np.round(spool[i]/pl,2)}')
