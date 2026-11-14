import json,sys,numpy as np
from scipy.optimize import least_squares
TRUE=np.array([[0,-1900,-280],[1645.44826719,950,-280],[-1645.44826719,950,-280],[0,0,1900.]])
FOUND=np.array([[0,-1907.51,-275.89],[1651.62,953.54,-275.89],[-1651.94,953.79,-275.89],[0.02,0,1880.34]])
def load(path):
    d=json.load(open(path)); k=np.array(d['config']['mm_per_degree'])
    rows=[]
    for si,s in enumerate(d['sweeps']):
        for p in s['data_points']:
            rows.append((si,s['drive_anchor'],s['sensor_anchor'],np.array(p['mu'])*k,p.get('sensor_force_n',np.nan)))
    return d,rows
def solve_pos(A,dl,idx,p0):
    L0=np.linalg.norm(A,axis=1)
    f=lambda p:[np.linalg.norm(A[i]-p)-L0[i]-dl[i] for i in idx]
    r=least_squares(f,p0,xtol=1e-14,ftol=1e-14,gtol=1e-14)
    return r.x,np.abs(r.fun).max()
def residuals(A,rows):
    out=[];p=np.zeros(3);last=-1
    for si,drv,sen,dl,F in rows:
        if si!=last: p=np.array([0,0,-50.]);last=si   # start below: gravity side
        idx=[i for i in range(4) if i!=sen]
        p,err=solve_pos(A,dl,idx,p)
        pred=np.linalg.norm(A[sen]-p)-np.linalg.norm(A[sen])
        out.append((si,drv,sen,p.copy(),dl[sen]-pred,err))
    return out
if __name__=='__main__':
    d,rows=load(sys.argv[1])
    for name,A in(('TRUE',TRUE),('FOUND',FOUND)):
        R=residuals(A,rows)
        r=np.array([x[4] for x in R])
        print(name,'rms',np.sqrt((r**2).mean()),'max',np.abs(r).max(),'solve err',max(x[5] for x in R))
        if name=='TRUE' or len(sys.argv)>2:
            for (si,drv,sen,p,res,err),row in zip(R,rows):
                print(si,drv,sen,np.round(row[3],2),'p',np.round(p,1),'res %.3f'%res)
