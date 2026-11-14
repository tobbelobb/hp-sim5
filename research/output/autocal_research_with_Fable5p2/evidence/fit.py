import json,sys,pickle,numpy as np
from scipy.optimize import least_squares
np.set_printoptions(precision=2,suppress=True,linewidth=250)
TRUE=np.array([[0,-1900,-280],[1645.44826719,950,-280],[-1645.44826719,950,-280],[0,0,1900.]])
def unpack(x): # 9 params: Ay, zlow, Bx,By, Cx,Cy, Dx,Dy,Dz
    return np.array([[0,x[0],x[1]],[x[2],x[3],x[1]],[x[4],x[5],x[1]],[x[6],x[7],x[8]]])
def pack(A): return np.array([A[0,1],A[0,2],A[1,0],A[1,1],A[2,0],A[2,1],A[3,0],A[3,1],A[3,2]])
def pos_from(A,L0,dl,idx,p):
    for _ in range(30):   # Newton on 3 sphere equations
        v=p-A[idx]; n=np.linalg.norm(v,axis=1)
        f=n-L0[idx]-dl[idx]; J=v/n[:,None]
        p=p-np.linalg.solve(J,f)
        if np.abs(f).max()<1e-10: break
    return p
def resid(x,rows):
    A=unpack(x); L0=np.linalg.norm(A,axis=1); out=[]; last=-1
    for si,sen,dl in rows:
        if si!=last: p=np.array([0,0,-50.]); last=si
        idx=[i for i in range(4) if i!=sen]
        p=pos_from(A,L0,dl,idx,p)
        out.append(dl[sen]-(np.linalg.norm(A[sen]-p)-L0[sen]))
    return np.array(out)
def fit(rows,x0=None,label=''):
    x0=pack(TRUE)+np.array([5,-5,5,5,-5,5,3,-3,8.]) if x0 is None else x0
    r=least_squares(resid,x0,args=(rows,),x_scale=100,xtol=1e-12,ftol=1e-12)
    A=unpack(r.x); e=A-TRUE
    print(f'{label:34s} rms_res={np.sqrt((r.fun**2).mean()):.4f}mm  anchor err max={np.abs(e).max():6.2f} rms={np.sqrt((e**2).mean()):5.2f}  A.y {e[0,1]:+.2f} zlow {e[0,2]:+.2f} B {e[1,0]:+.2f},{e[1,1]:+.2f} C {e[2,0]:+.2f},{e[2,1]:+.2f} D {e[3,0]:+.2f},{e[3,1]:+.2f},{e[3,2]:+.2f}')
    return r
def load(dirn,nsweeps=None):
    d=json.load(open(dirn+'/sweeps.json')); k=np.array(d['config']['mm_per_degree'])
    out,_=pickle.load(open(dirn+'/rrd.pkl','rb'))
    t,pos=out[('/world/machines/default/bodies/Effector_16','translation')]
    rows=[]
    for si,s in enumerate(d['sweeps'][:nsweeps]):
        for p in s['data_points']:
            i=min(np.searchsorted(t,p['timestamp_ms']/1000),len(t)-1)
            pp=pos[i]*1000-np.array([0,0,100.])
            geom=np.linalg.norm(TRUE-pp,axis=1)-np.linalg.norm(TRUE,axis=1)
            rows.append(dict(si=si,drv=s['drive_anchor'],sen=s['sensor_anchor'],fix=s['fixed_anchors'],enc=np.array(p['mu'])*k,geom=geom,pos=pp))
    return rows
def variant(rows,which):  # which: set of roles replaced by true geometry
    o=[]
    for r in rows:
        dl=r['enc'].copy()
        for a in range(4):
            role='sen' if a==r['sen'] else 'drv' if a==r['drv'] else 'fix'
            if role in which or (role=='fix' and ('fixD' in which and a==3 or 'fixL' in which and a!=3)): dl[a]=r['geom'][a]
        o.append((r['si'],r['sen'],dl))
    return o
if __name__=='__main__':
    rows=load(sys.argv[1],int(sys.argv[2]) if len(sys.argv)>2 else None)
    print('points',len(rows))
    fit(variant(rows,set()),label='raw encoder')
    fit(variant(rows,{'sen','drv','fix'}),label='all lines true geometry')
    fit(variant(rows,{'sen'}),label='only sensor corrected')
    fit(variant(rows,{'drv'}),label='only drive corrected')
    fit(variant(rows,{'fixL'}),label='only fixed low anchor corrected')
    fit(variant(rows,{'fixD'}),label='only fixed D corrected')
    fit(variant(rows,{'drv','fix'}),label='drive+fixed corrected')
    # sensitivity: Jacobian at truth
    v=variant(rows,{'sen','drv','fix'}); x=pack(TRUE); J=np.array([(resid(x+h,v)-resid(x-h,v))/2e-3 for h in np.eye(9)*1e-3]).T
    sv=np.linalg.svd(J/np.sqrt(len(v)),compute_uv=False); print('singular values of d(res)/d(param) per-point-rms:',sv)
    print('=> 1/sv (mm param error per mm rms coherent residual):',1/sv)
    # what stretch-knowledge accuracy is needed: scale the (geom-enc) correction by factor f
    for f in (0.5,0.8,0.9,0.95,1.0):
        v=[(r['si'],r['sen'],r['enc']+f*(r['geom']-r['enc'])) for r in rows]; fit(v,label=f'{int(f*100)}% of stretch removed')
