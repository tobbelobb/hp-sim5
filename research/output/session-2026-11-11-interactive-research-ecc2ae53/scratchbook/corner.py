import json,sys,numpy as np
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from contour import load,contour
H=float(sys.argv[1]); files=sys.argv[2:]
fig,ax=plt.subplots(1,3,figsize=(17,5.5))
for fn in files:
    d,tip=load(fn); h,e=contour(tip); t=np.array(d['t']); se=np.array(d['servoErr'],float)
    m=(h==H)&~np.isnan(e)
    x,y=tip[:,0],tip[:,1]
    # arc coordinate along the +x side (descending y) then -y side (descending x), 0 at the +x,-y corner
    s=np.where(np.abs(x-H)<np.abs(y+H), -(y+H), H-x)  # +x side: s<0 before the corner; -y side: s>0 after
    m&=np.abs(s)<300
    ax[0].plot(s[m],e[m],'.',ms=1.5,label=fn.replace('.json',''))
    ax[1].plot(x[m]+50*e[m]*(np.abs(x[m]-H)<np.abs(y[m]+H)), y[m]-50*e[m]*(np.abs(x[m]-H)>=np.abs(y[m]+H)),'.',ms=1,label=fn)
    tm=t[m]; k=(t>=tm.min()-0.5)&(t<=tm.max())
    for i,a in enumerate(d['axes'][:3]): ax[2].plot(t[k],se[k,i],lw=.7,label=f'{fn[:14]} {a}')
ax[0].set_xlabel('arc distance from +x,-y corner (mm; <0 = +x side)'); ax[0].set_ylabel('contour error mm (+out)'); ax[0].legend(markerscale=8)
ax[1].set_aspect('equal'); ax[1].set_title(f'tip near +x,-y corner, square {H:.0f}, contour error x50')
ax[2].set_ylabel('servo error rad'); ax[2].set_xlabel('t s'); ax[2].legend(fontsize=7)
fig.tight_layout(); fig.savefig(f'corner_{int(H)}.png',dpi=95)
