import json,sys,numpy as np
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from contour import load,contour
fn=sys.argv[1]; t0,t1=(float(sys.argv[2]),float(sys.argv[3])) if len(sys.argv)>3 else (0,1e9)
d,tip=load(fn); h,e=contour(tip); t=np.array(d['t']); m=(t>=t0)&(t<=t1)
pos=np.array(d['pos'])*1000; T=np.array(d['tension']); ph=np.array(d['phase']); se=np.array(d['servoErr'],dtype=float)
fig,ax=plt.subplots(5,1,figsize=(13,13),sharex=True)
ax[0].plot(t[m],e[m],lw=.7); ax[0].set_ylabel('contour err mm')
for i in range(3): ax[1].plot(t[m],pos[m,i]-(0 if i<2 else 0),lw=.7,label='xyz'[i])
ax[1].legend(); ax[1].set_ylabel('effector pos mm')
for i in range(T.shape[1]): ax[2].plot(t[m],T[m,i],lw=.7,label=f'path{i}')
ax[2].legend(); ax[2].set_ylabel('end-span force N')
for i,a in enumerate(d['axes']): ax[3].plot(t[m],ph[m,i],lw=.7,label=a)
ax[3].legend(); ax[3].set_ylabel('encoder-commanded rad')
for i,a in enumerate(d['axes']): ax[4].plot(t[m],se[m,i],lw=.7,label=a)
ax[4].legend(); ax[4].set_ylabel('servoError')
fig.tight_layout(); fig.savefig(sys.argv[4] if len(sys.argv)>4 else 'timeline.png',dpi=90)
