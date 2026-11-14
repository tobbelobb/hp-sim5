import json,sys,pickle,numpy as np
np.set_printoptions(precision=3,suppress=True,linewidth=250)
sys.path.insert(0,sys.argv[0].rsplit('/',1)[0]); from fit import TRUE
d=json.load(open(sys.argv[1]+'/sweeps.json')); k=np.array(d['config']['mm_per_degree'])
out,_=pickle.load(open(sys.argv[1]+'/rrd.pkl','rb'))
t,pos=out[('/world/machines/default/bodies/Effector_16','translation')]
grp=[['AL_35','AR_36'],['BL_37','BR_38'],['CL_39','CR_40'],['D0_41','D1_42','D2_43']]
Y=np.array([528.3,528.3,528.3,398.9]); n=np.array([2,2,2,3]); L0=np.linalg.norm(TRUE,axis=1)
rows=[]
for si,s in enumerate(d['sweeps'][:3]):
    for pi,p in enumerate(s['data_points']):
        i=min(np.searchsorted(t,p['timestamp_ms']/1000),len(t)-1)
        pp=pos[i]*1000-np.array([0,0,100.])
        F=np.array([sum(out[('/cable_forces/default/CablePath'+c,'scalars')][1][i][0] for c in g) for g in grp])
        sen=s['sensor_anchor'] if pi<len(s['data_points'])//2 else s['drive_anchor']; u=(TRUE-pp); Lg=np.linalg.norm(u,axis=1); u/=Lg[:,None]
        idx=[a for a in range(4) if a!=sen]
        Ts=p['sensor_force_n']; rhs=-(Ts*u[sen]+np.array([0,0,-0.081*9.82]))
        T=np.zeros(4); T[idx]=np.linalg.solve(u[idx].T,rhs); T[sen]=Ts
        enc=np.array(p['mu'])*k; geom=Lg-L0
        rows.append((si,sen,F,T,geom-enc,(Y+Lg)/n))
F=np.array([r[2] for r in rows]);T=np.array([r[3] for r in rows]);e=np.array([r[4] for r in rows]);c=np.array([r[5] for r in rows])
print('recorded F vs statics T: max abs diff per anchor',np.abs(F-T).max(0),' sample',F[0],T[0])
ref=np.array([0,0,0,0.081*9.82])
for a in range(4):
    x=(T[:,a]-ref[a])*c[:,a]   # N*mm : stretch = x/EA
    A=np.c_[x,np.ones(len(x))]; (slope,icpt),*_=np.linalg.lstsq(A,e[:,a],rcond=None)
    print('anchor',a,'effective EA per line = %.0f N'%(1/slope),'intercept %.3f mm'%icpt,'fit rms %.4f mm'%np.sqrt(((A@[slope,icpt]-e[:,a])**2).mean()), 'range of geom-enc: %.3f..%.3f'%(e[:,a].min(),e[:,a].max()))
print('rows where |F-T|>0.5:')
for r,(si,sen,Fr,Tr,er,cr) in enumerate(rows):
    if np.abs(Fr-Tr).max()>0.5: print(r,si,'sen',sen,'F',Fr,'T',Tr)
