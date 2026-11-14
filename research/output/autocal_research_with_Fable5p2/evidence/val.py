import json,sys,pickle,numpy as np
np.set_printoptions(precision=3,suppress=True,linewidth=250)
sys.path.insert(0,sys.argv[0].rsplit('/',1)[0]); from elastic import *
dirn=sys.argv[1]; EA=float(sys.argv[2])
d=json.load(open(dirn+'/sweeps.json')); k=np.array(d['config']['mm_per_degree'])
out,_=pickle.load(open(dirn+'/rrd.pkl','rb'))
t,pos=out[('/world/machines/default/bodies/Effector_16','translation')]
grp=[['AL_35','AR_36'],['BL_37','BR_38'],['CL_39','CR_40'],['D0_41','D1_42','D2_43']]
err=[];errF=[]
for s in d['sweeps'][:3]:
    for pi,p in enumerate(s['data_points']):
        i=min(np.searchsorted(t,p['timestamp_ms']/1000),len(t)-1)
        pp=pos[i]*1000-np.array([0,0,100.])
        F=np.array([sum(out[('/cable_forces/default/CablePath'+c,'scalars')][1][i][0] for c in g) for g in grp])
        sen=s['sensor_anchor'] if pi<len(s['data_points'])//2 else s['drive_anchor']
        enc=np.array(p['mu'])*k
        e1,T1,_=encoders(pp,sen,p['sensor_force_n'],EA)      # commanded sensor force
        e2,T2,_=encoders(pp,sen,F[sen],EA)                   # realised sensor force
        err.append(e1-enc); errF.append(e2-enc)
err=np.array(err);errF=np.array(errF)
print(dirn.rsplit('/',1)[-1],'EA',EA,'model-vs-recorded encoder: commanded-force rms per anchor',np.sqrt((err**2).mean(0)),'max',np.abs(err).max(), '| realised-force rms',np.sqrt((errF**2).mean(0)),'max',np.abs(errF).max())
