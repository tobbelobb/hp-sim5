import pickle,json,numpy as np,sys
np.set_printoptions(precision=4,suppress=True,linewidth=250)
out,names=pickle.load(open(sys.argv[1]+'/rrd.pkl','rb'))
d=json.load(open(sys.argv[1]+'/sweeps.json'))
g=lambda ep,c='scalars':out[(ep,c)]
E='/world/machines/default/bodies/Effector_16'
t,pos=g(E,'translation'); _,q=g(E,'quaternion')
cab=['AL_35','AR_36','BL_37','BR_38','CL_39','CR_40','D0_41','D1_42','D2_43']
def at(tt):
    i=np.searchsorted(t,tt); i=min(i,len(t)-1)
    row={'t':t[i],'pos':pos[i]*1000,'q':q[i]}
    for c in cab:
        row[c]=dict(len=g('/line_lengths/default/CablePath'+c)[1][i]*1000,err=g('/line_errors/default/CablePath'+c)[1][i]*1000,F=g('/cable_forces/default/CablePath'+c)[1][i])
    return row
for tt in (0,1,5,20):
    r=at(tt); print('t',r['t'],'pos',r['pos'],'q',r['q'])
    for c in cab: print('  ',c,'len[cmd,actual,geom]',r[c]['len'],'err/stretch',r[c]['err'],'F',r[c]['F'])
pts=[(si,p) for si,s in enumerate(d['sweeps']) for p in s['data_points']]
print('timestamps', [round(p['timestamp_ms']/1000,1) for _,p in pts[:12]])
print()
TRUE=np.array([[0,-1900,-280],[1645.44826719,950,-280],[-1645.44826719,950,-280],[0,0,1900.]])
k=np.array(d['config']['mm_per_degree'])
L0=np.linalg.norm(TRUE,axis=1)
grp=[['AL_35','AR_36'],['BL_37','BR_38'],['CL_39','CR_40'],['D0_41','D1_42','D2_43']]
r0=at(5.0)
print('si drv sen | true pos (mm, rel home) | tilt mrad | enc dl | true geom dl (from pos) | enc-geom | sum F per anchor | mean stretch per anchor')
for si,p in pts:
    s=d['sweeps'][si]
    r=at(p['timestamp_ms']/1000)
    pp=r['pos']-np.array([0,0,100.])
    dl=np.array(p['mu'])*k
    geom=np.linalg.norm(TRUE-pp,axis=1)-L0
    F=[sum(r[c]['F'][0] for c in gg) for gg in grp]
    st=[np.mean([r[c]['err'][1] for c in gg]) for gg in grp]
    qq=r['q']; tilt=2000*np.linalg.norm(qq[:3])
    print(si,s['drive_anchor'],s['sensor_anchor'],pp,'%.2f'%tilt,dl,geom,dl-geom,np.array(F),np.array(st))
