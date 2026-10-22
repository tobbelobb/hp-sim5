from pathlib import Path
import sys,json
import numpy as np
from rerun.chunk import RrdReader
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(sys.argv[1]);p=next((D/'world.headless').glob('*.rrd'));r=RrdReader(p)
poses={}
for c in r.stream().filter(content=('/world/machines/default/bodies/Effector_16' if json.loads((D/'result.json').read_text())['machine']=='hp3' else '/world/machines/default/bodies/Effector_58')).to_chunks():
 b=c.to_record_batch()
 if 'Transform3D:translation'not in b.schema.names:continue
 for step,t,pos,q in zip(b.column('sim_step').to_pylist(),b.column('sim_time').cast('int64').to_pylist(),b.column('Transform3D:translation').to_pylist(),b.column('Transform3D:quaternion').to_pylist()):
  poses[step]=(t/1e9,pos[0],q[0])
a=list(v for _,v in sorted(poses.items()));t=np.array([v[0]for v in a]);x=np.array([v[1]for v in a]);q=np.array([v[2]for v in a])
q=q/np.linalg.norm(q,axis=1,keepdims=True)
phases=json.loads((D/'result.json').read_text())['phases'];windows=[]
for i,p0 in enumerate(phases):
 if p0.get('message')=='start':
  p1=next(v for v in phases[i+1:]if v['phase']==p0['phase']);windows.append((p0['phase'],p0['physicsMs']/1000,p1['physicsMs']/1000))
last=phases[-1]['physicsMs']/1000;windows.append(('held-observation',last,last+15))
metrics=[]
for name,start,end in windows:
 for tail in [None,10,5]:
  lo=start if tail is None else max(start,end-tail);m=(t>=lo)&(t<=end);xx=x[m];qq=q[m]
  if len(xx)<2:continue
  arc=np.linalg.norm(np.diff(xx,axis=0),axis=1).sum()*1000;chord=np.linalg.norm(xx[-1]-xx[0])*1000
  angular=np.degrees(2*np.arccos(np.clip(np.abs(np.sum(qq[1:]*qq[:-1],axis=1)),0,1))).sum()
  metrics.append(dict(phase=name,tailS=tail,startS=lo,endS=end,pathMm=float(arc),chordMm=float(chord),bboxDiagonalMm=float(np.linalg.norm(np.ptp(xx,axis=0))*1000),rotationPathDeg=float(angular)))
series={}
for content in ['/line_errors/**','/line_lengths/**','/cable_forces/**']:
 for c in r.stream().filter(content=content,components='Scalars:scalars').to_chunks():
  b=c.to_record_batch();path=b.schema.metadata[b'rerun:entity_path'].decode()
  for tt,val in zip(b.column('sim_time').cast('int64').to_pylist(),b.column('Scalars:scalars').to_pylist()):series.setdefault(path,[]).append([tt/1e9,val])
result={'rrd':str(p),'kind':'sampled actual production Rust Native physical telemetry; not hardware','metrics':metrics}
(D/'physical-metrics.json').write_text(json.dumps(result,indent=2));(D/'physical-series.json').write_text(json.dumps(dict(time=t.tolist(),position=x.tolist(),quaternion=q.tolist(),cableSeries=series)))
fig=plt.figure(figsize=(13,9));ax=fig.add_subplot(221,projection='3d');ax.plot(*x.T,linewidth=.8);ax.set(xlabel='X (m)',ylabel='Y (m)',zlabel='Z (m)',title='Actual effector trajectory')
ax2=fig.add_subplot(222)
for j,name in enumerate('XYZ'):ax2.plot(t,x[:,j]*1000,label=name)
ax2.set(xlabel='Physical time (s)',ylabel='Position (mm)',title='Effector position');ax2.legend()
ax3=fig.add_subplot(223);speed=np.linalg.norm(np.diff(x,axis=0),axis=1)/np.diff(t)*1000;ax3.plot(t[1:],speed,linewidth=.7);ax3.set(xlabel='Physical time (s)',ylabel='mm/s',title='Body speed (sampled)')
ax4=fig.add_subplot(224)
for path,v in series.items():
 if '/line_errors/'in path:
  v.sort();ax4.plot([z[0]for z in v],[z[1][-1]*1000 for z in v],linewidth=.6,label=path.split('/')[-1])
ax4.set(xlabel='Physical time (s)',ylabel='mm',title='Cable geometric minus actual length (negative = slack)')
for name,start,end in windows:
 for axx in [ax2,ax3,ax4]:axx.axvspan(start,end,alpha=.08,label=name)
 ax2.text(start,ax2.get_ylim()[1],name,rotation=90,fontsize=7,va='top')
fig.tight_layout();fig.savefig(D/'physical-smoke.png',dpi=150)
print(json.dumps(metrics,indent=2))
