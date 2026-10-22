import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT=Path(__file__).resolve().parent
runs={}
for file in sorted(ROOT.glob('E[0-9][0-9].json')):
    result=json.loads(file.read_text())
    runs[result['id']]=result

summaries=[]
for name,d in runs.items():
    final=d['rows'][-1]
    before=min(d['rows'],key=lambda r:abs(r['t']-20))
    max_abc=max(abs(w['angle']) for w in final['wheels'] if w['name'][5] in 'ABC')
    max_d=max(abs(w['angle']) for w in final['wheels'] if w['name'][5]=='D')
    summaries.append(dict(id=name,variant=d['variant'],backend=d['backend'],substeps=d['substeps'],mode=d['mode'],
        wall_s=d['wall_s'],seconds=d['seconds'],final_xyz_mm=(np.array(final['xyz'])*1000).tolist(),
        at10_xyz_mm=(np.array(min(d['rows'],key=lambda r:abs(r['t']-10))['xyz'])*1000).tolist(),
        max_abc_angle_rad=max_abc,max_d_angle_rad=max_d,
        max_angle_change_20_to_end_rad=max(abs(a['angle']-b['angle']) for a,b in zip(final['wheels'],before['wheels'])),
        constraints=sum(len(p['joints']) for p in final['paths']),
        max_total_material_error_m=max(abs(p['total']-sum(p['stored'])-sum(j['rest'] for j in p['joints'])) for p in final['paths']),
        artifact_sha256=(d.get('artifact') or {}).get('sha256')))
(ROOT/'motion-summary.json').write_text(json.dumps(summaries,indent=2)+'\n')

trace=[]
for name in ['E27','E28']:
    if name not in runs:continue
    d=runs[name]
    rows=d.get('solverTrace',{}).get('rows',[])
    for path in d['rows'][-1]['paths']:
        for j in path['joints']:
            matching=[r for r in rows if d['entityNames'][str(r['a'])]==j['a'] and d['entityNames'][str(r['b'])]==j['b']]
            if matching:
                last=matching[-1]
                trace.append(dict(run=name,path=path['name'],joint=j['name'],first_visit_force_N=j['force'],
                    accumulated_force_N=abs(last['previous']+last['delta'])/d['dt']**2,
                    trace=matching))
(ROOT/'multiplier-audit.json').write_text(json.dumps(trace,indent=2)+'\n')

pulley=json.loads((ROOT/'E16-pulley-convergence.json').read_text())
energy=[]
for d in pulley:
    final=d['samples'][-1]
    elastic=sum(t*t/(2*d['k']) for t in final['tensions'])
    kinetic=.5*d['I']*final['omega']**2
    energy.append(dict(dt=d['dt'],redistribution=d['redistribution'],initial_energy_J=.5*d['k']*.001**2,
        final_elastic_J=elastic,final_kinetic_J=kinetic,final_total_J=elastic+kinetic,
        angle=final['angle'],expected_angle=d['expected_abs_angle']))
(ROOT/'pulley-energy.json').write_text(json.dumps(energy,indent=2)+'\n')

fig,axes=plt.subplots(1,3,figsize=(15,4.2))
for name,label in [('E02','Original, one substep'),('E03','Original, four substeps'),('E18','No slip, one substep')]:
    if name not in runs:continue
    d=runs[name]
    axes[0].plot([r['t'] for r in d['rows']],[max(abs(w['angle']) for w in r['wheels']) for r in d['rows']],label=label)
axes[0].set_yscale('symlog',linthresh=.02)
axes[0].set(xlabel='Simulated time (s)',ylabel='Maximum wheel angle (rad)',title='Held motors: passive wheel drift')
for name,label in [('E04','Original / 1'),('E05','Original / 4'),('E19','No slip / 1'),('E20','No slip / 4'),('E26','No slip / 16')]:
    if name not in runs:continue
    d=runs[name]
    axes[1].plot([r['t'] for r in d['rows']],[1000*r['xyz'][1] for r in d['rows']],label=label)
axes[1].set(xlabel='Simulated time (s)',ylabel='Y (mm)',title='Controlled pull: timestep dependence')
for slip,color,label in [(True,'#b45a3c','Original rolling redistribution'),(False,'#247b73','Rotation carries cable')]:
    points=[d for d in energy if d['redistribution']==slip]
    axes[2].plot([d['dt']*1000 for d in points],[d['final_total_J']/d['initial_energy_J'] for d in points],'o-',color=color,label=label)
axes[2].axhline(1,color='black',linewidth=.8,linestyle='--',label='Initial energy / no external work')
axes[2].set(xlabel='Pulley timestep (ms)',ylabel='Energy at 1 s / initial energy',title='Isolated pulley: energy check')
for ax in axes:
    ax.grid(alpha=.2);ax.legend(fontsize=8)
fig.tight_layout()
fig.savefig(ROOT/'mechanics-summary.png',dpi=160)
fig.savefig(ROOT/'mechanics-summary.svg')

fig,ax=plt.subplots(figsize=(8,4))
ax.plot([-2,0,0,-2],[.7,.7,-.7,-.7],color='#287b8c',lw=2)
ax.plot([0,.45,.45,0],[.7,.5,-.5,-.7],color='#999999',lw=2,ls='--')
ax.scatter([0,0],[.7,-.7],s=65,color='#287b8c',zorder=3)
ax.annotate('entry port',(-.05,.7),(-1.1,1.0),arrowprops={'arrowstyle':'-'})
ax.annotate('exit port',(-.05,-.7),(-1.1,-1.1),arrowprops={'arrowstyle':'-'})
ax.text(.7,.05,'Fixed hidden cable length\n105.45 mm per ABC branch\n\nPort separation: 59.5 mm',va='center')
ax.text(-2.1,.9,'from anchor',fontsize=10);ax.text(-2.1,-1.0,'back to anchor',fontsize=10)
ax.text(-.3,1.4,'Two-port pinhole gate on one rigid effector',ha='center',fontsize=13)
ax.set(xlim=(-2.5,3.1),ylim=(-1.5,1.7));ax.axis('off');fig.tight_layout()
fig.savefig(ROOT/'gate-schematic.svg');fig.savefig(ROOT/'gate-schematic.png',dpi=150)
print(json.dumps(summaries,indent=2))
