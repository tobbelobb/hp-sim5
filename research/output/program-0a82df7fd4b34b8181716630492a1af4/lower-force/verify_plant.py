from pathlib import Path
import json,sys,hashlib
S=Path(__file__).resolve().parent;name=sys.argv[1];r=S/'runs'/name;h=r/'sweeps.headless';d=json.loads((h/'intervention-native.json').read_text());inv=json.loads((r/'invocation.json').read_text());expected={'SpoolA':(.0005,.00225),'SpoolB':(.0015,.003),'SpoolC':(.002,.00075),'SpoolD':(.00075,.0015)}
assert len(d['selected'])==4
for m in d['motors']:
 a,b=expected[m['name']];assert abs(m['coulombFriction']-a)<1e-14 and abs(m['stictionTorque']-b)<1e-14
for row in d['selected']:
 native=next(m for m in d['motors'] if m['entity']==row['entity'])
 for key,val in row['selected'].items():assert abs(native[key]-val)<1e-14
for x in d['rows']:
 if x.get('name','').startswith('Spool'):assert abs(x['radius_native']-.03)<1e-12
 if x.get('name','').startswith('Anchor'):assert x['mu_native']==0 and x.get('mu_ecs',0)==0
state=json.loads((h/'clock.json').read_text());assert state['settings']['enableLayering'] is False
assert state['physics_artifact']['sha256']=='66a8451bc1a42310e4dfae166288737e595f75bc1173c8f88e92163afb94a5d8'
assert inv['world']['sigma_deg']==.05 and inv['world']['noise_seed']==4201
assert '--buildup-factor' in inv['argv'] and inv['argv'][inv['argv'].index('--buildup-factor')+1]=='0'
receipt={'run':name,'passed':True,'motor_expected':expected,'native_sha256':state['physics_artifact']['sha256'],'radii_mm':[30]*4,'guide_mu':0,'layering':False,'buildup':0,'encoder_sigma_deg':.05,'seed':4201,'ecs_guide_note':'No coefficient component means the stock zero coefficient; packed native value verified zero.'}
(S/'evidence'/f'{name}-plant-check.json').write_text(json.dumps(receipt,indent=2)+'\n');print(json.dumps(receipt))
