from pathlib import Path
import json,hashlib,tarfile,math,re,sys
archive=Path(sys.argv[1]);receipt=json.loads(Path(sys.argv[2]).read_text());sha=lambda b:hashlib.sha256(b).hexdigest();assert sha(archive.read_bytes())==receipt['gzip_sha256'];expected={f['path']:f for f in receipt['files']};P={}
with tarfile.open(archive,'r:gz') as t:
 for m in t:
  b=t.extractfile(m).read();f=expected.pop(m.name);assert len(b)==f['bytes'] and sha(b)==f['sha256'];P[m.name]=b
assert not expected
read=lambda n,default=None:json.loads(P[n]) if n in P else default
M=read('evidence/measurements.json');truth=[[0,-1900,-280],[1645.448267190434,950,-280],[-1645.448267190434,950,-280],[0,0,1900]]
def metrics(a):
 if isinstance(a,dict):a=a.get('value')
 if not isinstance(a,list) or len(a)!=4:return None
 e=[math.dist(x[:3],y) for x,y in zip(a,truth)];return dict(per_anchor_error_mm=dict(zip('ABCD',e)),max_mm=max(e),rmse_mm=math.sqrt(sum(x*x for x in e)/4),all_below5=all(x<5 for x in e),anchors_over20=sum(x>20 for x in e))
def equal(a,b):
 assert (a is None)==(b is None)
 if a:
  for k in ['max_mm','rmse_mm']:assert abs(a[k]-b[k])<1e-9
  for k in 'ABCD':assert abs(a['per_anchor_error_mm'][k]-b['per_anchor_error_mm'][k])<1e-9
  for k in ['all_below5','anchors_over20']:assert a[k]==b[k]
rows=[];sourcehash=[]
for n,x in M.items():
 prefix=f'runs/{n}/';h=read(prefix+'sweeps.headless/manifest.json');inv=read(prefix+'invocation.json');freeze=read(prefix+'policy-frozen.json');assert h['total_steps']==x['total_steps'];assert abs(x['simulated_s']-x['total_steps']*.002)<1e-8
 points=[json.loads(line)['point'] for k,b in P.items() if k.startswith(prefix) and k.endswith('.partial-points.jsonl') for line in b.splitlines()];assert len(points)==x['physical_collected_points'];assert h['physics_engine']=='rust-native' and h['settings']['enableLayering'] is False
 if n=='invalid-path-startup':
  assert not read(prefix+'sweeps.headless/motor-selection.json') and not points and x['fit_count']==0 and x['total_steps']==71980
  rows.append({'run':n,'excluded_from_policy_inference':True,'steps':x['total_steps']});continue
 command=next((v for v in h.get('applied_parameters',[]) if v.startswith('M669')),None);anchors=[]
 if command:
  for axis in 'ABCD':
   hit=re.search(r'\b'+axis+r'([-+\d.eE]+):([-+\d.eE]+):([-+\d.eE]+)',command);assert hit;anchors.append(list(map(float,hit.groups())))
 equal(metrics(anchors),x['applied']);reply=h.get('firmware_final',{}).get('M669','');anchors=[]
 for axis in 'ABCD':
  hit=re.search(r'\b'+axis+r':\s*([-+\d.eE]+),\s*([-+\d.eE]+),\s*([-+\d.eE]+)',reply)
  if hit:anchors.append(list(map(float,hit.groups())))
 equal(metrics(anchors),x['readback_applied']);fits=[k for k in P if k.startswith(prefix+'stages/fit-') and k.endswith('.json')];assert len(fits)==x['fit_count'];saved={r['artifact']:r for r in x['iterations']}
 for f in fits:equal(metrics(read(f)['payload']['fit'].get('anchors')),saved[f]['errors'])
 assert sum(p.get('step_index')==p.get('step_count',0)-1 for p in points)==x['bidirectional_physical_sub_sweeps']
 assert len(read(prefix+'sweeps.json',{}).get('sweeps',[]))==x['dataset_sweep_pairs']
 assert h['physics_artifact']['sha256']=='66a8451bc1a42310e4dfae166288737e595f75bc1173c8f88e92163afb94a5d8'
 hashes=freeze.get('continuation_hashes',freeze['hashes'])
 for filename,value in inv['policy_files'].items():assert sha(P[prefix+'collector-policy/'+filename])==value
 assert sha(P[prefix+'service-snapshot.mjs'])==inv['script_sha256']['autocal_headless.mjs'];assert sha(P[prefix+'runner-snapshot.py'])==inv['script_sha256']['run.py']
 for key,value in hashes.items():
  if key.startswith('harness/') and '__pycache__' not in key:assert sha(P[prefix+'harness-source/'+key[len('harness/'):]])==value,key
 intervention=read(prefix+'sweeps.headless/intervention-native.json');assert len(intervention['selected'])==4;motors={m['entity']:m for m in intervention['motors']}
 for row in intervention['selected']:
  for k,v in row['selected'].items():assert abs(motors[row['entity']][k]-v)<1e-14
 if n.startswith('e-'):
  expect={'SpoolA':(.0005,.00225),'SpoolB':(.0015,.003),'SpoolC':(.002,.00075),'SpoolD':(.00075,.0015)}
  for m in motors.values():
   a,b=expect[m['name']];assert abs(m['coulombFriction']-a)<1e-14 and abs(m['stictionTorque']-b)<1e-14
 for row in intervention['rows']:
  if row.get('name','').startswith('Anchor'):assert row['mu_native']==0
  if row.get('name','').startswith('Spool'):assert row['radius_native']==.03
 sourcehash.append(h['autocal_source_sha256']);rows.append(dict(run=n,points=len(points),fits=len(fits),steps=x['total_steps'],applied_recomputed=x['applied'],readback_recomputed=x['readback_applied']))
assert len(set(sourcehash))==1
ledger=read('budget-ledger.json');assert ledger['completed_or_failed_collections']==7;assert sum(x['total_steps'] for x in M.values())+3003+ledger.get('verification_physics_steps',0)==ledger['physics_steps'];assert sum(x['fit_count'] for x in M.values())==ledger['fresh_fits'];assert sum(x['physical_collected_points'] for x in M.values())==ledger['physical_points']
for k in ['collections','fresh_fits','physics_steps','physical_points']:
 actual=ledger['completed_or_failed_collections'] if k=='collections' else ledger[k];assert actual<=ledger['limit'][k]
print(json.dumps(dict(passed=True,kind='offline fixed evidence hash and numerical recomputation',files_verified=len(P),runs=rows,budget=ledger,decision=read('evidence/decision.json')),indent=2))
