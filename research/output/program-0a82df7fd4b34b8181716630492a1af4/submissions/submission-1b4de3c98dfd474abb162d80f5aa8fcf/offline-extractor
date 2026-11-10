"""Recompute fixed numerical claims offline from the compact evidence archive."""
from pathlib import Path
import argparse,hashlib,json,math,re,tarfile,fnmatch
p=argparse.ArgumentParser();p.add_argument('--archive',type=Path,required=True);p.add_argument('--manifest',type=Path,required=True);a=p.parse_args();receipt=json.loads(a.manifest.read_text())
sha=lambda b:hashlib.sha256(b).hexdigest();assert sha(a.archive.read_bytes())==receipt['gzip_sha256'];expected={x['path']:x for x in receipt['files']};payload={}
with tarfile.open(a.archive,'r:gz') as t:
 for m in t:
  b=t.extractfile(m).read();f=expected.pop(m.name);assert len(b)==f['bytes'] and sha(b)==f['sha256'],m.name;payload[m.name]=b
assert not expected and all(Path(n).name!='events.jsonl' for n in payload)
def read(n,default=None):return json.loads(payload[n]) if n in payload else default
M=read('evidence/measurements.json');freeze=read('policy-frozen.json');truth=[[0,-1900,-280],[1645.448267190434,950,-280],[-1645.448267190434,950,-280],[0,0,1900]]
def metrics(anchors):
 if isinstance(anchors,dict):anchors=anchors.get('value')
 if not isinstance(anchors,list) or len(anchors)!=4:return None
 e=[math.dist(x[:3],y) for x,y in zip(anchors,truth)]
 return {'per_anchor_error_mm':dict(zip('ABCD',e)),'max_mm':max(e),'rmse_mm':math.sqrt(sum(x*x for x in e)/4),'all_below5':all(x<5 for x in e),'anchors_over20':sum(x>20 for x in e)}
def eq(m,s):
 assert (m is None)==(s is None)
 if m is None:return
 for k in ['max_mm','rmse_mm']:assert abs(m[k]-s[k])<1e-9,(k,m,s)
 for k in 'ABCD':assert abs(m['per_anchor_error_mm'][k]-s['per_anchor_error_mm'][k])<1e-9
 for k in ['all_below5','anchors_over20']:assert m[k]==s[k]
rows=[];sourcehash=[]
for name,x in M.items():
 prefix=f'runs/{name}/';h=read(prefix+'sweeps.headless/manifest.json');inv=read(prefix+'invocation.json');command=next((v for v in h.get('applied_parameters',[]) if v.startswith('M669')),None);aa=[]
 if command:
  for axis in 'ABCD':
   hit=re.search(r'\b'+axis+r'([-+\d.eE]+):([-+\d.eE]+):([-+\d.eE]+)',command);assert hit;aa.append(list(map(float,hit.groups())))
 applied=metrics(aa);eq(applied,x['applied']);rr=[];reply=h.get('firmware_final',{}).get('M669','')
 for axis in 'ABCD':
  hit=re.search(r'\b'+axis+r':\s*([-+\d.eE]+),\s*([-+\d.eE]+),\s*([-+\d.eE]+)',reply)
  if hit:rr.append(list(map(float,hit.groups())))
 eq(metrics(rr),x['readback_applied'])
 if applied:assert max(abs(v-w) for u,z in zip(aa,rr) for v,w in zip(u,z))<=.00500001 and x['readback_agrees_at_print_precision']
 fits=sorted(n for n in payload if n.startswith(prefix+'stages/fit-') and n.endswith('.json'));assert len(fits)==x['fit_count'];saved={r['artifact']:r for r in x['iterations']};tails=[]
 for n in fits:
  fit=read(n)['payload']['fit'];m=metrics(fit.get('anchors'));eq(m,saved[n]['errors'])
  if m:tails.append(m['max_mm'])
 assert (max(tails) if tails else None)==x['iteration_tail_max_mm']
 completed=sum(json.loads(line)['point'].get('step_index')==json.loads(line)['point'].get('step_count',0)-1 for n in payload if n.startswith(prefix) and n.endswith('.partial-points.jsonl') for line in payload[n].splitlines());assert completed==x['bidirectional_physical_sub_sweeps']
 points=sum(len(payload[n].splitlines()) for n in payload if n.startswith(prefix) and n.endswith('.partial-points.jsonl'));assert points==x['physical_collected_points']
 sweeps=read(prefix+'sweeps.json',{}).get('sweeps',[]);assert len(sweeps)==x['dataset_sweep_pairs'];assert sum(len(w.get('metadata',{}).get('direction_forces',[])) for w in sweeps)==x['dataset_direction_entries']
 assert h['total_steps']==x['total_steps'] and abs(h['total_steps']*.002-x['simulated_s'])<1e-8
 assert h['physics_engine']=='rust-native' and h['settings']['enableLayering'] is False
 assert h['physics_artifact']['sha256']==freeze['upstream_hashes']['src/wasm/cable_solver/target/native/physics.node']
 assert h['normal_completion']==x['normal_automatic_acceptance']
 for key,value in freeze['hashes'].items():
  n=prefix+'harness-source/'+str(Path(key).relative_to('harness'));assert sha(payload[n])==value,n
 policy='candidate' if inv['policy']=='sensor-4mid-v1' else 'default'
 for filename,value in inv['policy_files'].items():
  assert sha(payload[prefix+'collector-policy/'+filename])==value==freeze['hashes']['harness/'+policy+'/'+filename]
 assert sha(payload[prefix+'service-snapshot.mjs'])==freeze['hashes']['harness/autocal_headless.mjs']
 assert sha(payload[prefix+'runner-snapshot.py'])==freeze['hashes']['harness/run.py']
 intervention=read(prefix+'sweeps.headless/intervention-native.json');motors={r['entity']:r for r in intervention['motors']}
 for r in intervention['selected']:
  for k,v in r['selected'].items():assert abs(motors[r['entity']][k]-v)<1e-14
 for n in ['initial-native.json','terminal-native.json']:assert prefix+'sweeps.headless/'+n in payload
 sourcehash.append(h.get('autocal_source_sha256'));rows.append({'run':name,'applied_recomputed':applied,'fits':len(fits),'points':points,'steps':h['total_steps'],'readback_agrees':x['readback_agrees_at_print_precision'],'normal_completion':h['normal_completion']})
assert len(set(sourcehash))==1,'Implementation source hash differs across primary runs'
result={'kind':'offline fixed-evidence recomputation','passed':True,'compact_files_verified':len(payload),'omitted_duplicate_event_journals':receipt['compact_omissions'],'full_raw_archives_retained':receipt['raw_archive_references'],'runs':rows,'decision':read('evidence/decision.json'),'scope':'Numerical/hash/readback checks only. Successful checks do not establish hardware transfer or accept production adoption.'};print(json.dumps(result,indent=2))
