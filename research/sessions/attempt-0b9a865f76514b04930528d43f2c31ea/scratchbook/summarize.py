from pathlib import Path
import json,hashlib,subprocess
s=Path('research/sessions/attempt-0b9a865f76514b04930528d43f2c31ea'); e=s/'evidence'
def load(p): return json.loads(p.read_text()) if p.exists() else None
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
summary={}; omitted=[]
for label in ['old','current','old20','current20','old20corrected']:
 d=e/label
 if not (d/'sweeps.headless/manifest.json').exists(): continue
 m=load(d/'sweeps.headless/manifest.json');c=load(d/'sweeps.headless/clock.json');data=load(d/'sweeps.json')
 bundle={'command':load(d/'command.json'),'manifest':m,'clock':c,'console':(d/'console.log').read_text(),'full_auto_log':(d/'sweeps.full_auto.log').read_text() if (d/'sweeps.full_auto.log').exists() else None,'iterations':[json.loads(line) for line in (d/'sweeps.full_auto_log.jsonl').read_text().splitlines()] if (d/'sweeps.full_auto_log.jsonl').exists() else [],'stages':{p.name:load(p) for p in sorted((d/'sweeps.stages').glob('*.json'))},'force_tuning':load(d/'sweeps.json.force-tuning.json')}
 (e/f'{label}-raw.json').write_text(json.dumps(bundle,indent=2)+'\n')
 sw=data['sweeps'] if data else []
 summary[label]={'status':m['status'],'applied_parameters':m.get('applied_parameters'),'applied_anchor_errors_mm':None,'applied_rmse_mm':None,'max_applied_error_mm':None,'steps':c['total_steps'] if c else None,'simulated_seconds':c['simulated_s'] if c else None,'sweeps':len(sw),'points':sum(len(x['data_points']) for x in sw),'per_sweep_points':[len(x['data_points']) for x in sw],'sweeps_sha256':hashlib.sha256(json.dumps(sw,sort_keys=True,separators=(',',':')).encode()).hexdigest(),'dataset_sha256':sha(d/'sweeps.json') if data else None}
 truth=[[0,-1900,-280],[1645.448267190434,950,-280],[-1645.448267190434,950,-280],[0,0,1900]]
 fit=bundle['stages'].get('fit-001-default.json',{}).get('payload',{}).get('fit',{})
 anchors=fit.get('anchors',{}).get('value')
 if anchors:
  import math
  errors=[math.sqrt(sum((x-y)**2 for x,y in zip(a,t))) for a,t in zip(anchors,truth)]
  summary[label]['rejected_candidate_nominal_frame_errors_mm']=errors
  summary[label]['rejected_candidate_nominal_frame_rmse_mm']=math.sqrt(sum(x*x for x in errors)/4)
  summary[label]['rejected_candidate_nominal_frame_max_error_mm']=max(errors)
 for p in d.rglob('*'):
  if p.is_file(): omitted.append({'path':str(p.relative_to(s)),'bytes':p.stat().st_size,'sha256':sha(p),'retention':'raw bundle and committed dataset retain small numerical evidence; original workspace retains journal/recording and service files'})
for a,b in [('old','current'),('old20','current20'),('old20corrected','current20')]:
 if a in summary and b in summary: summary[f'{a}/{b}']={'sweeps_identical':summary[a]['sweeps_sha256']==summary[b]['sweeps_sha256']}
for p in (e/'native').glob('*.node'):omitted.append({'path':str(p.relative_to(s)),'bytes':p.stat().st_size,'sha256':sha(p),'retention':'workspace only; rebuild from committed recipe and exact engine source revision'})
(e/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');(e/'artifact-manifest.json').write_text(json.dumps(omitted,indent=2)+'\n');print(json.dumps(summary,indent=2))
