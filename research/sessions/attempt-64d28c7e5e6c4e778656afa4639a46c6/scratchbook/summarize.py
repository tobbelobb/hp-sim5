from pathlib import Path
import json,hashlib,re,math
s=Path('research/sessions/attempt-64d28c7e5e6c4e778656afa4639a46c6');e=s/'evidence'
def load(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
truth=[[0,-1900,-280],[1645.448267190434,950,-280],[-1645.448267190434,950,-280],[0,0,1900]]
def errors(anchors):
 er=[math.dist(a,t) for a,t in zip(anchors,truth)]
 return dict(per_anchor_mm=er,max_mm=max(er),rmse_mm=math.sqrt(sum(v*v for v in er)/4))
results={};omitted=[]
for label in ['stock-auto','historical-auto']:
 d=e/label
 if not (d/'result.json').exists():continue
 m=load(d/'sweeps.headless/manifest.json');clock=load(d/'sweeps.headless/clock.json');data=load(d/'sweeps.json')
 stages={p.name:load(p) for p in sorted((d/'sweeps.stages').glob('*.json'))};fit_scores=[]
 for name,stage in stages.items():
  if stage['kind']=='fit':
   fit=stage['payload']['fit'];fit_scores.append(dict(stage=name,**errors(fit['anchors']['value']),cost=fit['cost'],info_rank=fit['info_rank']))
 bundle=dict(command=load(d/'command.json'),result=load(d/'result.json'),manifest=m,clock=clock,console=(d/'console.log').read_text(),full_auto_log=(d/'sweeps.full_auto.log').read_text(),iteration_rows=[json.loads(line) for line in (d/'sweeps.full_auto_log.jsonl').read_text().splitlines()],stages=stages,force_checkpoints={p.name:load(p) for p in d.glob('*.force-tuning.json')},runtime_environment=load(d/'runtime-environment.json') if (d/'runtime-environment.json').exists() else {'NODE_OPTIONS':None,'HP_SIM5_NATIVE_ENGINE':None,'HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS':'5000000'})
 (e/f'{label}-raw.json').write_text(json.dumps(bundle,separators=(',',':'))+'\n')
 applied=next((c for c in m.get('applied_parameters',[]) if c.startswith('M669')),None)
 anchors=[[float(v) for v in re.search(r'\b'+name+r'([-+\d.eE]+:[-+\d.eE]+:[-+\d.eE]+)',applied)[1].split(':')] for name in 'ABCD'] if applied else None
 results[label]=dict(status=m['status'],normal_completion=m.get('normal_completion'),stop_reason=m.get('stop_reason'),applied_command=applied,applied_anchors_mm=anchors,applied_errors=errors(anchors) if anchors else None,truth_nominal_home_mm=truth,fit_candidates=fit_scores,steps=clock['total_steps'],simulated_s=clock['simulated_s'],wall_s=load(d/'result.json')['wall_s'],sweeps=len(data['sweeps']),points=sum(len(sw['data_points']) for sw in data['sweeps']),dataset_sha256=sha(d/'sweeps.json'),canonical_sweeps_sha256=hashlib.sha256(json.dumps(data['sweeps'],sort_keys=True,separators=(',',':')).encode()).hexdigest(),native_artifact=m['physics_artifact'],baked_scene_sha256=m['baked_scene_sha256'])
 vals=sorted(v for score in fit_scores for v in score['per_anchor_mm'])
 if vals:
  results[label]['fit_candidate_error_distribution_mm']={'count':len(vals),'median':vals[len(vals)//2],'p95_nearest_rank':vals[math.ceil(.95*len(vals))-1],'max':vals[-1]}
 results[label]['stage_source_hashes_agree']=(len({json.dumps(v['source_sha256'],sort_keys=True) for v in stages.values()})==1)
 for p in sorted(d.rglob('*')):
  if p.is_file():omitted.append(dict(path=str(p.relative_to(s)),bytes=p.stat().st_size,sha256=sha(p),retention='Exact stage JSON, logs, metadata and force checkpoints embedded in raw bundle; datasets committed separately; original RRD/journal/service files workspace-only.'))
if len(results)==2:
 a,b=[load(e/n/'sweeps.json') for n in ['stock-auto','historical-auto']]
 strip=lambda value:{k:v for k,v in value.items() if k!='tuned_at'}
 results['pair_checks']=dict(sweeps_exactly_equal=a['sweeps']==b['sweeps'],mm_per_degree_equal=a['config']['mm_per_degree']==b['config']['mm_per_degree'],m666_equal=a['config']['m666_adjusted_by_data_collector']==b['config']['m666_adjusted_by_data_collector'],force_tuning_equal_without_timestamp=strip(a['config']['force_tuning'])==strip(b['config']['force_tuning']),baked_scene_equal=results['stock-auto']['baked_scene_sha256']==results['historical-auto']['baked_scene_sha256'],applied_commands_equal=results['stock-auto']['applied_command']==results['historical-auto']['applied_command'],fit_source_hashes_equal=load(e/'stock-auto/sweeps.stages/fit-001-default.json')['source_sha256']==load(e/'historical-auto/sweeps.stages/fit-001-default.json')['source_sha256'])
(e/'summary.json').write_text(json.dumps(results,indent=2)+'\n');(e/'artifact-manifest.json').write_text(json.dumps(omitted,indent=2)+'\n')
print(json.dumps({k:{a:b for a,b in v.items() if a not in ['native_artifact','fit_candidates']} for k,v in results.items()},indent=2))
