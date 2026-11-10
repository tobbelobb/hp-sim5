"""Evaluate saved evidence only. Never imported by the collector or fitting process."""
from pathlib import Path
import json,math,re,sys,statistics,hashlib,datetime
S=Path(__file__).resolve().parents[1]
TRUTH=[[0.,-1900.,-280.],[1645.448267190434,950.,-280.],[-1645.448267190434,950.,-280.],[0.,0.,1900.]]
def read(p,default=None):
 try:return json.loads(p.read_text())
 except (OSError,ValueError):return default

def metrics(a):
 if isinstance(a,dict):a=a.get('value')
 if not isinstance(a,list) or len(a)!=4:return None
 try:errors=[math.dist(x[:3],y) for x,y in zip(a,TRUTH)]
 except (ValueError,TypeError):return None
 if not all(map(math.isfinite,errors)):return None
 return dict(anchors=a,per_anchor_error_mm=dict(zip('ABCD',errors)),max_mm=max(errors),rmse_mm=math.sqrt(sum(x*x for x in errors)/4),anchors_over20=sum(x>20 for x in errors),all_below5=all(x<5 for x in errors))

def summarize(directory):
 inv=read(directory/'invocation.json',{});world=inv.get('world',{})
 if world.get('split')=='evaluation' and not (S/'policy-frozen.json').exists():raise RuntimeError('Held-out scoring prohibited before policy freeze')
 h=read(directory/'sweeps.headless/manifest.json',{});clock=read(directory/'sweeps.headless/clock.json',{})
 rows=[]
 for p in sorted((directory/'stages').glob('fit-*.json')):
  d=read(p,{})['payload'];f=d['fit'];a=f.get('anchors');m=metrics(a)
  rows.append(dict(iteration=int(p.stem.split('-')[1]),errors=m,success=f.get('calibration',{}).get('success'),cost=f.get('cost'),info_rank=f.get('info_rank'),info_rank_deficient=f.get('info_rank_deficient'),warnings=f.get('warnings'),fit_seconds=d.get('fit_seconds'),artifact=str(p.relative_to(S))))
 log=[]
 p=directory/'sweeps.full_auto_log.jsonl'
 if p.exists():
  for line in p.read_text().splitlines():
   try:d=json.loads(line)
   except ValueError:continue
   if 'iteration' in d:log.append({k:d.get(k) for k in ['iteration','decision','score_ui','best_iteration','consensus_spread_mm','warnings','recovery','runs']})
 for row in rows:
  run_id=Path(row['artifact']).stem.split('-',2)[2]
  matching=[r for d in log if d['iteration']==row['iteration'] for r in d.get('runs',[]) if r.get('id')==run_id]
  row['production_metrics']=matching[0].get('metrics') if matching else None
  row['production_valid']=(matching[0].get('metrics',{}).get('valid') if matching else None)
 applied=None
 for line in h.get('applied_parameters',[]):
  if line.startswith('M669'):
   a=[]
   for key in 'ABCD':
    match=re.search(r'\b'+key+r'([-+\d.eE]+):([-+\d.eE]+):([-+\d.eE]+)',line)
    if not match:break
    a.append([float(x) for x in match.groups()])
   applied=metrics(a)
 reason=h.get('stop_reason');normal=reason in ['patience','spread-consensus'] and h.get('normal_completion') is True
 terminal=bool(read(directory/'exit.json')) or h.get('status') in ['complete','failed','interrupted']
 if normal and applied:category='accepted_bad_over20' if applied['anchors_over20'] else 'accepted_below20'
 elif reason=='max-steps':category='capped_without_normal_stopping'
 elif terminal:category='workflow_failure' if applied is None else 'workflow_failure_with_interim_application'
 else:category='running'
 journals=list(directory.glob('*.partial-points.jsonl'));points=sum(sum(1 for _ in p.open()) for p in journals)
 dataset=read(directory/'sweeps.json',{});sweeps=dataset.get('sweeps',[])
 physical_dataset_points=sum(len(w.get('data_points',w.get('points',[]))) for w in sweeps)
 errors=[r['errors']['max_mm'] for r in rows if r['errors']]
 noise_count=0;noise_sum=0.;noise_sq=0.
 p=directory/'sweeps.headless/events.jsonl'
 if p.exists():
  for line in p.open():
   if '"sensor_noise"' not in line:continue
   try:d=json.loads(line)
   except ValueError:continue
   for clean,noisy in zip(d.get('clean_angles_deg',[]),d.get('noisy_angles_deg',[])):
    e=noisy-clean;noise_count+=1;noise_sum+=e;noise_sq+=e*e
 sigma=math.sqrt(max(0,noise_sq/noise_count-(noise_sum/noise_count)**2)) if noise_count else None
 return dict(run_id=directory.name,world=world.get('id'),split=world.get('split'),category=category,status=h.get('status'),stop_reason=reason,normal_automatic_acceptance=normal,applied=applied,iterations=rows,fit_count=len(rows),iteration_count=len({r['iteration'] for r in rows}),invalid_fits=sum(r['errors'] is None or r['success'] is False or r.get('production_valid') is False for r in rows),underconstrained_penalty_fits=sum(bool((r.get('production_metrics') or {}).get('underconstrained_penalty')) for r in rows),iteration_tail_max_mm=max(errors) if errors else None,iteration_any_anchor_over20=sum(x>20 for x in errors),decisions=log,sweeps=len(sweeps),dataset_points=physical_dataset_points,physical_collected_points=points,total_steps=h.get('total_steps',clock.get('total_steps',0)),simulated_s=h.get('simulated_s',clock.get('simulated_s',0)),wall_s=h.get('wall_s'),service_wall_s=h.get('service_wall_s',clock.get('wall_s')),noise_observations=noise_count,noise_empirical_sigma_deg=sigma,noise_seed=world.get('noise_seed'),physics_engine=h.get('physics_engine'),artifact_sha256=h.get('physics_artifact',{}).get('sha256'),authored_scene_sha256=h.get('scene_sha256'),baked_scene_sha256=h.get('baked_scene_sha256'),source_sha256=h.get('source_sha256'),autocal_source_sha256=h.get('autocal_source_sha256'),source_revision=h.get('git_revision'),manifest=str((directory/'sweeps.headless/manifest.json').relative_to(S)),invocation=str((directory/'invocation.json').relative_to(S)))

if __name__=='__main__':
 runs=sys.argv[1:] or [p.name for p in (S/'runs').iterdir() if p.is_dir() and not p.name.startswith('eval')]
 out={r:summarize(S/'runs'/r) for r in runs};p=S/'evidence'/'measurements.json';existing=read(p,{})
 existing.update(out);p.write_text(json.dumps(existing,indent=2,allow_nan=False)+'\n')
 print(json.dumps({r:{k:v for k,v in d.items() if k in ['category','applied','iteration_count','invalid_fits','iteration_tail_max_mm','physical_collected_points','total_steps','simulated_s','wall_s','noise_empirical_sigma_deg']} for r,d in out.items()},indent=2))
