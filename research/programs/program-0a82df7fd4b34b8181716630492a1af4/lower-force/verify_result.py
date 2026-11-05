"""Saved evidence evaluator only; never imported by collection or fitting."""
from pathlib import Path
import json,re,sys,importlib.util,math
S=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('saved_evaluator',S/'harness/analyze.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
name=sys.argv[1];run=S/'runs'/name;out=S/'evidence/measurements.json';allm=json.loads(out.read_text());row=allm[name];h=json.loads((run/'sweeps.headless/manifest.json').read_text());reply=h.get('firmware_final',{}).get('M669','');a=[]
for axis in 'ABCD':
 hit=re.search(r'\b'+axis+r':\s*([-+\d.eE]+),\s*([-+\d.eE]+),\s*([-+\d.eE]+)',reply)
 if hit:a.append([float(x) for x in hit.groups()])
readback=m.metrics(a);row['firmware_final_M669']=reply;row['readback_applied']=readback
if row['applied'] and readback:
 deltas=[abs(v-w) for x,y in zip(row['applied']['anchors'],readback['anchors']) for v,w in zip(x,y)]
 row['readback_max_coordinate_delta_mm']=max(deltas);row['readback_agrees_at_print_precision']=max(deltas)<=.00500001
else:row['readback_agrees_at_print_precision']=False
row['readback_note']='M669 reply prints coordinates to0.01mm. Agreement permits0.005mm per coordinate. Command-rounded applied geometry and reply-rounded geometry retained separately.'
dataset=m.read(run/'sweeps.json',{});sweeps=dataset.get('sweeps',[]);row['dataset_sweep_pairs']=len(sweeps);row['bidirectional_physical_sub_sweeps']=sum(len(x.get('metadata',{}).get('direction_forces',[])) for x in sweeps)
row['dataset_direction_entries']=row['bidirectional_physical_sub_sweeps']
points=[json.loads(line)['point'] for p in run.glob('*.partial-points.jsonl') for line in p.open()]
row['bidirectional_physical_sub_sweeps']=sum(p.get('step_index')==p.get('step_count',0)-1 for p in points)
row['physical_sub_sweeps_with_points']=sum(p.get('step_index')==0 for p in points)
row['physical_sub_sweeps_started']=len(re.findall(r'Sub-sweep \d+/\d+:', (run/'sweeps.full_auto.log').read_text()))
row['sub_sweep_count_definition']='Dataset pairs and direction entries count persisted dataset metadata. Physical completed sub-sweeps count final step_index in all point journals, including failures/retries. Started sub-sweeps count collector log headings.'
end=(m.read(run/'exit.json',{}) or {}).get('finished_unix');start=(m.read(run/'invocation.json',{}) or {}).get('started_unix');row['primary_process_wall_s']=end-start if end and start else h.get('wall_s');row['primary_process_wall_source']='runner timestamps' if end and start else 'manifest fallback for failure without exit.json'
allm[name]=row;out.write_text(json.dumps(allm,indent=2,allow_nan=False)+'\n');(run/'verified-result.json').write_text(json.dumps(row,indent=2,allow_nan=False)+'\n')
print(json.dumps({k:row[k] for k in ['run_id','category','applied','readback_applied','readback_agrees_at_print_precision','dataset_sweep_pairs','bidirectional_physical_sub_sweeps','fit_count','total_steps','simulated_s','wall_s','iteration_tail_max_mm']},indent=2))
