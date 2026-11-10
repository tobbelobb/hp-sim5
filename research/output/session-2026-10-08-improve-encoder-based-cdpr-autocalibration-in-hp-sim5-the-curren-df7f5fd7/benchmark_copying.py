from pathlib import Path
import sys, time, json, copy, importlib.util, statistics
import numpy as np
sys.path.insert(0,str(Path.cwd()))
from autocal.stage_artifacts import read_stage_artifact, write_stage_artifact
from autocal.planning_pass import plan_ellipse_sweep, plan_next_ellipse_sweep
from autocal.history_selection import evaluate_history_candidates, rank_history_candidates
from autocal.tools.replay_stage import main
s=Path(__file__).resolve().parent
out=s/'E07'
out.mkdir(exist_ok=True)
fit_path=s/'E03/stages/fit-001-default.json'
history_path=s/'E03/stages/history-006.json'
f=read_stage_artifact(fit_path)['payload']
h=read_stage_artifact(history_path)['payload']
options={**f['planning_options'],'write_cfg':out/'bench.cfg.txt'}
def timed(fn,n=10):
    rows=[]
    for _ in range(n):
        start=time.perf_counter();value=fn();rows.append(time.perf_counter()-start)
    return value,rows
plan,plan_times=timed(lambda:plan_ellipse_sweep(f['fit'],f['dataset_path'],**options))
evaluated,eval_times=timed(lambda:evaluate_history_candidates(h['candidates'],**h['validation_options']))
ranked,rank_times=timed(lambda:rank_history_candidates(evaluated),100)
assert ranked[0][1]['iteration']==3
# JSON restore changes object identity, not decision keys.
assert [x[0] for x in evaluated]==[x[0] for x in h['evaluated']]
# Compare old planner (with recomputation) on frozen fits: replace only fitting
# dependencies in the original module, preserving its actual planning algorithm.
spec=importlib.util.spec_from_file_location('baseline_planning',s/'original_planning_pass.py')
old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
original_rank=old.rank_candidates_d_optimal
counter={'observed_matrices':0}
def rank_old(*args,**kwargs):
    return original_rank(*args,**kwargs)
# A paired full-plan comparison also exercises fitting; use embedded prefix.
snapshot=out/'frozen-input.json';snapshot.write_text(json.dumps(f['fit']['dataset']))
kwargs={**f['fit_options'],**options}
for key in ('fd_eps_mm','regularization','collector_args'):
    kwargs[key]=options[key]
# Toggle only metadata copying around complete old/new calls. The same public
# transformation still performs exactly the same scalar length calculations.
import autocal.filter_pass as filter_module
real_transform=filter_module.dataset_with_modeled_lengths
def baseline_transform(*args,**kwargs):
    kwargs["copy_metadata"]=True
    return real_transform(*args,**kwargs)
def baseline_full(*args,**kwargs):
    filter_module.dataset_with_modeled_lengths=baseline_transform
    try:
        return old.plan_next_ellipse_sweep(*args,**kwargs)
    finally:
        filter_module.dataset_with_modeled_lengths=real_transform
# Warm once in the same process; then alternate order to reduce cache effects.
old_times=[];new_times=[];exact=[]
baseline_full(snapshot,**kwargs)
for pair in range(7):
    outputs={}
    for name,fn in ([('old',baseline_full),('new',plan_next_ellipse_sweep)] if pair%2==0 else [('new',plan_next_ellipse_sweep),('old',baseline_full)]):
        start=time.perf_counter();outputs[name]=fn(snapshot,**kwargs)
        (old_times if name=='old' else new_times).append(time.perf_counter()-start)
    np.testing.assert_array_equal(outputs['old']['anchors'],outputs['new']['anchors'])
    assert outputs['old']['cost']==outputs['new']['cost']
    assert outputs['old']['ranked']==outputs['new']['ranked']
    exact.append(True)
# A/B planner matrix recomputation on identical frozen fits, alternating order.
import autocal.planning_pass as pp
real_rank=pp.rank_candidates_d_optimal
cached_times=[];recomputed_times=[]
def recalc(*args,**kwargs):
    kwargs.pop('observed_information',None)
    return real_rank(*args,**kwargs)
for i in range(30):
    pair_results={}
    for name,rank in ([('cached',real_rank),('recomputed',recalc)] if i%2==0 else [('recomputed',recalc),('cached',real_rank)]):
        pp.rank_candidates_d_optimal=rank
        start=time.perf_counter();pair_results[name]=plan_ellipse_sweep(f['fit'],snapshot,**options)
        (cached_times if name=='cached' else recomputed_times).append(time.perf_counter()-start)
    assert pair_results['cached']['ranked']==pair_results['recomputed']['ranked']
pp.rank_candidates_d_optimal=real_rank
# Real CLI integration checks: plan, evaluate, rank and frozen-input fit.
cli=[]
for stage,input_path in [('plan',fit_path),('evaluate',history_path),('rank',history_path),('fit',fit_path)]:
    start=time.perf_counter();main([stage,str(input_path),'--output',str(out/(stage+'-replay.json'))]);cli.append({'stage':stage,'in_process_seconds_with_io':time.perf_counter()-start})
restored_fit=read_stage_artifact(out/'fit-replay.json')['payload']['fit']
np.testing.assert_array_equal(restored_fit['anchors'],f['fit']['anchors'])
assert restored_fit['cost']==f['fit']['cost']
results=dict(fit_seconds_recorded=f['fit_seconds'],plan_seconds=plan_times,evaluate_seconds=eval_times,rank_seconds=rank_times,
old_full_plan_seconds=old_times,new_full_plan_seconds=new_times,full_plan_exact=exact,
cached_plan_seconds=cached_times,recomputed_plan_seconds=recomputed_times,cli=cli,selected_iteration=ranked[0][1]['iteration'],fit_replay_exact=True)
(out/'benchmark.json').write_text(json.dumps(results,indent=2))
print(json.dumps({k:statistics.median(v) for k,v in results.items() if isinstance(v,list) and v and type(v[0]) is float},indent=2))
