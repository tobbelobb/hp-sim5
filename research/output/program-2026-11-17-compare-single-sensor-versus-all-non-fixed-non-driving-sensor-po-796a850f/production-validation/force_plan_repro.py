import inspect,json,tempfile
from pathlib import Path
import autocal.planning_pass as planning
from autocal.equal_strain import InfeasibleForceProfileError
source=Path(__file__).resolve().parent/'failed-plan'
temporary=tempfile.TemporaryDirectory(prefix='hp5-force-plan-')
output=Path(temporary.name)
rows=[json.loads(l) for l in (source/'sweeps.full_auto_log.jsonl').read_text().splitlines()]
settings=rows[-1]['runs'][0]['settings']
fields=inspect.signature(planning.plan_next_ellipse_sweep).parameters
options={key:value for key,value in settings.items() if key in fields}
options.update(residuals_csv=None,candidate_count=41,delta_min=None,delta_max=None,regularization=1e-6,
 min_fixed_delta_spacing_mm=20.,write_cfg=output/'force-plan.cfg.txt',
 collector_output=output/'force-plan.json',collector_args=())
failures=[];original=planning.write_sensor_force_profile
def record(*args,**kwargs):
 try:return original(*args,**kwargs)
 except InfeasibleForceProfileError:
  failures.append(str(args[3]));raise
planning.write_sensor_force_profile=record
plan=planning.plan_next_ellipse_sweep(source/'sweeps.json',**options)
assert plan['collect_command'] and plan['collection_error'] is None
assert failures
result={'rejected_infeasible':failures,'selected':str(plan['best_cfg']),'collect_command':plan['collect_command']}
print(json.dumps(result,indent=2))
