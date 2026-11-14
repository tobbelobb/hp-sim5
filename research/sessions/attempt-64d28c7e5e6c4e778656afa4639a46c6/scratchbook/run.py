from pathlib import Path
import subprocess,os,json,time,signal,sys
root=Path.cwd();sys.path.insert(0,str(root))
from autocal.tools.native_snapshot import snapshot_environment
s=root/'research/sessions/attempt-64d28c7e5e6c4e778656afa4639a46c6';label=sys.argv[1];d=s/'evidence'/label;d.mkdir(exist_ok=False)
cmd=[str(root/'.venv/bin/python'),'autocal/autocal.py','--headless-sim','--recording-mode','events','--machine-type','hangprinter_3','--dataset',str(d/'sweeps.json'),'--rrf-config','sys/config_hp3.g','--base-radii','30,30,30,30','--buildup-factor','0','--find-radii','off','--find-buildup-factor','off','--optimizer-mode','fast','--solve-optimizer','lbfgsb','--solve-iterations','400','--solve-restarts','4','--max-steps','20','--sweep-points','20','--speedup','25','--stage-artifacts',str(d/'sweeps.stages')]
(d/'command.json').write_text(json.dumps(cmd,indent=2)+'\n')
base_env=snapshot_environment(Path(sys.argv[2])) if len(sys.argv)>2 else dict(os.environ)
env=dict(base_env,HP_SIM5_AUTOCAL_JOB='1',HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS='5000000',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONUNBUFFERED='1')
(d/'runtime-environment.json').write_text(json.dumps({k:env.get(k) for k in ['HP_SIM5_NATIVE_ENGINE','NODE_OPTIONS','HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS']},indent=2)+'\n')
start=time.monotonic();out=dict(label=label,wall_limit_s=480,max_physics_steps=5000000)
with (d/'console.log').open('w') as f:
 p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True,env=env)
 try:out['exit_code']=p.wait(timeout=480)
 except subprocess.TimeoutExpired:
  out['timeout']=True;os.killpg(p.pid,signal.SIGTERM)
  try:out['exit_code']=p.wait(timeout=8)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);out['exit_code']=p.wait()
out['wall_s']=time.monotonic()-start;(d/'result.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out))
