from pathlib import Path
import subprocess,os,json,time,signal,hashlib
root=Path.cwd(); s=root/'research/sessions/attempt-0b9a865f76514b04930528d43f2c31ea'; results=[]
contract=dict(order=['old','current'],baseline='2c0b86ff',current_revision=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),physics_change='c7713d82 open-loop inertia correction only',max_calibrations=2,wall_timeout_s_each=145,max_physics_steps_each=1200000,max_fit_iterations=3,sweep_points=6,radii_mm=[30]*4,buildup=0,layering=False,force_low_n=.01,force_mid_n=.05,force_max_n=.5,sensor_collection_force_n=1,truth_use='post-hoc scoring only',rng='Unmodified production RNG; no controlled common seed is exposed here.')
(s/'evidence/contract.json').write_text(json.dumps(contract,indent=2)+'\n')
for label in contract['order']:
 d=s/'evidence'/label;d.mkdir(exist_ok=False)
 cmd=[str(root/'.venv/bin/python'),'autocal/autocal.py','--headless-sim','--recording-mode','events','--machine-type','hangprinter_3','--dataset',str(d/'sweeps.json'),'--rrf-config','sys/config_hp3.g','--base-radii','30,30,30,30','--buildup-factor','0','--find-radii','off','--find-buildup-factor','off','--optimizer-mode','fast','--solve-optimizer','lbfgsb','--solve-iterations','400','--solve-restarts','4','--max-steps','3','--sweep-points','6','--speedup','25','--stage-artifacts',str(d/'sweeps.stages'),'--collector-args','--force-low','.01','--force-mid','.05','--force-max','.5','--sensor-collection-force','1','--no-auto-tune-force']
 (d/'command.json').write_text(json.dumps(cmd,indent=2)+'\n')
 env=dict(os.environ,HP_SIM5_NATIVE_ENGINE=str((s/f'evidence/native/{label}.node').resolve()),HP_SIM5_AUTOCAL_JOB='1',HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS='1200000',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
 start=time.monotonic();out=dict(label=label)
 with (d/'console.log').open('w') as f:
  p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True,env=env)
  try: out['exit_code']=p.wait(timeout=145)
  except subprocess.TimeoutExpired:
   out['timeout']=True;os.killpg(p.pid,signal.SIGTERM)
   try: out['exit_code']=p.wait(timeout=8)
   except subprocess.TimeoutExpired: os.killpg(p.pid,signal.SIGKILL);out['exit_code']=p.wait()
 out['wall_s']=time.monotonic()-start
 results.append(out);(s/'evidence/run-results.json').write_text(json.dumps(results,indent=2)+'\n');print(json.dumps(out),flush=True)
