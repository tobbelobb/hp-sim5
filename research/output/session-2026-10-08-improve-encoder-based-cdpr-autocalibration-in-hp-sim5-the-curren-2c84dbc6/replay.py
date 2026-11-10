import json,os,shutil,subprocess,sys,time
from pathlib import Path
sys.path.insert(0,str(Path.cwd()))
from autocal.tools.regress_calibration_logs import DATASETS,parse_log_file,parse_generated_log_path
ROOT=Path.cwd();OUT=Path(__file__).resolve().parent
variant=sys.argv[1];names=sys.argv[2:] or [s.name for s in DATASETS]
results=[]
for spec in DATASETS:
 if spec.name not in names:continue
 target=OUT/variant/spec.name;target.mkdir(parents=True,exist_ok=True)
 source=ROOT/'autocal/data/references'/f'{spec.name}.json';dataset=target/source.name
 shutil.copy2(source,dataset)
 cmd=[str(ROOT/'.venv/bin/python'),str(ROOT/'autocal/autocal.py'),'--sim','--machine-type',spec.machine_type,'--dataset',str(dataset),'--find-radii',spec.find_radii,'--base-radii',spec.base_radii,'--buildup-factor',spec.buildup_factor,'--no-collect','--full-auto-log',str(target/'events.jsonl'),*spec.extra_args]
 if variant.startswith('cache'):cmd[1]=str(OUT/'cache_cli.py')
 if variant.startswith('guard'):cmd[1]=str(OUT/'guard_cli.py')
 if variant.startswith('safe'):cmd[1]=str(OUT/'safe_cli.py')
 if variant.startswith('pointbucket'):cmd[1]=str(OUT/'pointbucket_cli.py')
 if variant.startswith('bucket'):cmd[1]=str(OUT/'bucket_cli.py')
 if variant.startswith('candidate'):cmd[1]=str(OUT/'candidate_cli.py')
 if variant.startswith('baseline_profile'):cmd[1]=str(OUT/'baseline_profile_cli.py')
 if variant.startswith('guard_profile'):cmd[1]=str(OUT/'guard_profile_cli.py')
 if variant=='baseline_slide':cmd[1]=str(OUT/'baseline_cli.py')
 if variant.startswith('cache_actual'):cmd[1]=str(ROOT/'autocal/autocal.py')
 env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1',MPLCONFIGDIR='/tmp/autocal-mpl',PYTHONHASHSEED='0',PYTHONFAULTHANDLER='1')
 if variant.startswith('cache_actual'):env['JAX_COMPILATION_CACHE_DIR']=str(OUT/'E10-jax-cache')
 if variant=='cache_actual_disabled':env['JAX_ENABLE_COMPILATION_CACHE']='false'
 if variant in ('cache_actual_disabled','cache_actual_repeat'):env['JAX_DEBUG_LOG_MODULES']='jax._src.compiler'
 t=time.perf_counter()
 with (target/'stdout.txt').open('w') as log:
  try:rc=subprocess.run(cmd,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=240).returncode
  except subprocess.TimeoutExpired:rc='timeout'
 result={'dataset':spec.name,'variant':variant,'seconds':time.perf_counter()-t,'returncode':rc,'command':cmd,'cache_enabled':env.get('JAX_ENABLE_COMPILATION_CACHE'),'cache_dir':env.get('JAX_COMPILATION_CACHE_DIR')}
 if rc==0:
  text=(target/'stdout.txt').read_text();p=parse_generated_log_path(text);path=Path(p)
  if not path.is_absolute():path=ROOT/path
  shutil.copy2(path,target/'calibration.log')
  parsed=parse_log_file(path)
  result.update({'summary':vars(parsed.summary),'iterations':len(parsed.iterations)})
 (target/'run.json').write_text(json.dumps(result,indent=2)+'\n');results.append(result)
 print(json.dumps(result),flush=True)
 (OUT/f'{variant}-runs.json').write_text(json.dumps(results,indent=2)+'\n')
