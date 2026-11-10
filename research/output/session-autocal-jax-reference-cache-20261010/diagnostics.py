"""Separate compiler diagnostics and full-precision fit checkpoints (not timing arms)."""
from collections import Counter
import json
import os
from pathlib import Path
import re
import runpy
import shutil
import sys
import time
from checks import parse_fixed_radii

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
SOURCE=ROOT/'output/extended-autocal/no-buildup-reference-001/sweeps.json'
REG_CACHE=HERE.parent/'autocal-jax-cache-20261010/disk-cache'
ns=runpy.run_path(str(ROOT/'autocal/tools/regress_calibration_logs.py'),run_name='cache_diagnostics')
for key in list(os.environ):
    if key.startswith('JAX_') or key.startswith('AUTOCAL_REFERENCE') or key.startswith('AUTOCAL_EXTENDED_REFERENCE'):
        os.environ.pop(key)
# With JAX's default root logger at NOTSET its debug-handler filter discards
# compiler messages. ERROR activates module debugging and retains native level 2.
os.environ['JAX_LOGGING_LEVEL']='ERROR'
os.environ['JAX_DEBUG_LOG_MODULES']='jax._src.compiler,jax._src.dispatch'
os.environ['JAX_PERSISTENT_CACHE_MIN_COMPILE_TIME_SECS']='0'
os.environ['TF_CPP_MIN_LOG_LEVEL']='2'
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS']:
    os.environ[key]='1'

trials=[('D11','diagnostic-warm')] if '--followup' in sys.argv else [('D8','regression-warm'),('D9','target-warm'),('D10','off')]
for trial,mode in trials:
    folder=HERE/trial
    folder.mkdir(exist_ok=False)
    path=folder/'sweeps.json'
    shutil.copy2(SOURCE,path)
    cache=folder/'cache'
    if mode!='off':
        source_cache=REG_CACHE if mode=='regression-warm' else HERE/'D8/cache' if mode=='diagnostic-warm' else HERE/'target-cache'
        shutil.copytree(source_cache,cache)
    else:
        cache.mkdir()
    spec=ns['DatasetSpec'](name=trial,machine_type='hangprinter_4',base_radii='30',buildup_factor='0',
        true_anchors=ns['SIM_3D_TRUE_ANCHORS'],true_radii=[30.0]*4,find_radii='off',
        extra_args=('--stage-artifacts',str(folder/'stages')))
    os.environ['JAX_ENABLE_COMPILATION_CACHE']='false' if mode=='off' else 'true'
    os.environ['JAX_COMPILATION_CACHE_DIR']=str(cache)
    print(f'START diagnostic {trial} {mode}',flush=True)
    start=time.monotonic()
    rc,output=ns['run_autocal'](repo_root=ROOT,dataset_path=path,dataset_spec=spec,
        full_auto_log=folder/'sweeps.full_auto.log')
    (folder/'stdout.log').write_text(output)
    if rc:
        raise RuntimeError(f'{trial} failed with {rc}')
    log=folder/'sweeps.full_auto.log'
    parsed=parse_fixed_radii(ns,log)
    combined=output+'\n'+log.read_text()
    summary=dict(trial=trial,mode=mode,elapsed_s=time.monotonic()-start,returncode=rc,
        hits=dict(Counter(module for module,key in set(re.findall(r"Persistent compilation cache hit for '([^']+)' with key '([^']+)'",combined)))),
        misses=dict(Counter(module for module,key in set(re.findall(r"PERSISTENT COMPILATION CACHE MISS for '([^']+)' with key '([^']+)'",combined)))),
        compatibility_warnings=combined.count('PjRt-IFRT does not track XLA executable versions'))
    (folder/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print('DONE diagnostic '+json.dumps(summary),flush=True)
