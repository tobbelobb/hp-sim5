"""Full-precision checkpoints with a warm cache at the same path as D3/D4/D7."""
from collections import Counter
import json
import os
from pathlib import Path
import re
import runpy
import shutil
import time
from checks import parse_fixed_radii, numeric_metrics
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
ns=runpy.run_path(str(ROOT/'autocal/tools/regress_calibration_logs.py'),run_name='warm_precision')
folder=HERE/'D15'
folder.mkdir(exist_ok=False)
path=folder/'sweeps.json'
shutil.copy2(ROOT/'output/extended-autocal/no-buildup-reference-001/sweeps.json',path)
for key in list(os.environ):
    if key.startswith('JAX_') or key.startswith('AUTOCAL_REFERENCE') or key.startswith('AUTOCAL_EXTENDED_REFERENCE'):
        os.environ.pop(key)
os.environ.update(JAX_LOGGING_LEVEL='ERROR',JAX_DEBUG_LOG_MODULES='jax._src.compiler',
    TF_CPP_MIN_LOG_LEVEL='2',JAX_ENABLE_COMPILATION_CACHE='true',
    JAX_PERSISTENT_CACHE_MIN_COMPILE_TIME_SECS='0',JAX_COMPILATION_CACHE_DIR=str(HERE/'target-cache'))
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS']:
    os.environ[key]='1'
spec=ns['DatasetSpec'](name='D15',machine_type='hangprinter_4',base_radii='30',buildup_factor='0',
    true_anchors=ns['SIM_3D_TRUE_ANCHORS'],true_radii=[30.0]*4,find_radii='off',
    extra_args=('--stage-artifacts',str(folder/'stages')))
start=time.monotonic()
rc,output=ns['run_autocal'](repo_root=ROOT,dataset_path=path,dataset_spec=spec,full_auto_log=folder/'sweeps.full_auto.log')
(folder/'stdout.log').write_text(output)
log=folder/'sweeps.full_auto.log'
combined=output+'\n'+log.read_text()
parsed=parse_fixed_radii(ns,log)
passed,lines,_=ns['report_dataset'](name='D15',ref=parse_fixed_radii(ns,HERE/'D1/sweeps.full_auto.log'),gen=parsed,
    dataset_spec=spec,tol_mm_total=1e-6,fail_on_score_mismatch=True,color=False)
(folder/'regression-comparison.txt').write_text('\n'.join(lines)+'\n')
result=dict(trial='D15',mode='target-warm-same-path',returncode=rc,elapsed_s=time.monotonic()-start,regression_pass=passed,
    exact_full_precision_metrics_equal=numeric_metrics(log)==numeric_metrics(HERE/'D1/sweeps.full_auto.log'),
    hits=dict(Counter(module for module,key in set(re.findall(r"Persistent compilation cache hit for '([^']+)' with key '([^']+)'",combined)))),
    misses=dict(Counter(module for module,key in set(re.findall(r"PERSISTENT COMPILATION CACHE MISS for '([^']+)' with key '([^']+)'",combined)))))
(folder/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
assert rc==0 and passed and result['exact_full_precision_metrics_equal']
