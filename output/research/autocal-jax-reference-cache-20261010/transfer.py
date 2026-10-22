"""Test the regression-warmed cache at its original path, preserving its bytes."""
from collections import Counter
from dataclasses import asdict
import hashlib
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
SOURCE=ROOT/'output/extended-autocal/no-buildup-reference-001/sweeps.json'
CACHE=HERE.parent/'autocal-jax-cache-20261010/disk-cache'
ns=runpy.run_path(str(ROOT/'autocal/tools/regress_calibration_logs.py'),run_name='same_path_transfer')
spec=ns['DatasetSpec'](name='reference',machine_type='hangprinter_4',base_radii='30',buildup_factor='0',
    true_anchors=ns['SIM_3D_TRUE_ANCHORS'],true_radii=[30.0]*4,find_radii='off')
def manifest(path):
    return {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in path.glob('*-cache')}
original=manifest(CACHE)
for key in list(os.environ):
    if key.startswith('JAX_') or key.startswith('AUTOCAL_REFERENCE') or key.startswith('AUTOCAL_EXTENDED_REFERENCE'):
        os.environ.pop(key)
os.environ['TF_CPP_MIN_LOG_LEVEL']='2'
os.environ['JAX_PERSISTENT_CACHE_MIN_COMPILE_TIME_SECS']='0'
os.environ['JAX_ENABLE_COMPILATION_CACHE']='true'
os.environ['JAX_COMPILATION_CACHE_DIR']=str(CACHE)
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS']:
    os.environ[key]='1'
baseline=parse_fixed_radii(ns,HERE/'D1/sweeps.full_auto.log')
for trial in ['D12','D13','D14']:
    folder=HERE/trial
    folder.mkdir(exist_ok=False)
    path=folder/'sweeps.json'
    shutil.copy2(SOURCE,path)
    # D14 is diagnostic only. D12/D13 match the timing arms' logging defaults.
    if trial=='D14':
        os.environ['JAX_LOGGING_LEVEL']='ERROR'
        os.environ['JAX_DEBUG_LOG_MODULES']='jax._src.compiler'
    backup=HERE/'original-regression-cache-backup'
    CACHE.rename(backup)
    try:
        shutil.copytree(backup,CACHE)
        print(f'START {trial} original-path regression-warm',flush=True)
        started=time.monotonic()
        rc,output=ns['run_autocal'](repo_root=ROOT,dataset_path=path,dataset_spec=spec,
                                  full_auto_log=folder/'sweeps.full_auto.log')
        elapsed=time.monotonic()-started
        (folder/'stdout.log').write_text(output)
        log=folder/'sweeps.full_auto.log'
        parsed=parse_fixed_radii(ns,log)
        passed,lines,_=ns['report_dataset'](name=trial,ref=baseline,gen=parsed,dataset_spec=spec,
            tol_mm_total=1e-6,fail_on_score_mismatch=True,color=False)
        (folder/'regression-comparison.txt').write_text('\n'.join(lines)+'\n')
        combined=output+'\n'+log.read_text()
        current=manifest(CACHE)
        row=dict(trial=trial,mode='regression-warm-same-path',elapsed_s=elapsed,returncode=rc,
            regression_pass=passed,exact_full_precision_metrics_equal=numeric_metrics(log)==numeric_metrics(HERE/'D1/sweeps.full_auto.log'),
            numerical=dict(summary=asdict(parsed.summary),iterations=[{k:getattr(it,k) for k in
                ('anchors','radii','fit_score_ui','rank_score','history_rank_score')} for it in parsed.iterations]),
            hits=dict(Counter(module for module,key in set(re.findall(r"Persistent compilation cache hit for '([^']+)' with key '([^']+)'",combined)))) if trial=='D14' else None,
            misses=dict(Counter(module for module,key in set(re.findall(r"PERSISTENT COMPILATION CACHE MISS for '([^']+)' with key '([^']+)'",combined)))) if trial=='D14' else None,
            new_cache_entries=sorted(set(current)-set(original)),
            compatibility_warnings=combined.count('PjRt-IFRT does not track XLA executable versions'))
        (folder/'summary.json').write_text(json.dumps(row,indent=2)+'\n')
        print('DONE '+json.dumps({k:row[k] for k in ('trial','elapsed_s','returncode','regression_pass','hits','misses')}),flush=True)
        assert rc==0 and passed and row['exact_full_precision_metrics_equal']
    finally:
        if CACHE.exists():
            CACHE.rename(folder/'cache-after')
        backup.rename(CACHE)
        assert manifest(CACHE)==original
