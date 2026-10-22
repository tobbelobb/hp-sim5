"""Initial replay series. D2/D6 are relocated-cache controls, not valid transfer tests.

Run transfer.py afterwards for the valid same-path regression-cache comparison.
"""
from collections import Counter
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import re
import runpy
import shutil
import subprocess
import time
from checks import parse_fixed_radii

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
SOURCE = ROOT / 'output/extended-autocal/no-buildup-reference-001/sweeps.json'
REG_CACHE = HERE.parent / 'autocal-jax-cache-20261010/disk-cache'
ns = runpy.run_path(str(ROOT / 'autocal/tools/regress_calibration_logs.py'), run_name='dataset_cache_benchmark')
spec = ns['DatasetSpec'](name='no-buildup-reference-001', machine_type='hangprinter_4',
    base_radii='30', buildup_factor='0', true_anchors=ns['SIM_3D_TRUE_ANCHORS'],
    true_radii=[30.0]*4, find_radii='off')

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def cache_manifest(path):
    return {p.name:sha(p) for p in sorted(path.glob('*-cache'))}

def numerical(parsed):
    return dict(summary=asdict(parsed.summary) if parsed.summary else None,
        iterations=[{k:getattr(it,k) for k in ('anchors','radii','fit_score_ui','rank_score','history_rank_score')}
                    for it in parsed.iterations])

source_hash = sha(SOURCE)
reg_manifest = cache_manifest(REG_CACHE)
(HERE/'provenance.json').write_text(json.dumps(dict(source=str(SOURCE), source_sha256=source_hash,
    original_log_sha256=sha(SOURCE.with_suffix('.full_auto.log')),
    regression_cache=str(REG_CACHE), regression_cache_manifest=reg_manifest,
    revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
    dataset_spec=asdict(spec)),indent=2)+'\n')
for key in list(os.environ):
    if key.startswith('JAX_') or key.startswith('AUTOCAL_REFERENCE') or key.startswith('AUTOCAL_EXTENDED_REFERENCE'):
        os.environ.pop(key)
os.environ['TF_CPP_MIN_LOG_LEVEL']='2'
os.environ['JAX_PERSISTENT_CACHE_MIN_COMPILE_TIME_SECS']='0'
os.environ['JAX_DEBUG_LOG_MODULES']='jax._src.compiler'
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS']:
    os.environ[key]='1'

rows=[]
baseline=None
target_cache=HERE/'target-cache'
for trial,mode in [('D1','off'),('D2','regression-warm'),('D3','cold-empty'),('D4','target-warm'),
                   ('D5','off'),('D6','regression-warm'),('D7','target-warm')]:
    folder=HERE/trial
    folder.mkdir(exist_ok=False)
    path=folder/'sweeps.json'
    shutil.copy2(SOURCE,path)
    cache=folder/'cache' if mode in ('regression-warm','off') else target_cache
    if mode=='regression-warm':
        shutil.copytree(REG_CACHE,cache)
    else:
        cache.mkdir(exist_ok=True)
    before_manifest=cache_manifest(cache)
    os.environ['JAX_ENABLE_COMPILATION_CACHE']='false' if mode=='off' else 'true'
    os.environ['JAX_COMPILATION_CACHE_DIR']=str(cache)
    print(f'START {trial} {mode}',flush=True)
    start=time.monotonic()
    rc,output=ns['run_autocal'](repo_root=ROOT,dataset_path=path,dataset_spec=spec,
                             full_auto_log=folder/'sweeps.full_auto.log')
    elapsed=time.monotonic()-start
    (folder/'stdout.log').write_text(output)
    parsed=parse_fixed_radii(ns, folder/'sweeps.full_auto.log') if rc==0 else None
    if parsed is not None and baseline is None:
        baseline=parsed
    comparison=None
    if parsed is not None:
        comparison,lines,_=ns['report_dataset'](name=trial,ref=baseline,gen=parsed,dataset_spec=spec,
            tol_mm_total=1e-6,fail_on_score_mismatch=True,color=False)
        (folder/'regression-comparison.txt').write_text('\n'.join(lines)+'\n')
    hits=Counter(re.findall(r"Persistent compilation cache hit for '([^']+)'",output))
    misses=Counter(re.findall(r"PERSISTENT COMPILATION CACHE MISS for '([^']+)'",output))
    row=dict(trial=trial,mode=mode,elapsed_s=elapsed,returncode=rc,regression_pass=comparison,
        compatibility_warnings=output.count('PjRt-IFRT does not track XLA executable versions'),
        hits=dict(hits),misses=dict(misses),cache_entries_before=len(before_manifest),
        numerical=numerical(parsed) if parsed else None)
    after_manifest=cache_manifest(cache)
    row['new_cache_entries']=sorted(set(after_manifest)-set(before_manifest))
    (folder/'summary.json').write_text(json.dumps(row,indent=2)+'\n')
    rows.append(row)
    (HERE/'results.json').write_text(json.dumps(rows,indent=2)+'\n')
    print('DONE '+json.dumps({k:row[k] for k in ('trial','mode','elapsed_s','returncode','regression_pass','compatibility_warnings','hits','misses')}),flush=True)
    if rc != 0 or parsed is None:
        raise RuntimeError(f'{trial} failed; see {folder}/stdout.log')

original=parse_fixed_radii(ns, SOURCE.with_suffix('.full_auto.log'))
ok,lines,_=ns['report_dataset'](name='original-collection-vs-replay',ref=original,gen=baseline,
    dataset_spec=spec,tol_mm_total=1e-6,fail_on_score_mismatch=True,color=False)
(HERE/'original-comparison.txt').write_text('\n'.join(lines)+'\n')
(HERE/'integrity.json').write_text(json.dumps(dict(source_unchanged=sha(SOURCE)==source_hash,
    regression_cache_unchanged=cache_manifest(REG_CACHE)==reg_manifest,
    original_comparison_pass=ok),indent=2)+'\n')
