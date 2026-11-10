"""Run the unchanged regression entry point with observational instrumentation."""
import argparse
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import runpy
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
parser = argparse.ArgumentParser()
parser.add_argument('mode', choices=['cold-on','off','warm-on'])
parser.add_argument('trial')
args = parser.parse_args()
output = HERE / args.trial
output.mkdir(exist_ok=False)
for key in ['JAX_COMPILATION_CACHE_DIR','JAX_PERSISTENT_CACHE_MIN_COMPILE_TIME_SECS']:
    os.environ.pop(key, None)
os.environ['JAX_ENABLE_COMPILATION_CACHE'] = 'false' if args.mode == 'off' else 'true'
if args.mode != 'off':
    os.environ['JAX_COMPILATION_CACHE_DIR'] = str(HERE / 'disk-cache')
    os.environ['JAX_PERSISTENT_CACHE_MIN_COMPILE_TIME_SECS'] = '0'
ns = runpy.run_path(str(ROOT/'autocal/tools/regress_calibration_logs.py'), run_name='cache_benchmark')
globals_ = ns['main'].__globals__
original = globals_['run_autocal']
rows = []
lock = threading.Lock()
started = time.monotonic()
def observed(**kwargs):
    spec, path = kwargs['dataset_spec'], kwargs['dataset_path']
    row = dict(dataset=spec.name, fixture_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
               scratch_dir=str(path.parent), start_s=time.monotonic()-started)
    before = time.monotonic()
    rc, text = original(**kwargs)
    row.update(returncode=rc, elapsed_s=time.monotonic()-before,
               compatibility_warnings=text.count('PjRt-IFRT does not track XLA executable versions'))
    (output/f'{spec.name}.stdout.log').write_text(text)
    log = path.with_name(path.stem+'.full_auto.log')
    if rc == 0 and log.exists():
        parsed = ns['parse_log_file'](log)
        row['numerical'] = dict(summary=asdict(parsed.summary) if parsed.summary else None,
          iterations=[{key:getattr(it,key) for key in ('anchors','radii','fit_score_ui','rank_score','history_rank_score')} for it in parsed.iterations])
    with lock:
        rows.append(row)
        (output/'datasets.json').write_text(json.dumps(rows,indent=2)+'\n')
        print('COMPLETED '+json.dumps({key:row[key] for key in ('dataset','returncode','elapsed_s','compatibility_warnings')}),flush=True)
    return rc, text
globals_['run_autocal'] = observed
sys.argv = [str(ROOT/'autocal/tools/regress_calibration_logs.py'),'--color','never']
rc = ns['main']()
summary = dict(mode=args.mode,trial=args.trial,elapsed_s=time.monotonic()-started,runner_returncode=rc,
  datasets=len(rows),child_failures=[r['dataset'] for r in rows if r['returncode']],
  compatibility_warnings=sum(r['compatibility_warnings'] for r in rows),
  runner_sha256=hashlib.sha256((ROOT/'autocal/tools/regress_calibration_logs.py').read_bytes()).hexdigest(),
  revision=os.popen('git rev-parse HEAD').read().strip())
(output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print('BATTERY_SUMMARY '+json.dumps(summary),flush=True)
