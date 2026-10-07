"""Reproduce compilation failures without datasets, JAX arrays, or execution.

Run with the repository's .venv/bin/python and PYTHONMALLOC=debug.
Child logs and summary.json are retained even when a child crashes.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import gc
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def replay(args):
    sys.path.insert(0, str(args.source_root))
    import numpy as np
    import jax
    from autocal import ellipse_objective_jax as objective

    source = args.source_root / 'autocal/ellipse_objective_jax.py'
    print(json.dumps({'python': sys.version, 'jax': jax.__version__,
                      'numpy': np.__version__, 'source_root': str(args.source_root),
                      'objective_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
                      'python_malloc': os.environ.get('PYTHONMALLOC'),
                      'xla_flags': os.environ.get('XLA_FLAGS')}), flush=True)
    cases = json.loads(args.signatures.read_text())
    start = time.monotonic()
    for cycle in range(args.cycles):
        for case in cases:
            signatures = [jax.ShapeDtypeStruct(tuple(a['shape']), np.dtype(a['dtype']))
                          for a in case['arguments']]
            print(json.dumps({'cycle': cycle, 'case': case['id'], 'stage': 'lower',
                              'elapsed_s': time.monotonic() - start}), flush=True)
            lowered = objective._COMPILED_VALUE_AND_GRAD.lower(*signatures, **case['kwargs'])
            if not args.lower_only:
                compiled = lowered.compile()
                del compiled
            print(json.dumps({'cycle': cycle, 'case': case['id'],
                              'stage': 'lowered' if args.lower_only else 'compiled',
                              'elapsed_s': time.monotonic() - start}), flush=True)
            del lowered, signatures
            gc.collect()
        jax.clear_caches()
        gc.collect()
    print('COMPLETE', len(cases) * args.cycles, flush=True)


def main():
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=Path, default=here.parents[4])
    parser.add_argument('--signatures', type=Path, default=here / 'signatures.json')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--workers', type=int, default=11)
    parser.add_argument('--cycles', type=int, default=1)
    parser.add_argument('--timeout', type=float, default=600)
    parser.add_argument('--lower-only', action='store_true')
    parser.add_argument('--gdb', action='store_true')
    parser.add_argument('--child', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.workers < 1 or args.cycles < 1 or args.timeout <= 0:
        parser.error('workers, cycles, and timeout must be positive')
    args.source_root = args.source_root.resolve()
    args.signatures = args.signatures.resolve()
    if args.child:
        replay(args)
        return 0
    if args.output is None:
        parser.error('--output is required')
    args.output.mkdir(parents=True, exist_ok=False)
    command = [sys.executable, str(Path(__file__).resolve()), '--child',
               '--source-root', str(args.source_root), '--signatures', str(args.signatures),
               '--cycles', str(args.cycles)]
    if args.lower_only:
        command.append('--lower-only')
    if args.gdb:
        command = ['gdb', '-q', '-batch', '--return-child-result',
                   '-x', str(here / 'diagnostic.gdb'), '--args', *command]
    env = os.environ.copy()
    env.update(PYTHONFAULTHANDLER='1', PYTHONHASHSEED='0', OMP_NUM_THREADS='1',
               OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1', NUMEXPR_NUM_THREADS='1')
    # Exclude the earlier investigation's signal hooks and alternate JAX install.
    env.pop('PYTHONPATH', None)
    env.pop('LD_PRELOAD', None)

    def run(i):
        start = time.monotonic()
        with (args.output / f'{i:02d}.txt').open('w') as log:
            try:
                result = subprocess.run(command, env=env, stdout=log,
                                        stderr=subprocess.STDOUT, timeout=args.timeout)
                row = {'id': i, 'returncode': result.returncode, 'timed_out': False}
            except subprocess.TimeoutExpired:
                row = {'id': i, 'returncode': None, 'timed_out': True}
        row['elapsed_s'] = time.monotonic() - start
        print(json.dumps(row), flush=True)
        return row

    (args.output / 'command.json').write_text(json.dumps(command, indent=2) + '\n')
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        results = list(pool.map(run, range(args.workers)))
    (args.output / 'summary.json').write_text(json.dumps(results, indent=2) + '\n')
    return int(any(r['returncode'] != 0 or r['timed_out'] for r in results))


if __name__ == '__main__':
    sys.exit(main())
