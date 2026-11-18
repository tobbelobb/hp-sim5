#!/usr/bin/env python3
"""Run the headless autocal benchmark on past main commits, several in parallel.

For each selected first-parent commit, check it out in a temporary worktree
(RRF submodules as submodule worktrees, shared .venv and node_modules), build or
reuse a cached rrf_simulator and native physics addon, run full-auto autocal for
each machine and score it with the *current* autocal/tools/autocal_benchmark.py.

By default, measure each distinct runtime input once. --repeat N runs N fresh
measurements of each explicitly selected revision and machine. results.jsonl
keeps individual measurements; summary.md shows medians by runtime input.
--contribute imports results into the tracked history contributions file.

  .venv/bin/python scripts/autocal_benchmark_backfill.py --since 2026-11-01
  .venv/bin/python scripts/autocal_benchmark_backfill.py --revs OLD NEW --machines hangprinter_4 --repeat 3
  .venv/bin/python scripts/autocal_benchmark_backfill.py --summary-only
  .venv/bin/python scripts/autocal_benchmark_backfill.py --contribute
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import fcntl
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import threading
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from autocal.tools.benchmark_history import (  # noqa: E402
    CONTRIBUTIONS, MACHINES, MAX_PHYSICS_STEPS, fingerprint, git, markdown_summary, measurement,
    merge_results, read_results, run_autocal, settings, verified_results, write_results,
)

RRF_SUBMODULES = ('RRF/ReprapFirmware', 'RRF/RRFLibraries', 'RRF/CANlib')
LOCK = threading.Lock()


def run(command, cwd, log):
    with open(log, 'a') as f:
        f.write(f'$ {" ".join(map(str, command))}\n')
        f.flush()
        return subprocess.run(command, cwd=cwd, stdout=f, stderr=subprocess.STDOUT).returncode


def select_commits(args):
    if args.revs:
        return [git('rev-parse', '--verify', f'{rev}^{{commit}}') for rev in args.revs]
    return git('rev-list', '--first-parent', '--reverse', f'--since={args.since}', 'main').split()


def build_rrf(commit, worktree, out, threads):
    binary = out / 'cache/rrf' / git('rev-parse', f'{commit}:RRF') / 'rrf_simulator'
    if not binary.exists():
        build = out / 'cache/rrf-build' / binary.parent.name
        log = out / 'cache/rrf-build.log'
        if run(['cmake', '-S', worktree / 'RRF', '-B', build], ROOT, log) or \
           run(['cmake', '--build', build, '--target', 'rrf_simulator', f'-j{threads}'], ROOT, log):
            raise RuntimeError(f'rrf_simulator build failed; see {log}')
        binary.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(build / 'rrf_simulator', binary)
        shutil.rmtree(build)
    (worktree / 'RRF/build').mkdir(parents=True, exist_ok=True)
    shutil.copy2(binary, worktree / 'RRF/build/rrf_simulator')


def build_native(commit, worktree, out):
    key = hashlib.sha256(' '.join(git('rev-parse', f'{commit}:{path}') for path in (
        'src/wasm/cable_solver', 'scripts/build_native_physics.mjs')).encode()).hexdigest()[:20]
    cache = out / 'cache/native' / key
    target = worktree / 'src/wasm/cable_solver/target/native'
    if not (cache / 'physics.node').exists():
        log = out / 'cache/native-build.log'
        # PGO only changes speed, not results; skip its training run.
        if run(['node', 'scripts/build_native_physics.mjs', '--no-pgo'], worktree, log):
            raise RuntimeError(f'native physics build failed; see {log}')
        cache.mkdir(parents=True, exist_ok=True)
        for name in ('physics.node', 'physics.node.json'):
            shutil.copy2(target / name, cache / name)
        shutil.rmtree(worktree / 'src/wasm/cable_solver/target', ignore_errors=True)
    target.mkdir(parents=True, exist_ok=True)
    for name in ('physics.node', 'physics.node.json'):
        shutil.copy2(cache / name, target / name)


def prepare(commit, out, threads, worktree):
    git('worktree', 'add', '--detach', str(worktree), commit)
    for sub in RRF_SUBMODULES:
        sha = git('rev-parse', f'{commit}:{sub}')
        if subprocess.run(['git', 'cat-file', '-e', f'{sha}^{{commit}}'], cwd=ROOT / sub).returncode:
            git('fetch', 'origin', sha, cwd=ROOT / sub)
        shutil.rmtree(worktree / sub, ignore_errors=True)
        git('worktree', 'add', '--detach', str(worktree / sub), sha, cwd=ROOT / sub)
    for link in ('.venv', 'node_modules'):
        (worktree / link).symlink_to(ROOT / link, target_is_directory=True)
    build_rrf(commit, worktree, out, threads)
    build_native(commit, worktree, out)


def remove_worktree(worktree):
    if not worktree.exists():
        return
    for sub in RRF_SUBMODULES:
        if (worktree / sub / '.git').exists():
            subprocess.run(['git', 'worktree', 'remove', '--force', str(worktree / sub)], cwd=ROOT / sub)
    subprocess.run(['git', 'worktree', 'remove', '--force', str(worktree)], cwd=ROOT)
    shutil.rmtree(worktree, ignore_errors=True)


def benchmark(commit, machine, repeat, worktree, out, timeout_s, max_steps):
    from autocal.tools.autocal_benchmark import benchmark_metrics

    row = measurement(commit, machine, repeat=repeat, max_steps=max_steps)
    directory = out / 'runs' / commit[:12] / machine / row['measurement_id']
    directory.mkdir(parents=True)
    dataset = directory / 'sweeps.json'
    log = directory / 'autocal.log'
    row['run_dir'] = str(directory.relative_to(out))
    try:
        code = run_autocal(machine, dataset, worktree, log, timeout_s, max_steps)
        details = log.read_text(errors='replace')[-2000:]
        if code:
            collection_log = dataset.with_suffix('.full_auto.log')
            if collection_log.exists():
                details += '\n' + collection_log.read_text(errors='replace')[-2000:]
            if code < 0 or 'stop requested (Ctrl-C)' in details:
                status = 'interrupted'
            elif 'physics step budget exhausted' in details.lower():
                status = 'step-cap'
            elif 'wall-clock deadline' in details or 'fetch failed' in details:
                status = 'retry'
            elif 'usage:' in details:
                status = 'unsupported'
            else:
                status = 'failed'
            row.update(status=status, reason=details[-1500:] or f'autocal exit {code}')
        else:
            manifest_path = dataset.with_suffix('.headless') / 'manifest.json'
            manifest = json.loads(manifest_path.read_text())
            if manifest.get('normal_completion') is False:
                raise RuntimeError('Calibration did not complete and apply parameters')
            row.update(benchmark_metrics(dataset, worktree), status='ok')
    except subprocess.TimeoutExpired:
        row.update(status='timeout', reason=f'Wall-clock timeout after {timeout_s:g} seconds')
    except Exception as error:
        row.update(status='failed', reason=f'{type(error).__name__}: {error}')
    record_result(out, row)
    print(commit[:12], machine, row['status'], row.get('accuracy_mm'), flush=True)


def record_result(out, record):
    path = out / 'results.jsonl'
    with LOCK:
        write_results(path, merge_results(read_results(path), [record]))


def write_summary(out):
    (out / 'summary.md').write_text(markdown_summary(merge_results(read_results(out / 'results.jsonl'))))
    print(f'Wrote {out / "summary.md"}')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--since', default='2026-11-01', help='first-parent main commits since this date')
    parser.add_argument('--revs', nargs='+', help='explicit commits instead of --since')
    parser.add_argument('--stride', type=int, default=1,
                        help="run every Nth distinct runtime input per machine; skipped inputs' commits are not reported")
    parser.add_argument('--machines', nargs='+', default=MACHINES, choices=MACHINES)
    # A run keeps about 1.5 cores busy (physics 1, rrf_simulator 0.5): 12 runs ~ 85% of 20 cores.
    parser.add_argument('--threads', type=int, default=24)
    parser.add_argument('--mem-gb', type=float, default=50)
    parser.add_argument('--threads-per-run', type=int, default=2, help='python fit + native physics')
    parser.add_argument('--mem-gb-per-run', type=float, default=4)
    # HEAD needs at most ~3M steps (HP4); broken older revisions otherwise burn 6M.
    parser.add_argument('--max-physics-steps', type=int, default=MAX_PHYSICS_STEPS)
    parser.add_argument('--timeout-min', type=float, default=90)
    parser.add_argument('--out', type=Path, default=ROOT / 'output/autocal-benchmark-backfill')
    # RRF's host simulator silently runs without config when its SD path is long.
    parser.add_argument('--worktree-dir', type=Path, default=Path.home() / '.cache/hpbf',
                        help='short path for checkouts')
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--summary-only', action='store_true')
    modes.add_argument('--contribute', action='store_true', help='import results into the tracked contributions file')
    parser.add_argument('--repeat', type=int, help='fresh measurements per explicit revision and machine')
    args = parser.parse_args()
    if args.stride < 1 or (args.repeat is not None and (args.repeat < 1 or not args.revs or args.stride != 1)):
        parser.error('--stride must be positive; --repeat requires --revs and a stride of 1')
    out = args.out.resolve()
    if args.contribute:
        rows = list(verified_results(read_results(out / 'results.jsonl') + read_results(out / 'retry.jsonl')))
        existing = read_results(CONTRIBUTIONS)
        combined = merge_results(existing, rows)
        write_results(CONTRIBUTIONS, combined)
        print(f'Added {len(combined) - len(existing)} measurements to {CONTRIBUTIONS.relative_to(ROOT)}')
        return
    if args.summary_only:
        return write_summary(out)
    (out / 'cache').mkdir(parents=True, exist_ok=True)
    with (out / '.backfill.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error(f'a backfill is already using {out}. Use a different --out.')
        run_backfill(args, out)


def run_backfill(args, out):
    known = {(row['machine'], row['fingerprint']) for row in read_results(out / 'results.jsonl')
             if row['status'] not in {'retry', 'timeout', 'interrupted', 'setup-failed'}
             and (row.get('settings') == settings(args.max_physics_steps)
                  or ('settings' not in row and args.max_physics_steps == MAX_PHYSICS_STEPS))}
    selection = [(c, m, fingerprint(c, m)) for c in dict.fromkeys(select_commits(args))
                 for m in dict.fromkeys(args.machines)]
    if args.stride > 1:
        keep = set(known)
        for machine in args.machines:
            order = list(dict.fromkeys(fp for _, m, fp in selection if m == machine))
            keep |= {(machine, fp) for fp in order[::-args.stride]}
        selection = [s for s in selection if (s[1], s[2]) in keep]
    todo = {}
    for commit, machine, fp in selection:
        if args.repeat or (machine, fp) not in known:
            known.add((machine, fp))
            todo.setdefault(commit, []).extend((machine, repeat) for repeat in range(1, (args.repeat or 1) + 1))
    todo = list(todo.items())
    workers = max(1, min(args.threads // args.threads_per_run, int(args.mem_gb // args.mem_gb_per_run)))
    print(f'{len(todo)} commits, {sum(len(m) for _, m in todo)} runs, {workers} at a time; results in {out}')
    args.worktree_dir.mkdir(parents=True, exist_ok=True)

    pending, futures = {}, []
    # One live commit per worker keeps every worker busy even when a single
    # slow machine (HP4) is all that remains of several commits.
    live = threading.Semaphore(workers)

    def finished(commit, worktree):
        with LOCK:
            pending[commit] -= 1
            last = pending[commit] == 0
        if last:
            remove_worktree(worktree)
            live.release()

    with ThreadPoolExecutor(workers) as pool:
        # Builds run here, one commit at a time, while earlier commits' runs proceed.
        for commit, machines in todo:
            live.acquire()
            worktree = Path(tempfile.mkdtemp(prefix='b', dir=args.worktree_dir))
            try:
                prepare(commit, out, args.threads, worktree)
            except Exception:
                error = traceback.format_exc()[-2000:]
                for machine, repeat in machines:
                    row = measurement(commit, machine, repeat=repeat, max_steps=args.max_physics_steps)
                    record_result(out, dict(row, status='setup-failed', reason=error.splitlines()[-1]))
                print(commit[:12], 'prepare failed', error.splitlines()[-1], flush=True)
                remove_worktree(worktree)
                live.release()
                continue
            pending[commit] = len(machines)
            for machine, repeat in machines:
                future = pool.submit(benchmark, commit, machine, repeat, worktree, out, args.timeout_min * 60,
                                     args.max_physics_steps)
                future.add_done_callback(lambda _, c=commit, w=worktree: finished(c, w))
                futures.append(future)
    for future in futures:
        future.result()
    write_summary(out)


if __name__ == '__main__':
    main()
