#!/usr/bin/env python3
"""Run the headless autocal benchmark on past main commits, several in parallel.

For each selected first-parent commit, check it out in a temporary worktree
(RRF submodules as submodule worktrees, shared .venv and node_modules), build or
reuse a cached rrf_simulator and native physics addon, run full-auto autocal for
each machine and score it with the *current* autocal/tools/autocal_benchmark.py.

Results go to OUT/results.jsonl, one line per (commit, machine), and are
resumable: finished pairs are skipped. OUT/summary.md tabulates them and marks
every metric that got worse than the previous commit's value.

  .venv/bin/python scripts/autocal_benchmark_backfill.py --since 2026-11-10
  .venv/bin/python scripts/autocal_benchmark_backfill.py --revs 921ced3c dfc0e3e3 HEAD
  .venv/bin/python scripts/autocal_benchmark_backfill.py --summary-only

Wall times are not comparable between commits here; builds and parallel runs
share the CPU. Simulated times are.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from autocal.tools.autocal_benchmark import benchmark_metrics  # noqa: E402

MACHINES = ('hangprinter_3', 'hangprinter_4', 'hangprinter_5', 'hangprinter_5_pulley_reduced')
# Commits touching none of these cannot change an autocal result.
RELEVANT_PATHS = ('autocal', 'src', 'integrations', 'public/usd_scenes', 'RRF', 'hp-sim-3d/app',
                  'scripts/autocal_headless.mjs', 'scripts/build_native_physics.mjs')
RRF_SUBMODULES = ('RRF/ReprapFirmware', 'RRF/RRFLibraries', 'RRF/CANlib')
METRICS = (('accuracy_mm', 'mm', 0.1), ('precision_mm', 'mm', 0.1),
           ('convergence_sim_s', 's', 0.02), ('total_sim_s', 's', 0.02))
LOCK = threading.Lock()


def git(*args, cwd=ROOT):
    return subprocess.run(['git', *args], cwd=cwd, check=True, capture_output=True, text=True).stdout.strip()


def run(command, cwd, log):
    with open(log, 'a') as f:
        f.write(f'$ {" ".join(map(str, command))}\n')
        f.flush()
        return subprocess.run(command, cwd=cwd, stdout=f, stderr=subprocess.STDOUT).returncode


def select_commits(args):
    if args.revs:
        return [git('rev-parse', rev) for rev in args.revs]
    commits = git('rev-list', '--first-parent', '--reverse', f'--since={args.since}', 'main', '--',
                  *RELEVANT_PATHS).split()
    return commits[::-args.stride][::-1]  # keep the newest commit whatever the stride


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


def prepare(commit, out, threads, worktrees):
    worktree = worktrees / commit[:12]
    remove_worktree(worktree)
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
    return worktree


def remove_worktree(worktree):
    if not worktree.exists():
        return
    for sub in RRF_SUBMODULES:
        if (worktree / sub / '.git').exists():
            subprocess.run(['git', 'worktree', 'remove', '--force', str(worktree / sub)], cwd=ROOT / sub)
    subprocess.run(['git', 'worktree', 'remove', '--force', str(worktree)], cwd=ROOT)
    shutil.rmtree(worktree, ignore_errors=True)
    for sub in (ROOT, *(ROOT / s for s in RRF_SUBMODULES)):
        subprocess.run(['git', 'worktree', 'prune'], cwd=sub)


def benchmark(commit, machine, worktree, out, timeout_s):
    directory = out / 'runs' / commit[:12] / machine
    shutil.rmtree(directory, ignore_errors=True)
    directory.mkdir(parents=True)
    dataset = directory / 'sweeps.json'
    env = {**os.environ, 'OMP_NUM_THREADS': '1', 'OPENBLAS_NUM_THREADS': '1', 'MKL_NUM_THREADS': '1',
           'XLA_FLAGS': '--xla_cpu_multi_thread_eigen=false intra_op_parallelism_threads=1',
           'HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS': '6000000'}
    record = dict(commit=commit, date=git('show', '-s', '--format=%cI', commit),
                  subject=git('show', '-s', '--format=%s', commit), machine=machine)
    try:
        with (directory / 'autocal.log').open('w') as log:
            result = subprocess.run([sys.executable, 'autocal/autocal.py', '--headless-sim', '--machine-type',
                                     machine, '--dataset', str(dataset), '--recording-mode', 'none',
                                     '--patience', '8'], cwd=worktree, env=env, stdout=log,
                                    stderr=subprocess.STDOUT, timeout=timeout_s)
        if result.returncode:
            raise RuntimeError(f'autocal exit {result.returncode}: '
                               + (directory / 'autocal.log').read_text()[-1500:])
        record['metrics'] = benchmark_metrics(dataset, worktree)
    except Exception as error:  # every failure is a result worth recording
        record['error'] = f'{type(error).__name__}: {error}'[-2000:]
    with LOCK, (out / 'results.jsonl').open('a') as f:
        f.write(json.dumps(record) + '\n')
    print(commit[:12], machine, record.get('metrics') or record['error'][:200], flush=True)


def load_results(out):
    path = out / 'results.jsonl'
    return [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []


def write_summary(out):
    """Newest commit first; a metric worse than the previous commit's is bold with ▲."""
    latest = {}
    for record in load_results(out):
        latest[record['commit'], record['machine']] = record
    order = list(dict.fromkeys(r['commit'] for r in sorted(latest.values(), key=lambda r: r['date'])))
    lines = ['# Autocal benchmark per commit', '',
             'Accuracy: worst applied anchor coordinate error. Precision: largest anchor difference '
             'between fits of alternating points. Converged: simulated s until all anchors within 2 mm. '
             'Total: simulated s for the whole run. ▲ marks a value worse than the previous commit.', '']
    for machine in MACHINES:
        lines += [f'## {machine}', '', '| Commit | Date | Accuracy | Precision | Converged | Total | Sweeps | Stop | Subject |',
                  '|---|---|---:|---:|---:|---:|---:|---|---|']
        rows, previous = [], {}
        for commit in order:
            record = latest.get((commit, machine))
            if record is None:
                continue
            head = f'| {commit[:10]} | {record["date"][:16]} |'
            if 'error' in record:
                rows.append(f'{head} error | | | | | | {record["subject"][:60]} — {record["error"][:80].replace("|", "/")} |')
                continue
            m, cells = record['metrics'], []
            for key, unit, margin in METRICS:
                value, old = m.get(key), previous.get(key)
                if value is None:
                    cells.append('never' if key == 'convergence_sim_s' else '–')
                    continue
                worse = old is not None and (value > old * (1 + margin) if unit == 's' else value > old + margin)
                text = f'{value:.2f}' if unit == 'mm' else f'{value:.0f}'
                cells.append(f'**{text} ▲**' if worse else text)
            previous = m
            rows.append(f'{head} {" | ".join(cells)} | {m.get("sweeps")} | {m.get("stop_reason")} | {record["subject"][:60]} |')
        lines += rows[::-1] + ['']
    (out / 'summary.md').write_text('\n'.join(lines))
    print(f'Wrote {out / "summary.md"}')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--since', default='2026-11-10', help='first-parent main commits since this date')
    parser.add_argument('--revs', nargs='+', help='explicit commits instead of --since')
    parser.add_argument('--stride', type=int, default=1, help='benchmark every Nth selected commit')
    parser.add_argument('--machines', nargs='+', default=MACHINES, choices=MACHINES)
    parser.add_argument('--threads', type=int, default=18)
    parser.add_argument('--mem-gb', type=float, default=50)
    parser.add_argument('--threads-per-run', type=int, default=2, help='python fit + native physics')
    parser.add_argument('--mem-gb-per-run', type=float, default=4)
    parser.add_argument('--timeout-min', type=float, default=90)
    parser.add_argument('--out', type=Path, default=ROOT / 'output/autocal-benchmark-backfill')
    # RRF's host simulator silently runs without config when its SD path is long.
    parser.add_argument('--worktree-dir', type=Path, default=Path.home() / '.cache/hpbf',
                        help='short path for checkouts')
    parser.add_argument('--summary-only', action='store_true')
    args = parser.parse_args()
    out = args.out.resolve()
    (out / 'cache').mkdir(parents=True, exist_ok=True)
    if args.summary_only:
        return write_summary(out)

    # Build failures are retried on the next invocation; run results are final.
    done = {(r['commit'], r['machine']) for r in load_results(out)
            if not r.get('error', '').startswith('prepare:')}
    todo = [(c, [m for m in args.machines if (c, m) not in done]) for c in select_commits(args)]
    todo = [(c, machines) for c, machines in todo if machines]
    workers = max(1, min(args.threads // args.threads_per_run, int(args.mem_gb // args.mem_gb_per_run)))
    print(f'{len(todo)} commits, {sum(len(m) for _, m in todo)} runs, {workers} at a time; results in {out}')

    pending, futures = {}, []
    # Prepare only enough commits ahead to keep every worker busy.
    live = threading.Semaphore(-(-workers // len(args.machines)) + 1)

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
            try:
                worktree = prepare(commit, out, args.threads, args.worktree_dir)
            except Exception:
                error = traceback.format_exc()[-2000:]
                with LOCK, (out / 'results.jsonl').open('a') as f:
                    for machine in machines:
                        f.write(json.dumps(dict(commit=commit, date=git('show', '-s', '--format=%cI', commit),
                                                subject=git('show', '-s', '--format=%s', commit),
                                                machine=machine, error=f'prepare: {error}')) + '\n')
                print(commit[:12], 'prepare failed', error.splitlines()[-1], flush=True)
                remove_worktree(args.worktree_dir / commit[:12])
                live.release()
                continue
            pending[commit] = len(machines)
            for machine in machines:
                future = pool.submit(benchmark, commit, machine, worktree, out, args.timeout_min * 60)
                future.add_done_callback(lambda _, c=commit, w=worktree: finished(c, w))
                futures.append(future)
    write_summary(out)


if __name__ == '__main__':
    main()
