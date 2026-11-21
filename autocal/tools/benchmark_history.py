"""Benchmark inputs, measurements, and history for local runs and CI."""

import ast
from datetime import datetime, timezone
from functools import lru_cache
import hashlib
from importlib import metadata
import json
import os
import platform
from pathlib import Path
import re
import signal
from statistics import median
import subprocess
import sys
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[2]
MACHINES = ('hangprinter_3', 'hangprinter_4', 'hangprinter_5', 'hangprinter_5_pulley_reduced')
MAX_PHYSICS_STEPS = 4_500_000
PATIENCE = 8
METRICS = ('accuracy_mm', 'precision_mm', 'convergence_sim_s', 'total_sim_s')
BACKFILL_DIR = ROOT / 'output/autocal-benchmark-backfill'
HISTORY_REF = 'refs/heads/benchmark-history'
FINGERPRINT_VERSION = 2
SCORER_ENTRY = 'autocal/tools/autocal_benchmark.py'
EXECUTION_INPUTS = ('autocal/tests/test_autocal_benchmark.py', 'package.json', '.github/workflows/nodejs.yml',
                    'scripts/autocal_benchmark_ci.py', 'autocal/tools/benchmark_history.py')
CATALOG = 'machines/catalog.json'
LEGACY_TRANSFORM = 'autocal/auto-calibration-simulation-for-hangprinter'
DEPENDENCY_INPUTS = ('package-lock.json', 'requirements.txt', 'autocal/requirements-jax-cpu.txt')


def git(*args, cwd=None):
    return subprocess.run(['git', *args], cwd=cwd or ROOT, check=True,
                          capture_output=True, text=True).stdout.strip()


SCENES = {'hangprinter_3': 'hp3', 'hangprinter_4': 'hp4', 'hangprinter_5': 'hp5',
          'hangprinter_5_pulley_reduced': 'hp5_pulley_reduced'}
ENTRY_POINTS = ('autocal/autocal.py', 'scripts/autocal_headless.mjs', 'autocal/control/cli/collect_sweep_data.mjs')
# Inputs read as directories rather than through imports. Firmware is its whole
# tree (including submodule commits) minus tests, docs and other machines' configs.
DATA_PREFIXES = ('src/wasm/cable_solver/', 'scripts/build_native_physics.mjs', 'autocal/schemas/',
                 'public/usd_scenes/cable_joint_typed_schema/', 'RRF/')
DATA_EXCLUDED = re.compile(r'(^|/)(tests?|benches|ai_docs|logs|gcodes|firmware)/|\.md$|^RRF/run/vsd/sys/'
                           r'|^RRF/(RepRapFirmware\.wiki|DuetWebControl|DuetSoftwareFramework)$')
PY_IMPORT = re.compile(r'^\s*(?:from\s+(\.*[\w.]*)\s+import\s+([\w\s,()*]+)|import\s+([\w.,\s]+))', re.M)
JS_IMPORT = re.compile(r"""(?:\bfrom\s*|\bimport\s*\(?\s*|require\(\s*|new URL\(\s*)['"](\.{1,2}/[^'"]+)['"]""")
JS_PATH_IN_PY = re.compile(r"""['"]([\w./-]+\.(?:m?js|json|g|usda))['"]""")


def python_imports(path, text, exists):
    """Find local modules and package initializers imported by Python source."""
    package = path.split('/')[:-1]
    for node in ast.walk(ast.parse(text, filename=path)):
        if isinstance(node, ast.ImportFrom):
            prefix = package[:len(package) - node.level + 1] if node.level else []
            module = prefix + (node.module.split('.') if node.module else [])
            imports = [module, *(module + [alias.name] for alias in node.names)]
        elif isinstance(node, ast.Import):
            imports = [alias.name.split('.') for alias in node.names]
        else:
            continue
        for parts in imports:
            if not parts:
                continue
            for depth in range(1, len(parts) + 1):
                initializer = '/'.join(parts[:depth]) + '/__init__.py'
                if exists(initializer):
                    yield initializer
            module_file = '/'.join(parts) + '.py'
            if exists(module_file):
                yield module_file


@lru_cache(maxsize=16)
def runtime_files(commit, version):
    """Repository files the headless run can load at this commit: the Python and
    JavaScript import closures from the entry points, plus DATA_PREFIXES."""
    tree = dict(line.split('\t', 1)[::-1] for line in git('ls-tree', '-r', commit).splitlines())
    blobs = {path: entry.split()[2] for path, entry in tree.items()}
    seen, todo = set(), [p for p in ENTRY_POINTS if p in blobs]
    while todo:
        path = todo.pop()
        if path in seen:
            continue
        seen.add(path)
        text = blob_text(blobs[path])
        if path.endswith('.py') and version == 2:
            todo.extend(python_imports(path, text, blobs.__contains__))
            todo.extend(p for p in JS_PATH_IN_PY.findall(text) if p in blobs)
        elif path.endswith('.py'):
            here = path.rsplit('/', 1)[0]
            for base, names, plain in PY_IMPORT.findall(text):
                modules = [m.strip() for m in plain.split(',')] if plain else [base]
                if base:
                    names = [n.strip().split(' as ')[0] for n in names.strip('()').split(',')]
                    modules += [f'{base}.{n}' if base.strip('.') else base + n for n in names if n]
                for module in modules:
                    dots = len(module) - len(module.lstrip('.'))
                    stem = module.lstrip('.').replace('.', '/')
                    roots = ['/'.join(here.split('/')[:len(here.split('/')) - dots + 1])] if dots else ['', 'autocal', here]
                    for root in roots:
                        for candidate in (f'{root}/{stem}.py', f'{root}/{stem}/__init__.py'):
                            candidate = candidate.lstrip('/')
                            if candidate in blobs:
                                todo.append(candidate)
            todo += [p for p in JS_PATH_IN_PY.findall(text) if p in blobs]
        elif path.endswith(('.js', '.mjs')):
            for spec in JS_IMPORT.findall(text):
                candidate = os.path.normpath(os.path.join(os.path.dirname(path), spec.split('?')[0]))
                if candidate in blobs:
                    todo.append(candidate)
    data = {p for p in blobs if p.startswith(DATA_PREFIXES) and not DATA_EXCLUDED.search(p)}
    return {p: blobs[p] for p in seen | data}, blobs


@lru_cache(maxsize=None)
def blob_text(blob):
    return subprocess.run(['git', 'cat-file', 'blob', blob], cwd=ROOT, capture_output=True,
                          text=True, errors='replace').stdout


def catalog_inputs(blobs, machine):
    files = {CATALOG, LEGACY_TRANSFORM, 'scripts/build_cable_wasm.mjs', *DEPENDENCY_INPUTS}
    files.update(p for p in blobs if p.startswith('RRF/run/vsd/sys/') and not Path(p).name.startswith('config'))
    if CATALOG in blobs:
        catalog = json.loads(blob_text(blobs[CATALOG]))
        entries = catalog['machines']
        scenes = {entry['scene'] for entry in entries
                  if machine in (entry['name'].lower(), *(alias.lower() for alias in entry['nicknames']))}
        files.update(entry['rrf_config'] for entry in entries
                     if entry['scene'] in scenes and entry.get('rrf_config'))
        pending = list(scenes)
        while pending:
            scene = pending.pop()
            if scene not in blobs or scene in files:
                continue
            files.add(scene)
            pending.extend(os.path.normpath(os.path.join(os.path.dirname(scene), ref))
                           for ref in re.findall(r'@([^@\n]+)@', blob_text(blobs[scene])))
    return files


def fingerprint(commit, machine, version=FINGERPRINT_VERSION):
    """Hash tracked runtime inputs. Version 1 is retained for backfill imports."""
    if version not in (1, 2):
        raise ValueError(f'Unsupported benchmark fingerprint version {version}')
    files, blobs = runtime_files(commit, version)
    scene = SCENES[machine]
    paths = {f'public/usd_scenes/{scene}.usda', f'RRF/run/vsd/sys/config_{scene}.g'}
    if version == 2:
        paths.update(catalog_inputs(blobs, machine))
    files = {**files, **{p: blobs[p] for p in paths if p in blobs}}
    return hashlib.sha256(json.dumps(sorted(files.items())).encode()).hexdigest()[:20]


def execution_fingerprint(commit):
    """Hash the CI benchmark harness separately from simulated runtime inputs."""
    _, blobs = runtime_files(commit, FINGERPRINT_VERSION)
    inputs = [(path, blobs.get(path)) for path in EXECUTION_INPUTS]
    return hashlib.sha256(json.dumps(inputs).encode()).hexdigest()


def scorer_fingerprint(root):
    """Hash the running scorer and its transitive local Python imports."""
    files, pending = {}, [SCORER_ENTRY]
    while pending:
        path = pending.pop()
        if path in files:
            continue
        source = (root / path).read_bytes()
        files[path] = hashlib.sha256(source).hexdigest()
        pending.extend(python_imports(path, source, lambda p: (root / p).is_file()))
    return 'sha256:' + hashlib.sha256(json.dumps(sorted(files.items())).encode()).hexdigest()


def installed_node_packages(modules):
    packages = {}
    for path in sorted(modules.glob('*')):
        if path.name.startswith('.'):
            continue
        candidates = sorted(path.glob('*')) if path.name.startswith('@') else [path]
        for package in candidates:
            manifest = package / 'package.json'
            if manifest.is_file():
                info = json.loads(manifest.read_text())
                name = str(package.relative_to(modules))
                packages[name] = info.get('version')
                for child, version in installed_node_packages(package / 'node_modules').items():
                    packages[f'{name}/node_modules/{child}'] = version
    return packages


@lru_cache(maxsize=8)
def dependency_environment(modules):
    return dict(python=platform.python_version(),
                python_packages=sorted([d.metadata['Name'].lower(), d.version] for d in metadata.distributions()),
                node=subprocess.check_output(['node', '--version'], text=True).strip(),
                node_packages=installed_node_packages(modules))


def settings(max_steps=MAX_PHYSICS_STEPS):
    return dict(max_physics_steps=max_steps, patience=PATIENCE, recording_mode='none',
                omp_threads=1, blas_threads=1, xla_cpu_multi_thread_eigen=False)


def run_autocal(machine, dataset, repo, log, timeout_s, max_steps=MAX_PHYSICS_STEPS, scene=None, extra_args=()):
    env = {key: os.environ[key] for key in ('PATH', 'HOME', 'LANG', 'LD_LIBRARY_PATH', 'HP_SIM5_COMPLIANCE_PROBE') if key in os.environ}
    if scene is not None:
        env['HP_SIM5_AUTOCAL_SCENE'] = str(scene)
    env.update(OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1',
               XLA_FLAGS='--xla_cpu_multi_thread_eigen=false intra_op_parallelism_threads=1',
               HP_SIM5_AUTOCAL_JOB='1', HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS=str(max_steps))
    command = [sys.executable, 'autocal/autocal.py', '--headless-sim', '--machine-type', machine,
               '--dataset', str(dataset), '--recording-mode', 'none', '--patience', str(PATIENCE), *extra_args]
    with log.open('w') as output, subprocess.Popen(
            command, cwd=repo, env=env, stdout=output, stderr=subprocess.STDOUT, start_new_session=True) as process:
        try:
            return process.wait(timeout=timeout_s)
        finally:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass


def measurement(commit, machine, *, repeat=1, max_steps=MAX_PHYSICS_STEPS, ci=False):
    run_id = os.environ.get('GITHUB_RUN_ID') if ci else None
    attempt = os.environ.get('GITHUB_RUN_ATTEMPT') if ci else None
    return dict(commit=commit, machine=machine, fingerprint=fingerprint(commit, machine),
                fingerprint_version=FINGERPRINT_VERSION,
                date=git('show', '-s', '--format=%cI', commit), subject=git('show', '-s', '--format=%s', commit),
                measurement_id=f'ci:{run_id}:{attempt}:{machine}' if run_id else uuid4().hex,
                recorded_at=datetime.now(timezone.utc).isoformat(), repeat=repeat, settings=settings(max_steps),
                scorer_revision=git('rev-parse', 'HEAD'),
                scorer=scorer_fingerprint(ROOT),
                execution_fingerprint=execution_fingerprint(commit) if ci else None,
                github_run_id=run_id, github_run_attempt=attempt)


def regressions(previous, current):
    """Metrics with a review-worthy degradation, keyed by metric."""
    findings = {}
    for key in METRICS:
        old, new = previous.get(key), current.get(key)
        if old is None:
            continue
        if new is None:
            findings[key] = f'{key} is unavailable (previously {old:.3g})'
            continue
        accuracy_crossing = key == 'accuracy_mm' and old <= 1 < new
        margin = old * 0.05 if key.endswith('_s') else max(0.2, old * 0.1)
        if accuracy_crossing or new > old + margin:
            target = ' (crossed the 1 mm target)' if accuracy_crossing else ''
            findings[key] = f'{key} increased from {old:.3f} to {new:.3f}{target}'
    return findings


def read_results(path):
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def merge_results(*sources):
    """Deduplicate transfers of the same observation while preserving repeated runs."""
    rows = {}
    for source in sources:
        for row in source:
            identity = row.get('measurement_id') or json.dumps(row, sort_keys=True)
            if rows.setdefault(identity, row) != row:
                raise ValueError(f'Conflicting rows for measurement {identity}')
    return sorted(rows.values(), key=lambda r: (datetime.fromisoformat(r['date']), r['commit'],
                                               r['machine'], r.get('recorded_at') or '',
                                               r.get('measurement_id') or ''))


def results_text(rows):
    return ''.join(json.dumps(row, allow_nan=False) + '\n' for row in rows)


def write_results(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f'.{path.name}.{uuid4().hex}')
    temporary.write_text(results_text(rows))
    temporary.replace(path)


def verified_results(rows):
    """Resolve legacy short SHAs and check each submission against its recorded commit."""
    for row in rows:
        machine = row['machine']
        if machine not in MACHINES or not re.fullmatch(r'[0-9a-f]{7,40}', row['commit']):
            raise ValueError(f'Invalid benchmark machine or commit: {machine} {row["commit"]}')
        commit = git('rev-parse', '--verify', row['commit'] + '^{commit}')
        if row['fingerprint'] != fingerprint(commit, machine, row.get('fingerprint_version', 1)):
            raise ValueError(f'Fingerprint mismatch for {machine} at {commit}')
        yield dict(row, commit=commit)


def protocol(row):
    return (json.dumps(row.get('settings'), sort_keys=True), row.get('scorer'),
            json.dumps(row.get('environment', row.get('dependency_inputs')), sort_keys=True))


def metric_medians(rows):
    result = {}
    for key in METRICS:
        values = [row[key] for row in rows if row.get(key) is not None]
        # Convergence requires a majority of runs to report a convergence time.
        minimum = len(rows) // 2 + 1 if key == 'convergence_sim_s' else 1
        result[key] = median(values) if len(values) >= minimum else None
    return result


def markdown_summary(rows, *, by_fingerprint=False):
    group_name = 'runtime fingerprint' if by_fingerprint else 'tested commit'
    review_note = '' if by_fingerprint else (
        '▲ marks a review threshold crossed since the preceding successful input under the same protocol. ')
    lines = ['# Autocal benchmark history', '',
             'Accuracy is maximum applied anchor coordinate error in mm. Precision is the largest '
             'split-fit anchor coordinate difference in mm. Convergence is simulated seconds to '
             'within 2 mm of truth. Total is simulated seconds for the whole run.', '',
             f'Rows group measurements by {group_name} and scoring protocol. Values are medians '
             'of successful runs. Convergence requires a majority to report a time. '
             + review_note + 'Individual results, wall times, and failures are in `results.jsonl`.', '']
    for machine in MACHINES:
        lines += [f'## {machine}', '',
                  '| Commit | Fingerprint | Date | OK / runs | Accuracy | Precision | Converged | Total | Sweeps | Stop | Status | Subject |',
                  '|---|---|---|---:|---:|---:|---:|---:|---:|---|---|---|']
        groups = {}
        for row in rows:
            if row['machine'] == machine:
                identity = (row.get('fingerprint_version', 1), row['fingerprint']) if by_fingerprint else row['commit']
                groups.setdefault((identity, protocol(row)), []).append(row)
        table, previous = [], {}
        for (_, config), group in groups.items():
            first = group[0]
            successes = [row for row in group if row['status'] == 'ok']
            m = metric_medians(successes)
            findings = regressions(previous.get(config, {}), m) if successes and not by_fingerprint else {}
            cells = []
            for key, value in m.items():
                if value is None:
                    cell = 'unavailable' if successes else '–'
                else:
                    cell = f'{value:.2f}' if key.endswith('_mm') else f'{value:.0f}'
                cells.append(f'**{cell} ▲**' if key in findings else cell)
            if successes:
                previous[config] = m
            sweeps = [row['sweeps'] for row in successes if row.get('sweeps') is not None]
            sweeps = f'{median(sweeps):g}' if sweeps else '–'
            stops = ', '.join(sorted({row['stop_reason'] for row in successes if row.get('stop_reason')}))
            status = ', '.join(sorted({row['status'] for row in group}))
            if first.get('settings') is None:
                status += ' (legacy)'
            subject = first['subject'][:80].replace('|', '&#124;')
            fp = f'v{first.get("fingerprint_version", 1)} {first["fingerprint"]}'
            table.append(f'| {first["commit"][:10]} | {fp} | {first["date"][:16]} | {len(successes)}/{len(group)} '
                         f'| {" | ".join(cells)} | {sweeps} | {stops} | {status} | {subject} |')
        lines += table[::-1] + ['']
    return '\n'.join(lines)
