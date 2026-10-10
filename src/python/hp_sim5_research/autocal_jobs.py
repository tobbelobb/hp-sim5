"""Bounded supervisor jobs running the production CLI in isolated worlds."""
import asyncio
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

from .experiments import write_json


def number(name, value, low, high, *, integer=False):
    if (type(value) not in ((int,) if integer else (int, float)) or
            not math.isfinite(value) or not low <= value <= high):
        raise ValueError(f'{name} must be {"an integer" if integer else "a number"} in [{low}, {high}]')
    return value


def settings(args):
    allowed = {'machine', 'layering', 'base_radii_mm', 'buildup_factor', 'find_radii',
               'find_buildup_factor', 'optimizer', 'budgets', 'collection', 'artifact_dir'}
    if not isinstance(args, dict) or set(args) - allowed:
        raise ValueError(f'Autocal settings must be among {sorted(allowed)}')
    result = {'machine': 'hp3', 'layering': False, 'base_radii_mm': [30.0], 'buildup_factor': 0.0,
              'find_radii': 'off', 'find_buildup_factor': 'off', **{k: v for k, v in args.items()
              if k not in ('optimizer', 'budgets', 'collection', 'artifact_dir')}}
    if result['machine'] not in ('hp3', 'hp4'):
        raise ValueError('machine must be hp3 or hp4')
    if type(result['layering']) is not bool:
        raise ValueError('layering must be boolean')
    radii = result['base_radii_mm']
    if not isinstance(radii, list) or len(radii) not in (1, 4):
        raise ValueError('base_radii_mm must contain one or four radii')
    for radius in radii:
        number('base_radii_mm', radius, 1, 200)
    number('buildup_factor', result['buildup_factor'], 0, 2)
    for key in ('find_radii', 'find_buildup_factor'):
        if result[key] not in ('off', 'global', 'per-anchor'):
            raise ValueError(f'{key} must be off, global or per-anchor')
    if not result['layering'] and (result['buildup_factor'] != 0 or result['find_buildup_factor'] != 'off'):
        raise ValueError('Nonzero or fitted buildup requires layering')
    optimizer = dict(mode='fast', solver='lbfgsb', solve_iterations=400, solve_restarts=4,
                     spool_outer_iters=3, spool_inner_iters=30)
    supplied = args.get('optimizer', {})
    if not isinstance(supplied, dict) or set(supplied) - (set(optimizer) | {'r0_bounds_mm', 'b_bounds'}):
        raise ValueError('Unknown optimizer setting')
    optimizer.update(supplied)
    if optimizer['mode'] not in ('fast', 'fast-fd', 'legacy') or optimizer['solver'] not in ('lbfgsb', 'trf'):
        raise ValueError('Unknown optimizer mode or solver')
    for key, high in [('solve_iterations', 2000), ('solve_restarts', 16),
                      ('spool_outer_iters', 20), ('spool_inner_iters', 2000)]:
        number(key, optimizer[key], 1, high, integer=True)
    for key, low, high in [('r0_bounds_mm', 1, 200), ('b_bounds', 0, 2)]:
        if key in optimizer:
            bounds = optimizer[key]
            if not isinstance(bounds, list) or len(bounds) != 2:
                raise ValueError(f'{key} must contain two bounds')
            for value in bounds:
                number(key, value, low, high)
            if bounds[0] >= bounds[1]:
                raise ValueError(f'{key} must be increasing')
    budgets = dict(max_steps=20, sweep_points=20, wall_timeout_s=3600,
                   max_physics_steps=5_000_000, threads=1)
    supplied = args.get('budgets', {})
    if not isinstance(supplied, dict) or set(supplied) - set(budgets):
        raise ValueError('Unknown budget setting')
    budgets.update(supplied)
    for key, low, high in [('max_steps', 1, 100), ('sweep_points', 3, 100),
                           ('wall_timeout_s', 1, 14400), ('max_physics_steps', 1, 10_000_000), ('threads', 1, 8)]:
        number(key, budgets[key], low, high, integer=True)
    collection = args.get('collection', {})
    forces = {'force_low_n', 'force_mid_n', 'force_max_n'}
    if not isinstance(collection, dict) or set(collection) - (forces | {'sensor_collection_force_n'}):
        raise ValueError('Unknown collection setting')
    if forces & set(collection):
        if not forces <= set(collection):
            raise ValueError('Supply all three collection force settings together')
        for key in forces:
            number(key, collection[key], .001, 10)
        if not collection['force_low_n'] < collection['force_mid_n'] <= collection['force_max_n']:
            raise ValueError('Collection forces must satisfy low < mid <= max')
    if 'sensor_collection_force_n' in collection:
        number('sensor_collection_force_n', collection['sensor_collection_force_n'], .001, 10)
    result.update(optimizer=optimizer, budgets=budgets, collection=dict(collection))
    return result


def command(root, directory, config):
    opt, budget = config['optimizer'], config['budgets']
    argv = [str(root / '.venv/bin/python'), 'autocal/autocal.py', '--headless-sim', '--extended-reference',
            '--machine-type', f'hangprinter_{config["machine"][-1]}', '--dataset', str(directory / 'sweeps.json'),
            '--rrf-config', f'sys/config_{config["machine"]}{"_w_line_layers" if config["layering"] else ""}.g',
            '--base-radii', ','.join(map(str, config['base_radii_mm'])),
            '--buildup-factor', str(config['buildup_factor']), '--find-radii', config['find_radii'],
            '--find-buildup-factor', config['find_buildup_factor'], '--optimizer-mode', opt['mode'],
            '--solve-optimizer', opt['solver'], '--stage-artifacts', str(directory / 'sweeps.stages')]
    for key in ('solve_iterations', 'solve_restarts', 'spool_outer_iters', 'spool_inner_iters'):
        argv += ['--' + key.replace('_', '-'), str(opt[key])]
    for key, flag in [('r0_bounds_mm', '--r0-bounds'), ('b_bounds', '--b-bounds')]:
        if key in opt:
            argv += [flag, ','.join(map(str, opt[key]))]
    for key in ('max_steps', 'sweep_points'):
        argv += ['--' + key.replace('_', '-'), str(budget[key])]
    collector = ['--line-layering'] if config['layering'] else []
    for key, value in config['collection'].items():
        collector += ['--' + key.removesuffix('_n').replace('_', '-'), str(value)]
    if 'force_low_n' in config['collection']:
        collector += ['--no-auto-tune-force']
    if collector:
        argv += ['--collector-args', *collector]
    return argv


def read_json(path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None


class AutocalJobs:
    def __init__(self, root, directory):
        self.root, self.directory = Path(root), Path(directory).resolve()
        self.jobs = {}
        self.active = None
        self.source_hashes = {str(path.relative_to(self.root)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in (self.root / 'scripts/research_runtime.py', Path(__file__))}

    def start(self, args):
        config = settings(args)
        if self.active is not None:
            raise ValueError('A full-autocal job is already active')
        job_id = uuid.uuid4().hex
        requested = args.get('artifact_dir')
        if requested is not None and (not isinstance(requested, str) or not requested.strip()):
            raise ValueError('artifact_dir must be a path within the session artifacts')
        directory = (self.directory / (requested or f'autocal/{job_id}')).resolve()
        if not directory.is_relative_to(self.directory) or directory == self.directory:
            raise ValueError('artifact_dir must be within the session artifacts')
        directory.mkdir(parents=True, exist_ok=False)
        job = dict(job_id=job_id, kind='full_autocal', status='running', settings=config,
                   directory=str(directory), command=command(self.root, directory, config),
                   started_unix_s=time.time(), stop_reason=None, exit_code=None)
        job['supervisor_source_sha256'] = self.source_hashes
        self.jobs[job_id] = job
        self.active = job_id
        write_json(directory / 'job.json', job)
        job['task'] = asyncio.create_task(self.run(job))
        return self.status(job_id)

    def status(self, job_id):
        if job_id not in self.jobs:
            raise ValueError('Unknown full-autocal job')
        job = self.jobs[job_id]
        result = {key: value for key, value in job.items() if key not in ('task', 'process', 'terminate_task')}
        directory = Path(job['directory'])
        headless = read_json(directory / 'sweeps.headless/manifest.json')
        if headless:
            result['headless'] = headless
        clock = read_json(directory / 'sweeps.headless/clock.json') or {}
        progress = {key: clock[key] for key in ('step', 'epoch', 'total_steps', 'simulated_s', 'queue_length') if key in clock}
        log = directory / 'sweeps.full_auto_log.jsonl'
        if log.exists():
            # A row can be partially written while fitting/collection continues.
            for line in reversed(log.read_text().splitlines()):
                try:
                    row = json.loads(line)
                except ValueError:
                    continue
                progress.update({key: row[key] for key in ('iteration', 'decision', 'selected_run', 'best_iteration', 'best_score_ui') if key in row})
                break
        journals = list(directory.glob('*.partial-points.jsonl'))
        if journals:
            progress['collected_points'] = sum(len(path.read_text().splitlines()) for path in journals)
        text_log = directory / 'sweeps.full_auto.log'
        if text_log.exists():
            with text_log.open('rb') as stream:
                stream.seek(max(0, text_log.stat().st_size - 4096))
                lines = stream.read().decode(errors='replace').splitlines()
            if lines:
                progress['latest_log_line'] = lines[-1]
        result['progress'] = progress
        result['artifacts'] = {str(path.relative_to(directory)): str(path) for path in sorted(directory.rglob('*'))
                               if path.is_file() and not path.is_symlink()}
        return result

    async def terminate(self, job):
        if 'terminate_task' not in job:
            job['terminate_task'] = asyncio.create_task(self.stop_owner(job))
        await asyncio.shield(job['terminate_task'])

    async def stop_owner(self, job):
        process = job.get('process')
        if process is not None and process.poll() is None:
            # Signal only the owner first so it holds/stops collection and drains
            # physics before retiring the recorder. All children share its group.
            process.send_signal(signal.SIGTERM)
            try:
                await asyncio.to_thread(process.wait, timeout=75)
            except subprocess.TimeoutExpired:
                job['cleanup_error'] = 'CLI did not drain within 75 seconds'
                process.kill()
                await asyncio.to_thread(process.wait)

    async def cancel(self, job_id):
        self.status(job_id)  # validate before mutating
        job = self.jobs[job_id]
        if job['status'] in ('running', 'cancelling'):
            job.update(status='cancelling', stop_reason=job['stop_reason'] or 'cancelled')
            await self.terminate(job)
            await asyncio.shield(job['task'])
        return self.status(job_id)

    async def close(self):
        if self.active is not None:
            await self.cancel(self.active)

    async def retire_children(self, job):
        process = job.get('process')
        if process is None:
            return
        directory = Path(job['directory']) / 'sweeps.headless'
        manifest = read_json(directory / 'manifest.json') or {}
        # On an owner crash, drain physics before the recorder; signalling the
        # whole group at once can lose the final physical boundary and RRD footer.
        children = manifest.get('owned_services', [])
        order = {'collector': 0, 'physics': 1, 'rrf': 2, 'recorder': 3}
        for child in sorted(children, key=lambda child: order[child['name']]):
            pid = child['pid']
            try:
                if os.getpgid(pid) != process.pid:
                    continue
                os.kill(pid, signal.SIGTERM)
                timeout = {'collector': 5, 'physics': 15, 'rrf': 5, 'recorder': 45}[child['name']]
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    stat = Path(f'/proc/{pid}/stat')
                    if not stat.exists() or stat.read_text().split(') ')[1].split()[0] == 'Z':
                        break
                    await asyncio.sleep(.05)
                else:
                    os.kill(pid, signal.SIGKILL)
                    job['cleanup_error'] = f'{child["name"]} did not drain before forced retirement'
            except ProcessLookupError:
                pass
            except FileNotFoundError:
                pass
        try:
            os.killpg(process.pid, signal.SIGTERM)
            await asyncio.sleep(.5)
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        if manifest and manifest.get('status') in ('starting', 'running'):
            manifest.update(status='interrupted' if job['stop_reason'] == 'cancelled' else 'failed',
                            normal_completion=False, finalized_by_supervisor=True)
            manifest.update(read_json(directory / 'clock.json') or {})
            manifest['recordings'] = [read_json(path) for path in directory.glob('hangprinter-*.json')]
            write_json(directory / 'manifest.json', manifest)

    async def run(self, job):
        directory = Path(job['directory'])
        budget = job['settings']['budgets']
        # Do not inherit supervisor credentials, external recorder/config targets,
        # API keys or caller-controlled autocal overrides into a fresh job.
        env = {key: os.environ[key] for key in ('PATH', 'HOME', 'LANG', 'LD_LIBRARY_PATH') if key in os.environ}
        env.update(PYTHONUNBUFFERED='1', MPLCONFIGDIR=str(directory / 'matplotlib'),
                   HP_SIM5_AUTOCAL_JOB='1', HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS=str(budget['max_physics_steps']))
        for key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
            env[key] = str(budget['threads'])
        try:
            if job['status'] != 'cancelling':
                with (directory / 'stdout.log').open('w') as stdout, (directory / 'stderr.log').open('w') as stderr:
                    process = job['process'] = subprocess.Popen(job['command'], cwd=self.root, env=env,
                        stdin=subprocess.DEVNULL, stdout=stdout, stderr=stderr, start_new_session=True)
                job['pid'] = process.pid
                wait = asyncio.create_task(asyncio.to_thread(process.wait))
                try:
                    await asyncio.wait_for(asyncio.shield(wait), budget['wall_timeout_s'])
                except asyncio.TimeoutError:
                    if job['status'] != 'cancelling':
                        job.update(status='cancelling', stop_reason='wall-time-budget')
                    await self.terminate(job)
                    await wait
                job['exit_code'] = process.returncode
        except Exception as error:
            job['error'] = str(error)
        finally:
            process = job.get('process')
            if process is not None:
                await self.terminate(job)
                await self.retire_children(job)
            headless = read_json(directory / 'sweeps.headless/manifest.json') or {}
            reason = job['stop_reason'] or headless.get('stop_reason')
            clock = read_json(directory / 'sweeps.headless/clock.json') or {}
            if clock.get('error') == 'Physics step budget exhausted':
                reason = job['stop_reason'] or 'physics-step-budget'
            if reason == 'cancelled':
                status = 'cancelled'
            elif reason in ('wall-time-budget', 'physics-step-budget') or (reason == 'max-steps' and job['exit_code'] == 0 and headless.get('status') == 'complete'):
                status = 'budget_exhausted'
            elif job['exit_code'] == 0 and headless.get('normal_completion') and not job.get('cleanup_error'):
                status = 'accepted'
            else:
                status = 'failed'
                reason = reason or 'cli-failure'
            job.update(status=status, stop_reason=reason, ended_unix_s=time.time(),
                       wall_s=time.time() - job['started_unix_s'])
            write_json(directory / 'job.json', self.status(job['job_id']))
            self.active = None
