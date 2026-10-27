"""Bounded, supervisor-owned integration suites; no caller-supplied commands."""
import asyncio
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

from .configuration import number
from .experiments import write_json
from .services import register_test_process


SUITES = {
    'headless-autocal': ('autocal/tests/test_headless_sim.py',
                         'tests/python/test_hangprinter_flight_recorder.py'),
    'research-services': ('tests/python/test_research_agent.py',
                          'tests/python/test_research_preflight.py',
                          'tests/python/test_research_test_jobs.py',
                          'tests/python/test_native_collector.py',
                          'tests/python/test_world_configuration.py',
                          'tests/python/test_autocal_jobs.py'),
}
MAX_TIMEOUT_S = 900


def job_environment(directory):
    # Never inherit service credentials or caller autocal/recorder overrides.
    env = {key: os.environ[key] for key in ('PATH', 'HOME', 'LANG', 'LD_LIBRARY_PATH') if key in os.environ}
    env.update(PYTHONUNBUFFERED='1', MPLCONFIGDIR=str(directory / 'matplotlib'),
               XDG_CACHE_HOME=str(directory / 'cache'), WARP_CACHE_PATH=str(directory / 'cache/warp'),
               OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1',
               MKL_NUM_THREADS='1', NUMEXPR_NUM_THREADS='1')
    env['HP_SIM5_TEST_PROCESS_REGISTRY'] = str(directory / 'process-groups.jsonl')
    # RRF's temporary VSD needs a short host path (<120 characters). pytest
    # artifacts are isolated with --basetemp; services archive their VSD inputs.
    return env


async def retire_group(process):
    """Retire descendants even when the group leader has already exited."""
    if process is None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
        await asyncio.sleep(.25)
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    await asyncio.to_thread(process.wait)


def registered_groups(directory):
    import json
    owned = set()
    markers = {}
    # A suite can itself exercise test jobs; each nested job owns its registry.
    for registry in directory.rglob('process-groups.jsonl'):
        for line in registry.read_text().splitlines():
            try:
                item = json.loads(line)
                markers.setdefault(item['group'], set()).add(
                    b'HP_SIM5_TEST_PROCESS_REGISTRY=' + os.fsencode(registry))
                pid = item['pid']
                birth = Path(f'/proc/{pid}/stat').read_text().split(') ')[1].split()[19]
                if birth == item['birth'] and os.getpgid(pid) == item['group']:
                    owned.add(item['group'])
            except (FileNotFoundError, ProcessLookupError, ValueError, KeyError, TypeError):
                continue
    # A detached group can survive its reaped leader. Verify a live member's
    # private job marker rather than trusting a stale/reused group ID.
    for stat in Path('/proc').glob('[0-9]*/stat'):
        try:
            fields = stat.read_text().split(') ')[1].split()
            group = int(fields[2])
            if group in markers and group not in owned:
                environment = set(stat.with_name('environ').read_bytes().split(b'\0'))
                if environment & markers[group]:
                    owned.add(group)
        except (OSError, ValueError, IndexError):
            continue
    return owned


async def retire_registered_groups(directory):
    for sig in (signal.SIGTERM, signal.SIGKILL):
        owned = registered_groups(directory)
        for pid in owned:
            try:
                os.killpg(pid, sig)
            except ProcessLookupError:
                pass
        if owned and sig == signal.SIGTERM:
            await asyncio.sleep(.25)


class TestJobs:
    __test__ = False

    def __init__(self, root, directory):
        self.root, self.directory = Path(root).resolve(), Path(directory).resolve()
        self.jobs = {}
        self.active = None

    def start(self, args):
        if not isinstance(args, dict) or set(args) - {'suite', 'timeout_s'}:
            raise ValueError('Supply only suite and timeout_s')
        suite = args.get('suite')
        if not isinstance(suite, str) or suite not in SUITES:
            raise ValueError(f'Select a named test suite: {", ".join(SUITES)}')
        timeout = number('timeout_s', args.get('timeout_s', 300), 1, MAX_TIMEOUT_S, integer=True)
        if self.active is not None:
            raise ValueError('A test job is already active; poll or cancel_test first')
        job_id = uuid.uuid4().hex
        directory = self.directory / 'tests' / job_id
        directory.mkdir(parents=True, exist_ok=False)
        argv = [str(self.root / '.venv/bin/python'), '-m', 'pytest', '-q', '-m', 'not slow',
                *SUITES[suite], '--basetemp', str(directory / 'pytest-temp'),
                '--junitxml', str(directory / 'junit.xml')]
        job = dict(job_id=job_id, kind='integration_tests', suite=suite, status='running',
                   outcome=None, stop_reason=None, exit_code=None, timeout_s=timeout,
                   started_unix_s=time.time(), directory=str(directory), command=argv, cwd=str(self.root))
        for name in ('stdout.log', 'stderr.log'):
            (directory / name).touch()
        self.jobs[job_id] = job
        self.active = job_id
        write_json(directory / 'job.json', job)
        job['task'] = asyncio.create_task(self.run(job))
        return self.status(job_id)

    def status(self, job_id):
        if not isinstance(job_id, str) or job_id not in self.jobs:
            raise ValueError('Unknown test job')
        job = self.jobs[job_id]
        result = {key: value for key, value in job.items() if key not in ('task', 'process')}
        directory = Path(job['directory'])
        result['artifacts'] = {str(path.relative_to(directory)): str(path) for path in sorted(directory.rglob('*'))
                               if path.is_file() and not path.is_symlink()}
        result['elapsed_s'] = job.get('wall_s', time.time() - job['started_unix_s'])
        result['log_tail'] = {}
        for name in ('stdout.log', 'stderr.log'):
            with (directory / name).open('rb') as stream:
                stream.seek(max(0, (directory / name).stat().st_size - 4096))
                result['log_tail'][name] = stream.read().decode(errors='replace')
        return result

    async def cancel(self, job_id):
        self.status(job_id)
        job = self.jobs[job_id]
        if job['status'] in ('running', 'cancelling'):
            job.update(status='cancelling', stop_reason='cancelled')
            process = job.get('process')
            if process is not None and process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
            await asyncio.shield(job['task'])
        return self.status(job_id)

    async def close(self):
        if self.active is not None:
            await self.cancel(self.active)

    async def run(self, job):
        started = time.monotonic()
        directory = Path(job['directory'])
        process = None
        try:
            if job['status'] != 'cancelling':
                with (directory / 'stdout.log').open('w') as stdout, (directory / 'stderr.log').open('w') as stderr:
                    process = job['process'] = subprocess.Popen(job['command'], cwd=self.root,
                        env=job_environment(directory), stdin=subprocess.DEVNULL,
                        stdout=stdout, stderr=stderr, start_new_session=True)
                job['pid'] = process.pid
                register_test_process(process)
                # Polling bounds cancellation even if children ignore SIGTERM.
                while process.poll() is None and job['status'] != 'cancelling':
                    if time.monotonic() - started >= job['timeout_s']:
                        job['stop_reason'] = 'timeout'
                        break
                    await asyncio.sleep(.05)
        except OSError as error:
            job.update(error=str(error), outcome='blocked', stop_reason='launch-unavailable',
                       next_step='Check .venv/bin/python and the named suite files in this checkout')
        finally:
            # Integration fixtures detach RuntimeService/RRF/recorder groups.
            # Retire those before their pytest owner, including on an owner crash.
            await retire_registered_groups(directory)
            await retire_group(process)
            await retire_registered_groups(directory)
            job['exit_code'] = process.returncode if process is not None else None
            reason = job['stop_reason']
            if reason == 'cancelled':
                job.update(status='cancelled', outcome='blocked')
            elif reason == 'timeout':
                job.update(status='failed', outcome='failed', next_step='Inspect preserved logs; choose a larger bounded timeout')
            elif job['outcome'] == 'blocked' or job['exit_code'] in (4, 5):
                job.update(status='blocked', outcome='blocked', stop_reason=reason or 'pytest-unavailable')
            else:
                outcome = 'passed' if job['exit_code'] == 0 else 'failed'
                job.update(status=outcome, outcome=outcome, stop_reason='pytest-exit')
            job.update(wall_s=time.monotonic() - started, ended_unix_s=time.time())
            self.active = None
            write_json(directory / 'job.json', {key: value for key, value in job.items() if key not in ('task', 'process')})
