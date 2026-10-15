import asyncio
import json
import os
from pathlib import Path
import signal
import sys

import pytest

from hp_sim5_research.autocal_jobs import AutocalJobs, command, settings

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize('args', [
    {'machine': 'hp5'}, {'layering': 1}, {'buildup_factor': .6},
    {'base_radii_mm': [float('nan')]}, {'base_radii_mm': [30, 30]},
    {'optimizer': {'mode': 'shell'}}, {'optimizer': {'r0_bounds_mm': [40, 30]}},
    {'budgets': {'max_steps': True}}, {'budgets': {'wall_timeout_s': 14401}},
    {'budgets': {'threads': 0}}, {'collector_args': ['--server', 'external']},
    {'collection': {'force_max_n': 9.2}},
    {'collection': {'force_low_n': .1, 'force_mid_n': .2, 'force_max_n': 11}},
    {'collection': {'sensor_collection_force_n': float('inf')}},
    {'recording_mode': 'debug'}, {'recording_mode': None}, {'recording_mode': True},
])
def test_autocal_rejects_unbounded_or_untyped_settings(args):
    with pytest.raises(ValueError):
        settings(args)


def test_autocal_command_runs_one_production_cli_with_explicit_configuration(tmp_path):
    cfg = settings({'machine': 'hp3', 'layering': True, 'buildup_factor': .6,
                    'find_radii': 'global', 'optimizer': {'r0_bounds_mm': [39, 40]}})
    argv = command(ROOT, tmp_path, cfg)
    assert argv[:2] == [str(ROOT / '.venv/bin/python'), 'autocal/autocal.py']
    assert argv[argv.index('--rrf-config') + 1] == 'sys/config_hp3_w_line_layers.g'
    assert argv[-2:] == ['--collector-args', '--line-layering']
    assert '--no-collect' not in argv and '--keep-sim-alive' not in argv
    cfg = settings({'collection': {'force_low_n': .01, 'force_mid_n': .23, 'force_max_n': 9.2}})
    assert '--no-auto-tune-force' in command(ROOT, tmp_path, cfg)


@pytest.mark.parametrize('mode', ['none', 'events', 'sampled', 'full'])
def test_autocal_recording_mode_reaches_cli(tmp_path, mode):
    assert settings({})['recording_mode'] == 'events'
    cfg = settings({'recording_mode': mode})
    argv = command(ROOT, tmp_path, cfg)
    assert argv[argv.index('--recording-mode') + 1] == mode
    assert '--extended-reference' not in argv


@pytest.mark.asyncio
async def test_autocal_validates_new_artifact_directory_and_exclusive_job(tmp_path):
    manager = AutocalJobs(ROOT, tmp_path)
    for path in ('../escape', str(tmp_path.parent / 'outside'), '.', ''):
        with pytest.raises(ValueError):
            manager.start({'artifact_dir': path})
    (tmp_path / 'link').symlink_to(tmp_path.parent, target_is_directory=True)
    with pytest.raises(ValueError):
        manager.start({'artifact_dir': 'link/escape'})
    job = manager.start({})
    directory = Path(job['directory'])
    (directory / 'sweeps.json.partial-points.jsonl').write_text('{}\n{}\n')
    (directory / 'sweeps.new_001.json.partial-points.jsonl').write_text('{}\n')
    assert manager.status(job['job_id'])['progress']['collected_points'] == 3
    with pytest.raises(ValueError, match='already active'):
        manager.start({})
    cancelled = await manager.cancel(job['job_id'])  # before process startup
    assert cancelled['status'] == 'cancelled' and cancelled['exit_code'] is None
    assert manager.active is None
    assert json.loads(Path(cancelled['artifacts']['job.json']).read_text())['status'] == 'cancelled'
    assert (await manager.cancel(job['job_id']))['status'] == 'cancelled'


async def finished(manager, job):
    await asyncio.wait_for(manager.jobs[job['job_id']]['task'], 10)
    return manager.status(job['job_id'])


@pytest.mark.asyncio
@pytest.mark.parametrize('normal,reason,code,expected', [
    (True, 'patience-or-threshold', 0, 'accepted'),
    (True, 'spread-consensus', 0, 'accepted'),
    (False, 'max-steps', 0, 'budget_exhausted'),
    (False, 'max-steps', 1, 'failed'),
    (False, None, 0, 'failed'),
    (False, None, 2, 'failed'),
])
async def test_autocal_terminal_state_requires_acceptance_evidence(tmp_path, normal, reason, code, expected):
    manager = AutocalJobs(ROOT, tmp_path)
    job = manager.start({})
    owned = manager.jobs[job['job_id']]
    directory = Path(job['directory']) / 'sweeps.headless'
    directory.mkdir()
    (directory / 'manifest.json').write_text(json.dumps({'normal_completion': normal, 'stop_reason': reason, 'status': 'complete'}))
    # Only lifecycle classification is synthetic; real collection has a separate acceptance check.
    owned['command'] = [sys.executable, '-c', f'raise SystemExit({code})']
    result = await finished(manager, job)
    assert result['status'] == expected and result['exit_code'] == code


@pytest.mark.asyncio
async def test_autocal_spawn_failure_preserves_manifest_and_releases_slot(tmp_path):
    manager = AutocalJobs(ROOT, tmp_path)
    job = manager.start({})
    manager.jobs[job['job_id']]['command'] = ['/does/not/exist']
    result = await finished(manager, job)
    assert result['status'] == 'failed' and result['error']
    assert manager.active is None and Path(result['artifacts']['stderr.log']).exists()


@pytest.mark.asyncio
async def test_autocal_does_not_inherit_credentials_or_external_targets(tmp_path, monkeypatch):
    for key in ('HP_SIM5_RUNTIME_TOKEN', 'OPENAI_API_KEY', 'AUTOCAL_REFERENCE_WS', 'AUTOCAL_RRF_SIM_CONFIG'):
        monkeypatch.setenv(key, 'test-value-must-not-reach-the-job')
    manager = AutocalJobs(ROOT, tmp_path)
    job = manager.start({})
    manager.jobs[job['job_id']]['command'] = [sys.executable, '-c',
        'import os,json; print(json.dumps(dict(os.environ)))']
    result = await finished(manager, job)
    assert 'test-value-must-not-reach-the-job' not in Path(result['artifacts']['stdout.log']).read_text()
    assert 'test-value-must-not-reach-the-job' not in json.dumps(result)


@pytest.mark.asyncio
@pytest.mark.parametrize('stop', ['cancel', 'close', 'timeout', 'crash'])
async def test_autocal_retires_children_even_when_owner_crashes(tmp_path, stop):
    manager = AutocalJobs(ROOT, tmp_path)
    job = manager.start({'budgets': {'wall_timeout_s': 1 if stop == 'timeout' else 30}})
    owned = manager.jobs[job['job_id']]
    pid_file = Path(job['directory']) / 'child.pid'
    child_code = 'import signal,time; signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(60)'
    script = ('import subprocess,sys,time; '
              f'p=subprocess.Popen([sys.executable,"-c",{child_code!r}]); '
              f'open({str(pid_file)!r},"w").write(str(p.pid)); '
              + ('time.sleep(.1); raise SystemExit(1)' if stop == 'crash' else 'time.sleep(60)'))
    owned['command'] = [sys.executable, '-c', script]
    for _ in range(100):
        if pid_file.exists():
            break
        await asyncio.sleep(.01)
    assert pid_file.exists()
    if stop == 'cancel':
        # Concurrent cancels/shutdown must signal the owner once, including during drain.
        await asyncio.gather(manager.cancel(job['job_id']), manager.close())
    elif stop == 'close':
        await manager.close()
    result = await finished(manager, job)
    assert result['status'] == ('failed' if stop == 'crash' else 'budget_exhausted' if stop == 'timeout' else 'cancelled')
    child = Path(f'/proc/{pid_file.read_text()}/stat')
    for _ in range(100):
        if not child.exists() or child.read_text().split()[2] == 'Z':
            break
        await asyncio.sleep(.01)
    assert not child.exists() or child.read_text().split()[2] == 'Z'
    with pytest.raises(ProcessLookupError):
        os.kill(owned['process'].pid, 0)


def test_supervised_signal_does_not_accept_best_so_far(tmp_path, monkeypatch):
    from argparse import Namespace
    from autocal.headless_sim import HeadlessSimulation
    monkeypatch.setenv('HP_SIM5_AUTOCAL_JOB', '1')
    service = HeadlessSimulation(Namespace(machine_type='hp3', find_radii='off', find_buildup_factor='off',
        rrf_config=None, config=None, dataset=tmp_path / 'sweeps.json'), [])
    with pytest.raises(SystemExit) as error:
        service.interrupt(signal.SIGTERM, None)
    assert error.value.code == 130 and service.manifest['stop_reason'] == 'cancelled'


@pytest.mark.slow
@pytest.mark.asyncio
@pytest.mark.parametrize('stop', ['cancel', 'shutdown', 'signal-shutdown', 'crash', 'physics-budget'])
async def test_supervised_live_stop_finalizes_rrd_without_mutating_hp4(tmp_path, stop):
    from hp_sim5_research.services import RuntimeService
    from rerun.chunk import RrdReader
    service = await asyncio.to_thread(RuntimeService(ROOT, tmp_path, record=False).start)
    try:
        before = await asyncio.to_thread(service.call, 'status')
        job = await asyncio.to_thread(service.call, 'start_autocal', machine='hp3',
                                     recording_mode='full',
                                     budgets={'max_physics_steps': 1000} if stop == 'physics-budget' else {})
        for _ in range(1200):
            state = await asyncio.to_thread(service.call, 'autocal_status', job_id=job['job_id'])
            if state['progress'].get('total_steps', 0) >= 1000:
                break
            assert state['status'] == 'running', state
            await asyncio.sleep(.1)
        assert state['progress'].get('total_steps', 0) >= 1000
        if stop in ('shutdown', 'signal-shutdown'):
            if stop == 'signal-shutdown':
                service.process.send_signal(signal.SIGTERM)
                await asyncio.to_thread(service.process.wait, timeout=90)
            else:
                await asyncio.to_thread(service.close)
            result = json.loads(Path(job['directory']).joinpath('job.json').read_text())
        else:
            if stop == 'cancel':
                result = await asyncio.to_thread(service.call, 'cancel_autocal', job_id=job['job_id'])
            else:
                if stop == 'crash':
                    os.kill(state['pid'], signal.SIGKILL)
                for _ in range(600):
                    result = await asyncio.to_thread(service.call, 'autocal_status', job_id=job['job_id'])
                    if result['status'] not in ('running', 'cancelling'):
                        break
                    await asyncio.sleep(.1)
            assert result['status'] == ('failed' if stop == 'crash' else 'budget_exhausted' if stop == 'physics-budget' else 'cancelled')
        if stop in ('cancel', 'shutdown', 'signal-shutdown'):
            assert result['status'] == 'cancelled' and result['exit_code'] == 130
        headless = result['headless']
        assert headless['status'] == ('interrupted' if stop in ('cancel', 'shutdown', 'signal-shutdown') else 'failed')
        assert not headless['normal_completion']
        assert not headless.get('applied_parameters') and headless['recording_drained']
        recording, = headless['recordings']
        assert recording['status'] == 'finalized' and recording['physics_samples'] > 0
        assert not recording['autocal_complete']
        assert RrdReader(recording['rrd']).store()
        if stop not in ('shutdown', 'signal-shutdown'):
            after = await asyncio.to_thread(service.call, 'status')
            assert before['session_id'] == after['session_id']
            assert before['step'] == after['step'] == 0 and after['machine_design'] == 'hp4'
    finally:
        await asyncio.to_thread(service.close)
