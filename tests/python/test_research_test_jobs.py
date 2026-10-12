import asyncio
import json
import os
from pathlib import Path
import signal

import pytest

from hp_sim5_research.test_jobs import MAX_TIMEOUT_S, SUITES, TestJobs

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.asyncio
@pytest.mark.parametrize('args', [
    {}, {'suite': 'shell'}, {'suite': ['headless-autocal']}, {'suite': 'headless-autocal', 'argv': ['echo']},
    {'suite': 'headless-autocal', 'timeout_s': True}, {'suite': 'headless-autocal', 'timeout_s': 0},
    {'suite': 'headless-autocal', 'timeout_s': MAX_TIMEOUT_S + 1},
])
async def test_invalid_selection_creates_no_jobs(tmp_path, args):
    manager = TestJobs(ROOT, tmp_path)
    with pytest.raises(ValueError):
        manager.start(args)
    assert not manager.jobs and not list(tmp_path.iterdir())


@pytest.mark.asyncio
async def test_fixed_argv_isolation_and_cancel_before_spawn(tmp_path):
    manager = TestJobs(ROOT, tmp_path)
    job = manager.start({'suite': 'headless-autocal'})
    assert job['command'][:4] == [str(ROOT / '.venv/bin/python'), '-m', 'pytest', '-q']
    assert set(SUITES['headless-autocal']).issubset(job['command'])
    assert job['cwd'] == str(ROOT) and Path(job['directory']).is_relative_to(tmp_path)
    with pytest.raises(ValueError, match='already active'):
        manager.start({'suite': 'research-services'})
    cancelled = await manager.cancel(job['job_id'])
    assert cancelled['status'] == 'cancelled' and cancelled['outcome'] == 'blocked'
    assert cancelled['exit_code'] is None and manager.active is None
    assert json.loads(Path(cancelled['artifacts']['job.json']).read_text())['status'] == 'cancelled'
    assert await manager.cancel(job['job_id']) == manager.status(job['job_id'])


async def finished(manager, job):
    await asyncio.wait_for(manager.jobs[job['job_id']]['task'], 5)
    return manager.status(job['job_id'])


@pytest.mark.asyncio
@pytest.mark.parametrize('code,outcome', [(0, 'passed'), (1, 'failed'), (2, 'failed'), (4, 'blocked'), (5, 'blocked')])
async def test_exit_codes_preserve_logs_and_evidence(tmp_path, code, outcome):
    manager = TestJobs(ROOT, tmp_path)
    job = manager.start({'suite': 'research-services'})
    manager.jobs[job['job_id']]['command'] = [str(ROOT / '.venv/bin/python'), '-c',
        f'import sys; print("result"); print("stderr", file=sys.stderr); raise SystemExit({code})']
    result = await finished(manager, job)
    assert result['status'] == result['outcome'] == outcome
    assert result['exit_code'] == code and result['log_tail']['stdout.log'] == 'result\n'
    assert Path(result['artifacts']['stderr.log']).read_text() == 'stderr\n'
    assert manager.active is None


@pytest.mark.asyncio
async def test_spawn_failure_and_private_environment(tmp_path, monkeypatch):
    manager = TestJobs(ROOT, tmp_path)
    job = manager.start({'suite': 'research-services'})
    manager.jobs[job['job_id']]['command'] = ['/missing/interpreter']
    result = await finished(manager, job)
    assert result['status'] == 'blocked' and result['next_step']
    for key in ('HP_SIM5_RUNTIME_TOKEN', 'OPENAI_API_KEY', 'AUTOCAL_REFERENCE_WS', 'PYTEST_ADDOPTS'):
        monkeypatch.setenv(key, 'private-sentinel')
    next_job = manager.start({'suite': 'research-services'})
    manager.jobs[next_job['job_id']]['command'] = [str(ROOT / '.venv/bin/python'), '-c',
        'import os,json; print(json.dumps(dict(os.environ)))']
    result = await finished(manager, next_job)
    assert result['status'] == 'passed'
    assert 'private-sentinel' not in json.dumps(result)
    assert Path(job['directory']).exists()  # failed trial retained


@pytest.mark.asyncio
@pytest.mark.parametrize('stop', ['cancel', 'owner-exit'])
async def test_registered_detached_services_are_retired(tmp_path, stop):
    manager = TestJobs(ROOT, tmp_path)
    job = manager.start({'suite': 'research-services'})
    owned = manager.jobs[job['job_id']]
    pid_path = Path(job['directory']) / 'detached.pid'
    service_code = 'import signal,time; signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(30)'
    script = ('import subprocess,sys,time; '
              f'sys.path.insert(0,{str(ROOT / "src/python")!r}); '
              'from hp_sim5_research.services import register_test_process; '
              f'p=subprocess.Popen([sys.executable,"-c",{service_code!r}],start_new_session=True); '
              'register_test_process(p); '
              f'open({str(pid_path)!r},"w").write(str(p.pid)); '
              + ('time.sleep(.1)' if stop == 'owner-exit' else 'time.sleep(30)'))
    owned['command'] = [str(ROOT / '.venv/bin/python'), '-c', script]
    for _ in range(200):
        if pid_path.exists():
            break
        await asyncio.sleep(.01)
    assert pid_path.exists()
    if stop == 'cancel':
        await manager.cancel(job['job_id'])
    result = await finished(manager, job)
    assert result['status'] == ('cancelled' if stop == 'cancel' else 'passed')
    stat = Path(f'/proc/{pid_path.read_text()}/stat')
    for _ in range(100):
        if not stat.exists() or stat.read_text().split(') ')[1].split()[0] == 'Z':
            break
        await asyncio.sleep(.01)
    assert not stat.exists() or stat.read_text().split(') ')[1].split()[0] == 'Z'


@pytest.mark.asyncio
async def test_reaped_detached_leader_does_not_leave_its_grandchild(tmp_path):
    manager = TestJobs(ROOT, tmp_path)
    job = manager.start({'suite': 'research-services'})
    directory = Path(job['directory'])
    grandchild_pid = directory / 'grandchild.pid'
    leader_pid = directory / 'leader.pid'
    service = ('import subprocess,sys,time; '
               'p=subprocess.Popen([sys.executable,"-c","import time; time.sleep(30)"]); '
               f'open({str(grandchild_pid)!r},"w").write(str(p.pid)); time.sleep(.1)')
    script = (f'import subprocess,sys; sys.path.insert(0,{str(ROOT / "src/python")!r}); '
              'from hp_sim5_research.services import register_test_process; '
              f'p=subprocess.Popen([sys.executable,"-c",{service!r}],start_new_session=True); '
              'register_test_process(p); '
              f'open({str(leader_pid)!r},"w").write(str(p.pid)); p.wait()')
    manager.jobs[job['job_id']]['command'] = [str(ROOT / '.venv/bin/python'), '-c', script]
    try:
        result = await finished(manager, job)
        assert result['status'] == 'passed'
        assert not Path(f'/proc/{leader_pid.read_text()}/stat').exists()
        stat = Path(f'/proc/{grandchild_pid.read_text()}/stat')
        for _ in range(100):
            if not stat.exists() or stat.read_text().split(') ')[1].split()[0] == 'Z':
                break
            await asyncio.sleep(.01)
        assert not stat.exists() or stat.read_text().split(') ')[1].split()[0] == 'Z'
    finally:
        # The failing reproduction must also retire its known isolated group.
        if leader_pid.exists():
            try:
                os.killpg(int(leader_pid.read_text()), signal.SIGKILL)
            except ProcessLookupError:
                pass


@pytest.mark.asyncio
async def test_stale_registry_does_not_stop_an_unrelated_live_group(tmp_path):
    import subprocess
    unrelated = subprocess.Popen([str(ROOT / '.venv/bin/python'), '-c', 'import time; time.sleep(30)'],
                                 start_new_session=True)
    try:
        manager = TestJobs(ROOT, tmp_path)
        job = manager.start({'suite': 'research-services'})
        registry = Path(job['directory']) / 'process-groups.jsonl'
        registry.write_text(json.dumps({'pid': unrelated.pid, 'group': unrelated.pid, 'birth': 'stale'}) + '\n')
        manager.jobs[job['job_id']]['command'] = [str(ROOT / '.venv/bin/python'), '-c', 'pass']
        result = await finished(manager, job)
        assert result['status'] == 'passed'
        assert unrelated.poll() is None
    finally:
        unrelated.terminate()
        unrelated.wait(timeout=5)


@pytest.mark.asyncio
@pytest.mark.parametrize('stop', ['cancel', 'timeout', 'owner-exit', 'shutdown'])
async def test_cleanup_retires_stubborn_children_even_after_owner_exit(tmp_path, stop):
    manager = TestJobs(ROOT, tmp_path)
    job = manager.start({'suite': 'research-services', 'timeout_s': 1 if stop == 'timeout' else 30})
    owned = manager.jobs[job['job_id']]
    pid_path = Path(job['directory']) / 'child.pid'
    child_code = 'import signal,time; signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(30)'
    owned['command'] = [str(ROOT / '.venv/bin/python'), '-c',
        'import subprocess,sys,time,signal; signal.signal(signal.SIGTERM, signal.SIG_IGN); '
        f'p=subprocess.Popen([sys.executable,"-c",{child_code!r}]); '
        f'open({str(pid_path)!r},"w").write(str(p.pid)); '
        + ('time.sleep(.15)' if stop == 'owner-exit' else 'time.sleep(30)')]
    for _ in range(100):
        if pid_path.exists():
            break
        await asyncio.sleep(.01)
    assert pid_path.exists()
    if stop == 'cancel':
        await asyncio.gather(manager.cancel(job['job_id']), manager.cancel(job['job_id']))
    elif stop == 'shutdown':
        await manager.close()
    result = await finished(manager, job)
    assert result['status'] == ('failed' if stop == 'timeout' else 'passed' if stop == 'owner-exit' else 'cancelled')
    if stop == 'timeout':
        assert result['stop_reason'] == 'timeout' and result['exit_code'] == -signal.SIGKILL
    stat = Path(f'/proc/{pid_path.read_text()}/stat')
    assert not stat.exists() or stat.read_text().split(') ')[1].split()[0] == 'Z'
    with pytest.raises(ProcessLookupError):
        os.kill(owned['process'].pid, 0)
    assert manager.active is None and result['artifacts']['child.pid']


@pytest.mark.asyncio
@pytest.mark.parametrize('stop', ['shutdown', 'signal'])
async def test_live_supervisor_shutdown_retires_test_owned_services(tmp_path, stop):
    from hp_sim5_research.services import RuntimeService
    service = await asyncio.to_thread(RuntimeService(ROOT, tmp_path, record=False).start)
    try:
        before = await asyncio.to_thread(service.call, 'status')
        job = await asyncio.to_thread(service.call, 'start_test', suite='headless-autocal')
        registry = Path(job['directory']) / 'process-groups.jsonl'
        owned = []
        for _ in range(300):
            if registry.exists():
                owned = [json.loads(line) for line in registry.read_text().splitlines()]
                if len(owned) >= 2:
                    break
            state = await asyncio.to_thread(service.call, 'test_status', job_id=job['job_id'])
            assert state['status'] == 'running', state['log_tail']
            await asyncio.sleep(.05)
        assert len(owned) >= 2, 'The socket-dependent suite must have started real detached services'
        if stop == 'shutdown':
            await asyncio.to_thread(service.close)
        else:
            service.process.send_signal(signal.SIGTERM)
            await asyncio.to_thread(service.process.wait, timeout=20)
        assert service.process.returncode == 0
        final = json.loads(Path(job['directory']).joinpath('job.json').read_text())
        assert final['status'] == 'cancelled' and final['outcome'] == 'blocked'
        assert Path(job['directory']).joinpath('stdout.log').exists()
        for entry in owned:
            stat = Path(f'/proc/{entry["pid"]}/stat')
            assert not stat.exists() or stat.read_text().split(') ')[1].split()[0] == 'Z'
        archived = json.loads(Path(before['artifacts']['world_manifest']).read_text())
        assert archived['end_step'] == before['step'] == 0
    finally:
        await asyncio.to_thread(service.close)
