import asyncio
from io import BytesIO
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from hp_sim5_research import preflight as pf
from hp_sim5_research.configuration import world_configuration
from hp_sim5_research.services import supervised_preflight
from scripts import research_socket_probe
from scripts.research_runtime import Runtime

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.asyncio
async def test_non_json_runtime_result_returns_http_error_and_keeps_service_usable():
    from hp_sim5_research.services import request
    runtime = Runtime.__new__(Runtime)
    runtime.token = 'fixture-token'
    runtime.handlers = set()

    async def dispatch(operation, args):
        return {'score': float('inf')} if operation == 'broken' else {'ok': True}

    runtime.dispatch = dispatch
    server = await asyncio.start_server(runtime.connection, '127.0.0.1', 0)
    endpoint = f'http://127.0.0.1:{server.sockets[0].getsockname()[1]}'
    try:
        with pytest.raises(RuntimeError, match='JSON compliant'):
            await asyncio.to_thread(request, endpoint, 'broken', token=runtime.token, timeout=2)
        assert await asyncio.to_thread(request, endpoint, 'healthy', token=runtime.token, timeout=2) == {'ok': True}
    finally:
        server.close()
        await server.wait_closed()


@pytest.fixture
def runtime(tmp_path, monkeypatch):
    config = world_configuration(ROOT, {'machine': 'hp3', 'layering': False}, backend='headless-js')
    firmware = {'M115': 'RepRapFirmware', 'M669': 'Kinematics is Hangprinter\n'
                'A:0.00, -1900.00, -280.00\nB:1645.45, 950.00, -280.00\n'
                'C:-1645.45, 950.00, -280.00\nD:0.00, 0.00, 1900.00', 'M666': 'M666 A1'}
    state = dict(world_id='retained-world', generation=3, step=42,
                 accepted_settings=config.effective(), active_job='retained-collection',
                 active_autocal_job='retained-autocal', reset_required=False, connected=True,
                 services={'rrf': {'pid': 123, 'exit_code': None}}, source_changed={},
                 world_configuration={'effective_settings': config.effective()})
    runtime = SimpleNamespace(directory=tmp_path, configuration=config, status=lambda: state,
                              collector_url='http://collector', rrf_url='http://firmware', token='private-test-secret')
    monkeypatch.setattr(pf, 'request', lambda *args, **kwargs: {'connected': True, 'busy': True, 'firmware_initial': firmware})
    def object_model(url, **kwargs):
        result = ({'name': 'Hangprinter', 'anchors': [[0, -1900, -280], [1645.4, 950, -280],
                   [-1645.4, 950, -280], [0, 0, 1900]]} if 'key=move.kinematics' in url else {'status': 'idle'})
        return BytesIO(json.dumps({'result': result}).encode())
    monkeypatch.setattr(pf, 'urlopen', object_model)

    async def recorder(*args):
        return {'outcome': 'passed', 'kind': 'tool_check', 'handshake': True, 'finalized': True, 'readable': True}

    monkeypatch.setattr(pf, 'recorder_probe', recorder)
    return runtime


@pytest.mark.asyncio
async def test_healthy_preflight_leaves_active_world_and_command_network_unknown(runtime):
    before = json.dumps(runtime.status())
    result = await pf.preflight(runtime, ROOT, {'machine': 'hp3', 'layering': False})
    assert result['outcome'] == 'passed'
    assert result['command_network']['outcome'] == 'unknown'
    assert result['world']['step'] == 42 and result['world']['active_job'] == 'retained-collection'
    assert result['checks']['collector']['busy']
    assert result['checks']['capabilities']['test_suites']
    assert 'private-test-secret' not in json.dumps(result)
    assert json.dumps(runtime.status()) == before
    assert json.loads(Path(result['manifest']).read_text())['outcome'] == 'passed'


@pytest.mark.asyncio
@pytest.mark.parametrize('failure', ['world', 'collector', 'firmware', 'recorder', 'settings', 'stale', 'dependencies'])
async def test_blocked_preflight_preserves_each_actionable_failure(runtime, monkeypatch, failure):
    if failure == 'world':
        runtime.status()['reset_required'] = True
        runtime.status()['connected'] = False
    elif failure == 'collector':
        monkeypatch.setattr(pf, 'request', lambda *args, **kwargs: {'connected': False})
    elif failure == 'firmware':
        def disconnected(*args, **kwargs):
            raise ConnectionRefusedError('Connection refused')
        monkeypatch.setattr(pf, 'urlopen', disconnected)
    elif failure == 'recorder':
        async def failed(*args):
            return pf.blocked('No recorder acknowledgment', 'Inspect recorder.log')
        monkeypatch.setattr(pf, 'recorder_probe', failed)
    elif failure == 'stale':
        runtime.status()['source_changed']['python_requires_launcher_restart'] = True
    elif failure == 'dependencies':
        monkeypatch.setattr(pf.shutil, 'which', lambda name: None)
    expected = {'machine': 'hp4'} if failure == 'settings' else None
    before = json.dumps(runtime.status())
    result = await pf.preflight(runtime, ROOT, expected)
    name = {'world': 'services', 'stale': 'settings', 'dependencies': 'capabilities'}.get(failure, failure)
    assert result['outcome'] == result['checks'][name]['outcome'] == 'blocked'
    assert result['checks'][name]['next_step']
    assert json.dumps(runtime.status()) == before


@pytest.mark.asyncio
async def test_preflight_total_timeout_is_bounded(runtime, monkeypatch):
    async def hang(*args):
        await asyncio.sleep(10)
    monkeypatch.setattr(pf, 'recorder_probe', hang)
    monkeypatch.setattr(pf, 'PREFLIGHT_TIMEOUT_S', .02)
    result = await pf.preflight(runtime, ROOT)
    assert result['outcome'] == 'blocked' and result['timeout']
    assert result['wall_s'] < 1


@pytest.mark.asyncio
@pytest.mark.parametrize('live', [
    None, {'name': 'cartesian'}, {'name': 'hangprinter', 'anchors': [[1, 2, 3]] * 4},
    {'name': 'Hangprinter', 'anchors': None},
    {'name': 'Hangprinter', 'anchors': [[float('nan'), 0, 0]] * 4},
])
async def test_preflight_rejects_live_firmware_drift_despite_cached_identity(runtime, monkeypatch, live):
    def object_model(url, **kwargs):
        return BytesIO(json.dumps({'result': live if 'key=move.kinematics' in url else {'status': 'idle'}}).encode())
    monkeypatch.setattr(pf, 'urlopen', object_model)
    result = await pf.preflight(runtime, ROOT)
    assert result['checks']['collector']['outcome'] == 'passed'
    assert result['outcome'] == result['checks']['firmware']['outcome'] == 'blocked'
    assert result['checks']['firmware']['next_step']
    assert runtime.status()['step'] == 42


def test_disconnected_attachment_reports_unknown_command_network(monkeypatch):
    monkeypatch.delenv('HP_SIM5_RUNTIME_URL', raising=False)
    assert supervised_preflight()['outcome'] == 'blocked'
    monkeypatch.setenv('HP_SIM5_RUNTIME_URL', 'http://127.0.0.1:1')
    from hp_sim5_research import services
    def disconnected(*args, **kwargs):
        raise ConnectionRefusedError('Supervisor disconnected')
    monkeypatch.setattr(services, 'request', disconnected)
    result = supervised_preflight()
    assert result['outcome'] == 'blocked'
    assert result['checks']['supervisor']['error'] == 'Supervisor disconnected'
    assert result['command_network']['outcome'] == 'unknown'


def test_command_probe_reports_actual_socket_denial_and_skips_http(monkeypatch):
    def denied():
        raise PermissionError('Operation not permitted')
    monkeypatch.setattr(research_socket_probe.socket, 'socket', denied)
    result = research_socket_probe.probe()
    assert result['context'] == 'agent_command' and result['outcome'] == 'blocked'
    assert result['checks']['socket_creation']['outcome'] == 'blocked'
    assert result['checks']['http_round_trip']['outcome'] == 'unknown'


@pytest.mark.asyncio
async def test_runtime_preflight_does_not_dispatch_motion_and_preserves_jobs(tmp_path, monkeypatch):
    monkeypatch.setenv('HP_SIM5_RUNTIME_TOKEN', 'private-test-secret')
    runtime = Runtime('http://127.0.0.1:1', tmp_path)
    runtime.active_job, runtime.autocal.active = 'collection', 'autocal'
    async def check(attached, root, expected):
        assert attached.active_job == 'collection' and attached.autocal.active == 'autocal'
        with pytest.raises(ValueError, match='in progress'):
            await attached.dispatch('preflight', {})
        return {'outcome': 'passed'}
    monkeypatch.setattr('scripts.research_runtime.preflight', check)
    assert await runtime.dispatch('preflight', {}) == {'outcome': 'passed'}
    assert runtime.active_job == 'collection' and runtime.autocal.active == 'autocal'


@pytest.mark.asyncio
async def test_recorder_probe_timeout_retires_group(tmp_path, monkeypatch):
    import subprocess
    from hp_sim5_research.test_jobs import retire_group
    process = None
    original = subprocess.Popen
    def spawn(*args, **kwargs):
        nonlocal process
        process = original([str(ROOT / '.venv/bin/python'), '-c', 'import time; time.sleep(30)'], **kwargs)
        return process
    monkeypatch.setattr(pf.subprocess, 'Popen', spawn)
    # Cancellation exercises the same finally path as the overall preflight timeout.
    with pytest.raises(TimeoutError):
        await asyncio.wait_for(pf.recorder_probe(ROOT, tmp_path), .2)
    assert process.poll() is not None
    await retire_group(process)


@pytest.mark.asyncio
async def test_live_mcp_preflight_test_cancel_timeout_and_disconnection(tmp_path):
    from mcp import Client, StdioServerParameters
    from hp_sim5_research.services import RuntimeService
    service = await asyncio.to_thread(RuntimeService(ROOT, tmp_path, record=False).start)
    params = StdioServerParameters(command=str(ROOT / '.venv/bin/python'),
        args=[str(ROOT / 'scripts/hp_sim5_mcp.py')],
        env={'HP_SIM5_RUNTIME_URL': service.endpoint, 'HP_SIM5_RUNTIME_TOKEN': service.token})
    try:
        async with Client(params, read_timeout_seconds=30) as client:
            async def call(name, args=None):
                result = await client.call_tool(name, args or {})
                assert not result.is_error, result.content
                return result.structured_content
            before = await call('runtime_status')
            checked = await call('research_preflight')
            assert checked['outcome'] == 'passed' and checked['checks']['recorder']['readable']
            assert checked['command_network']['outcome'] == 'unknown'
            job = await call('start_test', {'suite': 'headless-autocal'})
            cancelled = await call('cancel_test', {'job_id': job['job_id']})
            assert cancelled['status'] == 'cancelled'
            job = await call('start_test', {'suite': 'headless-autocal', 'timeout_s': 1})
            for _ in range(100):
                state = await call('test_status', {'job_id': job['job_id']})
                if state['status'] != 'running':
                    break
                await asyncio.sleep(.05)
            assert state['status'] == 'failed' and state['stop_reason'] == 'timeout'
            after = await call('runtime_status')
            assert before['world_id'] == after['world_id'] and before['step'] == after['step'] == 0
            # These configuration commands change only the isolated test firmware.
            await call('send_gcode', {'line': 'M669 A1:2:3'})
            changed_anchors = await call('research_preflight')
            assert changed_anchors['checks']['firmware']['outcome'] == 'blocked'
            assert 'anchors differ' in changed_anchors['checks']['firmware']['error']
            await call('send_gcode', {'line': 'M669 K0'})
            wrong_kinematics = await call('research_preflight')
            assert wrong_kinematics['checks']['firmware']['outcome'] == 'blocked'
            assert 'not Hangprinter' in wrong_kinematics['checks']['firmware']['error']
            unchanged = await call('runtime_status')
            assert before['world_id'] == unchanged['world_id'] and unchanged['step'] == 0
            # Connectivity failure is a structured blocked check, not a successful capability claim.
            await asyncio.to_thread(service.close)
            disconnected = await call('research_preflight')
            assert disconnected['outcome'] == 'blocked'
    finally:
        await asyncio.to_thread(service.close)
