"""Real MCP/file transport checks, not authenticated-model or accuracy evidence."""
import importlib.util
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time

import pytest

from hp_sim5_research.services import RuntimeService

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('cloud_tool_bridge', ROOT / 'scripts/cloud/cloud_tool_bridge.py')
bridge = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bridge)


def until(predicate):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        result = predicate()
        if result:
            return result
        time.sleep(.02)
    pytest.fail('Tool relay did not reach its expected bounded state')


@pytest.fixture
def relay(tmp_path):
    session = tmp_path / 'session'
    session.mkdir()
    owner = tmp_path / 'request.json'
    bridge.write(owner, {'request_id': 'fixture-live-tools', 'state': 'running'})
    runtime = RuntimeService(ROOT, session / 'native', record=False, machine='hp3').start()
    bridge.write(session / 'connection.json', {'repo': str(ROOT), 'session_id': runtime.call('status')['session_id'],
                                              'runtime_endpoint': runtime.endpoint, 'runtime_token': runtime.token,
                                              'viewer_endpoint': None})
    process = None
    try:
        process, connection = bridge.start(ROOT, session, owner)
        yield process, connection, owner, runtime
    finally:
        bridge.stop(process)
        runtime.close()


def test_real_mcp_tools_work_without_sockets_in_requester(relay, monkeypatch):
    process, connection, owner, runtime = relay
    with monkeypatch.context() as patch:
        patch.setattr(socket, 'socket', lambda *a, **kw: pytest.fail('Requester attempted to create a socket'))
        tools = bridge.call(connection, 'hp_sim5', 'list_tools')
        assert len(tools) == 23 and any(t['name'] == 'run_experiment' for t in tools)
        status = bridge.call(connection, 'hp_sim5', 'call_tool', tool='runtime_status')['structuredContent']
        assert status['step'] == 0
        first = bridge.call(connection, 'hp_sim5', 'call_tool', tool='run_experiment',
                            arguments={'steps': 3, 'record': False})['structuredContent']
        assert first['status'] == 'complete'
        readback = bridge.call(connection, 'hp_sim5', 'call_tool', tool='read_experiment',
                               arguments={'run_id': first['run_id'], 'start_step': 0, 'limit': 4})['structuredContent']
        assert len(readback['samples']) == 4
        assert process.poll() is None
    bridge.stop(process)
    assert process.poll() is not None
    assert runtime.call('status')['step'] == 0
    assert Path(bridge.read(connection)['directory'], 'closed.json').is_file()


def test_unknown_and_expired_requests_do_not_call_tools(relay):
    process, connection, owner, runtime = relay
    descriptor = bridge.read(connection)
    directory = Path(descriptor['directory'])
    for name, overrides in [('stale', {'request_id': 'old-generation'}),
                            ('expired', {'expires': time.time() - 1}),
                            ('server', {'server': 'arbitrary-shell'})]:
        envelope = {'request_id': descriptor['request_id'], 'expires': time.time() + 5,
                    'operation': 'call_tool', 'server': 'hp_sim5', 'tool': 'step_physics', 'arguments': {'steps': 100}}
        envelope.update(overrides)
        bridge.write(directory / (name + '.request'), envelope)
        response = directory / (name + '.response')
        until(response.is_file)
        assert not bridge.read(response)['ok']
    assert runtime.call('status')['step'] == 0
    with pytest.raises(ValueError, match='hp_sim5 or rerun'):
        bridge.call(connection, 'shell', 'list_tools')
    bridge.write(owner, {'request_id': descriptor['request_id'], 'state': 'completed'})
    with pytest.raises(RuntimeError, match='has ended'):
        bridge.call(connection, 'hp_sim5', 'list_tools')
    until(lambda: process.poll() is not None)


def test_unanswered_call_is_bounded_and_not_retried(tmp_path):
    owner = tmp_path / 'request.json'
    bridge.write(owner, {'request_id': 'fixture-no-server', 'state': 'running'})
    directory = tmp_path / 'queue'
    directory.mkdir()
    connection = tmp_path / 'connection.json'
    bridge.write(connection, {'version': 1, 'request_id': 'fixture-no-server',
                              'owner_request': str(owner), 'directory': str(directory)})
    with pytest.raises(TimeoutError, match='outcome may be unknown'):
        bridge.call(connection, 'hp_sim5', 'call_tool', tool='step_physics', arguments={'steps': 1}, timeout=.03)
    assert not list(directory.iterdir())


def test_reviewer_has_no_tool_attachment(tmp_path):
    process, connection = bridge.start(ROOT, tmp_path, tmp_path / 'unused-owner.json')
    assert process is None and connection is None


def test_hard_bridge_owner_loss_closes_real_mcp_relay(relay, tmp_path):
    _, connection, _, runtime = relay
    session = Path(bridge.read(connection)['session'])
    queue = tmp_path / 'model-queue'
    process = subprocess.Popen([sys.executable, str(ROOT / 'scripts/cloud/cloud_client_bridge.py'), 'client',
                                '--cd', str(ROOT), '--session-dir', str(session),
                                '--output-last-message', str(session / 'final-message.md')],
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                               text=True, env=dict(os.environ, HP_SIM5_CLOUD_QUEUE=str(queue)))
    process.stdin.write('Lifecycle fixture, no authenticated model or scientific result.')
    process.stdin.close()
    try:
        def attached():
            for path in queue.glob('*/request.json'):
                request = bridge.read(path)
                if request.get('tool_connection_path'):
                    return path, request
        path, request = until(attached)
        tool_connection = Path(request['tool_connection_path'])
        assert len(bridge.call(tool_connection, 'hp_sim5', 'list_tools')) == 23
        process.kill()
        process.wait(timeout=5)
        descriptor = bridge.read(tool_connection)
        until(lambda: Path(descriptor['directory'], 'closed.json').exists())
        from hp_sim5_research.program_workers import process_live
        until(lambda: not process_live(descriptor['relay_process']))
        assert runtime.call('status')['step'] == 0
    finally:
        if process.poll() is None:
            process.kill()
        process.wait(timeout=5)


@pytest.mark.skipif(not os.environ.get('HP_SIM5_TEST_RERUN_VIEWER'), reason='Requires an installed headless graphics driver')
def test_real_rerun_mcp_uses_compatible_handshake(relay):
    _, connection, _, _ = relay
    session = Path(bridge.read(connection)['session'])
    descriptor = session / 'connection.json'
    spec = importlib.util.spec_from_file_location('viewer_test_launcher', ROOT / 'scripts/research_agent.py')
    launcher = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launcher)
    viewer, endpoint = launcher.start_viewer('headless', session)
    try:
        settings = bridge.read(descriptor)
        settings['viewer_endpoint'] = endpoint
        bridge.write(descriptor, settings)
        tools = bridge.call(connection, 'rerun', 'list_tools')
        assert {'rerun_get_viewer_state', 'rerun_save_screenshot'}.issubset({t['name'] for t in tools})
        result = bridge.call(connection, 'rerun', 'call_tool', tool='rerun_get_viewer_state')
        assert not result.get('isError')
    finally:
        launcher.stop_process(viewer)
