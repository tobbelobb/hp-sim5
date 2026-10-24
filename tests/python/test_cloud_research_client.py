"""Process-level transport fixtures, not live cloud model or release evidence."""
import hashlib
import json
import os
from pathlib import Path
import runpy
import signal
import subprocess
import sys
import time

import pytest


ROOT = Path(__file__).resolve().parents[2]
BRIDGE = ROOT / 'scripts/cloud/cloud_client_bridge.py'


def wait_for(predicate):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(.02)
    raise AssertionError('Cloud transport fixture timed out')


@pytest.fixture
def client(tmp_path):
    root = tmp_path / 'repo'
    final = root / 'output/session/final-message.md'
    final.parent.mkdir(parents=True)
    queue = tmp_path / 'queue'
    environment = dict(os.environ, HP_SIM5_CLOUD_QUEUE=str(queue))
    prompt = tmp_path / 'prompt.md'
    prompt.write_text('Transport fixture only, no scientific assignment.')
    with prompt.open('rb') as stdin:
        process = subprocess.Popen(
            [sys.executable, str(BRIDGE), 'client', 'exec', '--cd', str(root),
             '--output-last-message', str(final), '-c', 'private-config-marker'],
            stdin=stdin, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, env=environment)
    request_path = wait_for(lambda: next(iter(queue.glob('*/request.json')), None))
    request = json.loads(request_path.read_text())

    def command(operation, actor='fixture-actor', check=True):
        return subprocess.run(
            [sys.executable, str(BRIDGE), operation, request['request_id'], '--agent-id', actor],
            capture_output=True, text=True, env=environment, timeout=5, check=check)

    try:
        yield process, request_path, final, command
    finally:
        if process.poll() is None:
            process.kill()
        process.wait(timeout=5)


def test_actor_completion_preserves_real_final_message_and_omits_private_flags(client):
    process, path, final, command = client
    assert 'private-config-marker' not in path.read_text()
    command('bind')
    final.write_text('Actual fixture actor output. No scientific result.')
    command('complete')
    stdout, stderr = process.communicate(timeout=5)
    assert process.returncode == 0, stderr
    events = [json.loads(line) for line in stdout.splitlines()]
    assert events[-1]['type'] == 'cloud_agent.completed'
    assert events[-1]['agent_id'] == 'fixture-actor'
    response = json.loads(path.with_name('response.json').read_text())
    assert response['final_sha256'] == hashlib.sha256(final.read_bytes()).hexdigest()
    assert final.read_text() == 'Actual fixture actor output. No scientific result.'


def test_other_actor_cannot_acknowledge_completion(client):
    process, path, final, command = client
    command('bind')
    final.write_text('Original actor output')
    rejected = command('complete', actor='another-actor', check=False)
    assert rejected.returncode != 0
    assert 'Only the bound cloud actor' in rejected.stderr
    assert not path.with_name('response.json').exists()
    assert process.poll() is None
    assert final.read_text() == 'Original actor output'


def test_changed_final_message_is_not_success(client):
    process, path, final, command = client
    command('bind')
    request = json.loads(path.read_text())
    final.write_text('Changed after the acknowledged output')
    path.with_name('response.json').write_text(json.dumps({
        'request_id': request['request_id'], 'agent_id': 'fixture-actor',
        'state': 'completed', 'returncode': 0,
        'final_sha256': hashlib.sha256(b'Previous output').hexdigest()}))
    stdout, stderr = process.communicate(timeout=5)
    assert process.returncode != 0
    assert 'Actual actor final message missing or changed' in stderr
    assert 'cloud_agent.completed' not in stdout


def test_cancel_waits_for_actor_ack_and_keeps_partial_evidence(client):
    process, path, final, command = client
    command('bind')
    partial = final.with_name('partial.md')
    partial.write_text('Retained partial fixture evidence')
    process.send_signal(signal.SIGTERM)
    wait_for(lambda: json.loads(path.read_text())['state'] == 'cancel_requested')
    assert process.poll() is None
    command('cancelled')
    process.communicate(timeout=5)
    assert process.returncode == 130
    assert partial.read_text() == 'Retained partial fixture evidence'
    assert not final.exists()


def test_wrapper_install_repeats_and_preserves_custom_content(tmp_path):
    install = runpy.run_path(str(ROOT / 'scripts/cloud/install_research_client.py'))['install']
    root = tmp_path / 'repo with spaces'
    python = root / '.venv/bin/python'
    python.parent.mkdir(parents=True)
    python.symlink_to(sys.executable)
    script = root / 'scripts/cloud/cloud_client_bridge.py'
    script.parent.mkdir(parents=True)
    script.write_text(BRIDGE.read_text())
    tools = tmp_path / 'tools with spaces'
    install(root, tools)
    install(root, tools)
    wrapper = tools / 'cloud-research-client/bin/codex'
    result = subprocess.run([str(wrapper), '--version'], capture_output=True, text=True, check=True)
    assert 'cloud-session transport' in result.stdout
    wrapper.write_text('User-maintained wrapper\n')
    with pytest.raises(SystemExit, match='Preserving modified'):
        install(root, tools)
    assert wrapper.read_text() == 'User-maintained wrapper\n'
