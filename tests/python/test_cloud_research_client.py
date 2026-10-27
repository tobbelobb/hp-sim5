"""Process-level transport fixtures, not live cloud model or release evidence."""
import hashlib
import json
import os
from pathlib import Path
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
def client(tmp_path, request):
    root = tmp_path / 'repo'
    final = root / 'output/session/final-message.md'
    if getattr(request, 'param', None) in ('default-review', 'alternate-review'):
        store = '.research-programs' if request.param == 'default-review' else '.research-programs-custom'
        final = root / store / 'reviews' / ('review-run-' + 'a' * 32) / 'final-message.md'
    final.parent.mkdir(parents=True)
    queue = tmp_path / 'queue'
    environment = dict(os.environ, HP_SIM5_CLOUD_QUEUE=str(queue))
    prompt = tmp_path / 'prompt.md'
    prompt.write_text('Transport fixture only, no scientific assignment.')
    runner = BRIDGE
    if getattr(request, 'param', None) == 'jump-clock':
        runner = tmp_path / 'clock.py'
        runner.write_text('import runpy, time\n'
                          'clock = iter(range(0, 100000000, 3600))\n'
                          'time.monotonic = lambda: next(clock)\n'
                          f'runpy.run_path({str(BRIDGE)!r}, run_name="__main__")\n')
    with prompt.open('rb') as stdin:
        process = subprocess.Popen(
            [sys.executable, str(runner), 'client', '--cd', str(root),
             '--output-last-message', str(final)],
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


@pytest.mark.parametrize('client', ['output', 'default-review', 'alternate-review'], indirect=True)
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


@pytest.mark.parametrize('relative', [
    '.git/final-message.md',
    '.research-programs/programs.sqlite3',
    '.research-programs/reviews/review-run-invalid/final-message.md',
    '.research-programs/reviews/review-run-' + 'a' * 32 + '/connection.json',
    '../outside/final-message.md',
])
def test_cloud_final_path_rejects_nonreview_control_and_external_paths(tmp_path, relative):
    root = tmp_path / 'repo'
    root.mkdir()
    queue = tmp_path / 'queue'
    result = subprocess.run(
        [sys.executable, str(BRIDGE), 'client', '--cd', str(root),
         '--output-last-message', str(root / relative)], input='Fixture only',
        capture_output=True, text=True, timeout=5,
        env=dict(os.environ, HP_SIM5_CLOUD_QUEUE=str(queue)))
    assert result.returncode == 2
    assert 'Final message must' in result.stderr
    assert not queue.exists()


def test_cloud_review_final_path_rejects_symlink_escape(tmp_path):
    root = tmp_path / 'repo'
    root.mkdir()
    outside = tmp_path / 'outside'
    outside.mkdir()
    (root / '.research-programs').symlink_to(outside, target_is_directory=True)
    final = root / '.research-programs/reviews' / ('review-run-' + 'a' * 32) / 'final-message.md'
    result = subprocess.run(
        [sys.executable, str(BRIDGE), 'client', '--cd', str(root),
         '--output-last-message', str(final)], input='Fixture only',
        capture_output=True, text=True, timeout=5)
    assert result.returncode == 2
    assert 'Final message must' in result.stderr


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


@pytest.mark.parametrize('client', ['jump-clock'], indirect=True)
def test_transport_has_no_independent_ten_minute_deadline(client):
    process, path, final, command = client
    time.sleep(.15)
    assert json.loads(path.read_text())['state'] == 'awaiting_agent'
    assert process.poll() is None
    command('bind')
    final.write_text('Completed within the controller allocation')
    command('complete')
    process.communicate(timeout=5)
    assert process.returncode == 0


def test_cloud_commands_do_not_invoke_a_standalone_client(tmp_path, monkeypatch):
    from hp_sim5_research import coding_client
    monkeypatch.setenv('HP_SIM5_RESEARCH_CLIENT', 'cloud')
    argv = coding_client.codex_command(ROOT, tmp_path, batch=True)
    assert argv[:3] == [sys.executable, str(BRIDGE), 'client']
    assert 'codex' not in argv and 'exec' not in argv
    with pytest.raises(ValueError, match='current cloud chat'):
        coding_client.codex_command(ROOT, tmp_path)
    with pytest.raises(ValueError, match='another coding CLI'):
        coding_client.claude_command(tmp_path, batch=True)
    with pytest.raises(ValueError, match='inherit'):
        coding_client.codex_command(ROOT, tmp_path, batch=True, model='local-only-setting')


@pytest.mark.parametrize('markers', [False, True])
@pytest.mark.parametrize('selected,expected', [(None, 'local'), ('local', 'local'), ('cloud', 'cloud')])
def test_mode_depends_only_on_explicit_selection(monkeypatch, markers, selected, expected):
    from hp_sim5_research.coding_client import research_client
    if markers:
        monkeypatch.setenv('CODEX_CLOUD', '1')
        monkeypatch.setenv('CODEX_HOME', '/run/codex-environment/codex-home')
    else:
        monkeypatch.delenv('CODEX_CLOUD', raising=False)
        monkeypatch.delenv('CODEX_HOME', raising=False)
    if selected is None:
        monkeypatch.delenv('HP_SIM5_RESEARCH_CLIENT', raising=False)
    else:
        monkeypatch.setenv('HP_SIM5_RESEARCH_CLIENT', selected)
    assert research_client() == expected


def test_automatic_mode_is_no_longer_supported(monkeypatch):
    from hp_sim5_research.coding_client import research_client
    monkeypatch.setenv('HP_SIM5_RESEARCH_CLIENT', 'auto')
    with pytest.raises(ValueError, match='must be local or cloud'):
        research_client()


def test_cloud_manager_rejects_nested_cli_before_creating_store(tmp_path):
    store = tmp_path / 'never-created'
    result = subprocess.run([str(ROOT / 'hp-sim5-research-program'), '--store', str(store),
                             '--cloud', '--codex', '--question', 'Must use current chat'],
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 2
    assert 'not allowed with argument' in result.stderr
    assert not store.exists()


@pytest.mark.parametrize('backend', ['codex', 'claude'])
def test_local_launcher_selects_local_despite_inherited_cloud_settings(tmp_path, backend):
    store = tmp_path / 'never-created'
    result = subprocess.run([str(ROOT / 'hp-sim5-research-program'), '--store', str(store),
                             '--' + backend, '--question', 'A local manager', '--dry-run'],
                            capture_output=True, text=True, timeout=5, check=True,
                            env=dict(os.environ, HP_SIM5_RESEARCH_CLIENT='cloud', CODEX_CLOUD='1',
                                     CODEX_HOME='/run/codex-environment/codex-home'))
    plan = json.loads(result.stdout)['result']
    assert plan['client_transport'] == 'local'
    assert plan['client']['argv'][0] == backend
    assert 'Bound execution mode: LOCAL' in plan['client']['instructions']
    assert '--cloud --question' not in plan['client']['instructions']
    assert not store.exists()


@pytest.mark.parametrize('mode', [['--serve'], ['--batch', '--prompt', 'A bounded worker']])
def test_cloud_worker_and_service_plans_need_no_standalone_cli(tmp_path, mode):
    session = tmp_path / 'never-created'
    result = subprocess.run([str(ROOT / 'hp-sim5-research-agent'), '--codex', *mode,
                             '--session-dir', str(session), '--dry-run'], capture_output=True, text=True,
                            env=dict(os.environ, HP_SIM5_RESEARCH_CLIENT='cloud'), timeout=5, check=True)
    argv = json.loads(result.stdout)['argv']
    assert 'codex' not in argv
    if '--serve' in mode:
        assert argv == []
    else:
        assert str(BRIDGE) in argv
    assert not session.exists()


def test_cloud_mode_survives_fresh_cli_shells(tmp_path):
    from hp_sim5_research.program_host import live, request
    store = tmp_path / 'store'
    environment = {k: v for k, v in os.environ.items()
                   if k not in ('CODEX_CLOUD', 'CODEX_HOME', 'HP_SIM5_RESEARCH_CLIENT')}
    result = subprocess.run([str(ROOT / 'hp-sim5-research-program'), '--store', str(store),
                             '--cloud', '--question', 'Cloud mode without PATH wrappers'],
                            capture_output=True, text=True, timeout=10, check=True, env=environment)
    opened = json.loads(result.stdout)['result']
    attachment = opened['attachment']
    try:
        assert opened['client'] is None
        assert opened['client_transport'] == attachment['client_transport'] == 'cloud'
        state = request(attachment, 'capabilities')
        assert state['ok']
        # An ordinary later shell cannot silently change an already live host.
        changed = subprocess.run([str(ROOT / 'hp-sim5-research-program'), '--store', str(store),
                                  '--program', attachment['program_id']], capture_output=True, text=True, timeout=5,
                                 env=dict(environment, HP_SIM5_RESEARCH_CLIENT='local'))
        assert changed.returncode == 2
        assert 'different client transport' in changed.stdout
    finally:
        request(attachment, 'shutdown')
        wait_for(lambda: not live(attachment))
