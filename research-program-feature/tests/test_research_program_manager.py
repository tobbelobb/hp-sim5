"""Real CLI/host protocols with fixture worker children, never live research proof."""
import hashlib
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import time
import tomllib

import pytest

from program.store import ProgramStore
from program.entry import client_plan
from program.host import live, open_manager, read_json, request
from program.workers import process_identity
from program import host as program_host

ROOT = Path(__file__).resolve().parents[2]
CLI = ROOT / 'hp-sim5-research-program'


def until(predicate, timeout=5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = predicate()
        if result:
            return result
        time.sleep(.02)
    pytest.fail('Timed out waiting for the expected bounded process observation')


def cli(*arguments, ok=True):
    completed = subprocess.run([str(CLI), *map(str, arguments)], capture_output=True, text=True, timeout=15)
    result = json.loads(completed.stdout)
    assert completed.returncode == (0 if ok else 2), completed.stderr + completed.stdout
    assert result['ok'] is ok
    assert 'Traceback' not in completed.stderr
    return result.get('result') if ok else result['error']


@pytest.mark.parametrize('backend', ['codex', 'claude', None])
@pytest.mark.parametrize('flags,grill,questions', [
    ([], False, False), (['--grill-me'], True, False),
    (['--ask-questions'], False, True),
    (['--grill-me', '--ask-questions-along-the-way'], True, True),
    (['--grill-me', '--no-grill-me', '--no-ask-questions'], False, False),
    (['--dont-ask'], False, False),
])
def test_manager_communication_policy(tmp_path, backend, flags, grill, questions):
    opened = cli('--store', tmp_path / 'unused', '--question', 'Worth doing?', '--dry-run',
                 *(['--' + backend] if backend else []), *flags)
    assert not (tmp_path / 'unused').exists()
    assert opened['communication'] == {'grill_me': grill, 'ask_questions': questions}
    plan = opened['client']
    instructions = plan['instructions'] if plan else opened['manager_instructions']
    assert f'\"grill_me\": {str(grill).lower()}' in instructions
    assert f'\"ask_questions\": {str(questions).lower()}' in instructions
    assert 'Never treat silence' in instructions
    assert 'integration_finish' in instructions
    if backend == 'codex':
        assert f'features.default_mode_request_user_input={str(grill or questions).lower()}' in plan['argv']
        injected = next(a for a in plan['argv'] if a.startswith('developer_instructions='))
        assert json.loads(injected.split('=', 1)[1]) == instructions
    elif backend == 'claude':
        assert ('--disallowedTools' not in plan['argv']) is (grill or questions)
        assert '--append-system-prompt-file' in plan['argv']


def tool(attachment, operation, parameters=None, ok=True):
    return cli('tool', attachment['connection'], operation, '--params', json.dumps(parameters or {}), ok=ok)


def retire(attachments):
    for attachment in attachments:
        if live(attachment):
            request(attachment, 'shutdown')
    for attachment in attachments:
        until(lambda: not live(attachment))
        try:
            os.waitpid(attachment['process']['pid'], os.WNOHANG)
        except ChildProcessError:
            pass

        session = Path(attachment['session'])
        if session.is_relative_to(ROOT / 'research/programs'):
            shutil.rmtree(session.parent.parent)


@pytest.fixture
def manager(tmp_path):
    root = tmp_path / 'repo'
    root.mkdir()
    (root / 'scripts').symlink_to(ROOT / 'scripts', target_is_directory=True)
    (root / 'research-program-feature').symlink_to(ROOT / 'research-program-feature', target_is_directory=True)
    worker = root / 'hp-sim5-research-agent'
    worker.write_text(f'#!{sys.executable}\n' + '''import json, signal, sys, time
from pathlib import Path
args = sys.argv[1:]
session = Path(args[args.index('--session-dir') + 1])
session.mkdir(parents=True)
def stop(*_):
    (session / 'exit.json').write_text(json.dumps({'returncode': 130}))
    sys.exit(130)
signal.signal(signal.SIGTERM, stop)
(session / 'ready').write_text('fixture, no physics')
time.sleep(30)
''')
    worker.chmod(0o755)
    subprocess.run(['git', 'init', '-q', str(root)], check=True)
    subprocess.run(['git', '-C', str(root), 'add', '.'], check=True)
    subprocess.run(['git', '-C', str(root), '-c', 'user.name=fixture', '-c', 'user.email=f@f',
                    'commit', '-qm', 'fixture'], check=True)
    store_path = tmp_path / 'memory'
    with ProgramStore(store_path, create=True) as store:
        store.create_program('Which saved evidence changes the next decision?', program_id='p', question_id='q')
        store.create_program('Different question', program_id='other', question_id='other-q')
    attachments = []
    result = {'root': root, 'store': store_path, 'attachments': attachments}
    try:
        yield result
    finally:
        retire(attachments)


def attach(c, program='p'):
    result = open_manager(c['store'], c['root'], program)
    assert result['ok'], result
    attachment = result['result']['attachment']
    c['attachments'].append(attachment)
    return attachment


def assignment(a, identity='work'):
    route = tool(a, 'approach', {'question_id': 'q', 'title': 'Inspect saved data', 'reason': 'Cheapest discriminator'})
    result = tool(a, 'assignment', {'approach_id': route['id'], 'objective': 'Compare frozen baselines',
                                    'record_id': identity})
    tool(a, 'transition', {'record_id': result['id'], 'target': 'ready', 'reason': 'Explicit bounded assignment'})
    return result


def test_question_only_actual_cli_creates_owned_host_and_second_chat_opens_it(tmp_path):
    store_path = tmp_path / 'memory'
    first = cli('--store', store_path, '--question', 'Question without prepared state')
    attachment = first['attachment']
    try:
        assert first['overview']['program']['goal'] == 'Question without prepared state'
        assert first['overview']['work'] == []
        assert first['physics_world'] is False
        assert first['overview']['manager']['physics_world'] is False
        assert not list(Path(attachment['session']).glob('overview.*'))
        second = cli('--store', store_path, '--program', attachment['program_id'])
        assert second['attachment'] == attachment
        assert tool(attachment, 'worker_sessions') == []
        assert not list(Path(attachment['session']).rglob('native'))
        assert first['client_transport'] == 'local'
        assert not Path(attachment['session'], 'attachment.toml').exists()
    finally:
        retire([attachment])


def test_concurrent_chats_open_one_host_one_program_identity(manager):
    # Each opener is a fresh process and uses the same automatic startup path.
    code = ('import sys,json; from program.host import open_manager; '
            'print(json.dumps(open_manager(sys.argv[1],sys.argv[2],"p")))')
    environment = dict(os.environ, PYTHONPATH=os.pathsep.join([str(ROOT / 'src/python'), str(ROOT / 'research-program-feature')]))
    children = [subprocess.Popen([sys.executable, '-c', code, str(manager['store']), str(manager['root'])],
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=environment) for _ in range(3)]
    attachments = []
    for child in children:
        stdout, stderr = child.communicate(timeout=15)
        assert child.returncode == 0, stderr
        result = json.loads(stdout)
        assert result['ok'], result
        attachments.append(result['result']['attachment'])
    manager['attachments'].extend(attachments)
    assert attachments[0] == attachments[1] == attachments[2]
    assert tool(attachments[0], 'worker_sessions') == []


def test_client_with_invisible_host_pid_uses_protocol_and_does_not_spawn_replacement(manager, monkeypatch):
    a = attach(manager)
    monkeypatch.setattr(program_host, 'process_identity', lambda _: None)
    assert not live(a)  # Diagnostic from another PID namespace cannot see it.
    def no_process(*args, **kwargs):
        pytest.fail('A live locked controller must not be replaced')
    monkeypatch.setattr(program_host.subprocess, 'Popen', no_process)
    assert request(a, 'capabilities')['result']['client_transport'] == 'local'
    reopened = open_manager(manager['store'], manager['root'], 'p')
    assert reopened['result']['attachment'] == a
    assert request(dict(a, host_id='old-generation'), 'overview')['error']['code'] == 'stale_attachment'


def test_unanswered_file_request_is_bounded_and_never_claims_a_pid_is_dead(tmp_path):
    queue = tmp_path / 'controller-queue'
    queue.mkdir()
    descriptor = {'store': str(tmp_path), 'host_id': 'absent'}
    result = request(descriptor, 'capabilities', timeout=.05)
    assert result['error']['code'] == 'outcome_unknown'
    assert not list(queue.iterdir())


def test_recovery_cannot_change_local_store_to_cloud_after_host_stops(manager, monkeypatch):
    a = attach(manager)
    assert request(a, 'shutdown')['ok']
    until(lambda: not live(a))
    monkeypatch.setenv('HP_SIM5_RESEARCH_CLIENT', 'cloud')
    with pytest.raises(ValueError, match='different client transport'):
        open_manager(manager['store'], manager['root'], 'p')
    assert read_json(manager['store'] / 'controller.json')['host_id'] == a['host_id']


def test_fresh_cli_calls_retain_owned_worker_share_attachment_pause_then_cancel(manager):
    first = attach(manager)
    work = assignment(first)
    overview = tool(first, 'overview')
    assert overview['work'][0]['id'] == work['id']
    started_at = time.monotonic()
    worker = tool(first, 'worker_start', {'settings': {'machine': 'hp3'}, 'assignment_id': work['id']})
    assert time.monotonic() - started_at < 2
    assert worker['details']['state'] == 'running'
    until(lambda: Path(worker['details']['session_dir'], 'ready').exists())
    second = attach(manager)
    assert first == second
    assert tool(second, 'worker_status', {'attempt_id': worker['id']})['owned']
    error = tool(second, 'worker_start', {'settings': {'machine': 'hp3'}, 'assignment_id': work['id']}, ok=False)
    assert error['code'] == 'invalid_request'
    assert len(tool(second, 'worker_sessions')) == 1
    paused = tool(second, 'pause', {'reason': 'Human wants to reconsider new work'})
    assert paused['program']['state'] == 'paused'
    assert tool(second, 'worker_status', {'attempt_id': worker['id']})['details']['state'] == 'running'
    assert live({'process': worker['details']['process']})
    assert tool(second, 'review_start', {'submission_id': 'absent'}, ok=False)['code'] == 'invalid_request'
    tool(first, 'resume', {'reason': 'Proceed with explicit calls'})
    note = tool(first, 'steering', {'text': 'Keep the existing inputs', 'subject_id': work['id']})
    assert note['details']['actor'] == 'human'
    tool(second, 'worker_cancel', {'attempt_id': worker['id']})
    until(lambda: tool(first, 'worker_status', {'attempt_id': worker['id']})['details']['state'] == 'cancelled')
    final = tool(first, 'worker_status', {'attempt_id': worker['id']})
    assert final['details']['cleanup'] == 'launcher_finally_completed'
    assert final['details']['scientific_state'] == 'unassessed'
    assert Path(final['details']['session_dir'], 'exit.json').is_file()


def test_program_bound_identity_graph_review_and_structured_errors(manager):
    first, other = attach(manager), attach(manager, 'other')
    assert first['host_id'] == other['host_id']
    assert first['controller_id'] != other['controller_id']
    assert tool(other, 'overview')['program']['id'] == 'other'
    assert tool(first, 'question', {'question': 'Cheapest useful observation?'})['program_id'] == 'p'
    questions = tool(first, 'query', {'query': 'open_questions'})
    assert questions['program_id'] == 'p'
    assert tool(other, 'overview')['program']['revision'] == 1
    for operation, parameters, code in [
        ('overview', {'program_id': 'other'}, 'invalid_request'),
        ('worker_start', {}, 'invalid_request'),
        ('query', {'query': 'support', 'subject_id': 'absent'}, 'invalid_request'),
        ('review_plan', {'submission_id': 'absent'}, 'invalid_request'),
        ('review_status', {'run_id': 'absent'}, 'invalid_request'),
        ('review_apply', {'review_id': 'absent'}, 'invalid_request'),
        ('review_cancel', {'run_id': 'absent'}, 'invalid_request'),
        ('does_not_exist', {}, 'unknown_operation'),
        ('autonomous_continue', {}, 'unknown_operation'),
        ('parallel', {}, 'unknown_operation'),
        ('integration_prepare', {}, 'invalid_request'),
        ('integration_check', {}, 'invalid_request'),
        ('integration_status', {'integration_id': 'absent'}, 'invalid_request'),
        ('_open', {'program_id': 'other'}, 'invalid_request'),
    ]:
        assert tool(first, operation, parameters, ok=False)['code'] == code
    assert request(first, [], {})['error']['code'] == 'unknown_operation'
    assert request(first, 'overview', [])['error']['code'] == 'invalid_request'
    forged = dict(first, program_id='other')
    assert request(forged, 'overview')['error']['code'] == 'wrong_attachment'
    stale = dict(first, host_id='past-generation')
    assert request(stale, 'overview')['error']['code'] == 'stale_attachment'
    bad = subprocess.run([str(CLI), 'tool', first['connection'], 'overview', '--params', '{'],
                         capture_output=True, text=True, timeout=5)
    assert bad.returncode == 2
    assert json.loads(bad.stdout)['error']['code'] == 'invalid_request'
    assert 'Traceback' not in bad.stderr
    assert 'route_plan' not in tool(first, 'capabilities')['operations']
    available = tool(first, 'capabilities')
    assert available['physics_world'] is False
    assert {'submit', 'review_start', 'worker_start', 'stop', 'query'} <= available['operations'].keys()
    tool(other, 'shutdown')
    reopened = attach(manager, 'other')
    assert reopened['host_id'] == first['host_id']
    assert reopened['controller_id'] != other['controller_id']
    assert request(other, 'overview')['error']['code'] == 'wrong_attachment'


def test_stop_cancels_owned_children_and_prevents_more_dispatch(manager):
    a = attach(manager)
    work = assignment(a)
    worker = tool(a, 'worker_start', {'settings': {'machine': 'hp3'}, 'assignment_id': work['id']})
    until(lambda: Path(worker['details']['session_dir'], 'ready').exists())
    assert tool(a, 'stop', {'reason': 'Human stop'})['program']['state'] == 'stopped'
    until(lambda: tool(a, 'worker_status', {'attempt_id': worker['id']})['details']['state'] == 'cancelled')
    assert tool(a, 'worker_start', {'settings': {'machine': 'hp3'}, 'assignment_id': work['id']}, ok=False)['code'] == 'invalid_request'
    assert tool(a, 'resume', {'reason': 'Stopped is final'}, ok=False)['code'] == 'invalid_request'


def test_fixed_submission_review_cancel_and_admission_through_fresh_cli(manager, tmp_path, monkeypatch):
    # Separate executable supplies a fixture judgement after inspecting fixed bytes.
    worker_path = manager['root'] / 'hp-sim5-research-agent'
    worker_path.write_text(worker_path.read_text().replace('time.sleep(30)', 'sys.exit(0)'))
    binaries = tmp_path / 'bin'
    binaries.mkdir()
    hold, entered = tmp_path / 'hold-review', tmp_path / 'review-entered'
    hold.touch()
    fixture = json.loads((ROOT / 'research-program-feature/tests/fixtures/review/cases.json').read_text())['scoped_negative']
    reviewer = binaries / 'codex'
    reviewer.write_text(f'#!{sys.executable}\n' + f'''import hashlib, json, re, sys, time
from pathlib import Path
prompt = sys.stdin.read()
directory = Path(re.search(r'Target directory: (.+)', prompt).group(1))
manifest = json.loads((directory / 'manifest.json').read_text())
for asset in manifest['assets']:
    assert hashlib.sha256((directory / asset['id']).read_bytes()).hexdigest() == asset['sha256']
Path({str(entered)!r}).touch()
while Path({str(hold)!r}).exists():
    time.sleep(.02)
response = {fixture['review']!r}
response.update(submission_id=re.search(r'Submission: (.+)', prompt).group(1),
                submission_sha256=re.search(r'SHA-256: (.+)', prompt).group(1),
                checks=[{{'kind': 'inspected', 'description': 'Read frozen fixture bytes, no reproduction',
                         'asset_ids': [a['id'] for a in manifest['assets'] ]}}])
Path(sys.argv[sys.argv.index('--output-last-message') + 1]).write_text(json.dumps(response))
''')
    reviewer.chmod(0o755)
    monkeypatch.setenv('PATH', str(binaries) + os.pathsep + os.environ['PATH'])
    a = attach(manager)
    work = assignment(a)
    attempt = tool(a, 'worker_start', {'settings': {'machine': 'hp3'}, 'assignment_id': work['id']})
    until(lambda: tool(a, 'worker_status', {'attempt_id': attempt['id']})['details']['state'] == 'submitted')
    claim = tool(a, 'claim', {'title': fixture['claim'], 'scope': {'evidence_type': 'synthetic', 'configuration': 'fixture'}})
    selected = []
    for role, data in [('report', fixture['claim']), ('abstract', fixture['review']['reason']),
                       ('evidence', json.dumps(fixture['measurements']))]:
        path = tmp_path / (role + '.txt')
        path.write_text(data)
        selection = {'id': role, 'role': role, 'path': str(path)}
        if role == 'evidence':
            artifact = tool(a, 'artifact', {'reference': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                                            'evidence_type': 'synthetic', 'source_revision': attempt['details']['source_revision']})
            selection['artifact_id'] = artifact['id']
        selected.append(selection)
    submission = tool(a, 'submit', {'attempt_id': attempt['id'], 'claim_id': claim['id'],
                                    'base_revision': attempt['details']['source_revision'], 'config': {'fixture': True},
                                    'input_identities': ['fixture-input'], 'selected': selected, 'producer_checks': []})
    verified = tool(a, 'submission_verify', {'submission_id': submission['id']})
    assert verified[1] == []
    assert tool(a, 'review_plan', {'submission_id': submission['id']})['argv'][0] == 'codex'
    run = tool(a, 'review_start', {'submission_id': submission['id']})
    until(entered.exists)
    tool(a, 'pause', {'reason': 'Pause only new dispatch'})
    assert tool(a, 'review_status', {'run_id': run['id']})['details']['state'] == 'running'
    with ProgramStore(manager['store']) as store:
        with pytest.raises(ValueError, match='historical or unverified'):
            store.reviews(manager['root']).cancel('p', run['id'])
    tool(a, 'review_cancel', {'run_id': run['id']})
    until(lambda: tool(a, 'review_status', {'run_id': run['id']})['details']['state'] == 'cancelled')
    cancelled = tool(a, 'review_status', {'run_id': run['id']})
    assert cancelled['details']['review_id'] is None
    tool(a, 'resume', {'reason': 'Review the fixed evidence'})
    hold.unlink()
    second = tool(a, 'review_start', {'submission_id': submission['id']})
    until(lambda: tool(a, 'review_status', {'run_id': second['id']})['details']['state'] == 'completed')
    review = tool(a, 'review_status', {'run_id': second['id']})
    admission = tool(a, 'review_apply', {'review_id': review['details']['review_id']})
    assert admission['details']['decision'] == 'keep'
    graph = tool(a, 'query', {'query': 'support', 'subject_id': claim['id']})
    assert graph['states'][claim['id']]['assessment_state'] == 'reviewed'
    with ProgramStore(manager['store']) as store, ProgramStore(tmp_path / 'restored', create=True) as restored:
        restored.import_snapshot(store.snapshot('p'))
        assert restored.get_record('p', run['id'])['details']['state'] == 'cancelled'
    assert tool(a, 'submission_invalidate', {'submission_id': submission['id'], 'reason': 'Fixture withdrawal'})['kind'] == 'submission_event'


def test_dead_and_zombie_hosts_are_not_reused_on_open(manager):
    # Keep an exited child unreaped to exercise the PID1-zombie failure directly.
    zombie = subprocess.Popen([sys.executable, '-c', 'pass'])
    identity = process_identity(zombie.pid)
    try:
        until(lambda: Path(f'/proc/{zombie.pid}/stat').read_text().rsplit(') ', 1)[1].startswith('Z '))
        assert not live({'process': identity})
        descriptor = {'process': identity, 'host_id': 'dead-host', 'root': str(manager['root'])}
        (manager['store'] / 'controller.json').write_text(json.dumps(descriptor))
        opened = attach(manager)
        assert opened['host_id'] != 'dead-host'
        assert live(opened)
        assert tool(opened, 'worker_sessions') == []
    finally:
        zombie.wait(timeout=5)


@pytest.mark.parametrize('backend', ['codex', 'claude'])
def test_manager_client_commands_only_attach_orchestration_and_dry_run_is_inert(tmp_path, backend):
    directory = tmp_path / 'does-not-exist'
    planned = cli('--store', directory, '--' + backend, '--question', 'Compare options', '--dry-run', '--model', 'chosen')
    assert not directory.exists()
    plan = planned['client']
    assert plan['physics_world'] is False
    assert 'chosen' in plan['argv']
    command = ' '.join(plan['argv'])
    assert 'research_program_mcp.py' not in command
    assert 'research_attach.py' not in command
    assert 'hp-sim5-research-program tool' in plan['instructions']
    assert '.agents/skills/cloud-research-client/scripts/' not in plan['instructions']
    if backend == 'codex':
        for name in ('hp_sim5', 'rerun', 'hp_sim5_program'):
            option = next(a for a in plan['argv'] if a.startswith(f'mcp_servers.{name}='))
            assert tomllib.loads(option)['mcp_servers'][name]['enabled'] is False
    else:
        assert '--strict-mcp-config' in plan['argv']
        assert 'mcp__hp_sim5_program' not in command


@pytest.mark.skipif(shutil.which('codex') is None, reason='Installed Codex config validation is optional')
@pytest.mark.parametrize('inherited', [{}, {'hp_sim5': {'url': 'https://example.invalid/mcp'}}])
def test_installed_codex_accepts_disabled_manager_transports(tmp_path, inherited):
    opened = cli('--store', tmp_path / 'store', '--question', 'Validate manager attachment only')
    a = opened['attachment']
    try:
        plan = client_plan(ROOT, a, 'codex')
        configs = [plan['argv'][i+1] for i, arg in enumerate(plan['argv'])
                   if arg == '-c' and plan['argv'][i+1].startswith('mcp_servers.')]
        from hp_sim5_research.coding_client import toml_literal
        for overrides in (configs,):
            command = ['codex', '-c', 'mcp_servers=' + toml_literal(inherited)]
            for override in overrides:
                command += ['-c', override]
            checked = subprocess.run(command + ['mcp', 'list', '--json'], capture_output=True, text=True, timeout=10)
            assert checked.returncode == 0, checked.stderr
            servers = {s['name']: s for s in json.loads(checked.stdout)}
            assert not servers['hp_sim5']['enabled'] and not servers['rerun']['enabled']
            assert servers['hp_sim5']['transport']['type'] == servers['rerun']['transport']['type'] == 'stdio'
            assert not servers['hp_sim5_program']['enabled']
    finally:
        retire([a])


def test_manager_forwarded_provider_and_model_share_existing_helpers(tmp_path):
    attachment = {'program_id': 'p', 'session': str(tmp_path), 'connection': str(tmp_path / 'connection.json')}
    plan = client_plan(ROOT, attachment, 'codex', model='base', model_provider='openai',
                       client_args=['--model', 'forwarded', '-c', 'model_provider="existing-provider"'])
    assert plan['model_provider'] == 'existing-provider'
    assert plan['argv'].count('--model') == 1
    assert 'base' not in plan['argv']
    assert 'model_provider="openai"' not in plan['argv']




def test_missing_worker_machine_does_not_charge_or_spawn(manager):
    a = attach(manager)
    work = assignment(a)
    before = tool(a, 'overview')
    error = tool(a, 'worker_start', {'assignment_id': work['id']}, ok=False)
    assert error['code'] == 'invalid_request'
    assert 'settings.machine' in error['message']
    assert tool(a, 'worker_sessions') == []
    assert tool(a, 'overview') == before
    assert 'Required hp3/hp4' in tool(a, 'capabilities')['worker_machine']


def test_manager_answers_worker_question_through_bound_cli(manager):
    a = attach(manager)
    work = assignment(a)
    worker = tool(a, 'worker_start', {'assignment_id': work['id'], 'settings': {'machine': 'hp4'}})
    session = Path(worker['details']['session_dir'])
    until(lambda: (session / 'ready').exists())
    (session / 'manager-question.json').write_text(json.dumps({'id': 'baseline', 'question': 'Which baseline?'}))
    status = tool(a, 'worker_status', {'attempt_id': worker['id']})
    assert status['manager_question']['id'] == 'baseline'
    answer = 'Use the "saved" baseline.\nDo not refit.'
    response = tool(a, 'worker_answer', {'attempt_id': worker['id'], 'question_id': 'baseline', 'answer': answer})
    assert response['manager_question'] is None
    assert response['details']['runtime']['wall_deadline'] == worker['details']['runtime']['wall_deadline']
    assert json.loads(Path(status['manager_answer_path']).read_text()) == {'id': 'baseline', 'answer': answer}


def test_program_cannot_be_marked_complete_without_delivery(manager):
    a = attach(manager)
    error = tool(a, 'transition', {'record_id': 'p', 'target': 'completed', 'reason': 'No delivery'}, ok=False)
    assert error['code'] == 'invalid_request'
    with ProgramStore(manager['store']) as store:
        with pytest.raises(ValueError, match='Invalid program transition'):
            store.transition('p', 'p', 'completed', 'Skip review and integration')
    assert tool(a, 'overview')['program']['state'] == 'active'
