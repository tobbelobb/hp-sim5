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

from hp_sim5_research.program import ProgramStore
from hp_sim5_research.program_entry import client_plan
from hp_sim5_research.program_host import live, open_manager, read_json, request
from hp_sim5_research.program_workers import process_identity
from hp_sim5_research import program_host

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


@pytest.fixture
def manager(tmp_path):
    root = tmp_path / 'repo'
    root.mkdir()
    (root / 'scripts').symlink_to(ROOT / 'scripts', target_is_directory=True)
    (root / 'research/prompts').mkdir(parents=True)
    (root / 'research/prompts/research-manager-workflow.txt').write_bytes((ROOT / 'research/prompts/research-manager-workflow.txt').read_bytes())
    (root / 'research/prompts/reviewer.txt').write_bytes((ROOT / 'research/prompts/reviewer.txt').read_bytes())
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
                                    'record_id': identity, 'allocation': {'wall_minutes': 1, 'fresh_fits': 0}})
    tool(a, 'transition', {'record_id': result['id'], 'target': 'ready', 'reason': 'Explicit bounded assignment'})
    return result


def test_question_only_actual_cli_creates_owned_host_and_second_chat_opens_it(tmp_path):
    store_path = tmp_path / 'memory'
    first = cli('--store', store_path, '--question', 'Question without prepared state')
    attachment = first['attachment']
    try:
        assert first['overview']['program']['goal'] == 'Question without prepared state'
        assert first['overview']['next_allocation']['state'] == 'unallocated'
        assert first['physics_world'] is False
        assert first['overview']['manager']['physics_world'] is False
        for key in ('overview', 'snapshot'):
            assert Path(first['overview']['manager']['links'][key]).is_file()
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
    code = ('import sys,json; from hp_sim5_research.program_host import open_manager; '
            'print(json.dumps(open_manager(sys.argv[1],sys.argv[2],"p")))')
    environment = dict(os.environ, PYTHONPATH=str(ROOT / 'src/python'))
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


def test_result_commit_runs_in_host_even_when_client_cannot_run_git(manager, tmp_path):
    root = manager['root']
    for name, value in (('user.name', 'Fixture'), ('user.email', 'fixture@example.invalid')):
        subprocess.run(['git', '-C', str(root), 'config', name, value], check=True)
    (root / '.gitignore').write_text('/research/programs/\n')
    (root / 'unrelated.txt').write_text('Unrelated staged work')
    subprocess.run(['git', '-C', str(root), 'add', 'unrelated.txt'], check=True)
    a = attach(manager)
    # The client is a fresh process. Blocking Git there must not block the host.
    code = '''import json, runpy, subprocess, sys
from pathlib import Path
a = json.loads(sys.argv[1])
main = runpy.run_path(sys.argv[2])['main']
main.__globals__['ROOT'] = Path(a['root'])
def denied(*args, **kwargs):
    raise AssertionError('Client must not run Git')
subprocess.run = denied
assert main(['--store', a['store'], 'save', a['program_id'], '--commit']) == 0
'''
    completed = subprocess.run([sys.executable, '-c', code, json.dumps(a), str(ROOT / 'scripts/research_program.py')], capture_output=True, text=True,
                               env=dict(os.environ, PYTHONPATH=str(ROOT / 'src/python')), timeout=10, check=True)
    saved = json.loads(completed.stdout)
    assert saved['commit']
    staged = subprocess.check_output(['git', '-C', str(root), 'diff', '--cached', '--name-only'], text=True)
    assert staged.strip() == 'unrelated.txt'
    committed = subprocess.check_output(['git', '-C', str(root), 'show', '--pretty=', '--name-only', 'HEAD'], text=True)
    assert all(p.startswith('research/programs/p/') for p in committed.splitlines() if p)


@pytest.mark.skipif(not os.environ.get('HP_SIM5_TEST_REAL_SANDBOX'),
                    reason='Requires an installed Codex sandbox and a parent running outside that sandbox')
def test_real_command_sandbox_reopens_host_and_commits_with_git_read_only(manager, tmp_path):
    root = manager['root']
    for name, value in (('user.name', 'Fixture'), ('user.email', 'fixture@example.invalid')):
        subprocess.run(['git', '-C', str(root), 'config', name, value], check=True)
    (root / '.gitignore').write_text('/research/programs/\n')
    (root / 'unrelated.txt').write_text('Unrelated staged work')
    subprocess.run(['git', '-C', str(root), 'add', 'unrelated.txt'], check=True)
    a = attach(manager)
    client = tmp_path / 'sandbox_client.py'
    client.write_text('''import json, subprocess, sys
from pathlib import Path
sys.path.insert(0, sys.argv[1])
from hp_sim5_research.program_host import open_manager, request, live
a = json.loads(sys.argv[2])
assert not live(a), 'The controller must be invisible in this PID namespace'
denied = subprocess.run(['git', '-C', a['root'], 'add', '--', 'unrelated.txt'], capture_output=True, text=True)
assert denied.returncode != 0 and 'index.lock' in denied.stderr, denied.stderr
opened = open_manager(a['store'], a['root'], a['program_id'])
assert opened['ok'] and opened['result']['attachment'] == a, opened
saved = request(a, 'save', {'commit': True})
assert saved['ok'] and saved['result']['commit'], saved
print(json.dumps({'isolated_pid_namespace': True, 'client_git_read_only': True, 'saved': saved['result']['commit']}))
''')
    profile = {'type': 'managed', 'file_system': {'type': 'restricted', 'entries': [
        {'path': {'type': 'special', 'value': {'kind': 'root'}}, 'access': 'read'},
        {'path': {'type': 'path', 'path': str(tmp_path)}, 'access': 'write'},
        {'path': {'type': 'path', 'path': str(root / '.git')}, 'access': 'read'}]}, 'network': 'restricted'}
    completed = subprocess.run([
        'codex-linux-sandbox', '--sandbox-policy-cwd', str(root), '--command-cwd', str(root),
        '--permission-profile', json.dumps(profile), '--', sys.executable, str(client),
        str(ROOT / 'src/python'), json.dumps(a)], executable=os.environ['HP_SIM5_TEST_REAL_SANDBOX'],
        capture_output=True, text=True, timeout=20)
    assert completed.returncode == 0, completed.stderr + completed.stdout
    assert json.loads(completed.stdout)['client_git_read_only']
    assert subprocess.check_output(['git', '-C', str(root), 'diff', '--cached', '--name-only'], text=True).strip() == 'unrelated.txt'


def test_fresh_cli_calls_retain_owned_worker_share_attachment_pause_then_cancel(manager):
    first = attach(manager)
    work = assignment(first)
    overview = tool(first, 'overview')
    assert overview['next_allocation']['assignment_id'] == work['id']
    assert overview['next_allocation']['allocation']['wall_minutes'] == 1
    started_at = time.monotonic()
    worker = tool(first, 'worker_start', {'assignment_id': work['id']})
    assert time.monotonic() - started_at < 2
    assert worker['details']['state'] == 'running'
    until(lambda: Path(worker['details']['session_dir'], 'ready').exists())
    second = attach(manager)
    assert first == second
    assert tool(second, 'worker_status', {'attempt_id': worker['id']})['owned']
    error = tool(second, 'worker_start', {'assignment_id': work['id']}, ok=False)
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
    assert tool(first, 'strategy')['routes'] == []
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
    worker = tool(a, 'worker_start', {'assignment_id': work['id']})
    until(lambda: Path(worker['details']['session_dir'], 'ready').exists())
    assert tool(a, 'stop', {'reason': 'Human stop'})['program']['state'] == 'stopped'
    until(lambda: tool(a, 'worker_status', {'attempt_id': worker['id']})['details']['state'] == 'cancelled')
    assert tool(a, 'worker_start', {'assignment_id': work['id']}, ok=False)['code'] == 'invalid_request'
    assert tool(a, 'resume', {'reason': 'Stopped is final'}, ok=False)['code'] == 'invalid_request'


def test_fixed_submission_review_cancel_and_admission_through_fresh_cli(manager, tmp_path, monkeypatch):
    # Separate executable supplies a fixture judgement after inspecting fixed bytes.
    worker_path = manager['root'] / 'hp-sim5-research-agent'
    worker_path.write_text(worker_path.read_text().replace('time.sleep(30)', 'sys.exit(0)'))
    binaries = tmp_path / 'bin'
    binaries.mkdir()
    hold, entered = tmp_path / 'hold-review', tmp_path / 'review-entered'
    hold.touch()
    fixture = json.loads((ROOT / 'tests/fixtures/program-review/cases.json').read_text())['scoped_negative']
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
    tool(a, 'allocate', {'task_limit': 1, 'wall_minutes': 12, 'review_minutes': 10, 'retry_limit': 2})
    attempt = tool(a, 'worker_start', {'assignment_id': work['id']})
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
    assert plan['instructions'].startswith('Bound execution mode: LOCAL.')
    assert 'Do not use --cloud or the cloud-research-client skill' in plan['instructions']
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


def test_strategy_funding_steering_and_worker_contract_through_fresh_cli(manager):
    a = attach(manager)
    for identity, priority in [('saved', 10), ('model', 20)]:
        tool(a, 'approach', {'question_id': 'q', 'title': identity, 'reason': 'Compare credible routes', 'record_id': identity})
        tool(a, 'route_plan', dict(approach_id=identity, hypothesis='A fair common fold distinguishes ' + identity,
             scope={'method': identity, 'evidence': 'simulated'}, priority=priority,
             estimated_cost={'wall_minutes': 17, 'fresh_fits': 2}, uncertainty='Fair fold availability',
             decision_value='Decide collection versus model work', novelty='Applied model comparison',
             observation='Count truly common unseen sweeps', reopening_condition='Changed folds or model',
             reason='Resolve the cheapest uncertainty', tradeoffs=['Scouting is cheaper than collection'],
             objections=['Prediction is not physical accuracy']))
    params = dict(approach_id='saved', objective='Scout four saved sources for fair folds',
                  allocation={'wall_minutes': 8, 'fresh_fits': 0, 'collections': 0}, kind='scouting',
                  execution={'source_limit': 4},
                  explanations=[{'approach_id': 'saved', 'explanation': 'Membership selection is the issue'},
                                {'approach_id': 'model', 'explanation': 'Model inadequacy dominates'}],
                  outcomes={'gap': {'observation': 'No shared fair fold found', 'finding': 'inconclusive', 'actions': []}})
    first = tool(a, 'commission', params)
    worker = tool(a, 'worker_start', {'assignment_id': first['assignment']['id']})
    until(lambda: Path(worker['details']['session_dir'], 'ready').exists())
    prompt = Path(worker['details']['attempt_dir'], 'prompt.txt').read_text()
    assert 'source_limit' in prompt and 'Model inadequacy dominates' in prompt
    params.update(approach_id='model', kind='tooling', objective='Build a bounded fold-inspection helper')
    second = tool(a, 'commission', params)
    note = tool(a, 'steering', {'text': 'Radius bias might matter', 'intent': 'hunch',
                               'links': ['https://example.org/hunch'], 'provenance': {'channel': 'chat'}})
    assert note['accepted_input']['details']['intent'] == 'hunch'
    instruction = tool(a, 'steering', {'text': 'Fund the tooling route next with three fresh fits', 'intent': 'instruction',
         'next_assignment_id': second['assignment']['id'], 'allocation': {'wall_minutes': 41, 'fresh_fits': 3}})
    overview = tool(a, 'overview')
    assert overview['next_allocation']['assignment_id'] == instruction['accepted_input']['details']['next_assignment_id']
    assert overview['next_allocation']['allocation']['fresh_fits'] == 3
    assert overview['brief'] and len(overview['strategy']['routes']) == 2
    assert tool(a, 'worker_status', {'attempt_id': worker['id']})['details']['state'] == 'running'
    tool(a, 'worker_cancel', {'attempt_id': worker['id']})
    until(lambda: tool(a, 'worker_status', {'attempt_id': worker['id']})['details']['state'] == 'cancelled')
