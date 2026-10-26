"""Finite controller faults and real CLI/file-host protocols with fixture children."""
import copy
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from unittest.mock import patch

import pytest

from hp_sim5_research.program import ProgramError, ProgramStore
from hp_sim5_research.program_manager import ProgramManager
from hp_sim5_research.program_workers import process_identity, process_live
from .test_research_program_manager import manager, attach, assignment, tool, until, ROOT
from .test_research_program_review import context, submit, reviewer, CASES


def local_manager(c):
    m = ProgramManager(c['store'], c['root'], 'p', 'fixture-manager', c['tmp'] / 'manager', reviews=c['controller'])
    m.controller.recover('p', handoff_attempt_id=c['attempt']['id'], reason='Fixture worker had no world; missing exit manifest retained')
    return m


def ready(m, identity, minutes=.01):
    s = m.store
    s.add_assignment('p', 'route', 'Distinct bounded observation ' + identity, {'wall_minutes': minutes}, record_id=identity)
    s.transition('p', identity, 'ready', 'Fixture selection')


def test_cloud_setup_failure_does_not_allocate_or_charge_attempt(context, monkeypatch):
    m = local_manager(context)
    ready(m, 'new')
    monkeypatch.setenv('HP_SIM5_RESEARCH_CLIENT', 'cloud')
    blocked = {'ok': False, 'checks': {'launcher_sockets': {'ok': False, 'error': 'EPERM'}}}
    monkeypatch.setattr('hp_sim5_research.program_controller.runtime_check', lambda *a, **kw: blocked)
    before = m.store.snapshot('p')['records']
    with pytest.raises(ProgramError, match='no worker attempt was charged'):
        m.controller.run_once('p', 'new', settings={'machine': 'hp3'})
    assert m.store.snapshot('p')['records'] == before
    allocation = m.controller.allocate('p', 1, 10)
    with pytest.raises(ProgramError, match='no worker attempt was charged'):
        m.controller.run_once('p', 'new', allocation_id=allocation['id'], settings={'machine': 'hp3'})
    details = m.controller.current(allocation['id'])['details']
    assert details['reservations'] == []
    assert details['wall_reserved_seconds'] == 0


def test_cloud_setup_failure_holds_queue_without_repeated_dispatch(context, monkeypatch):
    m = local_manager(context)
    ready(m, 'new')
    monkeypatch.setenv('HP_SIM5_RESEARCH_CLIENT', 'cloud')
    calls = []
    def blocked(*args, **kwargs):
        calls.append(True)
        return {'ok': False}
    monkeypatch.setattr('hp_sim5_research.program_controller.runtime_check', blocked)
    allocation = m.controller.allocate('p', 1, 10)
    m.controller.run_within_allocation('p', allocation['id'], ['new'], settings={'machine': 'hp3'})
    m.controller.poll()
    details = m.controller.current(allocation['id'])['details']
    assert len(calls) == 1
    assert details['reservations'] == details['queue'] == []
    assert details['events'][-1]['event'] == 'queue_dispatch_blocked'


def test_slow_cloud_setup_check_renews_lease_without_extending_wall_budget(context, monkeypatch):
    m = local_manager(context)
    m.workers = m.store.workers(context['root'], executable=context['tmp'] / 'worker')
    ready(m, 'new')
    allocation = m.controller.allocate('p', 1, 10)
    wall_deadline = allocation['details']['deadline']
    now = time.time()
    clock = [now]
    monkeypatch.setattr(time, 'time', lambda: clock[0])
    monkeypatch.setenv('HP_SIM5_RESEARCH_CLIENT', 'cloud')
    def checked(*args, **kwargs):
        clock[0] += 20
        return {'ok': True}
    monkeypatch.setattr('hp_sim5_research.program_controller.runtime_check', checked)
    attempt = m.controller.run_once('p', 'new', allocation_id=allocation['id'], settings={'machine': 'hp3'})
    m.workers.children[attempt['id']].wait(timeout=5)
    details = m.controller.current(allocation['id'])['details']
    assert details['lease_until'] > clock[0]
    assert details['deadline'] == wall_deadline
    assert len(details['reservations']) == 1


def set_launcher(manager, action):
    p = manager['root'] / 'hp-sim5-research-agent'
    source = p.read_text().replace('time.sleep(30)', action)
    p.write_text(source)


def test_finite_reservations_concurrency_backlog_pause_and_poll_revision(manager):
    a = attach(manager)
    one, two = assignment(a, 'one'), assignment(a, 'two')
    for args in ({'task_limit': 0, 'wall_minutes': 10}, {'task_limit': True, 'wall_minutes': 10},
                 {'task_limit': 2, 'wall_minutes': 10, 'review_minutes': 5}):
        assert tool(a, 'allocate', args, ok=False)['code'] == 'invalid_request'
    allocation = tool(a, 'allocate', {'task_limit': 2, 'wall_minutes': 4, 'review_minutes': .2})
    worker = tool(a, 'run_once', {'assignment_id': one['id'], 'allocation_id': allocation['id']})
    until(lambda: Path(worker['details']['session_dir'], 'ready').exists())
    # Wait for initial file inventory, then repeated elapsed observations must
    # leave semantic revisions stable while an unchanged investigation runs.
    tool(a, 'overview')
    revision = tool(a, 'overview')['program']['revision']
    for _ in range(3):
        assert tool(a, 'overview')['program']['revision'] == revision
    assert tool(a, 'run_once', {'assignment_id': two['id'], 'allocation_id': allocation['id']}, ok=False)['code'] == 'invalid_request'
    tool(a, 'pause', {'reason': 'Reconsider future dispatch'})
    assert tool(a, 'worker_status', {'attempt_id': worker['id']})['details']['state'] == 'running'
    assert tool(a, 'run_once', {'assignment_id': two['id']}, ok=False)['code'] == 'invalid_request'
    tool(a, 'resume', {'reason': 'Keep the bounded allocation'})
    tool(a, 'worker_usage', {'attempt_id': worker['id'], 'report': {'fits': 2, 'claimed_steps': 17}})
    runtime = tool(a, 'runtime_status')['allocations'][0]['details']
    assert runtime['wall_reserved_seconds'] == 72
    assert runtime['reservations'][0]['usage']['worker_reports'][0]['source'] == 'worker_report'
    assert 'shell compute' in runtime['reservations'][0]['usage']['unmetered']
    tool(a, 'cancel_attempt', {'attempt_id': worker['id']})
    until(lambda: tool(a, 'worker_status', {'attempt_id': worker['id']})['details']['state'] == 'cancelled')
    tool(a, 'run_once', {'assignment_id': two['id'], 'allocation_id': allocation['id']})
    assert len(tool(a, 'runtime_status')['allocations'][0]['details']['reservations']) == 2
    assert tool(a, 'run_once', {'assignment_id': one['id'], 'allocation_id': allocation['id']}, ok=False)['code'] == 'invalid_request'
    tool(a, 'stop_program', {'reason': 'Finished controller fixture'})


def test_wall_expiry_reaches_existing_cleanup_and_keeps_separate_allowance(manager):
    a = attach(manager)
    route = tool(a, 'approach', {'question_id': 'q', 'title': 'Deadline', 'reason': 'Short fixture'})
    work = tool(a, 'assignment', {'approach_id': route['id'], 'objective': 'Wait briefly', 'allocation': {'wall_minutes': .008}})
    tool(a, 'transition', {'record_id': work['id'], 'target': 'ready', 'reason': 'Deadline fixture'})
    allocation = tool(a, 'allocate', {'task_limit': 1, 'wall_minutes': .1, 'review_minutes': .01, 'cleanup_seconds': 2})
    worker = tool(a, 'run_once', {'assignment_id': work['id'], 'allocation_id': allocation['id']})
    ended = until(lambda: (r if (r := tool(a, 'worker_status', {'attempt_id': worker['id']}))['details']['state'] == 'cancelled' else None))
    assert ended['details']['cleanup'] == 'launcher_finally_completed'
    assert any(o['event'] == 'wall_expired' for o in ended['details']['observations'])
    d = tool(a, 'runtime_status')['allocations'][0]['details']
    assert d['cleanup_deadline'] == d['deadline'] + 2
    assert d['reservations'][0]['worker_seconds'] == .48


@pytest.mark.parametrize('action,cleanup', [('sys.exit(19)', 'unverified'), ('time.sleep(30)', 'launcher_finally_completed')])
def test_crash_and_cancelled_start_keep_task_charge_and_partial_evidence(manager, action, cleanup):
    set_launcher(manager, action)
    a = attach(manager)
    work = assignment(a)
    worker = tool(a, 'run_once', {'assignment_id': work['id']})
    if action.startswith('time'):
        until(lambda: Path(worker['details']['session_dir'], 'ready').exists())
        tool(a, 'cancel_attempt', {'attempt_id': worker['id']})
    ended = until(lambda: (r if (r := tool(a, 'worker_status', {'attempt_id': worker['id']}))['details']['state'] in ('failed', 'cancelled') else None))
    assert ended['details']['cleanup'].startswith(cleanup)
    assert ended['details']['artifacts']
    new = assignment(a, 'handoff')
    if cleanup == 'unverified':
        assert tool(a, 'run_once', {'assignment_id': new['id']}, ok=False)['code'] == 'invalid_request'
        tool(a, 'recover', {'handoff_attempt_id': worker['id'], 'reason': 'No world in fixture; retain unknown shutdown boundary'})
    assert len(tool(a, 'runtime_status')['allocations'][0]['details']['reservations']) == 1


def test_stale_lease_cancelled_popen_and_restart_never_replay_assignment(context):
    c = context
    m = local_manager(c)
    ready(m, 'new')
    allocation = m.controller.allocate('p', 2, 2, review_minutes=.1)
    allocation['details']['lease_until'] = 0
    m.controller.save(allocation)
    with pytest.raises(ProgramError, match='lease'):
        m.controller.run_once('p', 'new', allocation_id=allocation['id'])
    allocation['details']['lease_until'] = time.time() + 15
    m.controller.save(allocation)
    original_popen = subprocess.Popen
    def cancel_start(argv, *args, **kwargs):
        if argv[0] == 'git':
            return original_popen(argv, *args, **kwargs)
        raise KeyboardInterrupt
    with patch('hp_sim5_research.program_workers.subprocess.Popen', side_effect=cancel_start):
        with pytest.raises(KeyboardInterrupt):
            m.controller.run_once('p', 'new', allocation_id=allocation['id'])
    d = m.controller.current(allocation['id'])['details']
    assert len(d['reservations']) == 1
    attempt = c['store'].get_record('p', d['reservations'][0]['attempt_id'])
    assert attempt['details']['state'] == 'launch_failed'
    restarted = ProgramManager(c['store'], c['root'], 'p', 'fresh-manager', c['tmp'] / 'fresh')
    assert restarted.controller.current(allocation['id'])['details']['state'] == 'interrupted'
    assert not restarted.workers.children
    with pytest.raises(ProgramError):
        restarted.controller.run_once('p', 'new')


def test_reused_pid_and_live_snapshot_do_not_authorize_adoption_or_signals(context, monkeypatch):
    c = context
    m = local_manager(c)
    original = copy.deepcopy(c['attempt'])
    d = original['details']
    d.update(state='running', cleanup='not_requested', exited_at=None)
    d['owner']['process'] = dict(process_identity(os.getpid()), birth='stale-owner')
    d['process'] = dict(process_identity(os.getpid()), birth='reused-pid')
    m.workers._save('p', original['id'], d)
    def no_signal(*args):
        pytest.fail('Recovery signalled a historical PID')
    monkeypatch.setattr(os, 'kill', no_signal)
    monkeypatch.setattr(os, 'killpg', no_signal)
    result = m.controller.recover('p')
    observation = result['reconciled'][0]
    assert observation['launcher_birth_matches'] is False
    assert observation['supervisor']['state'] == 'unknown'
    assert observation['adopted'] is False
    historical = c['store'].get_record('p', original['id'])
    assert historical['details']['state'] == 'interrupted'
    assert historical['details']['cleanup'].startswith('unknown')
    assert m.workers.cancel('p', original['id'])['details']['state'] == 'interrupted'


def test_review_retry_limit_historical_funding_roundtrip_and_stop_during_review(context):
    c = context
    submission = submit(c)
    m = local_manager(c)
    funded = m.controller.fund_review('p', submission['id'], wall_minutes=.1, retry_limit=1)
    # Configure a real separate fixture client with its existing test helper.
    planned = reviewer(c, submission, mode='hold', wall_minutes=.001)
    c['controller'].cancel('p', planned['id'])
    run = m.controller.review_start('p', submission['id'], wall_minutes=.03)
    m.reviews.cancel('p', run['id'])
    until(lambda: m.reviews.status('p', run['id'])['details']['state'] == 'cancelled')
    with pytest.raises(ProgramError, match='retry limit'):
        m.controller.review_start('p', submission['id'], wall_minutes=.03)
    # Reconciliation preserves the fixed history through an exported restore.
    snapshot = c['store'].snapshot('p')
    with ProgramStore(c['tmp'] / 'restored', create=True) as restored:
        restored.import_snapshot(snapshot)
        assert restored.get_record('p', funded['id'])['details']['reservations'][0]['worker_seconds'] == 0
    m.stop('p', 'Stop after a cancelled review')
    assert not [r for r in c['store'].snapshot('p')['records'] if r['kind'] == 'admission']


def test_stop_during_review_and_late_completed_review_cannot_admit(context):
    c = context
    submission = submit(c)
    m = local_manager(c)
    m.controller.fund_review('p', submission['id'], wall_minutes=1, retry_limit=2)
    reviewer(c, submission, mode='hold', wall_minutes=.001)
    # Stop owns and cancels even when the review was launched just before stop.
    run = m.controller.review_start('p', submission['id'], wall_minutes=.1)
    m.stop('p', 'Human stops during the review')
    until(lambda: m.reviews.status('p', run['id'])['details']['state'] == 'cancelled')
    late = reviewer(c, submission)
    with pytest.raises(ProgramError, match='Late review'):
        m.controller.review_apply('p', late['details']['review_id'])
    assert not [r for r in c['store'].snapshot('p')['records'] if r['kind'] == 'admission']


def test_recovered_review_transfers_backlog_and_recovers_admission_cache(context):
    c = context
    submission = submit(c)
    m = local_manager(c)
    old = m.controller.fund_review('p', submission['id'], wall_minutes=1)
    old['details']['state'] = 'interrupted'
    m.controller.save(old)
    fresh = m.controller.fund_review('p', submission['id'], wall_minutes=1)
    assert m.controller.current(old['id'])['details']['reservations'][0]['superseded_by'] == fresh['id']
    run = reviewer(c, submission)
    a = m.controller.current(fresh['id'])
    a['details']['reservations'][0]['reviews'].append({'run_id': run['id'], 'reserved_seconds': 30, 'state': 'completed'})
    m.controller.save(a)
    # Simulate a crash after durable admission but before the reservation cache.
    admission = m.reviews.apply('p', run['details']['review_id'])
    m.poll()
    d = m.controller.current(fresh['id'])['details']['reservations'][0]
    assert d['admission_id'] == admission['id'] and d['review_pending'] is False
    ready(m, 'next')
    # Validation reaches the new assignment rather than the old review backlog.
    current = m.controller.allocate('p', 1, 1, review_minutes=.1)
    assert m.controller._ready(current, 'next') == .6


def test_launcher_deadline_survives_absent_host_and_second_cancel_during_cleanup(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location('deadline_launcher', ROOT / 'scripts/research_agent.py')
    launcher = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launcher)
    services = []
    class Service:
        def __init__(self, *args, **kwargs):
            services.append(self)
            self.closed = False
        def start(self):
            time.sleep(5)
        def close(self):
            # The host can send cancellation as the launcher's alarm expires.
            # That second interrupt must not abort this existing cleanup path.
            os.kill(os.getpid(), signal.SIGTERM)
            time.sleep(.01)
            self.closed = True
    monkeypatch.setattr(launcher, 'RuntimeService', Service)
    monkeypatch.setattr(launcher, 'ensure_native_physics', lambda: None)
    monkeypatch.setattr(launcher, 'write_index', lambda *_: None)
    session = tmp_path / 'deadline-session'
    monkeypatch.setattr(sys, 'argv', ['research_agent.py', '--codex', '--batch', '--viewer', 'none', '--sidebar-section', 'none',
                                    '--prompt', 'Bound startup independently of tool host', '--session-dir', str(session),
                                    '--wall-deadline', str(time.time() + .3)])
    assert launcher.main() == 130
    assert services[0].closed
    assert json.loads((session / 'exit.json').read_text())['returncode'] == 130
    assert signal.getitimer(signal.ITIMER_REAL)[0] == 0


def test_review_guard_deadline_survives_absent_host_polling(context):
    c = context
    submission = submit(c)
    run = reviewer(c, submission, mode='hold', wall_minutes=.005)
    process = c['controller'].children[run['id']]
    process.wait(timeout=3)
    ended = c['controller'].status('p', run['id'])
    assert ended['details']['state'] == 'timed_out'
    assert Path(ended['details']['directory'], 'wall-expired').is_file()
    assert ended['details']['guard_argv'][0] == sys.executable
    assert ended['details']['argv'][0] == 'codex'


def test_worker_retry_limit_requires_distinct_bounded_assignments(manager):
    set_launcher(manager, "(session / 'exit.json').write_text(json.dumps({'returncode': 19}))\nsys.exit(19)")
    a = attach(manager)
    first, second, third = [assignment(a, i) for i in ('first', 'second', 'third')]
    allocation = tool(a, 'allocate', {'task_limit': 3, 'wall_minutes': 5, 'review_minutes': .1, 'retry_limit': 2})
    w1 = tool(a, 'run_once', {'assignment_id': first['id'], 'allocation_id': allocation['id']})
    until(lambda: tool(a, 'worker_status', {'attempt_id': w1['id']})['details']['state'] == 'failed')
    w2 = tool(a, 'run_once', {'assignment_id': second['id'], 'allocation_id': allocation['id'], 'retry_of': w1['id']})
    until(lambda: tool(a, 'worker_status', {'attempt_id': w2['id']})['details']['state'] == 'failed')
    error = tool(a, 'run_once', {'assignment_id': third['id'], 'allocation_id': allocation['id'], 'retry_of': w2['id']}, ok=False)
    assert 'retry limit' in error['message']
    assert len(tool(a, 'runtime_status')['allocations'][0]['details']['reservations']) == 2


def test_several_assignments_real_cli_transcript_and_fresh_host_continuation(manager, tmp_path, monkeypatch):
    set_launcher(manager, "(session / 'exit.json').write_text(json.dumps({'returncode': 0}))\nsys.exit(0)")
    binaries = tmp_path / 'fixture-bin'
    binaries.mkdir()
    client = binaries / 'codex'
    response = copy.deepcopy(CASES['scoped_negative']['review'])
    client.write_text(f'#!{sys.executable}\n' + f'''import hashlib, json, re, sys
from pathlib import Path
prompt = sys.stdin.read()
directory = Path(re.search(r'Target directory: (.+)', prompt).group(1))
manifest = json.loads((directory / 'manifest.json').read_text())
for asset in manifest['assets']:
    assert hashlib.sha256((directory / asset['id']).read_bytes()).hexdigest() == asset['sha256']
response = {response!r}
response.update(submission_id=re.search(r'Submission: (.+)', prompt).group(1), submission_sha256=re.search(r'SHA-256: (.+)', prompt).group(1), checks=[{{'kind': 'inspected', 'description': 'Inspected frozen synthetic fixture bytes; no numerical reproduction', 'asset_ids': [a['id'] for a in manifest['assets']]}}])
Path(sys.argv[sys.argv.index('--output-last-message') + 1]).write_text(json.dumps(response))
''')
    client.chmod(0o755)
    monkeypatch.setenv('PATH', str(binaries) + os.pathsep + os.environ['PATH'])
    a = attach(manager)
    transcript = {'kind': 'actual CLI and file host with fixture worker/reviewer children', 'live_release_evidence': False,
                  'commands': [], 'assignments': [], 'host_generations': [a['host_id']]}
    def invoke(operation, parameters=None):
        argv = [str(ROOT / 'hp-sim5-research-program'), 'tool', a['connection'], operation, '--params', json.dumps(parameters or {})]
        result = subprocess.run(argv, capture_output=True, text=True, timeout=15)
        payload = json.loads(result.stdout)
        transcript['commands'].append({'argv': argv, 'returncode': result.returncode, 'stdout': payload, 'stderr': result.stderr})
        assert result.returncode == 0, result.stderr + result.stdout
        assert payload['ok'] is True
        return payload['result']
    for identity, objective in [('membership', 'Inspect frozen membership disagreement'),
                                ('common-fold', 'Look for compatible common unseen folds'),
                                ('model-scope', 'Inspect the saved model adequacy limits')]:
        route = invoke('approach', {'question_id': 'q', 'title': objective, 'reason': 'Distinct finite discriminator'})
        work = invoke('assignment', {'approach_id': route['id'], 'objective': objective,
                                     'allocation': {'wall_minutes': .5}, 'record_id': identity})
        invoke('transition', {'record_id': identity, 'target': 'ready', 'reason': 'Manager selected this distinct assignment'})
        transcript['assignments'].append({'id': identity, 'objective': objective})
    funded = invoke('allocate', {'task_limit': 2, 'wall_minutes': 3, 'review_minutes': .2})
    invoke('run_within_allocation', {'allocation_id': funded['id'], 'assignment_ids': ['membership', 'common-fold']})
    for index, identity in enumerate(('membership', 'common-fold', 'model-scope')):
        if index == 2:
            invoke('shutdown')
            until(lambda: not process_live(a['process']))
            a = attach(manager)
            transcript['host_generations'].append(a['host_id'])
            assert transcript['host_generations'][0] != a['host_id']
            restored = invoke('overview')
            assert len([r for r in restored['records'] if r['kind'] == 'admission']) == 2
            invoke('allocate', {'task_limit': 1, 'wall_minutes': 2, 'review_minutes': .2})
            invoke('run_once', {'assignment_id': identity})
        def ended():
            matches = [r for r in tool(a, 'worker_sessions') if r['details']['assignment_id'] == identity]
            return matches[-1] if matches and matches[-1]['details']['state'] == 'submitted' else None
        attempt = until(ended)
        invoke('worker_status', {'attempt_id': attempt['id']})
        # Submitted result holds the queue at its separate-review boundary.
        sessions = invoke('worker_sessions')
        assert len(sessions) == index + 1
        claim = invoke('claim', {'title': 'Synthetic scoped negative for ' + identity,
                                'scope': {'evidence_type': 'synthetic', 'configuration': identity}})
        selected = []
        for role in ('report', 'abstract', 'evidence'):
            p = tmp_path / f'{identity}-{role}.txt'
            p.write_text('Synthetic fixture only. ' + role + ' ' + identity)
            selected.append({'id': role, 'role': role, 'path': str(p)})
        submission = invoke('submit', {'attempt_id': attempt['id'], 'claim_id': claim['id'],
                                      'base_revision': attempt['details']['source_revision'], 'config': {'fixture': identity},
                                      'input_identities': [identity], 'selected': selected, 'producer_checks': []})
        run = invoke('review_start', {'submission_id': submission['id'], 'wall_minutes': .05})
        completed = until(lambda: (r if (r := tool(a, 'review_status', {'run_id': run['id']}))['details']['state'] == 'completed' else None))
        invoke('review_status', {'run_id': run['id']})
        if index == 0:
            invoke('pause', {'reason': 'Fresh chat inspects the first reviewed result'})
        admitted = invoke('review_apply', {'review_id': completed['details']['review_id']})
        assert admitted['details']['decision'] == 'keep'
        if index == 0:
            assert attach(manager)['host_id'] == a['host_id']
            invoke('overview')
            invoke('resume', {'reason': 'Continue the second distinct funded assignment'})
    final = invoke('overview')
    assert len([r for r in final['records'] if r['kind'] == 'admission']) == 3
    assert final['program']['state'] == 'active'
    assert all(x['details']['state'] != 'active' for x in final['runtime']['allocations'])
    invoke('stop_program', {'reason': 'Fixture transcript complete; scientific program remains unresolved'})
    transcript['final_snapshot'] = final
    output = manager['root'] / 'controller-transcript.json'
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(transcript, indent=2) + '\n')


def test_explicit_steering_holds_finite_queue_without_cancelling_running_worker(manager):
    a = attach(manager)
    first, second = assignment(a, 'running'), assignment(a, 'pending')
    grant = tool(a, 'allocate', {'task_limit': 2, 'wall_minutes': 3, 'review_minutes': .1})
    tool(a, 'run_within_allocation', {'allocation_id': grant['id'], 'assignment_ids': [first['id'], second['id']]})
    worker = until(lambda: (items[0] if (items := tool(a, 'worker_sessions')) else None))
    tool(a, 'steering', {'text': 'Use a shorter pending comparison', 'intent': 'instruction',
                         'next_assignment_id': second['id'], 'allocation': {'wall_minutes': .5}})
    runtime = tool(a, 'runtime_status')['allocations'][0]['details']
    assert runtime['queue'] == []
    assert tool(a, 'worker_status', {'attempt_id': worker['id']})['details']['state'] == 'running'
    assert tool(a, 'overview')['next_allocation']['allocation'] == {'wall_minutes': .5}


def test_actual_host_crash_reconciles_orphan_without_adoption_or_duplicate_dispatch(manager):
    a = attach(manager)
    work = assignment(a, 'before-crash')
    child = tool(a, 'run_once', {'assignment_id': work['id']})
    until(lambda: Path(child['details']['session_dir'], 'ready').exists())
    try:
        assert process_live(a['process'])
        os.kill(a['process']['pid'], signal.SIGKILL)
        until(lambda: not process_live(a['process']))
        fresh = attach(manager)
        assert fresh['host_id'] != a['host_id']
        reconciled = tool(fresh, 'worker_status', {'attempt_id': child['id']})
        assert reconciled['details']['state'] == 'interrupted'
        assert reconciled['details']['recovery']['launcher_birth_matches'] is True
        assert reconciled['details']['recovery']['adopted'] is False
        assert process_live(child['details']['process'])
        next_work = assignment(fresh, 'bounded-handoff')
        assert tool(fresh, 'run_once', {'assignment_id': next_work['id']}, ok=False)['code'] == 'invalid_request'
        # Cancel does not use exported historical PIDs as authority.
        assert tool(fresh, 'cancel_attempt', {'attempt_id': child['id']})['details']['state'] == 'interrupted'
        assert process_live(child['details']['process'])
        tool(fresh, 'recover', {'handoff_attempt_id': child['id'], 'reason': 'Fixture has no physics; explicitly retain unknown stop boundary'})
        next_child = tool(fresh, 'run_once', {'assignment_id': next_work['id']})
        assert next_child['id'] != child['id']
        assert len(tool(fresh, 'worker_sessions')) == 2
    finally:
        # Test-owned orphan fixture cleanup, never production controller adoption.
        if process_live(child['details']['process']):
            os.killpg(child['details']['process']['pid'], signal.SIGKILL)
        until(lambda: not process_live(child['details']['process']))


def test_empty_allocation_roundtrip_and_failed_review_transfer_are_atomic(context, monkeypatch):
    c = context
    submission = submit(c)
    m = local_manager(c)
    initial = m.controller.allocate('p', 1, 2, review_minutes=.1)
    with ProgramStore(c['tmp'] / 'empty-restore', create=True) as restored:
        restored.import_snapshot(c['store'].snapshot('p'))
        assert restored.get_record('p', initial['id'])['details']['wall_reserved_seconds'] == 0
    m.controller.stop_allocation('p', initial['id'], 'Prepare saved evidence funding')
    old = m.controller.fund_review('p', submission['id'], wall_minutes=1)
    old['details']['state'] = 'interrupted'
    m.controller.save(old)
    insert = c['store']._insert
    def broken_insert(*args, **kwargs):
        result = insert(*args, **kwargs)
        if args[1] == 'runtime_allocation':
            raise RuntimeError('Fixture crash during atomic funding')
        return result
    before = c['store'].snapshot('p')
    monkeypatch.setattr(c['store'], '_insert', broken_insert)
    with pytest.raises(RuntimeError, match='atomic funding'):
        m.controller.fund_review('p', submission['id'], wall_minutes=1)
    assert c['store'].snapshot('p') == before


def test_invalid_worker_options_do_not_spend_or_strand_a_startup_reservation(manager):
    a = attach(manager)
    work = assignment(a)
    for extra in ({'settings': {'unsupported': True}}, {'instructions': 42}):
        assert tool(a, 'run_once', dict(assignment_id=work['id'], **extra), ok=False)['code'] == 'invalid_request'
    assert tool(a, 'runtime_status')['allocations'] == []
    child = tool(a, 'run_once', {'assignment_id': work['id']})
    assert child['details']['state'] in ('running', 'starting')


@pytest.mark.parametrize('reachable', [True, False])
def test_recovery_reads_current_supervisor_and_does_not_adopt_final_snapshot(context, monkeypatch, reachable):
    c = context
    m = local_manager(c)
    record = copy.deepcopy(c['attempt'])
    d = record['details']
    d.update(state='running', cleanup='not_requested', exited_at=None)
    d['owner']['process'] = dict(process_identity(os.getpid()), birth='lost-controller')
    session = Path(d['session_dir'])
    session.mkdir(parents=True, exist_ok=True)
    endpoint = 'http://127.0.0.1:12345'
    (session / 'session.json').write_text(json.dumps({'session_dir': str(session), 'launcher_session_id': 'fixture-launcher',
                                                     'repo': str(c['root']), 'branch': 'fixture', 'source_revision': c['base']}))
    (session / 'launch.json').write_text(json.dumps({'runtime_endpoint': endpoint, 'runtime_status': {'session_id': 'saved-final-world'}}))
    (session / 'connection.json').write_text(json.dumps({'runtime_endpoint': endpoint, 'session_id': 'saved-final-world',
                                                        'repo': str(c['root']), 'runtime_token': 'fixture-token'}))
    observations = []
    def current_status(url, operation, *, token, timeout):
        observations.append((url, operation, timeout))
        assert token == 'fixture-token'
        if not reachable:
            raise OSError('Fixture supervisor unavailable')
        return {'session_id': 'current-reset-world', 'active_job': 'fixture-running-job', 'recording': True}
    monkeypatch.setattr('hp_sim5_research.services.request', current_status)
    m.workers._save('p', record['id'], d)
    result = m.controller.recover('p')['reconciled'][0]
    assert observations == [(endpoint, 'status', .25)]
    assert result['adopted'] is False
    if reachable:
        assert result['supervisor']['state'] == 'live_observed'
        assert result['supervisor']['world_matches_saved'] is False
        assert result['supervisor']['observation']['active_job'] == 'fixture-running-job'
    else:
        assert result['supervisor']['state'] == 'unknown'
    assert c['store'].get_record('p', record['id'])['details']['state'] == 'interrupted'


def test_stop_preserves_unverified_review_boundary_without_signalling_reused_identity(context):
    c = context
    submission = submit(c)
    m = local_manager(c)
    m.controller.fund_review('p', submission['id'], wall_minutes=1)
    reviewer(c, submission, mode='hold', wall_minutes=.001)
    run = m.controller.review_start('p', submission['id'], wall_minutes=.1)
    run['details']['process']['birth'] = 'reused-review-pid'
    m.reviews._save('p', run['id'], run['details'])
    with patch('hp_sim5_research.program_review.os.killpg', side_effect=AssertionError('Unverified process was signalled')):
        stopped = m.stop('p', 'Stop with a changed process birth identity')
    assert stopped['program']['state'] == 'stopped'
    assert stopped['manager']['cleanup_errors']
    assert c['store'].get_record('p', run['id'])['details']['state'] == 'interrupted'
