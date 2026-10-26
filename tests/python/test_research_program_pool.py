"""Bounded pool checks with real child processes/Git worktrees and synthetic worlds."""
import concurrent.futures
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
from unittest.mock import patch

import pytest

from hp_sim5_research.program import ProgramError, ProgramStore
from hp_sim5_research.program_manager import ProgramManager
from hp_sim5_research import program_pool as pool
from hp_sim5_research.program_workers import process_identity, process_live
from .test_research_program_manager import manager, attach, assignment, tool, until, ROOT
from .test_research_program_review import context, submit, reviewer
from .test_research_program_controller import local_manager, ready


def configure(a, workers=2, backlog=2):
    return tool(a, 'configure_pool', {'worker_limit': workers, 'review_backlog_limit': backlog,
                                     'reason': 'Finite fixture allocation'})


def grant(a, tasks=3, workers=2, backlog=2):
    return tool(a, 'allocate', {'task_limit': tasks, 'wall_minutes': 5, 'review_minutes': .1,
                                'worker_limit': workers, 'review_backlog_limit': backlog})


def isolated_launcher(c):
    # Exercise the production launcher's actual Git worktree/link function.
    # Only the supervisor/world and coding client are fake.
    (c['root'] / '.venv').symlink_to(Path(sys.executable).parent.parent, target_is_directory=True)
    script = c['root'] / 'hp-sim5-research-agent'
    script.write_text(f'#!{sys.executable}\n' + '''import json, os, runpy, signal, sys, time, uuid
from pathlib import Path
args = sys.argv[1:]
session = Path(args[args.index('--session-dir') + 1])
module = runpy.run_path(str(Path.cwd() / 'scripts/research_agent.py'))
create = module['create_worktree']
create.__globals__['ROOT'] = Path.cwd()
worktree = create(session.name, Path.cwd().parent / 'worktrees')
session.mkdir(parents=True)
(session / 'session.json').write_text(json.dumps({'session_dir': str(session),
 'launcher_session_id': uuid.uuid4().hex, 'repo': str(worktree['path']),
 'branch': worktree['branch'], 'source_revision': worktree['base']}))
(session / 'launch.json').write_text(json.dumps({'runtime_status': {'session_id': 'synthetic-' + uuid.uuid4().hex}}))
(session / 'partial.txt').write_text('retained fixture evidence')
def stop(*_):
 (session / 'exit.json').write_text(json.dumps({'returncode': 130}))
 sys.exit(130)
signal.signal(signal.SIGTERM, stop)
(session / 'ready').write_text('fake world; real worktree')
while not (session / 'finish').exists():
 time.sleep(.01)
(session / 'exit.json').write_text(json.dumps({'returncode': 0}))
''')
    script.chmod(0o755)


def sessions_ready(a, count):
    def check():
        sessions = tool(a, 'worker_sessions')
        return sessions if len(sessions) == count and all(Path(r['details']['session_dir'], 'ready').exists() for r in sessions) else None
    return until(check)


def test_two_routes_real_worktrees_artifacts_worlds_atomic_queue_and_cancel(manager):
    isolated_launcher(manager)
    a = attach(manager)
    configure(a)
    first, second, third = [assignment(a, i) for i in ('one', 'two', 'three')]
    allocation = grant(a)
    tool(a, 'run_within_allocation', {'allocation_id': allocation['id'], 'assignment_ids': [first['id'], second['id'], third['id']]})
    workers = sessions_ready(a, 2)
    workers = tool(a, 'worker_sessions')
    for field in ('session_dir', 'attempt_dir', 'runtime_session_id', 'worktree_id', 'launcher_session_id'):
        assert len({r['details'][field] for r in workers}) == 2
    assert len({r['details']['worktree']['branch'] for r in workers}) == 2
    for r in workers:
        d = r['details']
        assert '--worktree' in d['argv']
        assert Path(d['worktree_id'], 'output').is_symlink()
        assert Path(d['worktree_id'], 'output').resolve() == manager['root'] / 'output'
        assert 'Shared builds' in Path(d['attempt_dir'], 'prompt.txt').read_text()
    runtime = tool(a, 'runtime_status')
    assert runtime['pool']['store_workers_reserved'] == 2
    assert runtime['pool']['store_future_review_obligations'] == 2
    assert len(runtime['allocations'][0]['details']['queue']) == 1
    assert runtime['allocations'][0]['details']['wall_reserved_seconds'] == 132
    assert tool(a, 'run_once', {'assignment_id': first['id'], 'allocation_id': allocation['id']}, ok=False)['code'] == 'invalid_request'
    # Hold selection before cancellation frees a slot.
    tool(a, 'steering', {'text': 'Keep partial evidence and hold queued work', 'intent': 'instruction'})
    tool(a, 'cancel_attempt', {'attempt_id': workers[0]['id']})
    until(lambda: tool(a, 'worker_status', {'attempt_id': workers[0]['id']})['details']['state'] == 'cancelled')
    assert process_live(workers[1]['details']['process'])
    assert Path(workers[0]['details']['session_dir'], 'partial.txt').read_text() == 'retained fixture evidence'
    assert tool(a, 'runtime_status')['allocations'][0]['details']['queue'] == []
    tool(a, 'stop_program', {'reason': 'Stop every owned pool worker'})
    until(lambda: tool(a, 'worker_status', {'attempt_id': workers[1]['id']})['details']['state'] == 'cancelled')
    transcript = {'scope': 'Actual CLI/file-host, real Git worktrees, fake supervisor/world/client; no production science',
        'pool': runtime['pool'], 'allocation': runtime['allocations'][0],
        'workers': [{'id': r['id'], 'assignment_id': r['details']['assignment_id'],
                     'session_dir': r['details']['session_dir'], 'attempt_dir': r['details']['attempt_dir'],
                     'synthetic_world_id': r['details']['runtime_session_id'], 'worktree': r['details']['worktree'],
                     'argv': r['details']['argv']} for r in workers],
        'final_states': [r['details']['state'] for r in tool(a, 'worker_sessions')]}
    (manager['root'] / 'pool-evidence.json').write_text(json.dumps(transcript, indent=2) + '\n')


def test_pool_alone_preserves_default_future_review_backpressure(manager):
    a = attach(manager)
    configure(a, backlog=1)
    one, two = assignment(a, 'one'), assignment(a, 'two')
    allocation = grant(a, tasks=2, backlog=2)
    first = tool(a, 'run_once', {'assignment_id': one['id'], 'allocation_id': allocation['id']})
    until(lambda: Path(first['details']['session_dir'], 'ready').exists())
    error = tool(a, 'run_once', {'assignment_id': two['id'], 'allocation_id': allocation['id']}, ok=False)
    assert 'review backlog' in error['message']
    configure(a, backlog=2)
    # Per-allocation default also remains one, regardless of the store cap.
    tool(a, 'run_once', {'assignment_id': two['id'], 'allocation_id': allocation['id']})
    assert len(tool(a, 'worker_sessions')) == 2


def other_assignment(a, identity='other-work'):
    route = tool(a, 'approach', {'question_id': 'other-q', 'title': 'Other route', 'reason': 'Independent program'})
    work = tool(a, 'assignment', {'approach_id': route['id'], 'objective': 'Wait independently',
                                 'record_id': identity, 'allocation': {'wall_minutes': 1}})
    tool(a, 'transition', {'record_id': work['id'], 'target': 'ready', 'reason': 'Fixture'})
    return work


def test_global_cap_backlog_and_stop_are_scoped_to_program(manager):
    a, b = attach(manager), attach(manager, 'other')
    configure(a)
    wa, wb, extra = assignment(a, 'a'), other_assignment(b), assignment(a, 'extra')
    aa, ab = grant(a, tasks=2), grant(b, tasks=1, workers=1)
    first = tool(a, 'run_once', {'assignment_id': wa['id'], 'allocation_id': aa['id']})
    second = tool(b, 'run_once', {'assignment_id': wb['id'], 'allocation_id': ab['id']})
    until(lambda: Path(second['details']['session_dir'], 'ready').exists())
    assert 'pool is full' in tool(a, 'run_once', {'assignment_id': extra['id'], 'allocation_id': aa['id']}, ok=False)['message']
    assert tool(a, 'cancel_attempt', {'attempt_id': second['id']}, ok=False)['code'] == 'invalid_request'
    tool(a, 'stop_program', {'reason': 'Only this program stops'})
    until(lambda: tool(a, 'worker_status', {'attempt_id': first['id']})['details']['state'] == 'cancelled')
    assert tool(b, 'worker_status', {'attempt_id': second['id']})['details']['state'] == 'running'
    assert process_live(second['details']['process'])
    assert tool(b, 'overview')['program']['state'] == 'active'


def test_many_programs_cannot_accumulate_unbounded_reviews(manager):
    isolated_launcher(manager)
    a, b = attach(manager), attach(manager, 'other')
    configure(a, backlog=1)
    first = tool(a, 'run_once', {'assignment_id': assignment(a)['id']})
    sessions_ready(a, 1)
    Path(first['details']['session_dir'], 'finish').touch()
    until(lambda: tool(a, 'worker_status', {'attempt_id': first['id']})['details']['state'] == 'submitted')
    extra = other_assignment(b)
    assert 'review backlog' in tool(b, 'run_once', {'assignment_id': extra['id']}, ok=False)['message']
    assert tool(b, 'runtime_status')['pool']['store_future_review_obligations'] == 1


def test_simultaneous_file_requests_preserve_two_charges_and_steering(manager):
    a = attach(manager)
    configure(a)
    works = [assignment(a, i) for i in ('one', 'two', 'three')]
    allocation = grant(a)
    # Raw CLI results permit either admission or backpressure without imposing order.
    def request_work(work):
        from hp_sim5_research.program_host import request
        return request(a, 'run_once', {'assignment_id': work['id'], 'allocation_id': allocation['id']})
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
        calls = [executor.submit(request_work, work) for work in works]
        note = executor.submit(tool, a, 'steering', {'text': 'Keep this concurrent note'})
        outcomes = [f.result() for f in calls]
        note.result()
    assert sum(r['ok'] for r in outcomes) == 2
    runtime = tool(a, 'runtime_status')['allocations'][0]['details']
    assert len(runtime['reservations']) == 2 and runtime['wall_reserved_seconds'] == 132
    assert len({r['assignment_id'] for r in runtime['reservations']}) == 2
    assert any(r['kind'] == 'human_input' and r['title'] == 'Keep this concurrent note' for r in tool(a, 'overview')['records'])


def test_shared_build_writes_need_exclusive_capacity_and_status_documents_boundary(manager):
    a = attach(manager)
    configure(a)
    one, two = assignment(a, 'one'), assignment(a, 'two')
    allocation = grant(a, tasks=2)
    first = tool(a, 'run_once', {'assignment_id': one['id'], 'allocation_id': allocation['id'], 'settings': {'shared_runtime_write': True}})
    until(lambda: Path(first['details']['session_dir'], 'ready').exists())
    assert 'Shared runtime' in tool(a, 'run_once', {'assignment_id': two['id'], 'allocation_id': allocation['id']}, ok=False)['message']
    runtime = tool(a, 'runtime_status')
    assert 'RRF/build' in runtime['shared_filesystem']
    assert 'Same-UID' in tool(a, 'overview')['brief']
    with ProgramStore(manager['store']) as store:
        with pytest.raises(ProgramError, match='Shared runtime'):
            pool.check_review(store)


def test_unopened_program_saved_worker_and_startup_reservations_hold_global_capacity(manager):
    a = attach(manager)
    configure(a)
    with ProgramStore(manager['store']) as store:
        # A saved process record belongs to an unopened program. No live process
        # is required and no saved PID may be signalled to make room.
        m = ProgramManager(store, manager['root'], 'other', 'unopened', manager['root'] / 'unopened')
        route = store.add_approach('other', 'other-q', 'Saved route', 'Fixture')
        work = store.add_assignment('other', route['id'], 'Charged startup', {'wall_minutes': 1})
        store.transition('other', work['id'], 'ready', 'Fixture')
        allocation = m.controller.allocate('other', 1, 3, review_minutes=.1)
        d = allocation['details']
        d['owner']['process']['birth'] = 'dead-controller-birth'
        d['reservations'].append({'assignment_id': work['id'], 'attempt_id': None, 'state': 'starting',
            'worker_seconds': 60, 'review_seconds': 6, 'reviews': [], 'review_pending': False})
        d['wall_reserved_seconds'] = 66
        m.controller.save(allocation)
    assert tool(a, 'runtime_status')['pool']['store_workers_reserved'] == 1
    configure(a, workers=1, backlog=2)
    new = assignment(a)
    assert 'pool is full' in tool(a, 'run_once', {'assignment_id': new['id']}, ok=False)['message']
    # Reopen that program with a new controller: it becomes interrupted and
    # still blocks dispatch until the explicit unknown-startup handoff.
    b = attach(manager, 'other')
    assert tool(b, 'runtime_status')['allocations'][0]['details']['state'] == 'interrupted'
    assert 'handoff' in tool(a, 'run_once', {'assignment_id': new['id']}, ok=False)['message']
    tool(b, 'recover', {'handoff_allocation_id': allocation['id'], 'reason': 'Fixture had no child/world; preserve unknown boundary'})
    assert tool(a, 'runtime_status')['pool']['store_workers_reserved'] == 0


def test_unknown_cleanup_in_other_program_blocks_dispatch_without_signalling(manager):
    a, b = attach(manager), attach(manager, 'other')
    configure(a)
    work = other_assignment(b)
    worker = tool(b, 'run_once', {'assignment_id': work['id']})
    until(lambda: Path(worker['details']['session_dir'], 'ready').exists())
    os.kill(worker['details']['process']['pid'], signal.SIGKILL)
    until(lambda: tool(b, 'worker_status', {'attempt_id': worker['id']})['details']['state'] == 'failed')
    extra = assignment(a)
    assert 'unknown cleanup' in tool(a, 'run_once', {'assignment_id': extra['id']}, ok=False)['message']
    tool(b, 'recover', {'handoff_attempt_id': worker['id'], 'reason': 'No real world in fixture, retain partial logs'})
    assert tool(a, 'run_once', {'assignment_id': extra['id']})['details']['state'] == 'running'


def test_running_attempt_premise_conflicts_preserve_partial_evidence_and_source_identity(manager):
    a = attach(manager)
    configure(a)
    works = [assignment(a, i) for i in ('dependent', 'independent')]
    shared = tool(a, 'artifact', {'reference': 'fixture/shared.csv', 'sha256': '1' * 64,
        'evidence_type': 'synthetic', 'source_revision': 'fixture', 'provenance': {'input_identities': ['shared-input']}})
    claim = tool(a, 'claim', {'title': 'Shared premise', 'scope': {'configuration': 'synthetic'}})
    tool(a, 'derive', {'conclusion_id': claim['id'], 'premise_ids': [], 'evidence_ids': [shared['id']], 'reason': 'Fixture support'})
    tool(a, 'route_plan', {'approach_id': works[0]['details']['approach_id'], 'hypothesis': 'Use premise', 'scope': {'configuration': 'synthetic'},
        'priority': 1, 'estimated_cost': {'wall_minutes': 1}, 'uncertainty': 'fixture', 'decision_value': 'fixture',
        'novelty': 'fixture', 'observation': 'fixture', 'reopening_condition': 'premise changes', 'reason': 'Fixture', 'dependency_ids': [claim['id']]})
    allocation = grant(a, tasks=2)
    workers = [tool(a, 'run_once', {'assignment_id': w['id'], 'allocation_id': allocation['id']}) for w in works]
    sessions_ready(a, 2)
    findings = [tool(a, 'assess', {'subject_id': claim['id'], 'outcome': outcome, 'evidence_ids': [shared['id']],
                                 'reason': 'Conflicting synthetic producer report', 'actor': actor})
                for outcome, actor in [('supported', 'worker-one'), ('rejected', 'worker-two')]]
    conflict_query = tool(a, 'query', {'query': 'support', 'subject_id': claim['id']})
    assert {r['id'] for r in findings} <= {r['id'] for r in conflict_query['records']}
    assert [r['id'] for r in conflict_query['sources']] == [shared['id']]
    event = tool(a, 'review_event', {'subject_id': shared['id'], 'event': 'withdrawal', 'reason': 'Other result invalidates shared input'})
    overview = tool(a, 'overview')
    affected = tool(a, 'worker_status', {'attempt_id': workers[0]['id']})
    conflict = affected['details']['premise_conflicts'][0]
    assert conflict['event_id'] == event['id'] and conflict['attempt_id'] == workers[0]['id']
    assert conflict['assignment_id'] == works[0]['id'] and conflict['state'] == 'review_required'
    assert tool(a, 'worker_status', {'attempt_id': workers[1]['id']})['details']['premise_conflicts'] == []
    assert affected['details']['state'] == 'running' and affected['details']['artifacts']
    assert any(r['id'] == shared['id'] and r['details']['provenance']['input_identities'] == ['shared-input'] for r in overview['records'])
    tool(a, 'overview')
    assert len(tool(a, 'worker_status', {'attempt_id': workers[0]['id']})['details']['premise_conflicts']) == 1
    with ProgramStore(manager['store']) as store, ProgramStore(manager['root'] / 'restored', create=True) as restored:
        restored.import_snapshot(store.snapshot('p'))
        assert restored.get_record('p', workers[0]['id'])['details']['premise_conflicts'] == affected['details']['premise_conflicts']


def test_pool_defaults_limits_and_host_restart_preserve_store_configuration(manager):
    a = attach(manager)
    defaults = tool(a, 'runtime_status')['pool']
    assert defaults['store_worker_limit'] == defaults['store_review_backlog_limit'] == 1
    for arguments in ({'worker_limit': 0}, {'worker_limit': 5}, {'worker_limit': True},
                      {'worker_limit': 2, 'review_backlog_limit': 17}):
        assert tool(a, 'configure_pool', dict(arguments, reason='Invalid fixture request'), ok=False)['code'] == 'invalid_request'
    configure(a)
    first, second = assignment(a, 'first'), assignment(a, 'second')
    allocation = tool(a, 'allocate', {'task_limit': 2, 'wall_minutes': 4, 'review_minutes': .1, 'review_backlog_limit': 2})
    assert allocation['details']['worker_limit'] == 1
    worker = tool(a, 'run_once', {'assignment_id': first['id'], 'allocation_id': allocation['id']})
    assert 'Program worker pool' in tool(a, 'run_once', {'assignment_id': second['id'], 'allocation_id': allocation['id']}, ok=False)['message']
    tool(a, 'shutdown')
    until(lambda: not process_live(a['process']))
    fresh = attach(manager)
    assert fresh['host_id'] != a['host_id']
    assert tool(fresh, 'runtime_status')['pool']['store_worker_limit'] == 2
    assert tool(fresh, 'runtime_status')['pool']['store_review_backlog_limit'] == 2


def test_review_capacity_is_global_and_stop_leaves_other_program_reviewer(context):
    c = context
    submission = submit(c)
    m = local_manager(c)
    m.controller.configure_pool('p', 2, 'Fixture', review_backlog_limit=2)
    m.controller.fund_review('p', submission['id'], wall_minutes=1)
    run = reviewer(c, submission, mode='hold')
    with pytest.raises(ProgramError, match='review concurrency'):
        m.controller.review_start('p', submission['id'], wall_minutes=.1)
    c['store'].create_program('Other program', program_id='other', question_id='other-q')
    other = ProgramManager(c['store'], c['root'], 'other', 'other-manager', c['tmp'] / 'other-manager', workers=m.workers, reviews=m.reviews)
    other.stop('other', 'Unrelated program stop')
    assert m.reviews.children[run['id']].poll() is None
    assert other.drained()
    m.stop('p', 'Owning program stops its review')
    until(lambda: m.reviews.status('p', run['id'])['details']['state'] == 'cancelled')


def test_startup_gap_excludes_workers_reviewers_and_preserves_queued_exclusive_item(context):
    c = context
    m = local_manager(c)
    m.controller.configure_pool('p', 2, 'Startup gap fixture', review_backlog_limit=2)
    ready(m, 'writer')
    ready(m, 'reader')
    allocation = m.controller.allocate('p', 2, 3, review_minutes=.1, worker_limit=2, review_backlog_limit=2)
    # Stop exactly after the atomic controller charge, before WorkerDispatcher
    # has made an attempt. Another connection sees the exclusive reservation.
    def gap(*args, **kwargs):
        with ProgramStore(c['store'].directory) as observer:
            with pytest.raises(ProgramError, match='Shared runtime'):
                pool.check_worker(observer, 'other', 'different')
            with pytest.raises(ProgramError, match='Shared runtime'):
                pool.check_review(observer)
        raise OSError('fixture crash in charged startup gap')
    with patch.object(m, '_launch_worker', side_effect=gap):
        with pytest.raises(OSError, match='startup gap'):
            m.controller.run_once('p', 'writer', allocation_id=allocation['id'], settings={'shared_runtime_write': True})
    charged = m.controller.current(allocation['id'])
    reservation = charged['details']['reservations'][0]
    assert reservation['state'] == 'interrupted' and reservation['attempt_id'] is None
    assert charged['details']['wall_reserved_seconds'] == 6.6
    # Explicit unknown-startup handoff retains the charge, then a normal
    # worker holds capacity while an exclusive queued item must stay queued.
    charged['details']['state'] = 'interrupted'
    m.controller.save(charged)
    m.controller.recover('p', handoff_allocation_id=charged['id'], reason='No child started in this fixture')
    # The fixture root has no launcher; use a held fake child without physics.
    script = c['root'] / 'hp-sim5-research-agent'
    script.write_text(f'#!{sys.executable}\nimport time\ntime.sleep(30)\n')
    script.chmod(0o755)
    second = m.controller.allocate('p', 2, 3, review_minutes=.1, worker_limit=2, review_backlog_limit=2)
    worker = m.controller.run_once('p', 'reader', allocation_id=second['id'])
    ready(m, 'queued-writer')
    current = m.controller.current(second['id'])
    current['details']['queue'] = [{'assignment_id': 'queued-writer', 'settings': {'shared_runtime_write': True}}]
    m.controller.save(current)
    try:
        m.poll()
        assert m.controller.current(second['id'])['details']['queue'][0]['assignment_id'] == 'queued-writer'
        assert m.workers.children[worker['id']].poll() is None
    finally:
        m.stop('p', 'Fixture cleanup')
        m.workers.children[worker['id']].wait(timeout=5)


def test_unopened_program_saved_attempt_is_not_adopted_or_signalled(manager):
    a = attach(manager)
    configure(a)
    with ProgramStore(manager['store']) as store:
        m = ProgramManager(store, manager['root'], 'other', 'old-owner', manager['root'] / 'old-owner')
        route = store.add_approach('other', 'other-q', 'Unopened saved worker', 'Fixture')
        work = store.add_assignment('other', route['id'], 'Wait', {'wall_minutes': 1})
        store.transition('other', work['id'], 'ready', 'Fixture')
        worker = m.workers.start('other', work['id'], settings={'worktree': True})
        child = m.workers.children[worker['id']]
        until(lambda: Path(worker['details']['session_dir'], 'ready').exists())
        os.kill(child.pid, signal.SIGKILL)
        child.wait(timeout=5)
        d = worker['details']
        d['owner']['process']['birth'] = 'previous-controller-birth'
        m.workers._save('other', worker['id'], d)
    # Reopen only p in the live host. The unopened program's saved attempt
    # occupies global capacity even though its historical PID is now dead.
    assert tool(a, 'runtime_status')['pool']['store_workers_reserved'] == 1
    configure(a, workers=1, backlog=2)
    extra = assignment(a)
    assert 'pool is full' in tool(a, 'run_once', {'assignment_id': extra['id']}, ok=False)['message']
    b = attach(manager, 'other')
    saved = tool(b, 'worker_status', {'attempt_id': worker['id']})
    assert saved['details']['state'] == 'interrupted' and saved['owned'] is False
    assert saved['details']['recovery']['adopted'] is False
    assert 'handoff' in tool(a, 'run_once', {'assignment_id': extra['id']}, ok=False)['message']
    tool(b, 'recover', {'handoff_attempt_id': worker['id'], 'reason': 'Fixture child reaped; no real world existed'})
    assert tool(a, 'run_once', {'assignment_id': extra['id']})['details']['state'] == 'running'


@pytest.mark.parametrize('cached_run', [False, True])
def test_review_startup_gap_recovery_requires_explicit_unknown_handoff(context, cached_run):
    c = context
    submission = submit(c)
    m = local_manager(c)
    allocation = m.controller.fund_review('p', submission['id'], wall_minutes=1)
    # Prepare a held fixture client, then stop its first preparation run.
    prepared = reviewer(c, submission, mode='hold')
    c['controller'].cancel('p', prepared['id'])
    until(lambda: c['controller'].status('p', prepared['id'])['details']['state'] == 'cancelled')
    original_start = m.reviews.start
    orphan = None
    def crash(program_id, submission_id, **options):
        nonlocal orphan
        if cached_run:
            orphan = original_start(program_id, submission_id, **options)
        raise KeyboardInterrupt
    # Model abrupt host loss, which cannot execute the controller's exception
    # bookkeeping. Restore the durable pre-spawn charge to its saved state.
    with patch.object(m.reviews, 'start', side_effect=crash):
        with pytest.raises(KeyboardInterrupt):
            m.controller.review_start('p', submission['id'], wall_minutes=.2)
    a = m.controller.current(allocation['id'])
    charge = a['details']['reservations'][0]['reviews'][0]
    charge['state'] = 'starting'
    m.controller.save(a)
    # The live controller cannot use this recovery escape while it owns work.
    with pytest.raises(ProgramError, match='interrupted'):
        m.controller.recover('p', handoff_review_charge_id=charge['charge_id'], reason='Still active')
    fresh = ProgramManager(c['store'], c['root'], 'p', 'fresh-generation', c['tmp'] / 'fresh-review')
    try:
        with pytest.raises(ProgramError, match='review concurrency'):
            pool.check_review(c['store'])
        if orphan:
            saved = c['store'].get_record('p', orphan['id'])
            assert saved['details']['state'] == 'interrupted'
            assert saved['details']['cleanup'].startswith('unknown')
            assert m.reviews.children[orphan['id']].poll() is None
        result = fresh.controller.recover('p', handoff_review_charge_id=charge['charge_id'],
            reason='Explicit new bounded review; old guard shutdown remains unknown')
        assert result['runtime']['pool']['store_reviewers_reserved'] == 0
        retained = fresh.controller.current(allocation['id'])['details']['reservations'][0]['reviews'][0]
        assert retained['reserved_seconds'] == 12 and retained['handoff']['reviewer_shutdown'] == 'unknown'
        if orphan:
            assert m.reviews.children[orphan['id']].poll() is None
            assert c['store'].get_record('p', orphan['id'])['details']['handoff']['reviewer_shutdown'] == 'unknown'
    finally:
        if orphan:
            os.killpg(m.reviews.children[orphan['id']].pid, signal.SIGKILL)
            m.reviews.children[orphan['id']].wait(timeout=5)
