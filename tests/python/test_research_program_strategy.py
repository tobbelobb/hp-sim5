"""Strategy state and real CLI calls; fixture observations are not live review proof."""
import copy
import json
from pathlib import Path
import sqlite3
import uuid

import pytest

from hp_sim5_research.program import ProgramError, ProgramStore
from hp_sim5_research.program_manager import ProgramManager
from hp_sim5_research.program_strategy import ProgramStrategy
from .test_research_program_review import context, submit, reviewer, CASES

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def strategy(context):
    store = context['store']
    controller = ProgramStrategy(store)
    controller.test_context = context
    for route in ('saved', 'model'):
        store.add_approach('p', 'q', route, 'Compare evidence before spending', record_id=route)
        profile(controller, route)
    yield store, controller


def profile(controller, route, **updates):
    params = dict(approach_id=route, hypothesis='Shared unseen sweeps can distinguish selection from model error ' + route,
                  scope={'machine': 'HP3', 'method': route, 'evidence': 'simulated'}, priority=10 if route == 'saved' else 20,
                  estimated_cost={'wall_minutes': 12, 'fresh_fits': 2, 'physics_steps': 10000, 'collections': 1},
                  uncertainty='Common folds may not exist', decision_value='A fair comparison decides whether to change selection',
                  novelty='Tests applied models rather than individual candidates', observation='Compare applied models on the same unseen sweep',
                  reopening_condition='A new common unseen fold or changed hypothesis', reason='Resolve comparability first',
                  tradeoffs=['Saved evidence is cheaper; a new fold costs collection'], objections=['Prediction may disagree with anchor error'],
                  retry_limit=2)
    params.update(updates)
    return controller.route('p', **params)


def fund(controller, route='saved', record_id=None, **updates):
    scope = {'machine': 'HP3', 'method': route, 'evidence': 'simulated'}
    outcomes = {
        'negative': dict(observation='Rule worsens an anchor and fair validation is absent', finding='rejected', actions=[
            dict(approach_id=route, target='retired', reason='Rule failed in the exact funded scope', scope=scope,
                 reopening_condition='Changed data or narrower hypothesis')]),
        'surprise': dict(observation='Residuals implicate model inadequacy', finding='unexpected', review_finding='supported', actions=[
            dict(approach_id='model', target='promising', reason='Residuals favor model adequacy investigation',
                 scope={'machine': 'HP3', 'method': 'model', 'evidence': 'simulated'}, reopening_condition='New adequate-model evidence')]),
        'unclear': dict(observation='No fair common fold', finding='inconclusive', actions=[]),
        'broken': dict(observation='Reader could not load the input', finding='tool_failure', actions=[]),
    }
    params = dict(approach_id=route, objective='Test applied models on a common fold', allocation={'wall_minutes': 17, 'fresh_fits': 2},
                  explanations=[dict(approach_id='saved', explanation='Candidate membership causes the error'),
                                dict(approach_id='model', explanation='The physical model is inadequate')], outcomes=outcomes,
                  execution={'physics_backend': 'native-python', 'recording_mode': 'events', 'simulated_autocal_speedup': 25}, record_id=record_id)
    params.update(updates)
    return controller.commission('p', **params)


def evidence(store, outcome, identity='assessment'):
    store.add_artifact('p', 'fixture/report.json', 'a' * 64, 'synthetic', 'fixture-v1', record_id=identity + '-bytes')
    store.graph().add_claim('p', 'Fixture claim, no production review', {'evidence': 'synthetic'}, record_id=identity + '-claim')
    return store.graph().add_assessment('p', identity + '-claim', outcome, [identity + '-bytes'],
                                        'Deterministic reported fixture', record_id=identity)['id']


def ended_review(controller, work, finding, *, verdict='keep', failed=False):
    c = dict(controller.test_context)
    worker = c['tmp'] / ('failed-worker' if failed else 'worker')
    if failed:
        worker.write_text((c['tmp'] / 'worker').read_text().replace('sys.exit(0)', 'sys.exit(9)'))
        worker.chmod(0o755)
    workers = c['store'].workers(c['root'], executable=worker)
    attempt = workers.start('p', work['assignment']['id'])
    workers.children[attempt['id']].wait(timeout=5)
    c['attempt'] = workers.status('p', attempt['id'])
    submission = submit(c, claim='fixture-' + uuid.uuid4().hex)
    response = copy.deepcopy(CASES['scoped_negative']['review'])
    response.update(finding=finding, verdict=verdict)
    run = reviewer(c, submission, response=response)
    return c['controller'].apply('p', run['details']['review_id'])


def end(store, work, state='completed'):
    identity = work['assignment']['id']
    store.transition('p', identity, 'running', 'Fixture only')
    store.transition('p', identity, state, 'Fixture observation ended')
    return identity


def test_broad_question_two_routes_funded_rival_and_scoped_negative(strategy):
    store, controller = strategy
    work = fund(controller)
    assert work['assignment']['details']['allocation']['fresh_fits'] == 2
    assert len(work['investigation']['details']['explanations']) == 2
    admission = ended_review(controller, work, 'rejected')
    citation = admission['id']
    controller.result('p', work['assignment']['id'], 'negative', [citation], 'Scoped negative fixture')
    board = controller.board('p')
    assert board['routes'][0]['state'] == 'retired'
    assert board['routes'][1]['state'] == 'live'
    rationale = store.graph().query('p', 'rationale', 'saved')
    assert citation in {r['id'] for r in rationale['records']}
    assert rationale['states'][admission['details']['claim_id']]['assessment_state'] == 'reviewed'
    snapshot = store.snapshot('p')
    with ProgramStore(store.directory.parent / 'second-manager', create=True) as other:
        other.import_snapshot(snapshot)
        assert ProgramStrategy(other).board('p') == board
        assert other.graph().query('p', 'rationale', 'saved') == rationale


def test_unexpected_promotes_rival_and_tool_failure_is_not_science(strategy):
    store, controller = strategy
    work = fund(controller)
    snapshot = store.get_record('p', 'model')
    controller.result('p', end(store, work, 'failed'), 'broken', [work['investigation']['id']], 'Reader failed, no finding')
    assert store.get_record('p', 'model') == snapshot
    assert all(r['inconclusive_count'] == 0 for r in controller.board('p')['routes'])
    work = fund(controller)
    admission = ended_review(controller, work, 'supported')
    controller.result('p', work['assignment']['id'], 'surprise', [admission['id']], 'Unexpected supported residual pattern')
    assert store.get_record('p', 'model')['details']['state'] == 'promising'


def test_repeated_inconclusive_parks_and_same_plan_cannot_reset_limit(strategy):
    store, controller = strategy
    for i in range(2):
        work = fund(controller)
        citation = ended_review(controller, work, 'inconclusive')['id']
        controller.result('p', work['assignment']['id'], 'unclear', [citation], f'No fair fold, trial {i}')
    route = controller.board('p')['routes'][0]
    assert route['state'] == 'parked' and route['futile'] and route['inconclusive_count'] == 2
    profile(controller, 'saved', reason='Same plan recorded again')
    controller.decide('p', 'saved', 'live', 'Attempt same hypothesis again', route['plan']['details']['scope'], 'New fold', [citation])
    with pytest.raises(ProgramError, match='reframe'):
        fund(controller)
    profile(controller, 'saved', hypothesis='Changed hypothesis about radius bias', observation='A matched radius discriminator')
    assert not controller.board('p')['routes'][0]['futile']


def test_withdrawal_traverses_surviving_derivations_and_requires_review(strategy):
    store, controller = strategy
    for identity in ('a', 'b'):
        store.add_artifact('p', identity, identity * 64, 'synthetic', 'fixture', record_id=identity)
    graph = store.graph()
    graph.add_claim('p', 'Conservative route support', {}, record_id='conclusion')
    graph.add_derivation('p', 'conclusion', [], ['a'], 'First source', record_id='d-a')
    graph.add_derivation('p', 'conclusion', [], ['b'], 'Alternative sufficient source', record_id='d-b')
    profile(controller, 'saved', dependency_ids=['conclusion'])
    event = graph.review_event('p', 'a', 'withdrawal', 'Fixture source withdrawn')
    original = store.get_record('p', 'a')
    board = controller.reconcile('p')
    saved = board['routes'][0]
    assert saved['state'] == 'live' and saved['review_required']
    assert saved['surviving_derivation_ids'] == ['d-b']
    with pytest.raises(ProgramError, match='Review dependencies'):
        fund(controller)
    before = store.get_program('p')['revision']
    assert controller.reconcile('p') == board
    assert store.get_program('p')['revision'] == before
    controller.decide('p', 'saved', 'live', 'Alternative d-b survives, retain narrower priority', saved['plan']['details']['scope'],
                      'A new contrary source', [event['id'], 'd-b'])
    assert not controller.board('p')['routes'][0]['review_required']
    graph.review_event('p', 'b', 'withdrawal', 'Last source withdrawn')
    board = controller.reconcile('p')
    assert board['routes'][0]['state'] == 'parked'
    assert not board['routes'][1]['review_required']
    assert store.get_record('p', 'a') == original


def test_hunch_preserved_instruction_reallocates_only_pending_work(strategy):
    store, controller = strategy
    running = fund(controller)
    store.transition('p', running['assignment']['id'], 'running', 'Unrelated active fixture')
    pending = fund(controller, 'model')
    old = copy.deepcopy(pending['assignment'])
    hunch = controller.steering('p', 'Maybe radius bias matters', intent='hunch', links=['https://example.org/idea'],
                                provenance={'channel': 'chat', 'speaker': 'human'})
    assert hunch['accepted_input']['details']['intent'] == 'hunch'
    assert controller.next_allocation('p', controller.board('p'))['assignment_id'] == old['id']
    instruction = controller.steering('p', 'Fund the model discriminator next with three fits', intent='instruction',
                                      next_assignment_id=old['id'], allocation={'wall_minutes': 42, 'fresh_fits': 3})
    next_id = instruction['accepted_input']['details']['next_assignment_id']
    next_allocation = controller.next_allocation('p', controller.board('p'))
    assert next_allocation['assignment_id'] == next_id
    assert next_allocation['allocation'] == {'wall_minutes': 42, 'fresh_fits': 3}
    assert store.get_record('p', old['id'])['details']['execution_state'] == 'cancelled'
    assert store.get_record('p', running['assignment']['id'])['details']['execution_state'] == 'running'
    assert old['details']['allocation'] == {'wall_minutes': 17, 'fresh_fits': 2}


def test_invalid_actions_atomic_and_strategy_history_immutable(strategy):
    store, controller = strategy
    before = store.snapshot('p')
    invalid = [
        lambda: profile(controller, 'saved', dependency_ids=['absent']),
        lambda: fund(controller, allocation={'wall_minutes': 0}),
        lambda: fund(controller, explanations=[dict(approach_id='saved', explanation='Only explanation')]),
        lambda: fund(controller, outcomes={'broken': dict(observation='Tool failed', finding='tool_failure', actions=[{}])}),
        lambda: fund(controller, outcomes={'wrong': dict(observation='Narrow method failed', finding='rejected', actions=[
            dict(approach_id='saved', target='retired', reason='Too broad', scope={'method': 'different'}, reopening_condition='Changed input')])}),
        lambda: controller.steering('p', 'hunch', intent='hunch', next_assignment_id='absent'),
    ]
    for action in invalid:
        with pytest.raises(ProgramError):
            action()
        assert store.snapshot('p') == before
    identity = controller.board('p')['routes'][0]['plan']['id']
    with pytest.raises(sqlite3.IntegrityError), store.transaction():
        store.connection.execute('DELETE FROM records WHERE id=?', (identity,))
    invalid_snapshot = copy.deepcopy(before)
    next(r for r in invalid_snapshot['records'] if r['id'] == identity)['details']['dependency_ids'] = ['absent']
    with ProgramStore(store.directory.parent / 'invalid', create=True) as other:
        with pytest.raises(ProgramError, match='strategy reference'):
            other.import_snapshot(invalid_snapshot)


def test_manager_tools_write_cited_brief_and_real_worker_contract(strategy, tmp_path):
    store, controller = strategy
    class Workers:
        children = {}
        def status(self, program_id, attempt_id):
            return {}
        cancel = status
        def sessions(self, program_id):
            return []
        def start(self, program_id, assignment_id, **kwargs):
            return dict(id='fixture-attempt', details={'state': 'failed'}, program_id=program_id, assignment_id=assignment_id, **kwargs)
    manager = ProgramManager(store, ROOT, 'p', 'test-controller', tmp_path / 'manager', workers=Workers())
    assert 'strategy' not in manager.capabilities('p')['unavailable']
    assert {'route_plan', 'commission', 'strategy_result', 'route_decide'} <= manager.operations.keys()
    work = fund(controller, kind='literature', allocation={'wall_minutes': 9, 'fresh_fits': 0, 'collections': 0},
                execution={'source_limit': 4, 'search_question': 'Do comparable applied models have an unbiased common fold?'})
    manager.controller.recover('p', handoff_attempt_id=controller.test_context['attempt']['id'],
                               reason='Fixture worker started no physics; acknowledge its missing exit manifest')
    response = manager.call('worker_start', {'assignment_id': work['assignment']['id']})
    contract = response['result']['instructions']
    assert 'literature' in contract and 'source_limit' in contract and 'explanations' in contract
    overview = manager.call('overview')['result']
    assert overview['next_allocation']['assignment_id'] == work['assignment']['id']
    assert overview['strategy']['routes'][0]['plan']['id'] in overview['brief']
    assert (tmp_path / 'manager/brief.md').read_text().strip() == overview['brief']


@pytest.mark.parametrize('boundary', ['withheld', 'invalidated', 'withdrawn', 'unrelated', 'reported_only'])
def test_automatic_outcomes_require_current_admission_for_exact_assignment(strategy, boundary):
    store, controller = strategy
    target = fund(controller)
    if boundary == 'reported_only':
        citation = evidence(store, 'rejected')
        end(store, target)
    else:
        producing = fund(controller) if boundary == 'unrelated' else target
        admission = ended_review(controller, producing, 'rejected', verdict='block' if boundary == 'withheld' else 'keep')
        citation = admission['id']
        if boundary == 'invalidated':
            controller.test_context['controller'].invalidate('p', admission['details']['submission_id'], 'Invalid fixed bytes')
        if boundary == 'withdrawn':
            store.graph().review_event('p', admission['details']['claim_id'], 'withdrawal', 'Measurement was misattributed')
        if boundary == 'unrelated':
            end(store, target)
    before = store.snapshot('p')
    with pytest.raises(ProgramError, match='effective reviewed admission'):
        controller.result('p', target['assignment']['id'], 'negative', [citation], 'This must not retire a route')
    assert store.snapshot('p') == before
    assert store.get_record('p', 'saved')['details']['state'] == 'live'


def test_failed_attempt_can_supply_scoped_reviewed_partial_result(strategy):
    store, controller = strategy
    work = fund(controller)
    admission = ended_review(controller, work, 'rejected', failed=True)
    assert store.get_record('p', work['assignment']['id'])['details']['execution_state'] == 'failed'
    result = controller.result('p', work['assignment']['id'], 'negative', [admission['id']], 'Reviewed partial data reject the scoped method')
    assert result['details']['finding'] == 'rejected'
    assert store.get_record('p', 'saved')['details']['state'] == 'retired'


def test_latest_plan_uses_append_order_even_with_clock_rollback(strategy, monkeypatch):
    store, controller = strategy
    import hp_sim5_research.program as program
    monkeypatch.setattr(program, '_now', lambda: '2026-10-24T00:00:00+00:00')
    profile(controller, 'saved', priority=1, record_id='z-old')
    monkeypatch.setattr(program, '_now', lambda: '2025-10-24T00:00:00+00:00')
    profile(controller, 'saved', priority=2, record_id='a-latest')
    assert next(r for r in controller.board('p')['routes'] if r['id'] == 'saved')['plan']['id'] == 'a-latest'
    with ProgramStore(store.directory.parent / 'restored-order', create=True) as restored:
        restored.import_snapshot(store.snapshot('p'))
        assert next(r for r in ProgramStrategy(restored).board('p')['routes'] if r['id'] == 'saved')['plan']['id'] == 'a-latest'


def test_funding_reads_dependencies_inside_its_write_transaction(strategy, monkeypatch):
    store, controller = strategy
    original = controller.board
    observed = []
    def board(program_id):
        observed.append(store.connection.in_transaction)
        return original(program_id)
    monkeypatch.setattr(controller, 'board', board)
    fund(controller, allocation={'wall_minutes': 57, 'collections': 1, 'physics_steps': 240000, 'fresh_fits': 4})
    assert observed == [True]
    latest = [r for r in controller.records('p').values() if r['kind'] == 'investigation'][-1]
    assert latest['details']['execution']['simulated_autocal_speedup'] == 25
    assert latest['details']['execution']['recording_mode'] == 'events'


def test_review_id_citation_reaches_submission_scope_and_effective_admission(strategy):
    store, controller = strategy
    work = fund(controller)
    admission = ended_review(controller, work, 'rejected')
    controller.result('p', work['assignment']['id'], 'negative', [admission['details']['review_id']], 'Use separately reviewed scoped evidence')
    query = store.graph().query('p', 'rationale', 'saved')
    cited = {r['id'] for r in query['records']}
    assert {admission['id'], admission['details']['review_id'], admission['details']['submission_id'], admission['details']['claim_id']} <= cited
    assert query['states'][admission['details']['claim_id']]['assessment_state'] == 'reviewed'
    assert query['sources'] and all(r['details']['evidence_type'] == 'synthetic' for r in query['sources'])


def test_later_withheld_review_requeues_route_with_full_history(strategy):
    store, controller = strategy
    work = fund(controller)
    admission = ended_review(controller, work, 'supported')
    profile(controller, 'saved', dependency_ids=[admission['details']['claim_id']])
    assert not controller.board('p')['routes'][0]['review_required']
    c = controller.test_context
    submission = store.get_record('p', admission['details']['submission_id'])
    response = copy.deepcopy(CASES['scoped_negative']['review'])
    response.update(finding='mixed', verdict='revise')
    run = reviewer(c, submission, response=response)
    unfavorable = c['controller'].apply('p', run['details']['review_id'])
    saved = controller.reconcile('p')['routes'][0]
    assert saved['review_required'] and unfavorable['id'] in saved['pending_review_event_ids']
    assert saved['state'] == 'parked'
    assert admission['id'] in {r['id'] for r in store.graph().query('p', 'rationale', 'saved')['records']}


def test_old_outcome_cannot_retire_a_reframed_broader_route(strategy):
    store, controller = strategy
    work = fund(controller)
    admission = ended_review(controller, work, 'rejected')
    profile(controller, 'saved', hypothesis='A broader family of truth-free selectors can help',
            scope={'method': 'selector family', 'machine': 'HP3'}, observation='Compare a different selector on an unbiased fold')
    before = store.snapshot('p')
    with pytest.raises(ProgramError, match='prior route framing'):
        controller.result('p', work['assignment']['id'], 'negative', [admission['id']], 'Old narrow failure must not retire the broader idea')
    assert store.snapshot('p') == before
