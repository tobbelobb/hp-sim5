"""Exercise manager strategy on inspected saved evidence, without a numerical replay."""
import hashlib
import json
from pathlib import Path
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/python'))
from hp_sim5_research.program import ProgramStore
from hp_sim5_research.program_manager import ProgramManager

OUTPUT = ROOT / 'output/research/sessions/2026-10-24-manager-strategy'
PROGRAM = 'autocal-evidence'


def main():
    seed_path = ROOT / 'research/program-graph/autocal-seed.json'
    seed = json.loads(seed_path.read_text())
    inspected = []
    for identity in ('cm-report', 'cm-results', 'cm-inputs', 'ranking-report', 'refresh-report'):
        artifact = next(r for r in seed['records'] if r['id'] == identity)
        path = ROOT / artifact['details']['reference']
        sha = hashlib.sha256(path.read_bytes()).hexdigest()
        assert sha == artifact['details']['sha256']
        inspected.append(dict(id=identity, path=str(path.relative_to(ROOT)), sha256=sha))
    measurements = json.loads((ROOT / next(r for r in seed['records'] if r['id'] == 'cm-results')['details']['reference']).read_text())
    assert measurements['summary']['additional_dataset']['worsened'] == 2
    assert measurements['summary']['additional_dataset']['improved'] == 1
    transcript = []
    with tempfile.TemporaryDirectory(prefix='program-strategy-012-') as temporary:
        with ProgramStore(Path(temporary) / 'store', create=True) as store:
            store.import_snapshot(seed)
            manager = ProgramManager(store, ROOT, PROGRAM, 'saved-evidence-exercise', Path(temporary) / 'chat')
            def call(operation, **parameters):
                result = manager.call(operation, parameters)
                assert result['ok'], result
                transcript.append(dict(operation=operation, parameters=parameters, result_id=result['result'].get('id') if isinstance(result['result'], dict) else None))
                return result['result']
            scopes = {
                'finite-membership': {'method': 'unconditional finite-only final consensus', 'evidence': 'saved simulated histories', 'producer': '72523b9d'},
                'unseen-sweeps': {'method': 'search compatible unseen whole sweeps', 'evidence': 'saved simulated histories'},
                'model-adequacy': {'method': 'diagnose model adequacy before selector changes', 'evidence': 'saved simulated reports'},
            }
            hypotheses = {
                'finite-membership': 'Unconditional finite-only membership improves final applied geometry without worsening another dataset',
                'unseen-sweeps': 'Compatible saved sweeps can distinguish selection effects from training overlap',
                'model-adequacy': 'Model inadequacy or identifiability can explain prediction and physical-error disagreement',
            }
            dependencies = {'finite-membership': ['blanket-retired'], 'unseen-sweeps': ['keep-defaults'], 'model-adequacy': ['ranking-selector-mixed']}
            for route, priority in [('finite-membership', 90), ('unseen-sweeps', 10), ('model-adequacy', 20)]:
                call('route_plan', approach_id=route, hypothesis=hypotheses[route], scope=scopes[route], priority=priority,
                     estimated_cost={'wall_minutes': 12, 'fresh_fits': 0, 'collections': 0},
                     uncertainty='Changed CM-E1 models have no common unseen fold; old study raw summaries are partly absent',
                     decision_value='Choose a fair saved-data discriminator before funding a fit-on-prefix replay',
                     novelty='Evaluate the actual applied anchor-plus-spool model on comparable folds',
                     observation='Find a common whole sweep unseen by every applied consensus member and spool fit',
                     reopening_condition='New compatible unseen data, a changed model, or a narrower truth-free selection hypothesis',
                     reason='CM-E1 has one improvement and two regressions; current defaults stay until fair validation exists',
                     tradeoffs=['Saved scouting is cheap; a new terminal holdout costs fitting or collection',
                                'A physically better saved fit need not have the lower prediction score'],
                     objections=['Shared fixture families are correlated', 'Frozen selection does not establish hardware accuracy'],
                     dependency_ids=dependencies[route], citation_ids=['cm-report', 'cm-results', 'refresh-report'],
                     record_id='plan-' + route)
            negative = call('route_decide', approach_id='finite-membership', target='retired', scope=scopes['finite-membership'],
                            reason='Inspected CM-E1 rejects the nonworsening default-change proposal in its frozen scope; broader selector research remains open',
                            reopening_condition='Common unseen-sweep evidence on the regressions or a narrower method',
                            citation_ids=['cm-mixed', 'cm-regressions', 'cm-holdout-gap', 'cm-report', 'assess-blanket-retired'])
            alternative = call('route_decide', approach_id='model-adequacy', target='promising', scope=scopes['model-adequacy'],
                               reason='Inspected mixed historical selectors justify keeping model adequacy as a rival; this is a practical judgment on unreviewed reports',
                               reopening_condition='Comparable validation showing the current model adequate',
                               citation_ids=['ranking-selector-mixed', 'refresh-mixed', 'incomparable-folds', 'refresh-report'])
            work = call('commission', approach_id='unseen-sweeps', objective='Scout four named saved sources for compatible common unseen sweeps',
                        allocation={'wall_minutes': 12, 'fresh_fits': 0, 'collections': 0}, kind='scouting',
                        explanations=[{'approach_id': 'unseen-sweeps', 'explanation': 'Training overlap hides a useful selector difference'},
                                      {'approach_id': 'model-adequacy', 'explanation': 'Model inadequacy dominates, even with a fair fold'}],
                        outcomes={'found': {'observation': 'A compatible sweep unused by all applied members exists', 'finding': 'supported', 'actions': []},
                                  'absent': {'observation': 'No compatible common sweep exists in the named saved sources', 'finding': 'inconclusive', 'actions': []},
                                  'failed': {'observation': 'A reader or missing file prevents inspection', 'finding': 'tool_failure', 'actions': []}},
                        execution={'source_limit': 4, 'sources': ['cm-inputs', 'cm-results', 'ranking-report', 'refresh-report']},
                        citation_ids=['cm-holdout-gap', 'cm-inputs', 'cm-results'], record_id='saved-scout')
            call('steering', text='I suspect radius-model bias, but first check saved fold compatibility', intent='hunch',
                 provenance={'source': 'worked manager case', 'author': 'task-012', 'human_role': 'illustrative fixture, not a real human instruction'},
                 links=['research/program-graph/README.md'])
            directed = call('steering', text='Illustrative accepted instruction: allow sixteen minutes for the four-source scout', intent='instruction',
                            next_assignment_id='saved-scout', allocation={'wall_minutes': 16, 'fresh_fits': 0, 'collections': 0},
                            provenance={'source': 'worked manager case', 'human_role': 'illustrative fixture, not actual new steering'})
            before = call('overview')
            event = call('review_event', subject_id='cm-regressions', event='withdrawal',
                         reason='COUNTERFACTUAL exercise only: inspect consequences if CM-E1 regressions were withdrawn', record_id='counterfactual-withdrawal')
            board = call('strategy')
            unseen = next(r for r in board['routes'] if r['id'] == 'unseen-sweeps')
            assert unseen['review_required'] and 'keep-via-history' in unseen['surviving_derivation_ids']
            assert not next(r for r in board['routes'] if r['id'] == 'model-adequacy')['review_required']
            call('route_decide', approach_id='unseen-sweeps', target='live', scope=scopes['unseen-sweeps'],
                 reason='Counterfactual review retains conservative priority through surviving keep-via-history; it does not reproduce CM-E1',
                 reopening_condition='Comparable validation or contrary model evidence',
                 citation_ids=[event['id'], 'keep-via-history', 'ranking-selector-mixed', 'refresh-mixed'])
            after = call('overview')
            assert after['next_allocation']['allocation']['wall_minutes'] == 16
            rationale = call('query', query='rationale', subject_id='unseen-sweeps')
            assert {'counterfactual-withdrawal', 'keep-via-history', 'cm-report'} <= {r['id'] for r in rationale['records']}
            snapshot = store.snapshot(PROGRAM)
            with ProgramStore(Path(temporary) / 'fresh-manager', create=True) as restored:
                restored.import_snapshot(snapshot)
                fresh = ProgramManager(restored, ROOT, PROGRAM, 'fresh-conversation', Path(temporary) / 'fresh-chat').overview(PROGRAM)
                assert fresh['brief'] == after['brief']
                assert fresh['strategy'] == after['strategy']
            OUTPUT.mkdir(parents=True, exist_ok=True)
            def save(name, value):
                (OUTPUT / name).write_text(json.dumps(value, indent=2) + '\n')
            save('worked-cases.json', dict(inspected_sources=inspected, reused_summary=measurements['summary']['additional_dataset'],
                 negative_decision=negative['id'], alternative_decision=alternative['id'],
                 funded_assignment=work['assignment']['id'], steered_assignment=directed['accepted_input']['details']['next_assignment_id'],
                 counterfactual_event=event['id'], surviving_derivations=unseen['surviving_derivation_ids'],
                 fresh_manager_reconstructed=True, operations=transcript,
                 boundary='Source reports and structured results inspected; no independent numerical reproduction or live separate review. Withdrawal and steering are illustrative cases only.',
                 numerical_use=dict(fits=0, physics_steps=0, collections=0, replays=0)))
            save('program.json', snapshot)
            save('rationale.json', rationale)
            (OUTPUT / 'brief-before.md').write_text(before['brief'] + '\n')
            (OUTPUT / 'brief-after.md').write_text(after['brief'] + '\n')
            assert manager.drained()
    print('Saved-source strategy cases passed; no worker or reviewer started.')


if __name__ == '__main__':
    main()
