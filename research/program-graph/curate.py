"""Build the inspected autocal seed and exercise real graph CLI queries; no science replay."""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/python'))
from hp_sim5_research.program import ProgramStore

OUTPUT = Path(__file__).resolve().parent
CM = 'output/research/sessions/2026-10-23-consensus-membership/'
NEGATIVE = 'output/research/sessions/b7deb1bcc9cb418388273e2b7b9adc8c/'
REFRESH = 'output/research/reference-refresh-october-7-2026/'
PENDING = 'output/research/sessions/a71aec03021f470796a789ba7382e28a/'
PROGRAM = 'autocal-evidence'


def save(name, value):
    (OUTPUT / name).write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n')


def build(store):
    metrics = json.loads((ROOT / CM / 'results.json').read_text())
    inputs = json.loads((ROOT / CM / 'inputs.json').read_text())
    hashes = ['sha256:' + row['sha256'] for row in inputs]
    checked = []
    for row in inputs:
        compressed = (ROOT / CM / row['input']).read_bytes()
        assert hashlib.sha256(compressed).hexdigest() == row['gzip_sha256'], row['input']
        assert hashlib.sha256(gzip.decompress(compressed)).hexdigest() == row['sha256'], row['input']
        checked.append({'path': CM + row['input'], 'gzip_sha256': row['gzip_sha256'], 'sha256': row['sha256']})
    assert metrics['summary']['additional_dataset']['worsened'] == 2
    assert all(not row['common_unseen_fold_ids'] for row in metrics['rows'] if row['delta_sum_anchor_error_mm'] != 0)
    store.create_program('What supports retiring blanket finite-prediction membership, and what remains unresolved?',
                         {'fresh_fits': 0, 'physics_runs': 0, 'collections': 0, 'inspection_only': True},
                         program_id=PROGRAM, question_id='q-calibration')
    graph = store.graph()
    sources = [
        ('cm-report', CM + 'report.md', '5400cefc', hashes, {}, []),
        ('cm-results', CM + 'results.json', metrics['source_revision'], hashes,
         {'producer_revision': '72523b9d', 'merge_distance_mm': 0.01, 'spread_limit_mm': 5}, []),
        ('cm-inputs', CM + 'inputs.json', '72523b9d', hashes,
         {'machine_labels': sorted({row['machine'] for row in inputs})}, []),
        ('ranking-report', NEGATIVE + 'report.md', 'aaf25cf7', ['historical-ranking-fixture-families'], {},
         [NEGATIVE + 'paired_regressions.json', NEGATIVE + 'heldout_audit.json']),
        ('refresh-report', REFRESH + 'report.md', '4daac40c', ['historical-ranking-fixture-families'],
         {'baseline_revision': 'b60856d9'}, [REFRESH + 'comparison.json']),
        ('attribution-report', 'research/autocal-filtering-lane02-integration.md', '87093cdd', ['layered-lane02-summary'], {},
         ['output/research/sessions/2026-10-22-autocal-filtering-integration/summary-comparison.json']),
        ('pending-abstract', PENDING + 'abstract.md', 'retrospective-2026-10-08', ['unfinished-baseline-notebook'], {}, []),
        ('pending-notebook', PENDING + 'research.md', 'not-recorded', ['unfinished-baseline-notebook'], {}, []),
        ('agenda', 'research/autocal-agenda.md', 'fd10fd94', hashes + ['historical-ranking-fixture-families'], {},
         ['output/research/sessions/2026-10-23-program-task-sanity/report.md']),
    ]
    inspected = []
    for identity, path, revision, lineage, config, missing in sources:
        content = (ROOT / path).read_bytes()
        digest = hashlib.sha256(content).hexdigest()
        for absent in missing:
            assert not (ROOT / absent).exists(), f'Inspection limitation changed: {absent}'
        provenance = {'config': config, 'input_identities': lineage, 'missing_artifacts': missing,
                      'inspection': 'Saved report/summary inspected; no independent numerical replay',
                      'report_checkout': 'f6bbda35'}
        store.add_artifact(PROGRAM, path, digest, 'simulated', revision, provenance=provenance, record_id=identity)
        inspected.append({'path': path, 'sha256': digest, 'missing_artifacts': missing})
    common = {'evidence': 'simulated', 'hardware': False, 'operation': 'saved-history selection',
              'source_revision': '5400cefc', 'machine': 'mixed stored fixture labels'}
    findings = [
        ('cm-mixed', 'CM-E1 additional datasets: finite-only membership improved 1, worsened 2, unchanged 5 of 8.',
         common, ['cm-results', 'cm-report'], 'supported'),
        ('cm-regressions', 'CM-E1: flexible_lines_2000 worsened +0.502998 mm and ten_points_w_buildup +0.094156 mm summed anchor error.',
         dict(common, machine='slideprinter', metric='sum of per-anchor Euclidean error mm'), ['cm-results'], 'supported'),
        ('cm-aggregate', 'CM-E1 summed anchor error 401.100276 to 400.797031 mm is a small net gain, with regressions.',
         dict(common, metric='anchor error sum, not RMSE'), ['cm-results'], 'mixed'),
        ('cm-holdout-gap', 'All three changed CM-E1 applied models have zero common unseen whole sweeps; predictive safety is unresolved.',
         dict(common, machine='slideprinter', training_overlap=True), ['cm-results', 'cm-report'], 'inconclusive'),
        ('cm-compatibility', 'All 11 chosen single-fit prediction scores match saved values at the reported frozen evaluations; fresh-fit equivalence is unestablished.',
         dict(common, producer_revision='72523b9d'), ['cm-results', 'cm-report'], 'supported'),
        ('earlier-gains', 'CM-E1 report preserves earlier two-of-three gains of 3.905 and 2.118 mm; original motivating session is unavailable here.',
         dict(common, inspection='secondary report only; different producer histories'), ['cm-report', 'agenda'], 'supported'),
        ('ranking-disagreement', 'Earlier negative study reports 22 adjacent ranking/physical disagreements across 7 of 11 datasets.',
         {'evidence': 'simulated', 'hardware': False, 'source_revision': 'aaf25cf7', 'metric': 'anchor plus 2pi weighted radius'},
         ['ranking-report'], 'supported'),
        ('ranking-selector-mixed', 'Earlier whole-sweep prediction selection was physically worse on 5 of 11 datasets, equal on 4, better on 2.',
         {'evidence': 'simulated', 'hardware': False, 'source_revision': 'aaf25cf7', 'raw_artifacts': 'missing here'},
         ['ranking-report'], 'mixed'),
        ('refresh-mixed', 'Refreshed-reference comparison reports 7.71% net composite-error gain with 3 improvements, 5 regressions, 3 unchanged.',
         {'evidence': 'simulated', 'hardware': False, 'baseline_revision': 'b60856d9', 'source_revision': '4daac40c',
          'metric': 'anchor plus 2pi weighted radius'}, ['refresh-report'], 'mixed'),
        ('incomparable-folds', 'Refreshed ranking candidates used different future folds; cross-study aggregates cannot establish a general selector.',
         {'evidence': 'simulated', 'hardware': False, 'source_revision': '4daac40c'}, ['refresh-report', 'agenda'], 'supported'),
        ('attribution-correction', 'Lane 02 layered alleged fallback gap compares individual fit 7.683 mm RMSE to final consensus 1.583 mm; it is an attribution error.',
         {'evidence': 'simulated', 'hardware': False, 'machine': 'slideprinter', 'source_revision': '87093cdd',
          'metric': 'anchor RMSE mm', 'raw_artifacts': 'missing here'}, ['attribution-report'], 'supported'),
        ('pending-baseline', 'Unfinished historical notebook reports ten_points_bigger_deltas selection regret 10.96–14.90 mm.',
         {'evidence': 'simulated', 'hardware': False, 'machine': 'slideprinter', 'source_revision': 'not-recorded'},
         ['pending-notebook'], 'supported'),
        ('pending-selector', 'Unfinished record has no completed held-out-selector outcome; successful exit does not supply the missing result.',
         {'evidence': 'simulated', 'hardware': False, 'source_revision': 'not-recorded', 'status': 'incomplete'},
         ['pending-abstract', 'pending-notebook'], 'not_tested'),
        ('cm-hp4-scope', 'CM-E1 includes five unchanged histories labelled hangprinter_4 and six slideprinter histories; it tests frozen simulation selection, with no live HP4 or hardware validation.',
         dict(common, machine='hangprinter_4', live_collection=False, note='Historical hp3 filenames are not machine identity'),
         ['cm-inputs', 'cm-results', 'cm-report'], 'supported'),
    ]
    for identity, title, scope, evidence, outcome in findings:
        graph.add_claim(PROGRAM, title, scope, record_id=identity)
        graph.add_derivation(PROGRAM, identity, [], evidence, 'Reported saved evidence, inspected without numerical replay', record_id='derive-' + identity)
        graph.add_assessment(PROGRAM, identity, outcome, evidence, 'Curated producer outcome; admission awaits task 010',
                             actor='task-008-curator', record_id='assess-' + identity)
    for identity, title in (
            ('keep-defaults', 'Preserve current consensus defaults while validation comparability remains unresolved.'),
            ('blanket-retired', 'Retire unconditional finite-prediction membership as a default-change proposal in the tested scope.')):
        graph.add_claim(PROGRAM, title, common, record_id=identity)
    graph.add_derivation(PROGRAM, 'keep-defaults', ['cm-regressions', 'cm-holdout-gap'], [],
                         'CM-E1 regressions AND absent fair validation justify retaining defaults', record_id='keep-via-cm')
    graph.add_derivation(PROGRAM, 'keep-defaults', ['ranking-selector-mixed', 'refresh-mixed', 'incomparable-folds'], [],
                         'Alternative conservative rationale: mixed historical selectors AND incomparable folds', record_id='keep-via-history')
    graph.add_derivation(PROGRAM, 'blanket-retired', ['cm-regressions', 'cm-holdout-gap'], [],
                         'Nonworsening failed AND no common unseen fold supports a deployable narrower rule', record_id='retire-via-cm')
    graph.add_assessment(PROGRAM, 'keep-defaults', 'supported', ['agenda'], 'Agenda decision imported; not admitted',
                         record_id='assess-keep-defaults')
    graph.add_assessment(PROGRAM, 'blanket-retired', 'supported', ['cm-report'], 'CM-E1 decision imported; not admitted',
                         record_id='assess-blanket-retired')
    store.add_question(PROGRAM, 'Can compatible common unseen sweeps assess the actual applied models?',
                       scope={'machine': 'slideprinter', 'hardware': False, 'status': 'proposed; not tested'}, record_id='q-holdout')
    store.add_question(PROGRAM, 'Does any saved selection result establish live HP4 or hardware calibration accuracy?',
                       scope={'machine': 'hangprinter_4', 'hardware_accuracy': 'unresolved'}, record_id='q-hardware')
    store.add_question(PROGRAM, 'Did the unfinished historical selector investigation complete?',
                       scope={'status': 'incomplete result; no documented completion'}, record_id='q-pending')
    graph.add_relation(PROGRAM, 'scope_limit', 'cm-hp4-scope', 'q-hardware', 'Frozen fixture evidence cannot establish live/hardware accuracy')
    graph.add_relation(PROGRAM, 'challenge', 'cm-regressions', 'earlier-gains', 'Challenges generalization; earlier scoped gains are preserved')
    graph.add_relation(PROGRAM, 'challenge', 'earlier-gains', 'cm-regressions', 'Discussion cycle retains the differing producer scopes')
    graph.add_relation(PROGRAM, 'support', 'cm-holdout-gap', 'q-holdout', 'Gap motivates a bounded saved-data search')
    graph.add_relation(PROGRAM, 'scope_limit', 'pending-selector', 'q-pending', 'Pending work is not a negative selector result')
    for identity, title, question, reason in (
            ('finite-membership', 'Blanket finite-prediction consensus membership', 'q-calibration', 'Motivating gains prompted a broader comparison'),
            ('unseen-sweeps', 'Find compatible saved unseen whole sweeps', 'q-holdout', 'Cheapest fair-validation discriminator'),
            ('model-adequacy', 'Investigate model adequacy and identifiability', 'q-calibration', 'Alternative if fair folds are unavailable')):
        store.add_approach(PROGRAM, question, title, reason, record_id=identity)
    store.transition(PROGRAM, 'finite-membership', 'retired', 'CM-E1 rejects unconditional nonworsening; keep current defaults')
    store.transition(PROGRAM, 'model-adequacy', 'parked', 'First resolve saved-data feasibility; no model change funded')
    decision = store.record_note(PROGRAM, 'decision', 'Retire blanket finite-only rule; preserve current defaults and search saved common folds.',
                                 subject_id='finite-membership', evidence_ids=['cm-report', 'agenda'], record_id='retirement')
    graph.add_relation(PROGRAM, 'decision', decision['id'], 'finite-membership', 'Dated route retirement rationale')
    graph.add_relation(PROGRAM, 'support', 'blanket-retired', decision['id'], 'Scoped negative decision, not universal ranking rejection')
    graph.add_relation(PROGRAM, 'provenance', 'q-calibration', 'agenda', 'Program question and competing routes come from the inspected agenda')
    return {'reports_and_summaries': inspected, 'byte_checked_history_envelopes': checked,
            'extent': 'Nine reports/summaries/structured result sources; 11 compressed/raw history hashes checked. '
                      'Inspected CM result rows and input identities, not every candidate telemetry value. '
                      'No whole-corpus extraction, fresh fit, numerical reproduction, collection or hardware work.',
            'findings': len(findings) + 2, 'numerical_replays': 0}


def main():
    with tempfile.TemporaryDirectory(prefix='program-graph-008-') as temporary:
        directory = Path(temporary) / 'manager'
        with ProgramStore(directory, create=True) as store:
            inspection = build(store)
            seed = store.snapshot(PROGRAM)
        save('autocal-seed.json', seed)
        save('inspection.json', inspection)
        # Exercise the exported/imported seed through fresh production CLI processes.
        restored = Path(temporary) / 'restored'
        transcript = []
        def cli(*args):
            command = [str(ROOT / 'hp-sim5-research-program'), '--store', str(restored), *args]
            result = subprocess.run(command, capture_output=True, text=True, timeout=30)
            assert result.returncode == 0, result.stderr
            output = json.loads(result.stdout)
            transcript.append({'argv': ['./hp-sim5-research-program', '--store', '$STORE', *args], 'result': output})
            return output
        cli('import', str(OUTPUT / 'autocal-seed.json'))
        rationale = cli('query', PROGRAM, 'rationale', 'finite-membership')
        assert rationale['states']['blanket-retired']['structural_support']
        cli('query', PROGRAM, 'support', 'cm-regressions')
        cli('query', PROGRAM, 'upstream', 'keep-defaults')
        cli('query', PROGRAM, 'open_questions')
        hp4 = cli('query', PROGRAM, 'search', '--text', 'hangprinter_4', '--scope', '{"machine":"hangprinter_4"}')
        assert hp4['matches'] == ['cm-hp4-scope', 'q-hardware']
        cli('query', PROGRAM, 'support', 'pending-selector')
        cli('review-event', PROGRAM, 'Counterfactual exercise: withdraw the necessary CM-E1 regression premise',
            '--subject', 'cm-regressions', '--event', 'withdrawal', '--id', 'exercise-withdrawal')
        affected = cli('query', PROGRAM, 'downstream', 'cm-regressions')
        assert not affected['states']['retire-via-cm']['structural_support']
        assert not affected['states']['keep-via-cm']['structural_support']
        assert affected['states']['keep-via-history']['structural_support']
        assert affected['states']['keep-defaults']['structural_support']
        outside = cli('query', PROGRAM, 'support', 'attribution-correction')
        assert not outside['states']['attribution-correction']['review_required']
        cli('review-event', PROGRAM, 'Counterfactual new configuration; preserve historical measurement',
            '--subject', 'cm-results', '--event', 'applicability', '--id', 'exercise-source-change',
            '--context', '{"source_revision":"hypothetical-next","config":{"machine":"live-HP4"}}')
        historical = cli('query', PROGRAM, 'support', 'cm-compatibility')
        assert historical['states']['cm-compatibility']['structural_support']
        assert historical['states']['cm-compatibility']['new_applicability'][0]['state'] == 'unestablished'
        save('queries.json', transcript)
    print('Curated 16 unassessed findings; 12 CLI calls passed; no numerical replay.')


if __name__ == '__main__':
    main()
