"""Offline graph semantics and fresh-process CLI checks; no numerical runs."""
import copy
import json
from pathlib import Path
import sqlite3
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/python'))
from hp_sim5_research.program import ProgramError, ProgramStore


class GraphTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.store = ProgramStore(self.directory / 'memory', create=True)
        self.addCleanup(self.store.close)
        self.store.create_program('Which scoped conclusion survives?', program_id='p', question_id='q')
        self.graph = self.store.graph()

    def artifact(self, identity, inputs=None, sha='a'):
        return self.store.add_artifact('p', f'reports/{identity}.md', sha * 64, 'simulated', 'source-old',
                                       provenance={'config': {'machine': 'HP3'},
                                                   'input_identities': inputs or [identity]}, record_id=identity)

    def claim(self, identity, scope=None):
        return self.graph.add_claim('p', identity, {'machine': 'HP3'} if scope is None else scope, record_id=identity)

    def derive(self, identity, conclusion, premises=None, evidence=None):
        return self.graph.add_derivation('p', conclusion, premises or [], evidence or [], 'frozen evidence', record_id=identity)

    def query(self, name, subject=None, **options):
        return self.graph.query('p', name, subject, **options)

    def test_and_premises_alternatives_withdrawal_and_unrelated_claim(self):
        for i in ('a', 'b', 'c'):
            self.artifact(i, sha=i)
        for i in ('first', 'second', 'alternative', 'conclusion', 'downstream', 'unrelated'):
            self.claim(i)
        self.derive('d1', 'first', evidence=['a'])
        self.derive('d2', 'second', evidence=['b'])
        self.derive('d3', 'alternative', evidence=['c'])
        self.derive('both', 'conclusion', ['first', 'second'])
        self.derive('either', 'conclusion', ['alternative'])
        self.derive('last', 'downstream', ['conclusion'])
        self.derive('outside', 'unrelated', evidence=['c'])
        original = self.store.get_record('p', 'first')
        event = self.graph.review_event('p', 'first', 'withdrawal', 'Measurement attribution was incorrect', record_id='withdraw')
        self.assertEqual(event['details']['affected_claim_ids'], ['conclusion', 'downstream', 'first'])
        self.assertEqual(event['details']['affected_derivation_ids'], ['both', 'last'])
        result = self.query('downstream', 'first')
        self.assertEqual(result['matches'], ['conclusion', 'downstream', 'first'])
        self.assertFalse(result['states']['both']['structural_support'])
        self.assertTrue(result['states']['either']['structural_support'])
        self.assertTrue(result['states']['downstream']['structural_support'])
        self.assertTrue(result['states']['conclusion']['review_required'])
        self.assertEqual(result['states']['conclusion']['assessment_state'], 'unassessed')
        self.assertEqual(self.store.get_record('p', 'first'), original)
        outside = self.query('support', 'unrelated')
        self.assertFalse(outside['states']['unrelated']['review_required'])
        self.graph.review_event('p', 'c', 'withdrawal', 'Second independent path withdrawn')
        self.assertFalse(self.query('support', 'downstream')['states']['downstream']['structural_support'])
        self.claim('later')
        self.derive('new-dependent', 'later', ['first'])
        later = self.query('support', 'later')['states']['later']
        self.assertTrue(later['review_required'])
        self.assertFalse(later['structural_support'])

    def test_no_grounding_from_support_edges_or_discussion_cycles(self):
        self.claim('a')
        self.claim('b')
        self.graph.add_relation('p', 'support', 'a', 'b', 'A supports B')
        self.graph.add_relation('p', 'challenge', 'b', 'a', 'B challenges A')
        result = self.query('support', 'a')
        self.assertEqual(len(result['relations']), 2)
        self.assertFalse(result['states']['a']['structural_support'])
        self.derive('ab', 'b', ['a'])
        before = self.store.snapshot('p')
        with self.assertRaisesRegex(ProgramError, 'Cyclic required premises'):
            self.derive('ba', 'a', ['b'])
        self.assertEqual(self.store.snapshot('p'), before)

    def test_revision_changes_preserve_old_measurements_and_claim_versions(self):
        self.artifact('data')
        old = self.claim('old')
        self.derive('evidence', 'old', evidence=['data'])
        context = {'source_revision': 'source-new', 'config': {'machine': 'HP4'}}
        self.graph.review_event('p', 'data', 'applicability', 'New simulator/config needs assessment', context=context)
        result = self.query('support', 'old')
        self.assertTrue(result['states']['old']['structural_support'])
        self.assertEqual(result['states']['old']['availability'], 'recorded')
        self.assertEqual(result['states']['old']['new_applicability'], [{'context': context, 'state': 'unestablished'}])
        new = self.graph.add_claim('p', 'Changed scope', {'machine': 'HP4'}, prior_id='old', record_id='new')
        self.assertEqual(new['details']['version'], 2)
        self.assertEqual(new['details']['claim_key'], 'old')
        self.assertEqual(self.store.get_record('p', 'old'), old)
        self.assertFalse(self.query('support', 'new')['states']['new']['structural_support'])
        self.assertEqual(self.query('search', text='Changed scope', scope={'machine': 'HP4'})['matches'], ['new'])
        reordered = self.store.snapshot('p')
        reordered['records'].reverse()
        with ProgramStore(self.directory / 'reordered', create=True) as restored:
            restored.import_snapshot(reordered)
            self.assertEqual(restored.get_record('p', 'old'), old)
            self.assertTrue(restored.graph().query('p', 'support', 'old')['states']['old']['structural_support'])
        next(r for r in reordered['records'] if r['id'] == 'new')['details'].pop('claim_key')
        with ProgramStore(self.directory / 'malformed-version', create=True) as restored:
            with self.assertRaisesRegex(ProgramError, 'Supersession|claim_key'):
                restored.import_snapshot(reordered)

    def test_diagnostics_are_atomic_and_graph_history_is_immutable(self):
        self.artifact('data')
        self.claim('a')
        before = self.store.snapshot('p')
        requests = [
            ('Missing graph reference: missing', lambda: self.derive('bad', 'a', ['missing'])),
            ('duplicate input identities', lambda: self.derive('bad', 'a', ['data', 'data'])),
            ('Duplicate input identities', lambda: self.derive('bad', 'a', ['data'], ['data'])),
            ('Duplicate graph identity', lambda: self.claim('a')),
            ('at least one ID', lambda: self.graph.add_assessment('p', 'a', 'supported', [], 'No citation')),
            ('artifact config', lambda: self.store.add_artifact('p', 'bad', 'c' * 64, 'imported', 'rev', provenance={'config': []})),
            ('duplicate input identities', lambda: self.store.add_artifact('p', 'bad', 'c' * 64, 'imported', 'rev', provenance={'input_identities': ['same', 'same']})),
            ('Stale program revision', lambda: self.graph.add_claim('p', 'stale', {}, expected_revision=1)),
        ]
        for diagnostic, request in requests:
            with self.subTest(diagnostic=diagnostic), self.assertRaisesRegex(ProgramError, diagnostic):
                request()
            self.assertEqual(self.store.snapshot('p'), before)
        for kind in ('claim', 'artifact'):
            identity = 'a' if kind == 'claim' else 'data'
            for sql in ('UPDATE records SET title=\'bad\' WHERE id=?', 'DELETE FROM records WHERE id=?'):
                with self.assertRaises(sqlite3.IntegrityError), self.store.transaction() as connection:
                    connection.execute(sql, (identity,))

    def test_shared_input_citations_assessments_route_and_open_questions(self):
        self.artifact('data', ['same-measurements'])
        self.artifact('copy', ['same-measurements'])
        self.claim('finding')
        self.derive('source', 'finding', evidence=['data'])
        self.graph.add_assessment('p', 'finding', 'mixed', ['copy'], 'Historical report inspected', actor='curator')
        route = self.store.add_approach('p', 'q', 'Keep defaults', 'Evidence remains mixed', record_id='route')
        decision = self.store.record_note('p', 'decision', 'Retire blanket rule', subject_id=route['id'], evidence_ids=['data'], record_id='choice')
        self.graph.add_relation('p', 'support', 'finding', decision['id'], 'Mixed evidence limits a default change')
        self.graph.add_relation('p', 'decision', decision['id'], 'route', 'Concrete route rationale')
        result = self.query('rationale', 'route')
        self.assertIn('finding', [r['id'] for r in result['records']])
        self.assertEqual(result['shared_inputs']['same-measurements'], ['copy', 'data'])
        self.assertEqual(result['shared_inputs']['sha256:' + 'a' * 64], ['copy', 'data'])
        self.assertEqual(result['states']['finding']['assessment_state'], 'unassessed')
        self.assertEqual(self.query('open_questions')['matches'], ['q'])
        upstream = self.query('upstream', 'finding')
        self.assertEqual(upstream['matches'], ['data', 'finding'])
        self.assertEqual(upstream['sources'][0]['details']['source_revision'], 'source-old')
        self.graph.add_relation('p', 'provenance', 'q', 'data', 'Question came from saved observations')
        self.assertTrue(self.query('open_questions')['sources'])

    def test_graph_snapshot_roundtrip_and_import_rejection(self):
        self.artifact('data')
        self.claim('a')
        self.claim('b')
        self.derive('base', 'a', evidence=['data'])
        self.derive('derived', 'b', ['a'])
        self.graph.add_assessment('p', 'b', 'supported', ['data'], 'Reported, not admitted')
        snapshot = self.store.snapshot('p')
        with ProgramStore(self.directory / 'restored', create=True) as restored:
            restored.import_snapshot(snapshot)
            self.assertEqual(restored.snapshot('p'), snapshot)
            self.assertEqual(restored.graph().query('p', 'upstream', 'b'), self.query('upstream', 'b'))
        for mutation, diagnostic in (
                (lambda s: s['records'][2]['details'].update(knowledge_state='accepted'), 'unassessed'),
                (lambda s: s['records'][-1]['details'].update(admission_state='accepted'), 'cannot admit'),
                (lambda s: s['records'][4]['details'].update(premise_ids=['b']), 'Cyclic'),
                (lambda s: s['records'][5]['details'].update(premise_ids=['missing']), 'Missing graph reference'),
                (lambda s: s['records'][1]['details']['provenance'].update(input_identities=['x', 'x']), 'duplicate input'),
        ):
            candidate = copy.deepcopy(snapshot)
            mutation(candidate)
            with ProgramStore(self.directory / 'invalid', create=True) as restored:
                with self.assertRaisesRegex(ProgramError, diagnostic):
                    restored.import_snapshot(candidate)
                self.assertEqual(restored.connection.execute('SELECT count(*) FROM programs').fetchone()[0], 0)

    def test_cli_graph_roundtrip_and_useful_errors(self):
        def cli(*args, success=True):
            result = subprocess.run([str(ROOT / 'hp-sim5-research-program'), '--store', str(self.directory / 'memory'), *args],
                                    capture_output=True, text=True, timeout=20)
            self.assertEqual(result.returncode, 0 if success else 2, result.stderr)
            self.assertNotIn('Traceback', result.stderr)
            return json.loads(result.stdout) if success else result.stderr
        cli('claim', 'p', 'Observed in HP3 simulation', '--scope', '{"machine":"HP3"}', '--id', 'observed')
        self.artifact('data')
        cli('derive', 'p', 'Frozen result', '--conclusion', 'observed', '--evidence', 'data', '--id', 'proof')
        cli('assess', 'p', 'Producer assessment', '--subject', 'observed', '--outcome', 'supported', '--evidence', 'data')
        cli('relate', 'p', 'Observation limits question', '--relation', 'scope_limit', '--source', 'observed', '--target', 'q')
        cli('review-event', 'p', 'Relevant source changed', '--subject', 'data', '--event', 'applicability', '--context', '{"source_revision":"new"}')
        result = cli('query', 'p', 'support', 'observed')
        self.assertTrue(result['states']['observed']['structural_support'])
        self.assertEqual(len(result['sources']), 1)
        self.assertIn('Missing graph reference: absent', cli('query', 'p', 'support', 'absent', success=False))
        self.assertIn('duplicate input', cli('derive', 'p', 'Bad', '--conclusion', 'observed', '--premise', 'data', '--premise', 'data', success=False))
        self.assertIn('SHA-256', cli('artifact', 'p', 'Bad hash', '--sha256', 'invalid', '--evidence-type', 'simulated',
                                    '--source-revision', 'rev', success=False))


if __name__ == '__main__':
    unittest.main()
