"""Bounded standard-library CLI smoke checks; no supervisor or physics needed."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'src/python'))
from hp_sim5_research.archive import initial_abstract, saved_autocal_status

INVOCATIONS = []


def cli(script, *args, expected=0):
    argv = [sys.executable, str(ROOT / 'scripts' / script), *map(str, args)]
    run = subprocess.run(argv, cwd=ROOT, capture_output=True, text=True, timeout=10)
    INVOCATIONS.append(dict(command=argv, expected_exit=expected, exit_code=run.returncode,
                            stdout=run.stdout, stderr=run.stderr))
    if run.returncode != expected:
        raise AssertionError((argv, run.returncode, run.stdout, run.stderr))
    return run.stdout


def fixture(parent):
    tree = parent / 'old-worktree'
    session = tree / 'session'
    session.mkdir(parents=True)
    (session / 'abstract.md').write_text(
        '# Synthetic claim\n\n## Decision handoff\n'
        '- Scope: fixture only.\n- Review: synthetic.\n'
        '- Decision: preserve defaults.\n- Next: observer-next-wave.\n'
        '[Measurements](metrics.json)\n')
    (session / 'report.md').write_text('Synthetic portability check; no accuracy claim.\n')
    (session / 'research.md').write_text('No new scientific experiment.\n')
    (session / 'metrics.json').write_text('{"fixture_only": true}\n')
    (session / 'job.json').write_text(json.dumps(dict(status='accepted', exit_code=0, pid=1)))
    external = tree / 'external-input.json'
    external.write_text('{"saved_input": [1, 2, 3]}\n')
    selection = dict(claim='Synthetic smoke fixture', scope='Selected bytes only',
                     decision='Preserve defaults', source_revision='fixture-revision',
                     files=[dict(path='metrics.json'), dict(path='job.json', role='job'),
                            dict(path='inputs/history.json', source=str(external), role='input',
                                 sha256=hashlib.sha256(external.read_bytes()).hexdigest())])
    return tree, session, selection


class SmokeTests(unittest.TestCase):
    def test_catalog_build_search_pending_and_legacy(self):
        with tempfile.TemporaryDirectory(prefix='program-catalog-') as temporary:
            root = Path(temporary)
            tree, source, _ = fixture(root)
            sessions = root / 'catalog/sessions'
            sessions.mkdir(parents=True)
            complete = sessions / ('a' * 32)
            shutil.copytree(source, complete)
            (sessions / 'descriptive-result').symlink_to(complete.name, target_is_directory=True)
            pending = sessions / 'pending'; pending.mkdir()
            (pending / 'abstract.md').write_text(initial_abstract('Unanswered radius question'))
            legacy = sessions / 'legacy'; legacy.mkdir()
            (legacy / 'exit.json').write_text('{"returncode": 0}')
            cli('research_index.py', '--root', sessions.parent)
            index = (sessions.parent / 'index.md').read_text()
            self.assertEqual(index.count('## [descriptive-result]'), 1)
            self.assertNotIn('## [' + 'a' * 32 + ']', index)
            self.assertIn('No research result is available yet', index)
            self.assertIn('Status: abstract missing', index)
            found = cli('research_index.py', '--root', sessions.parent, '--search', 'observer-next-wave')
            self.assertIn('- Scope: fixture only.', found)
            self.assertIn('[Measurements](sessions/descriptive-result/metrics.json)', found)
            self.assertNotIn('## [pending]', found)

    def test_handoff_relocation_without_original_paths(self):
        with tempfile.TemporaryDirectory(prefix='program-relocate-') as temporary:
            root = Path(temporary)
            tree, session, selection = fixture(root)
            (session / 'unselected.rrd').write_text('unneeded synthetic recording')
            (session / 'connection.json').write_text('unselected synthetic attachment')
            plan = root / 'selection.json'; plan.write_text(json.dumps(selection))
            exported = root / 'export'
            json.loads(cli('research_handoff.py', 'export', session, plan, exported))
            relocated = root / 'new-host'; shutil.move(exported, relocated)
            shutil.rmtree(tree); plan.unlink()
            recovered = json.loads(cli('research_handoff.py', 'show', relocated))
            self.assertEqual(recovered['integrity'], 'verified_selected_files')
            self.assertEqual(recovered['decision'], 'Preserve defaults')
            self.assertEqual(len(recovered['files']), 6)
            self.assertTrue(all(not Path(f['source']).exists() for f in recovered['files']))
            self.assertTrue(all(f['integrity'] == 'ok' for f in recovered['files']))
            self.assertEqual(recovered['jobs'][0]['observation'], 'terminal')
            self.assertEqual(recovered['jobs'][0]['scientific_success'], 'not_assessed')
            self.assertFalse((relocated / 'unselected.rrd').exists())
            self.assertFalse((relocated / 'connection.json').exists())
            (relocated / 'inputs/history.json').write_text('corruption after transfer')
            partial = json.loads(cli('research_handoff.py', 'show', relocated, expected=1))
            self.assertEqual(partial['integrity'], 'partial')
            self.assertTrue(any('SHA-256 mismatch' in d for d in partial['diagnostics']))
            self.assertTrue((relocated / 'report.md').exists())

    def test_partial_export_and_invalid_request(self):
        with tempfile.TemporaryDirectory(prefix='program-partial-') as temporary:
            root = Path(temporary)
            _, session, selection = fixture(root)
            selection['files'][0]['sha256'] = '0' * 64
            selection['files'].append(dict(path='missing-input.json', role='input'))
            (session / 'job.json').write_text('{interrupted write')
            plan = root / 'selection.json'; plan.write_text(json.dumps(selection))
            exported = root / 'partial'
            partial = json.loads(cli('research_handoff.py', 'export', session, plan, exported, expected=1))
            self.assertEqual(partial['integrity'], 'partial')
            self.assertEqual(partial['jobs'][0]['observation'], 'unavailable')
            self.assertTrue(any('Source differs' in d for d in partial['diagnostics']))
            self.assertTrue(any('missing-input.json' in d for d in partial['diagnostics']))
            self.assertTrue((exported / 'report.md').exists())
            selection['files'].append(dict(path='../escape'))
            plan.write_text(json.dumps(selection))
            cli('research_handoff.py', 'export', session, plan, root / 'invalid', expected=2)
            self.assertFalse((root / 'invalid').exists())

    def test_job_states_never_imply_liveness_or_scientific_success(self):
        with tempfile.TemporaryDirectory(prefix='program-status-') as temporary:
            path = Path(temporary) / 'job.json'
            for status in ('accepted', 'failed', 'cancelled', 'budget_exhausted', 'running', 'cancelling'):
                path.write_text(json.dumps(dict(status=status, pid=1, exit_code=0)))
                result = json.loads(cli('research_handoff.py', 'job', path))
                expected = 'terminal' if status not in ('running', 'cancelling') else 'historical_nonterminal'
                self.assertEqual(result['observation'], expected)
                self.assertEqual(result['live_status'], 'unknown')
                self.assertEqual(result['scientific_success'], 'not_assessed')
                with patch('os.kill', side_effect=AssertionError('No signalling')), \
                     patch('os.getpgid', side_effect=AssertionError('No PID adoption')):
                    self.assertEqual(saved_autocal_status(path)['saved_status'], status)
            path.write_text('{corrupt')
            result = json.loads(cli('research_handoff.py', 'job', path, expected=1))
            self.assertEqual(result['observation'], 'unavailable')


if __name__ == '__main__':
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(SmokeTests)
    outcome = unittest.TextTestRunner(verbosity=2).run(suite)
    summary = dict(command='.venv/bin/python ' + str(Path(__file__).relative_to(ROOT)),
                   source_revision=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                   tests_run=outcome.testsRun, failures=len(outcome.failures), errors=len(outcome.errors),
                   passed=outcome.wasSuccessful(), cli_invocations=len(INVOCATIONS),
                   actual_budget=dict(fresh_fits=0, physics_steps=0, collections=0, prediction_evaluations=0),
                   scope='Synthetic CLI/lifecycle smoke checks; no calibration or independent-agent evaluation')
    (HERE / 'cli-invocations.json').write_text(json.dumps(INVOCATIONS, indent=2) + '\n')
    (HERE / 'smoke-results.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary))
    raise SystemExit(0 if outcome.wasSuccessful() else 1)
