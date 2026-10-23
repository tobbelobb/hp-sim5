"""Offline storage/CLI contracts. Run directly with .venv/bin/python; no pytest needed."""
import copy
import json
import os
from pathlib import Path
import sqlite3
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/python'))
from hp_sim5_research.program import ProgramError, ProgramStore, render_markdown


class ProgramTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)
        self.store_path = self.directory / 'manager'

    def store(self, create=True):
        return ProgramStore(self.store_path, create=create)

    def cli(self, *args, store=None, success=True):
        result = subprocess.run([str(ROOT / 'hp-sim5-research-program'), '--store', str(store or self.store_path),
                                 *args], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0 if success else 2, result.stderr)
        self.assertNotIn('Traceback', result.stderr)
        return result

    def child(self, code, *args):
        environment = dict(os.environ, PYTHONPATH=str(ROOT / 'src/python'))
        return subprocess.Popen([sys.executable, '-c', code, str(self.store_path), *args], env=environment,
                                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    def test_question_alone_and_fresh_process_cli(self):
        created = json.loads(self.cli('create', 'Can the same data improve calibration?').stdout)
        self.assertEqual(created['constraints'], {})
        identity = created['id']
        for command in ('open', 'status', 'overview'):
            overview = json.loads(self.cli(command, identity).stdout)
            self.assertEqual(overview['program'], created)
            self.assertEqual(overview['records'][0]['id'], created['question_id'])
            self.assertEqual(overview['live_alternatives'], [])
            self.assertEqual(overview['assignments'], [])
            self.assertIn('No scientific result', overview['scientific_result'])
        self.assertEqual(self.store_path.stat().st_mode & 0o777, 0o700)
        self.assertEqual((self.store_path / 'programs.sqlite3').stat().st_mode & 0o777, 0o600)

    def test_cli_vertical_slice_and_lossless_export_import(self):
        constraints = {'goal': 'same data', 'hardware': False, 'max_fits': 2}
        program = json.loads(self.cli('create', 'Compare approaches', '--constraints', json.dumps(constraints), '--id', 'pilot').stdout)
        question = json.loads(self.cli('question', 'pilot', 'Which observations distinguish routes?', '--scope', '{"machine":"HP3"}').stdout)
        approaches = []
        for title in ('Screen saved candidates', 'Change the fit'):
            approaches.append(json.loads(self.cli('approach', 'pilot', title, '--question', question['id'],
                                                   '--reason', 'Compare decision value').stdout))
        assignment = json.loads(self.cli('assignment', 'pilot', 'Inspect saved candidates', '--approach', approaches[0]['id'],
                                          '--allocation', '{"wall_minutes":5,"fresh_fits":0}').stdout)
        artifact = json.loads(self.cli('artifact', 'pilot', 'output/research/sessions/example/metrics.json',
                                      '--sha256', 'a' * 64, '--evidence-type', 'synthetic',
                                      '--source-revision', 'candidate-123', '--provenance', '{"dataset":"shared-input"}').stdout)
        self.cli('decision', 'pilot', 'Prefer saved evidence', '--subject', approaches[0]['id'], '--evidence', artifact['id'])
        self.cli('steering', 'pilot', 'Preserve the input data', '--actor', 'human')
        for state in ('ready', 'running', 'completed'):
            result = json.loads(self.cli('transition', 'pilot', assignment['id'], state, '--reason', f'Fixture {state}').stdout)
            self.assertEqual(result['details']['execution_state'], state)
            self.assertEqual(result['details']['review_state'], 'unreviewed')
            self.assertEqual(result['details']['knowledge_state'], 'unassessed')
        self.cli('transition', 'pilot', approaches[1]['id'], 'parked', '--reason', 'Too costly for now')
        before = json.loads(self.cli('overview', 'pilot').stdout)
        self.assertEqual(before['program']['constraints'], constraints)
        self.assertEqual(before['live_alternatives'], [approaches[0]['id']])
        self.assertEqual(before['assignments'], [assignment['id']])
        self.assertEqual(before['program']['question_id'], program['question_id'])
        exported = self.cli('export', 'pilot').stdout
        source = self.directory / 'program.json'
        source.write_text(exported)
        relocated = self.directory / 'restored'
        self.cli('import', str(source), store=relocated)
        self.assertEqual(json.loads(self.cli('export', 'pilot', store=relocated).stdout), json.loads(exported))
        markdown = self.cli('export', 'pilot', '--format', 'markdown').stdout
        self.assertIn(artifact['id'], markdown)
        self.assertIn('a' * 64, markdown)
        self.assertIn('Preserve the input data', markdown)
        self.assertIn('No scientific result', markdown)

    def test_invalid_allocations_do_not_mutate_program(self):
        with self.store() as store:
            program = store.create_program('Bound assignments')
            approach = store.add_approach(program['id'], program['question_id'], 'Inspect', 'Cheap')
            before = store.snapshot(program['id'])
            for allocation in ({}, {'fresh_fits': 1}, {'wall_minutes': 0}, {'wall_minutes': -1},
                               {'wall_minutes': float('inf')}, {'wall_minutes': float('nan')},
                               {'wall_minutes': True}, {'wall_minutes': 1, 'fresh_fits': -1}, None):
                with self.subTest(allocation=allocation), self.assertRaises(ProgramError):
                    store.add_assignment(program['id'], approach['id'], 'Invalid', allocation)
                self.assertEqual(store.snapshot(program['id']), before)
            valid = store.add_assignment(program['id'], approach['id'], 'Small probe', {'wall_minutes': 2})
            self.assertNotIn('fresh_fits', valid['details']['allocation'])

    def test_rejections_are_atomic_and_revisions_detect_stale_views(self):
        with self.store() as store:
            program = store.create_program('Check transactions', program_id='program', question_id='root')
            approach = store.add_approach('program', 'root', 'Route A', 'Reason', record_id='route')
            assignment = store.add_assignment('program', 'route', 'Inspect', {'wall_minutes': 2}, record_id='work')
            before = store.snapshot('program')
            requests = [
                lambda: store.create_program('Duplicate', program_id='program'),
                lambda: store.create_program('Duplicate root', program_id='new', question_id='root'),
                lambda: store.create_program('Global identity', program_id='route'),
                lambda: store.add_question('program', 'Duplicate', record_id='route'),
                lambda: store.add_question('program', 'Program ID', record_id='program'),
                lambda: store.add_question('program', 'Stale', expected_revision=program['revision']),
                lambda: store.add_approach('program', 'work', 'Wrong type', 'Reason'),
                lambda: store.transition('program', assignment['id'], 'completed', 'Skipped running'),
                lambda: store.transition('program', approach['id'], 'live', 'Same state'),
                lambda: store.record_note('program', 'decision', 'Missing evidence', evidence_ids=['absent']),
            ]
            for request in requests:
                with self.assertRaises((ProgramError, sqlite3.IntegrityError)):
                    request()
                self.assertEqual(store.snapshot('program'), before)
            self.assertEqual(store.connection.execute('SELECT count(*) FROM programs').fetchone()[0], 1)
            self.assertEqual(assignment['revision'], 1)
            changed = store.transition('program', assignment['id'], 'ready', 'Prepared',
                                       expected_revision=before['program']['revision'])
            self.assertEqual(changed['revision'], 2)
            self.assertEqual(store.get_program('program')['revision'], before['program']['revision'] + 1)

    def test_route_and_program_state_guards_with_append_only_notes(self):
        with self.store() as store:
            program = store.create_program('State guards')
            identity = program['id']
            approach = store.add_approach(identity, program['question_id'], 'Route', 'Reason')
            work = store.add_assignment(identity, approach['id'], 'Probe', {'wall_minutes': 1})
            store.transition(identity, approach['id'], 'parked', 'Higher cost')
            with self.assertRaises(ProgramError):
                store.transition(identity, work['id'], 'ready', 'Must reject parked route')
            with self.assertRaises(ProgramError):
                store.add_assignment(identity, approach['id'], 'Must reject', {'wall_minutes': 1})
            store.transition(identity, approach['id'], 'live', 'New information')
            store.transition(identity, identity, 'paused', 'Human pause')
            with self.assertRaises(ProgramError):
                store.transition(identity, work['id'], 'ready', 'Must reject paused program')
            store.transition(identity, identity, 'active', 'Human resume')
            store.transition(identity, work['id'], 'ready', 'Prepared')
            note = store.record_note(identity, 'steering', 'Keep original data', actor='human')
            before = store.snapshot(identity)
            for statement in ('UPDATE records SET title=\'changed\' WHERE id=?', 'DELETE FROM records WHERE id=?'):
                with self.assertRaises(sqlite3.IntegrityError), store.transaction() as connection:
                    connection.execute(statement, (note['id'],))
                self.assertEqual(store.snapshot(identity), before)

    def test_concurrent_processes_keep_every_update(self):
        with self.store() as store:
            program = store.create_program('Concurrent research')
        children = [self.child(
            'import sys\nfrom hp_sim5_research.program import ProgramStore\n'
            'with ProgramStore(sys.argv[1]) as store:\n'
            ' for index in range(15):\n'
            '  store.record_note(sys.argv[2], "decision", f"worker-{sys.argv[3]}-{index}")\n',
            program['id'], str(worker)) for worker in range(4)]
        try:
            for child in children:
                stdout, stderr = child.communicate(timeout=30)
                self.assertEqual(child.returncode, 0, stdout + stderr)
        finally:
            for child in children:
                if child.poll() is None:
                    child.kill()
                    child.communicate()
        with self.store(False) as store:
            snapshot = store.snapshot(program['id'])
            notes = [r for r in snapshot['records'] if r['kind'] == 'decision']
            self.assertEqual(len(notes), 60)
            self.assertEqual(len({r['id'] for r in notes}), 60)
            self.assertEqual(len({r['title'] for r in notes}), 60)
            self.assertEqual(snapshot['program']['revision'], 61)

    def test_concurrent_initialization(self):
        children = [self.child(
            'import sys\nfrom hp_sim5_research.program import ProgramStore\n'
            'with ProgramStore(sys.argv[1], create=True) as store:\n'
            ' store.create_program("Independent question")\n') for _ in range(4)]
        try:
            for child in children:
                stdout, stderr = child.communicate(timeout=30)
                self.assertEqual(child.returncode, 0, stdout + stderr)
        finally:
            for child in children:
                if child.poll() is None:
                    child.kill()
                    child.communicate()
        with self.store(False) as store:
            self.assertEqual(store.connection.execute('SELECT count(*) FROM programs').fetchone()[0], 4)

    def test_killed_writer_recovers_without_partial_state(self):
        with self.store() as store:
            program = store.create_program('Survive interruption')
            before = store.snapshot(program['id'])
        child = self.child(
            'import sys\nfrom hp_sim5_research.program import ProgramStore\n'
            'with ProgramStore(sys.argv[1]) as store:\n'
            ' with store.transaction():\n'
            '  store._touch(sys.argv[2])\n'
            '  store._insert(sys.argv[2], "decision", "Uncommitted", {})\n'
            '  print("transaction open", flush=True)\n'
            '  sys.stdin.read()\n', program['id'])
        try:
            self.assertEqual(child.stdout.readline().strip(), 'transaction open')
        finally:
            child.kill()
            child.communicate(timeout=10)
        with self.store(False) as store:
            self.assertEqual(store.snapshot(program['id']), before)
            self.assertEqual(store.connection.execute('PRAGMA integrity_check').fetchone()[0], 'ok')
            store.record_note(program['id'], 'decision', 'Recovered writer')
            self.assertEqual(store.get_program(program['id'])['revision'], 2)

    def test_import_validation_and_duplicate_identity_rollback(self):
        with self.store() as store:
            program = store.create_program('Portable', program_id='source', question_id='source-root')
            snapshot = store.snapshot('source')
            for mutate in (
                lambda s: s.update(format_version=99),
                lambda s: s.update(format_version=True),
                lambda s: s['program'].update(question_id='missing'),
                lambda s: s['program'].update(created_at='invalid'),
                lambda s: s['program'].update(state=[]),
                lambda s: s['records'].append(copy.deepcopy(s['records'][0])),
                lambda s: s['records'][0].update(id=None),
                lambda s: s['records'][0].update(kind=[]),
                lambda s: s['records'][0]['details'].update(knowledge_state='accepted'),
            ):
                bad = copy.deepcopy(snapshot)
                mutate(bad)
                with self.assertRaises(ProgramError):
                    store.import_snapshot(bad)
                self.assertEqual(store.snapshot('source'), snapshot)
        with ProgramStore(self.directory / 'destination', create=True) as store:
            store.create_program('Existing', question_id='source-root')
            with self.assertRaises(sqlite3.IntegrityError):
                store.import_snapshot(snapshot)
            self.assertEqual(store.connection.execute('SELECT count(*) FROM programs').fetchone()[0], 1)
            with self.assertRaises(ProgramError):
                store.get_program(program['id'])
        broken = self.directory / 'broken.json'
        broken.write_text('{"format_version":99}')
        self.cli('import', str(broken), success=False)
        broken.write_text('not JSON')
        self.cli('import', str(broken), success=False)
        self.assertIn('source-root', render_markdown(snapshot))

    def test_refuse_symlinks_permissions_and_unknown_schema(self):
        with self.store() as store:
            store.create_program('Safe location')
        alias = self.directory / 'worker-output'
        alias.symlink_to(self.store_path, target_is_directory=True)
        for linked in (alias, alias / 'nested'):
            with self.assertRaises(ProgramError):
                ProgramStore(linked, create=True)
        database = self.store_path / 'programs.sqlite3'
        moved = self.directory / 'actual.sqlite3'
        database.rename(moved)
        database.symlink_to(moved)
        with self.assertRaises(ProgramError):
            self.store(False)
        database.unlink()
        moved.rename(database)
        journal = self.store_path / 'programs.sqlite3-journal'
        journal.symlink_to(self.directory / 'absent')
        with self.assertRaises(ProgramError):
            self.store(False)
        journal.unlink()
        os.chmod(self.store_path, 0o777)
        with self.assertRaises(ProgramError):
            self.store(False)
        os.chmod(self.store_path, 0o700)
        with sqlite3.connect(database) as connection:
            connection.execute('PRAGMA user_version = 999')
        self.cli('open', 'absent', success=False)
        with self.assertRaisesRegex(ProgramError, 'schema 999'):
            self.store()
        missing = self.directory / 'missing'
        with self.assertRaises(FileNotFoundError):
            ProgramStore(missing)
        self.assertFalse(missing.exists())


if __name__ == '__main__':
    unittest.main()
