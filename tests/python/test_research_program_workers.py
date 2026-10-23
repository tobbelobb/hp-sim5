"""Real child processes and real ProgramStore, with fake launchers; no physics."""
import copy
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/python'))
from hp_sim5_research.program import ProgramError, ProgramStore
from hp_sim5_research.program_workers import ACTIVE, WorkerDispatcher, process_identity


FIXTURE = '''
import json, os, signal, sys, time
from pathlib import Path
args = sys.argv[1:]
def option(flag):
    return args[args.index(flag) + 1]
session = Path(option('--session-dir'))
prompt = Path(option('--prompt-file')).read_bytes()
mode = MODE
if mode == 'auth':
    print('Authentication unavailable; sign in with the existing client', file=sys.stderr, flush=True)
    sys.exit(17)
session.mkdir(parents=True)
(session / 'prompt.txt').write_bytes(prompt)
(session / 'received-argv.json').write_text(json.dumps(args))
if mode == 'startup':
    print('partial startup', file=sys.stderr, flush=True)
    time.sleep(60)
    sys.exit(1)
def stopped(signum, frame):
    if mode == 'ignore':
        return
    (session / 'cleanup.txt').write_text('supervisor cleanup reached')
    if mode != 'broken-cleanup':
        observation = {'returncode': 130}
        if mode == 'reported-cleanup':
            observation['cleanup_errors'] = ['fixture shutdown failed']
        (session / 'exit.json').write_text(json.dumps(observation))
    sys.exit(130)
signal.signal(signal.SIGTERM, stopped)
(session / 'session.json').write_text(json.dumps({
    'session_dir': str(session), 'launcher_session_id': 'launcher-session',
    'repo': '/returned/worktree', 'branch': 'research/actual', 'source_revision': 'actual-source'}))
(session / 'launch.json').write_text(json.dumps({'runtime_status': {'session_id': 'physics-world'}}))
with (session / 'events.jsonl').open('w') as events:
    events.write('{"type":"result","result":"SUCCESS"}\\n' if mode != 'partial' else '{"type":')
(session / 'final-message.md').write_text('SUCCESS is not scientific acceptance\\n')
print('child output', flush=True)
print('child diagnostic', file=sys.stderr, flush=True)
if mode in ('hold', 'partial', 'ignore', 'broken-cleanup', 'reported-cleanup'):
    time.sleep(60)
code = 3 if mode == 'nonzero' else 0
(session / 'exit.json').write_text(json.dumps({'returncode': code}))
sys.exit(code)
'''


class WorkerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.root = self.directory / 'repo'
        self.root.mkdir()
        subprocess.run(['git', 'init', '-q', str(self.root)], check=True)
        subprocess.run(['git', '-C', str(self.root), '-c', 'user.name=fixture', '-c', 'user.email=f@f',
                        'commit', '--allow-empty', '-qm', 'fixture'], check=True)
        self.store = ProgramStore(self.directory / 'manager', create=True)
        self.addCleanup(self.store.close)
        self.store.create_program('Which evidence changes the decision?', {'keep': '$literal'}, program_id='p', question_id='q')
        self.store.add_approach('p', 'q', 'Inspect one input', 'Cheap scoped observation', record_id='route')
        self.new_assignment('a')
        self.dispatchers = []
        self.addCleanup(self.retire)

    def new_assignment(self, identity):
        self.store.add_assignment('p', 'route', 'Inspect a "quoted" input\nPreserve `text` and $variables',
                                  {'wall_minutes': 2, 'fresh_fits': 0, 'collections': 0}, record_id=identity)
        self.store.transition('p', identity, 'ready', 'Fixture is ready')

    def dispatcher(self, mode='normal', missing=False):
        script = self.directory / ('missing' if missing else 'fake-launcher-' + mode)
        if not missing:
            script.write_text(f'#!{sys.executable}\n' + FIXTURE.replace('MODE', repr(mode)))
            script.chmod(0o755)
        result = self.store.workers(self.root, executable=script)
        self.dispatchers.append(result)
        return result

    def retire(self):
        for dispatcher in self.dispatchers:
            for process in dispatcher.children.values():
                if process.poll() is None:
                    if process_identity(process.pid) is not None:
                        os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)

    def until(self, dispatcher, identity, predicate, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            record = dispatcher.status('p', identity)
            if predicate(record):
                return record
            time.sleep(.01)
        self.fail(f'Worker did not reach expected observation: {record}')

    def finish(self, dispatcher, identity):
        return self.until(dispatcher, identity, lambda r: r['details']['state'] not in ACTIVE)

    def started(self, dispatcher, identity):
        return self.until(dispatcher, identity, lambda r: 'runtime_session_id' in r['details'])

    def test_exact_assignment_prompt_argv_artifacts_and_no_admission(self):
        dispatcher = self.dispatcher()
        instructions = 'Read "names"; preserve $targets and `angles`.\nDo not run physics. €'
        before_graph = self.store.graph().query('p', 'open_questions')
        started = dispatcher.start('p', 'a', instructions=instructions,
                                   settings={'backend': 'codex', 'viewer': 'none', 'record': False,
                                             'model_provider': 'openai', 'model': 'chosen-model',
                                             'worktree': True, 'worktree_dir': str(self.directory / 'work trees'),
                                             'machine': 'hp3', 'physics_backend': 'native-python',
                                             'client_args': ['--resume', 'thread', '-c', 'custom="$literal"']})
        record = self.finish(dispatcher, started['id'])
        d = record['details']
        self.assertEqual(d['state'], 'submitted')
        self.assertEqual(d['returncode'], 0)
        self.assertEqual(d['scientific_state'], 'unassessed')
        self.assertEqual(d['cleanup'], 'launcher_finally_completed')
        self.assertEqual(d['launcher_session_id'], 'launcher-session')
        self.assertEqual(d['runtime_session_id'], 'physics-world')
        self.assertEqual(d['worktree']['repo'], '/returned/worktree')
        self.assertEqual(d['worktree_id'], '/returned/worktree')
        self.assertEqual(d['worktree']['source_revision'], 'actual-source')
        self.assertNotEqual(d['owner']['process']['pid'], d['process']['pid'])
        self.assertEqual(d['process']['group'], d['process']['pid'])
        prompt = (Path(d['attempt_dir']) / 'prompt.txt').read_bytes()
        self.assertEqual(hashlib.sha256(prompt).hexdigest(), d['prompt_sha256'])
        self.assertEqual(prompt, (Path(d['session_dir']) / 'prompt.txt').read_bytes())
        self.assertTrue(prompt.decode().endswith(instructions))
        self.assertIn('"fresh_fits": 0', prompt.decode())
        argv = json.loads((Path(d['session_dir']) / 'received-argv.json').read_text())
        self.assertEqual(argv, d['argv'][1:])
        self.assertEqual(argv[argv.index('--') + 1:], ['--resume', 'thread', '-c', 'custom="$literal"'])
        self.assertIn('--batch', argv)
        self.assertIn('--no-record', argv)
        self.assertIn('--worktree', argv)
        self.assertEqual(self.store.get_record('p', 'a')['details']['execution_state'], 'completed')
        self.assertEqual(self.store.get_record('p', 'a')['details']['review_state'], 'unreviewed')
        after_graph = self.store.graph().query('p', 'open_questions')
        self.assertGreater(after_graph.pop('program_revision'), before_graph.pop('program_revision'))
        self.assertEqual(after_graph, before_graph)
        self.assertEqual(len(dispatcher.sessions('p')), 1)
        self.assertTrue(any(a['path'].endswith('/events.jsonl') for a in d['artifacts']))
        self.assertTrue(all(not a['partial'] for a in d['artifacts']))
        snapshot = self.store.snapshot('p')
        with ProgramStore(self.directory / 'restored', create=True) as restored:
            restored.import_snapshot(snapshot)
            self.assertEqual(snapshot, restored.snapshot('p'))
            historical = restored.workers(self.root).status('p', record['id'])
            self.assertFalse(historical['owned'])
            self.assertIn('historical', historical['observation'])
        invalid = copy.deepcopy(snapshot)
        next(r for r in invalid['records'] if r['kind'] == 'attempt')['details']['scientific_state'] = 'accepted'
        with ProgramStore(self.directory / 'invalid', create=True) as restored:
            with self.assertRaisesRegex(ProgramError, 'cannot admit'):
                restored.import_snapshot(invalid)
            invalid = copy.deepcopy(snapshot)
            next(r for r in invalid['records'] if r['kind'] == 'attempt')['details'].pop('diagnostics')
            with self.assertRaisesRegex(ProgramError, 'required attempt fields'):
                restored.import_snapshot(invalid)

    def test_nonzero_auth_and_missing_executable(self):
        for mode in ('nonzero', 'auth', 'missing'):
            with self.subTest(mode=mode):
                assignment = 'a' if mode == 'nonzero' else mode
                if assignment != 'a':
                    self.new_assignment(assignment)
                dispatcher = self.dispatcher(mode, missing=mode == 'missing')
                record = dispatcher.start('p', assignment)
                record = self.finish(dispatcher, record['id'])
                d = record['details']
                self.assertEqual(d['state'], 'launch_failed' if mode == 'missing' else 'failed')
                self.assertEqual(self.store.get_record('p', assignment)['details']['execution_state'], 'failed')
                if mode == 'auth':
                    self.assertIn('Authentication unavailable', (Path(d['attempt_dir']) / 'stderr.log').read_text())
                if mode == 'missing':
                    self.assertIn('FileNotFoundError', d['diagnostics'][0])
                    self.assertTrue(any(a['path'].endswith('stderr.log') for a in d['artifacts']))
                self.assertEqual(d['scientific_state'], 'unassessed')

    def test_responsive_partial_logs_and_supervisor_cancellation(self):
        dispatcher = self.dispatcher('partial')
        before = time.monotonic()
        record = dispatcher.start('p', 'a')
        self.assertLess(time.monotonic() - before, 2)
        self.started(dispatcher, record['id'])
        before = time.monotonic()
        status = dispatcher.status('p', record['id'])
        self.assertLess(time.monotonic() - before, .5)
        self.assertEqual(status['details']['state'], 'running')
        self.assertTrue(any(a['partial'] and a['path'].endswith('events.jsonl') for a in status['details']['artifacts']))
        with self.assertRaisesRegex(ProgramError, 'already reserved'):
            self.new_assignment('b')
            self.dispatcher().start('p', 'b')
        self.store.transition('p', 'a', 'failed', 'Controller recorded failure before cleanup')
        before = time.monotonic()
        dispatcher.cancel('p', record['id'])
        self.assertLess(time.monotonic() - before, .5)
        stopped = self.finish(dispatcher, record['id'])['details']
        self.assertEqual(stopped['state'], 'cancelled')
        self.assertEqual(self.store.get_record('p', 'a')['details']['execution_state'], 'failed')
        self.assertEqual(stopped['cleanup'], 'launcher_finally_completed')
        self.assertTrue(any(a['partial'] and a['path'].endswith('events.jsonl') for a in stopped['artifacts']))
        session = Path(stopped['session_dir'])
        self.assertEqual((session / 'events.jsonl').read_text(), '{"type":')
        self.assertIn('supervisor cleanup reached', (session / 'cleanup.txt').read_text())

    def test_interrupted_startup_and_missing_cleanup_manifest(self):
        for mode in ('startup', 'broken-cleanup', 'reported-cleanup'):
            with self.subTest(mode=mode):
                if mode != 'startup':
                    self.new_assignment(mode)
                dispatcher = self.dispatcher(mode)
                record = dispatcher.start('p', 'a' if mode == 'startup' else mode)
                if mode == 'startup':
                    self.until(dispatcher, record['id'], lambda r: (Path(r['details']['attempt_dir']) / 'stderr.log').stat().st_size > 0)
                else:
                    self.started(dispatcher, record['id'])
                dispatcher.cancel('p', record['id'])
                stopped = self.finish(dispatcher, record['id'])['details']
                self.assertEqual(stopped['state'], 'cancelled')
                self.assertIn('failed' if mode == 'reported-cleanup' else 'unverified', stopped['cleanup'])
                self.assertTrue(Path(stopped['attempt_dir']).exists())

    def test_partial_and_malformed_manifests_are_diagnostics(self):
        dispatcher = self.dispatcher('hold')
        record = dispatcher.start('p', 'a')
        status = self.started(dispatcher, record['id'])
        manifest = Path(status['details']['session_dir']) / 'launch.json'
        for content in ('{"runtime_status":', '{"runtime_status": []}', 'x' * (1024 * 1024 + 1)):
            with self.subTest(content=content[:30]):
                manifest.write_text(content)
                status = dispatcher.status('p', record['id'])
                self.assertEqual(status['details']['state'], 'running')
                self.assertTrue(any(d.startswith('Partial manifest:') for d in status['details']['diagnostics']))
        manifest.write_text('{"runtime_status": {"session_id": "recovered"}}')
        status = dispatcher.status('p', record['id'])
        self.assertEqual(status['details']['runtime_session_id'], 'recovered')
        self.assertEqual(status['details']['diagnostics'], [])
        dispatcher.cancel('p', record['id'])
        self.finish(dispatcher, record['id'])

    def test_force_stop_reports_detached_cleanup_unknown(self):
        dispatcher = self.dispatcher('ignore')
        record = dispatcher.start('p', 'a')
        self.started(dispatcher, record['id'])
        dispatcher.cancel('p', record['id'], grace_seconds=0)
        stopped = self.finish(dispatcher, record['id'])['details']
        self.assertEqual(stopped['returncode'], -signal.SIGKILL)
        self.assertIn('failed', stopped['cleanup'])
        self.assertIn('detached services unverified', stopped['cleanup'])

    def test_historical_and_reused_birth_identity_are_never_signalled(self):
        dispatcher = self.dispatcher('hold')
        record = dispatcher.start('p', 'a')
        self.started(dispatcher, record['id'])
        historical = self.store.workers(self.root)
        with patch('hp_sim5_research.program_workers.os.kill') as kill:
            with self.assertRaisesRegex(ProgramError, 'historical'):
                historical.cancel('p', record['id'])
            kill.assert_not_called()
        original = self.store.get_record('p', record['id'])['details']
        forged = copy.deepcopy(original)
        forged['process']['birth'] = 'historical-reused-pid-birth'
        dispatcher._save('p', record['id'], forged)
        with patch('hp_sim5_research.program_workers.os.kill') as kill:
            with self.assertRaisesRegex(ProgramError, 'ownership'):
                dispatcher.cancel('p', record['id'])
            kill.assert_not_called()
        self.assertEqual(dispatcher.status('p', record['id'])['details']['state'], 'ownership_lost')
        self.assertIsNone(dispatcher.children[record['id']].poll())
        dispatcher._save('p', record['id'], original)
        dispatcher.cancel('p', record['id'])
        self.finish(dispatcher, record['id'])

    def test_keyboard_interrupt_during_spawn_keeps_attempt(self):
        dispatcher = self.dispatcher()
        with patch('hp_sim5_research.program_workers.subprocess.Popen', side_effect=KeyboardInterrupt):
            # Git inspection also uses Popen, so patch only the spawn after git calls.
            with patch('hp_sim5_research.program_workers.subprocess.run', return_value=subprocess.CompletedProcess([], 0, 'fixture')):
                with self.assertRaises(KeyboardInterrupt):
                    dispatcher.start('p', 'a')
        records = dispatcher.sessions('p')
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0]['details']['state'], 'launch_failed')
        self.assertEqual(records[0]['details']['cleanup'], 'no_process_started')

    def test_validation_refuses_unready_and_invalid_settings_without_spawning(self):
        dispatcher = self.dispatcher()
        for settings in ({'backend': 'other'}, {'backend': 'claude', 'model_provider': 'openai'},
                         {'client_args': '--resume x'}, {'record': 1}, {'worktree_dir': '/tmp/x'}, []):
            with self.subTest(settings=settings), self.assertRaises(ProgramError):
                dispatcher.start('p', 'a', settings=settings)
        self.assertEqual(dispatcher.sessions('p'), [])
        self.store.transition('p', 'p', 'paused', 'Pause before dispatch')
        with self.assertRaisesRegex(ProgramError, 'active'):
            dispatcher.start('p', 'a')

    def test_adapter_argv_passes_real_launcher_dry_run_for_both_clients(self):
        for backend in ('codex', 'claude'):
            with self.subTest(backend=backend):
                if backend == 'claude':
                    self.new_assignment(backend)
                dispatcher = self.dispatcher()
                record = dispatcher.start('p', 'a' if backend == 'codex' else backend,
                                          settings={'backend': backend, 'model': 'example-model',
                                                    'machine': 'hp3', 'viewer': 'none', 'record': False,
                                                    'sidebar_section': 'none', 'goal_mode': False,
                                                    'client_args': ['--resume', 'thread-id']},
                                          instructions='Preserve `literal` and $variables')
                record = self.finish(dispatcher, record['id'])
                argv = [str(ROOT / 'hp-sim5-research-agent'), '--dry-run', *record['details']['argv'][1:]]
                result = subprocess.run(argv, cwd='/tmp', capture_output=True, text=True, timeout=15)
                self.assertEqual(result.returncode, 0, result.stderr)
                plan = json.loads(result.stdout)
                self.assertEqual(plan['mode'], 'batch')
                self.assertEqual(plan['backend'], backend)
                self.assertEqual(plan['prompt'].encode(), (Path(record['details']['attempt_dir']) / 'prompt.txt').read_bytes())
                self.assertIn('--resume', plan['argv'])
                self.assertFalse(Path(record['details']['session_dir']).name == 'dry-run')
                self.assertIn('--model', plan['argv'])
                # Worktree dry-run plans without creating a world or directory.
                worktree = subprocess.run([*argv[:2], '--worktree', '--worktree-dir', str(self.directory / 'new-worktrees'),
                                          *argv[2:]], capture_output=True, text=True, timeout=15)
                self.assertEqual(worktree.returncode, 0, worktree.stderr)
                self.assertEqual(json.loads(worktree.stdout)['mode'], 'worktree')
                self.assertFalse((self.directory / 'new-worktrees').exists())


if __name__ == '__main__':
    unittest.main()
