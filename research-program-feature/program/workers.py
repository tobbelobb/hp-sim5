"""Bounded worker adapter for the existing launcher; no physics or client SDK imports.

Keep this dispatcher alive and poll status after start/cancel. The retained host
enforces deadlines; saved process identities alone never authorize signals.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

import machines

from .store import ProgramError, _allocation, _identity, _json, _now, _object, _text, _wall_minutes
from . import pool
from hp_sim5_research.client_transport import client_adapter
from .communication import pending_question, worker_policy


ACTIVE = ('starting', 'running', 'cancelling', 'ownership_lost')
TERMINAL = ('submitted', 'failed', 'cancelled', 'launch_failed', 'interrupted')
FILES = ('session.json', 'launch.json', 'exit.json', 'prompt.txt', 'events.jsonl',
         'final-message.md', 'research.md', 'report.md', 'abstract.md',
         'instructions.md', 'attachment.toml', 'attachment.json', 'connection.json',
         'manager-question.json', 'manager-answer.json',
         'native/runtime.log', 'viewer.log')


def runtime_check(root, *, physics_backend='headless-js'):
    """Check in a launcher's environment, including when its host predates setup."""
    argv = [sys.executable, str(Path(root) / 'scripts/research_runtime_check.py'),
            '--physics-backend', physics_backend]
    adapter = client_adapter()
    if adapter:
        argv = adapter.runtime_check_command(root, argv)
    result = subprocess.run(argv, cwd=root, capture_output=True, text=True, timeout=55)
    if result.returncode not in (0, 1) or not result.stdout.strip():
        raise ProgramError(f'Runtime setup check failed: {result.stderr[-2000:].strip()}')
    checked = json.loads(result.stdout)
    return adapter.runtime_check_result(checked) if adapter else checked


def process_identity(pid):
    """Linux birth identity, including boot and UID; absence never authorizes a signal."""
    try:
        proc = Path(f'/proc/{pid}')
        fields = (proc / 'stat').read_text().rsplit(') ', 1)[1].split()
        return {'pid': pid, 'birth': fields[19], 'group': int(fields[2]),
                'boot_id': Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                'uid': proc.stat().st_uid, 'host': socket.gethostname()}
    except (OSError, ValueError, IndexError):
        return None


def process_live(identity):
    if not isinstance(identity, dict) or identity != process_identity(identity.get('pid')):
        return False
    try:
        return Path(f'/proc/{identity["pid"]}/stat').read_text().rsplit(') ', 1)[1].split()[0] not in ('Z', 'X')
    except OSError:
        return False


def validate_attempts(records):
    """Validate export references, without treating imported process IDs as live."""
    for record in records.values():
        if record['kind'] != 'attempt':
            continue
        details = record['details']
        required = {'assignment_id', 'state', 'scientific_state', 'owner', 'process',
                    'source_revision', 'source_dirty', 'settings', 'argv', 'session_dir', 'attempt_dir',
                    'prompt_sha256', 'observations', 'artifacts', 'started_at', 'exited_at',
                    'returncode', 'cleanup', 'diagnostics'}
        if not required <= details.keys() or ('wall_minutes' not in details and 'allocation' not in details):
            raise ProgramError('Missing required attempt fields')
        assignment = records.get(details.get('assignment_id'))
        if assignment is None or assignment['kind'] != 'assignment':
            raise ProgramError('Attempt requires an assignment reference')
        if details.get('state') not in (*ACTIVE, *TERMINAL):
            raise ProgramError('Invalid attempt state')
        if details.get('scientific_state') != 'unassessed':
            raise ProgramError('Worker execution cannot admit scientific knowledge')
        if 'wall_minutes' in details:
            _wall_minutes(details['wall_minutes'])
        else:
            _allocation(details['allocation'])
        for name in ('owner', 'settings'):
            _object(details.get(name), name)
        for name in ('session_dir', 'attempt_dir', 'source_revision', 'prompt_sha256', 'cleanup'):
            _text(details.get(name), name)
        if (not isinstance(details['diagnostics'], list) or
                not all(isinstance(item, str) for item in details['diagnostics']) or
                not isinstance(details['artifacts'], list) or
                (details['process'] is not None and not isinstance(details['process'], dict)) or
                (details['returncode'] is not None and type(details['returncode']) is not int)):
            raise ProgramError('Invalid attempt process/artifacts/diagnostics')
        if (not isinstance(details.get('argv'), list) or
                not all(isinstance(arg, str) for arg in details['argv']) or
                not isinstance(details.get('observations'), list)):
            raise ProgramError('Invalid attempt argv/observations')
        for field, kinds in (('premise_ids', ('claim', 'artifact')), ('premise_plan_ids', ('route_plan',)),
                             ('premise_event_ids_at_start', ('review_event',))):
            if field in details and (not isinstance(details[field], list) or any(
                    i not in records or records[i]['kind'] not in kinds for i in details[field])):
                raise ProgramError('Invalid attempt premise references')
        for conflict in details.get('premise_conflicts', []):
            if (conflict.get('attempt_id') != record['id'] or conflict.get('assignment_id') != assignment['id'] or
                    conflict.get('event_id') not in records or records[conflict['event_id']]['kind'] != 'review_event' or
                    not set(conflict.get('premise_ids', [])) <= set(details.get('premise_ids', []))):
                raise ProgramError('Invalid attempt premise conflict')


class WorkerDispatcher:
    """Own only processes started by this instance; capacity is shared across a store.

    A stale active reservation blocks new dispatch rather than adopting a PID.
    The caller must keep polling, including after cancellation. No background
    thread uses the store and no status call waits for the investigation to end.
    """

    def __init__(self, store, root, *, executable=None):
        self.store = store
        self.root = Path(root).resolve()
        self.executable = str(Path(executable).absolute() if executable else self.root / 'hp-sim5-research-agent')
        self.owner = {'id': _identity('dispatcher'), 'process': process_identity(os.getpid())}
        if self.owner['process'] is None:
            raise ProgramError('Worker dispatch requires Linux process birth identity')
        self.children = {}
        self.deadlines = {}

    @staticmethod
    def _settings(settings):
        options = dict(_object({} if settings is None else settings, 'worker settings'))
        allowed = {'backend', 'model_provider', 'model', 'worktree', 'worktree_dir',
                   'physics_backend', 'machine', 'viewer', 'record', 'client_args',
                   'dont_ask', 'goal_mode', 'sidebar_section', 'shared_runtime_write'}
        if set(options) - allowed:
            raise ProgramError(f'Unknown worker settings: {sorted(set(options) - allowed)}')
        result = dict(backend='codex', physics_backend='headless-js',
                      viewer='headless', record=True, worktree=False, client_args=[],
                      dont_ask=False, goal_mode=True, shared_runtime_write=False)
        result.update(options)
        if 'machine' in result:
            try:
                result['machine'] = machines.machine(result['machine'])['name']
            except ValueError as error:
                raise ProgramError(str(error)) from None
        for name, choices in (('backend', ('codex', 'claude')),
                              ('physics_backend', ('headless-js',)),
                              ('viewer', ('none', 'headless', 'window'))):
            if result[name] not in choices:
                raise ProgramError(f'Unsupported {name}: {result[name]}')
        for name in ('record', 'worktree', 'dont_ask', 'goal_mode', 'shared_runtime_write'):
            if type(result[name]) is not bool:
                raise ProgramError(f'{name} must be boolean')
        if result.get('worktree_dir') and not result['worktree']:
            raise ProgramError('worktree_dir requires worktree')
        if result['backend'] == 'claude' and result.get('model_provider'):
            raise ProgramError('model_provider is only supported with Codex')
        if (not isinstance(result['client_args'], list) or
                not all(isinstance(arg, str) and '\0' not in arg for arg in result['client_args'])):
            raise ProgramError('client_args must be an argv list')
        for name in ('model', 'model_provider', 'worktree_dir', 'sidebar_section'):
            if name in result:
                _text(result[name], name)
        adapter = client_adapter()
        if adapter:
            adapter.validate_worker_settings(result)
        return result

    def assignment_settings(self, program_id, assignment_id, settings=None):
        """Resolve optional machine guidance and reject conflicting worker settings."""
        options = dict(_object({} if settings is None else settings, 'worker settings'))
        assignment = self.store.get_record(program_id, assignment_id, 'assignment')
        commissioned = assignment['details'].get('settings', {})
        if 'machine' in commissioned and 'machine' in options and (
                self._settings({'machine': commissioned['machine']})['machine'] !=
                self._settings({'machine': options['machine']})['machine']):
            raise ProgramError('Worker machine conflicts with commissioned settings.machine')
        options = dict(commissioned, **options)
        return self._settings(options)

    def start(self, program_id, assignment_id, *, settings=None, instructions='', runtime=None):
        """Reserve and spawn once; return without waiting for services or a client.

        The assignment snapshot and unchanged additional instructions become one
        UTF-8 prompt file, passed by absolute argv path through worktree re-exec.
        An optional wall timeout requests the existing launcher's cleanup.
        """
        options = self.assignment_settings(program_id, assignment_id, settings)
        if not isinstance(instructions, str):
            raise ProgramError('instructions must be a string')
        attempt_id = _identity('attempt')
        directory = self.store.directory / 'workers' / attempt_id
        session = (self.root / 'research/output' / program_id / f'session-{attempt_id}').resolve()
        if session.exists():
            raise ProgramError('Worker session directory must be new')
        revision = subprocess.run(['git', '-C', str(self.root), 'rev-parse', 'HEAD'],
                                  capture_output=True, text=True, check=True).stdout.strip()
        dirty = subprocess.run(['git', '-C', str(self.root), 'status', '--porcelain'],
                               capture_output=True, text=True, check=True).stdout
        with self.store.transaction():
            program = self.store.get_program(program_id)
            assignment = self.store.get_record(program_id, assignment_id, 'assignment')
            approach = self.store.get_record(program_id, assignment['details']['approach_id'], 'approach')
            if (program['state'] != 'active' or approach['details']['state'] not in ('live', 'promising') or
                    assignment['details']['execution_state'] != 'ready'):
                raise ProgramError('Dispatch requires an active program/approach and ready assignment')
            pool.check_worker(self.store, program_id, assignment_id, exclusive=options['shared_runtime_write'])
            if pool.limit(self.store) > 1 and not options['worktree']:
                raise ProgramError('A parallel worker pool requires separate launcher worktrees')
            minutes = assignment['details'].get('wall_minutes')
            if minutes is None and isinstance(assignment['details'].get('allocation'), dict):
                minutes = assignment['details']['allocation'].get('wall_minutes')
            minutes = _wall_minutes(minutes)
            directory.mkdir(mode=0o700, parents=True)
            contract = {'program_id': program_id, 'question': assignment['title'],
                        'session_dir': str(session)}
            if 'machine' in options:
                contract['machine'] = options['machine']
            prompt = ('# Research question\n\n' + assignment['title'] +
                      ('\n\nMachine: ' + options['machine'] + ' (machines/catalog.json)' if 'machine' in options else '') +
                      '\n\nSession artifacts: ' + str(session) +
                      worker_policy(session) +
                      (client_adapter().worker_instructions() if client_adapter() else '') +
                      '\n\n' + instructions)
            prompt_path = directory / 'prompt.txt'
            prompt_path.write_bytes(prompt.encode('utf-8'))
            (directory / 'assignment.json').write_text(json.dumps(contract, indent=2, ensure_ascii=False) + '\n')
            argv = [self.executable, '--' + options['backend'], '--batch', '--prompt-file', str(prompt_path),
                    '--session-dir', str(session), '--physics-backend', options['physics_backend'],
                    '--viewer', options['viewer']]
            wall_deadline = time.time() + minutes * 60 if minutes is not None else None
            if runtime and runtime.get('wall_deadline') is not None:
                wall_deadline = min(wall_deadline, runtime['wall_deadline']) if wall_deadline else runtime['wall_deadline']
            if wall_deadline is not None:
                argv += ['--wall-deadline', str(wall_deadline)]
            for name in ('model_provider', 'model', 'worktree_dir', 'sidebar_section'):
                if name in options:
                    value = options[name]
                    if name == 'worktree_dir':
                        value = str(Path(value).expanduser().resolve())
                    argv += ['--' + name.replace('_', '-'), value]
            for key, flag in (('worktree', '--worktree'), ('dont_ask', '--dont-ask')):
                if options[key]:
                    argv.append(flag)
            if not options['record']:
                argv.append('--no-record')
            if not options['goal_mode']:
                argv.append('--no-goal-mode')
            if options['client_args']:
                argv += ['--', *options['client_args']]
            details = {'assignment_id': assignment_id, 'state': 'starting', 'scientific_state': 'unassessed',
                       'wall_minutes': minutes, 'owner': self.owner, 'process': None,
                       'source_revision': revision, 'source_dirty': bool(dirty), 'settings': options,
                       'argv': argv, 'session_dir': str(session), 'attempt_dir': str(directory),
                       'prompt_sha256': hashlib.sha256(prompt.encode('utf-8')).hexdigest(),
                       'observations': [{'at': _now(), 'event': 'reserved'}], 'artifacts': [],
                       'started_at': None, 'exited_at': None, 'returncode': None,
                       'cleanup': 'not_requested', 'diagnostics': []}
            details['runtime'] = dict(runtime or {})
            if wall_deadline is not None:
                details['runtime']['wall_deadline'] = wall_deadline
                self.deadlines[attempt_id] = time.monotonic() + max(0, wall_deadline - time.time())
            self.store._touch(program_id)
            self.store._insert(program_id, 'attempt', assignment['title'], details, attempt_id)
            assignment['details']['execution_state'] = 'running'
            self._write(assignment_id, assignment['details'])
        process = None
        try:
            with (directory / 'stdout.log').open('wb') as stdout, (directory / 'stderr.log').open('wb') as stderr:
                process = subprocess.Popen(argv, cwd=self.root, stdin=subprocess.DEVNULL,
                                           stdout=stdout, stderr=stderr, start_new_session=True)
            self.children[attempt_id] = process
            details['process'] = process_identity(process.pid)
            details['started_at'] = _now()
            details['state'] = 'running'
            details['observations'].append({'at': _now(), 'event': 'spawned', 'pid': process.pid})
            self._save(program_id, attempt_id, details)
        except BaseException as error:
            details['diagnostics'].append(f'{type(error).__name__}: {error}')
            if process is None:
                details.update(state='launch_failed', exited_at=_now(), cleanup='no_process_started')
                self._save(program_id, attempt_id, details, assignment_state='failed')
            else:
                self.children[attempt_id] = process
                details['process'] = process_identity(process.pid)
                self._save(program_id, attempt_id, details)
                self.cancel(program_id, attempt_id)
            if not isinstance(error, OSError):
                raise
        return self.status(program_id, attempt_id)

    def _write(self, record_id, details):
        self.store.connection.execute('UPDATE records SET details_json=?, revision=revision+1, updated_at=? WHERE id=?',
                                      (_json(details), _now(), record_id))

    def _save(self, program_id, attempt_id, details, *, assignment_state=None):
        with self.store.transaction():
            self.store._touch(program_id)
            self._write(attempt_id, details)
            if assignment_state:
                assignment = self.store.get_record(program_id, details['assignment_id'], 'assignment')
                if assignment['details']['execution_state'] == 'running':
                    assignment['details']['execution_state'] = assignment_state
                    self._write(assignment['id'], assignment['details'])

    def _owned(self, attempt_id, details):
        process = self.children.get(attempt_id)
        return (process is not None and details['owner'] == self.owner and details['process'] is not None and
                details['process'] == process_identity(process.pid))

    @staticmethod
    def _inventory(details):
        session, directory = Path(details['session_dir']), Path(details['attempt_dir'])
        details['artifacts'] = []
        details['diagnostics'] = [d for d in details['diagnostics'] if not d.startswith('Partial manifest:')]
        for path in [*(session / name for name in FILES), *(directory / name for name in ('stdout.log', 'stderr.log', 'prompt.txt', 'assignment.json'))]:
            try:
                info = path.stat()
                details['artifacts'].append({'path': str(path), 'bytes': info.st_size,
                                             'partial': details['state'] != 'submitted'})
            except FileNotFoundError:
                continue
        for name in ('session.json', 'launch.json', 'exit.json'):
            path = session / name
            if not path.exists():
                continue
            try:
                with path.open('rb') as stream:
                    raw = stream.read(1024 * 1024 + 1)
                if len(raw) > 1024 * 1024:
                    raise ValueError('manifest exceeds 1 MiB')
                manifest = _object(json.loads(raw), 'launcher manifest')
                if name == 'session.json':
                    if Path(manifest['session_dir']).resolve() != session:
                        raise ValueError('returned session path differs from requested session')
                    details['launcher_session_id'] = manifest['launcher_session_id']
                    details['worktree_id'] = manifest['repo']
                    details['worktree'] = {key: manifest[key] for key in ('repo', 'branch', 'source_revision')}
                elif name == 'launch.json':
                    details['runtime_session_id'] = _object(manifest.get('runtime_status', {}), 'runtime status').get('session_id')
                else:
                    details['launcher_exit'] = manifest
                    if manifest.get('cleanup_errors'):
                        details['cleanup_errors'] = manifest['cleanup_errors']
                        details['cleanup'] = 'failed; launcher reported cleanup errors'
            except (OSError, ValueError, KeyError, TypeError) as error:
                details['diagnostics'].append(f'Partial manifest: {name}: {error}')

    def status(self, program_id, attempt_id):
        record = self.store.get_record(program_id, attempt_id, 'attempt')
        details = record['details']
        clarification = pending_question(details['session_dir'])
        record = dict(record, manager_question=clarification,
                      manager_answer_path=str(Path(details['session_dir']) / 'manager-answer.json'))
        process = self.children.get(attempt_id)
        if process is None:
            before = _json(details)
            self._inventory(details)
            if _json(details) != before:
                self._save(program_id, attempt_id, details)
                record.update(self.store.get_record(program_id, attempt_id, 'attempt'))
            return dict(record, observation='historical; not adopted', owned=False)
        before = _json(details)
        returncode = process.poll()
        assignment_state = None
        if returncode is not None and details['state'] in ACTIVE:
            cancelled = details['state'] == 'cancelling'
            details.update(returncode=returncode, exited_at=_now(),
                           state='cancelled' if cancelled else 'submitted' if returncode == 0 else 'failed')
            details['observations'].append({'at': _now(), 'event': 'exited', 'returncode': returncode})
            assignment_state = 'cancelled' if cancelled else 'completed' if returncode == 0 else 'failed'
        elif returncode is None and details['state'] in ACTIVE:
            if not self._owned(attempt_id, details):
                details['state'] = 'ownership_lost'
                details['cleanup'] = 'refused_unverified_process'
            elif details['state'] == 'cancelling' and time.time() >= details['cancel_deadline']:
                # Only the birth-verified launcher group, never arbitrary saved PIDs.
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                    details['cleanup'] = 'failed; forced launcher group stop, detached services unverified'
                    details['observations'].append({'at': _now(), 'event': 'forced_stop'})
                    details['cancel_deadline'] = float('1e100')
                except OSError as error:
                    details['cleanup'] = f'failed: {error}'
            elif (details['state'] in ('starting', 'running') and not details.get('runtime', {}).get('expired') and
                  (time.time() >= details.get('runtime', {}).get('wall_deadline', float('inf')) or
                   time.monotonic() >= self.deadlines.get(attempt_id, float('inf')))):
                details['observations'].append({'at': _now(), 'event': 'wall_expired'})
                details['runtime']['expired'] = True
                self._save(program_id, attempt_id, details)
                return self.cancel(program_id, attempt_id, grace_seconds=details.get('runtime', {}).get('cleanup_seconds', 190))
        self._inventory(details)
        if details['state'] in TERMINAL and details['cleanup'] != 'no_process_started':
            if not details['cleanup'].startswith('failed'):
                details['cleanup'] = ('launcher_finally_completed' if 'launcher_exit' in details else
                                      'unverified; launcher exit manifest missing')
        if details['started_at'] and details['state'] in TERMINAL:
            from datetime import datetime
            end = datetime.fromisoformat(details['exited_at']).timestamp() if details['exited_at'] else time.time()
            details['observed_wall_seconds'] = max(0, end - datetime.fromisoformat(details['started_at']).timestamp())
        if _json(details) != before:
            self._save(program_id, attempt_id, details, assignment_state=assignment_state)
            record.update(self.store.get_record(program_id, attempt_id, 'attempt'))
        return dict(record, owned=self._owned(attempt_id, details), observation='observed child')

    def answer(self, program_id, attempt_id, question_id, answer):
        """Answer a pending worker question by ID; validate and write its mailbox atomically."""
        _text(question_id, 'question_id')
        _text(answer, 'answer')
        payload = (_json({'id': question_id, 'answer': answer}) + '\n').encode('utf-8')
        if len(payload) > 16384:
            raise ProgramError('Worker answer exceeds the 16 KiB mailbox limit')
        record = self.status(program_id, attempt_id)
        if not record['owned'] or record['details']['state'] not in ('starting', 'running'):
            raise ProgramError('Worker answer requires an owned active attempt')
        if not record['manager_question'] or record['manager_question']['id'] != question_id:
            raise ProgramError('question_id does not match a pending worker question; inspect worker_status')
        destination = Path(record['manager_answer_path'])
        temporary = destination.with_suffix('.tmp')
        try:
            temporary.write_bytes(payload)
            temporary.replace(destination)
        finally:
            temporary.unlink(missing_ok=True)
        return self.status(program_id, attempt_id)

    def cancel(self, program_id, attempt_id, *, grace_seconds=190):
        """Request launcher cleanup, then return. Poll until exit, including on failure.

        Runtime shutdown can take 160s + 15s + fallback termination; the default
        grace allows that path. An explicit shorter grace can lose cleanup.
        """
        if (isinstance(grace_seconds, bool) or not isinstance(grace_seconds, (int, float)) or
                not math.isfinite(grace_seconds) or grace_seconds < 0):
            raise ProgramError('grace_seconds must be finite and nonnegative')
        record = self.status(program_id, attempt_id)
        details = record['details']
        if details['state'] in TERMINAL or details['state'] == 'cancelling':
            return record
        if not self._owned(attempt_id, details):
            raise ProgramError('Refusing to signal historical or unverified worker ownership')
        details.update(state='cancelling', cleanup='requested', cancel_deadline=time.time() + grace_seconds)
        details['observations'].append({'at': _now(), 'event': 'cancel_requested'})
        self._save(program_id, attempt_id, details)
        try:
            # Signal the launcher first; it retires the client, runtime and Viewer.
            os.kill(self.children[attempt_id].pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        except OSError as error:
            details['cleanup'] = f'failed: {error}'
            self._save(program_id, attempt_id, details)
        return self.status(program_id, attempt_id)

    def sessions(self, program_id):
        """Durable paths for human inspection, with explicit current ownership."""
        identities = [row[0] for row in self.store.connection.execute(
            "SELECT id FROM records WHERE program_id=? AND kind='attempt' ORDER BY rowid", (program_id,))]
        return [self.status(program_id, identity) for identity in identities]
