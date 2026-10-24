"""One local file-queue host per store retains child ownership across CLI calls.

No sockets, model API, physics service or automatic assignment loop. The files
are private accident prevention for cooperative same-UID clients, not isolation.
"""
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback
import uuid

from .program import ProgramError, ProgramStore, _identity, _json, _object
from .program_manager import ProgramManager, failure
from .program_workers import process_identity


def write_json(path, value):
    temporary = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    try:
        with open(temporary, 'x', opener=lambda p, flags: os.open(p, flags, 0o600)) as stream:
            stream.write(_json(value) + '\n')
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def read_json(path):
    with Path(path).open('rb') as stream:
        raw = stream.read(1024 * 1024 + 1)
    if len(raw) > 1024 * 1024:
        raise ProgramError('Request or descriptor exceeds 1 MiB')
    return _object(json.loads(raw), 'request or descriptor')


def live(descriptor):
    identity = descriptor.get('process')
    if not isinstance(identity, dict) or identity != process_identity(identity.get('pid')):
        return False
    try:
        return Path(f'/proc/{identity["pid"]}/stat').read_text().rsplit(') ', 1)[1].split()[0] not in ('Z', 'X')
    except OSError:
        return False


def request(descriptor, operation, parameters=None, *, timeout=10):
    """Never retry a mutation. A timeout after pickup has an unknown outcome."""
    if not live(descriptor):
        return failure('controller_unavailable', 'Controller is not live; reopen this program. No process was adopted')
    directory = Path(descriptor['store']) / 'controller-queue'
    identity = uuid.uuid4().hex
    queued, response = directory / (identity + '.request'), directory / (identity + '.response')
    deadline = time.time() + timeout
    write_json(queued, {'host_id': descriptor['host_id'], 'program_id': descriptor.get('program_id'),
                        'controller_id': descriptor.get('controller_id'), 'operation': operation,
                        'parameters': parameters, 'expires': deadline})
    try:
        while time.time() < deadline:
            if response.exists():
                return json.loads(response.read_text())
            if not live(descriptor):
                return failure('controller_lost', 'Controller exited during request; inspect durable status before retrying')
            time.sleep(.02)
        return failure('outcome_unknown', 'Controller response timed out; inspect status before retrying a mutation')
    finally:
        queued.unlink(missing_ok=True)
        response.unlink(missing_ok=True)


def open_manager(store_path, root, program_id):
    """Start an owned host when needed, then open a program without dispatching."""
    root = Path(root).resolve()
    with ProgramStore(store_path) as store:
        store.get_program(program_id)
        directory = store.directory
    # Serialize the check and startup, including simultaneous fresh chats.
    with (directory / 'controller-start.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        descriptor_path = directory / 'controller.json'
        descriptor = read_json(descriptor_path) if descriptor_path.exists() else {}
        if live(descriptor):
            if descriptor['root'] != str(root):
                raise ProgramError('Store controller belongs to another repository')
        else:
            with (directory / 'controller.log').open('ab') as output:
                process = subprocess.Popen([sys.executable, str(root / 'scripts/research_program_host.py'),
                                            '--store', str(directory), '--root', str(root)],
                                           cwd=root, stdin=subprocess.DEVNULL, stdout=output, stderr=output,
                                           start_new_session=True)
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                descriptor = read_json(descriptor_path) if descriptor_path.exists() else {}
                if live(descriptor) and descriptor['process']['pid'] == process.pid:
                    break
                if process.poll() is not None:
                    raise ProgramError('Controller startup failed; inspect controller.log')
                time.sleep(.02)
            else:
                raise ProgramError('Controller startup timed out; inspect controller.log before reopening')
        return request(descriptor, '_open', {'program_id': program_id})


def serve(store_path, root):
    """Poll requests and owned child exits on the SQLite owner thread."""
    with ProgramStore(store_path) as store, (Path(store_path) / 'controller.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return 2
        root = Path(root).resolve()
        queue = store.directory / 'controller-queue'
        queue.mkdir(mode=0o700, exist_ok=True)
        host = {'version': 1, 'host_id': _identity('host'), 'store': str(store.directory),
                'root': str(root), 'process': process_identity(os.getpid())}
        managers, attachments = {}, {}
        stopping = False

        def shutdown(*_):
            nonlocal stopping
            stopping = True

        signal.signal(signal.SIGTERM, shutdown)
        signal.signal(signal.SIGINT, shutdown)
        write_json(store.directory / 'controller.json', host)
        while True:
            for path in sorted(queue.glob('*.request')):
                result = None
                try:
                    envelope = read_json(path)
                    if envelope.get('expires', 0) < time.time():
                        continue
                    if envelope.get('host_id') != host['host_id']:
                        result = failure('stale_attachment', 'Attachment belongs to an earlier controller; reopen the program')
                    elif envelope.get('operation') == '_open':
                        if envelope.get('program_id') is not None or envelope.get('controller_id') is not None:
                            raise ProgramError('A bound tool attachment cannot open another program; use the entry CLI')
                        if stopping:
                            raise ProgramError('Host is shutting down; wait for cleanup before reopening')
                        program_id = _identity('program', _object(envelope['parameters'], 'parameters')['program_id'])
                        store.get_program(program_id)
                        if program_id not in managers:
                            controller_id = _identity('controller')
                            session = root / 'output/research/managers' / program_id / controller_id
                            managers[program_id] = ProgramManager(store, root, program_id, controller_id, session)
                            connection = store.directory / 'managers' / program_id / 'connection.json'
                            connection.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
                            attachments[program_id] = dict(host, program_id=program_id, controller_id=controller_id,
                                                          connection=str(connection), session=str(session))
                            write_json(connection, attachments[program_id])
                        manager = managers[program_id]
                        if manager.stopping:
                            raise ProgramError('Program controller is shutting down; wait for cleanup before reopening')
                        result = {'ok': True, 'result': {'attachment': attachments[program_id],
                                                       'overview': manager.overview(program_id)}}
                    else:
                        program_id = envelope.get('program_id')
                        manager = managers.get(program_id) if isinstance(program_id, str) else None
                        if manager is None or envelope.get('controller_id') != manager.controller_id:
                            result = failure('wrong_attachment', 'Program and controller identity do not match; reopen this program')
                        else:
                            result = manager.call(envelope.get('operation'), envelope.get('parameters'))
                except (OSError, ValueError, TypeError, KeyError) as error:
                    result = failure('invalid_request', str(error))
                except Exception:
                    traceback.print_exc()
                    result = failure('internal_error', 'Host request failed; inspect controller.log')
                finally:
                    if result is not None:
                        write_json(path.with_suffix('.response'), result)
                    path.unlink(missing_ok=True)
            for manager in managers.values():
                try:
                    if stopping and not manager.stopping:
                        manager.shutdown(manager.program_id)
                    manager.poll()
                except Exception:
                    traceback.print_exc()
            if (stopping or managers) and all(m.stopping and m.drained() for m in managers.values()):
                return 0
            for program_id, manager in tuple(managers.items()):
                if manager.stopping and manager.drained():
                    del managers[program_id]
                    del attachments[program_id]
            # Expired client responses are not program evidence.
            for path in queue.glob('*.response'):
                try:
                    if time.time() - path.stat().st_mtime > 60:
                        path.unlink(missing_ok=True)
                except FileNotFoundError:
                    pass
            time.sleep(.05)
