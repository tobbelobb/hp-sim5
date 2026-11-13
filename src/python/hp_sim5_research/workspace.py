"""Stable investigation identities; launcher state stays outside the research corpus."""
from contextlib import contextmanager
import fcntl
import json
import os
from pathlib import Path
import subprocess
import shlex
import tempfile
import uuid

from .archive import session_name


def common_root(root):
    result = subprocess.run(['git', '-C', str(root), 'rev-parse', '--path-format=absolute', '--git-common-dir'],
                            capture_output=True, text=True, check=True)
    return Path(result.stdout.strip()).parent


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as stream:
        temporary = Path(stream.name)
        json.dump(value, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')
    try:
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def read_json(path):
    return json.loads(Path(path).read_text())


@contextmanager
def lock(path, *, blocking=True):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('a') as stream:
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | (0 if blocking else fcntl.LOCK_NB))
        except BlockingIOError:
            raise ValueError('This investigation is already open; use a separate researcher') from None
        yield


class Sessions:
    def __init__(self, root):
        self.root = Path(root).resolve()
        self.state = common_root(self.root) / '.research-state'

    def path(self, identity):
        if Path(identity).name != identity or identity in ('.', '..'):
            raise ValueError('Invalid investigation ID')
        return self.state / 'sessions' / (identity + '.json')

    def save(self, session):
        write_json(self.path(session['id']), session)

    def open(self, *, resume=None, directory=None, topic='research', backend=None, conversation=None, dry_run=False):
        session = None
        if resume:
            if resume == 'latest':
                resume = read_json(self.state / 'last-agent.json')['id']
            candidate = Path(resume).expanduser()
            if candidate.is_dir():
                directory = candidate.resolve()
                matches = [read_json(p) for p in (self.state / 'sessions').glob('*.json')
                           if Path(read_json(p)['directory']).resolve() == directory]
                # Old launchers kept their manifest in the artifact directory.
                old = directory / 'session.json'
                session = read_json(old) if old.is_file() else {}
                if session.get('backend'):
                    session['clients_started'] = {session['backend']: True}
                identity = matches[0]['id'] if matches else directory.name
                saved = self.path(identity)
                session = read_json(saved) if saved.is_file() else dict(session, id=identity, directory=str(directory))
            elif self.path(resume).is_file():
                session = read_json(self.path(resume))
            else:
                # Committed evidence remains reopenable on a different host.
                candidate = self.root / 'research/output' / resume
                if not candidate.is_dir():
                    raise ValueError(f'Unknown investigation: {resume}; supply its ID or artifact directory')
                return self.open(resume=str(candidate), backend=backend, conversation=conversation, dry_run=dry_run)
        if session is None:
            identity = (Path(directory).name if directory and Path(directory).name.startswith('session-')
                        else 'session-' + session_name(topic))
            directory = Path(directory).expanduser().resolve() if directory else self.root / 'research/output' / identity
            if directory.exists():
                raise ValueError('Artifact directory exists; use --resume DIRECTORY')
            if self.path(identity).exists():
                raise ValueError('Investigation already exists; use --resume ID')
            if any(Path(read_json(p)['directory']).resolve() == directory
                   for p in (self.state / 'sessions').glob('*.json')):
                raise ValueError('Artifact path is already reserved; use --resume with its investigation ID')
            session = {'id': identity, 'directory': str(directory), 'topic': topic, 'repo': str(self.root),
                       'backend': backend, 'conversations': {}}
        if directory and Path(session['directory']).resolve() != Path(directory).resolve():
            raise ValueError('--session-dir differs from the resumed investigation')
        session.setdefault('conversations', {})
        session.setdefault('topic', topic)
        session.setdefault('repo', str(self.root))
        if backend:
            session['backend'] = backend
        if conversation:
            if not session.get('backend'):
                raise ValueError('Binding a conversation requires --codex or --claude')
            session['conversations'][session['backend']] = conversation
        if not dry_run:
            self.save(session)
            write_json(self.state / 'last-agent.json', {'id': session['id']})
        return session

    def run_directory(self, session):
        directory = self.state / 'runs' / session['id'] / uuid.uuid4().hex
        directory.mkdir(parents=True, mode=0o700)
        return directory


def role(root, name):
    return (Path(root) / 'research-program/roles' / (name + '.md')).read_text().strip()


def researcher_handoff(root, session):
    if not Path(session['repo']).is_dir():
        raise ValueError(f'Saved workspace is unavailable: {session["repo"]}; restore committed evidence before resuming')
    directory = session['directory']
    relative = (str(Path(directory).relative_to(session['repo']))
                if Path(directory).is_relative_to(session['repo']) else 'research/output/' + session['id'])
    return {'id': session['id'], 'session_dir': directory, 'repo': session['repo'],
            'artifact_relative': relative,
            'instructions': role(root, 'researcher'),
            'task': f"{session['topic']}\n\nInvestigation: {session['id']}\nArtifacts: {directory}\n"
                    'Continue existing evidence. In a platform-provided worktree, bind this handoff and use its returned artifact path:\n'
                    f"./hp-sim5-research-agent --resume {shlex.quote(session['id'])} --workspace\n"
                    'If the ID is absent on this host, reopen copied evidence by its directory, or prepare the same ID locally:\n'
                    f'./hp-sim5-research-agent --session-dir {shlex.quote(relative)} --prompt {shlex.quote(session["topic"])}\n'
                    'Start tools only if needed:\n'
                    f"./hp-sim5-research-agent --resume {shlex.quote(session['id'])} --start-runtime\n"
                    'The returned tool connection supports shell calls without command sockets.'}
