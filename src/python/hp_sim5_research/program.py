"""Transactional manager-owned program memory; no physics or worker imports."""
from contextlib import contextmanager
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import sqlite3
import stat
import uuid


SCHEMA_VERSION = 1
KINDS = ('question', 'approach', 'assignment', 'artifact', 'decision', 'steering',
         'claim', 'derivation', 'relation', 'assessment', 'review_event', 'attempt',
         'submission', 'submission_event', 'review_run', 'review', 'admission',
         'route_plan', 'investigation', 'strategy_event', 'human_input')
TRANSITIONS = {
    'program': {'active': {'paused', 'stopped'}, 'paused': {'active', 'stopped'}, 'stopped': set()},
    'approach': {'live': {'promising', 'parked', 'retired'},
                 'promising': {'live', 'parked', 'retired'},
                 'parked': {'live', 'promising', 'retired'}, 'retired': {'live'}},
    'assignment': {'planned': {'ready', 'cancelled'}, 'ready': {'running', 'cancelled'},
                   'running': {'completed', 'failed', 'cancelled'},
                   'completed': set(), 'failed': set(), 'cancelled': set()},
}


class ProgramError(ValueError):
    """An invalid request, stale revision or unsupported saved format."""


def _text(value, label):
    if not isinstance(value, str) or not value.strip():
        raise ProgramError(f'{label} must be a nonempty string')
    return value


def _json(value):
    try:
        return json.dumps(value, allow_nan=False, sort_keys=True)
    except (TypeError, ValueError) as error:
        raise ProgramError(f'Expected finite JSON data: {error}') from error


def _object(value, label):
    if not isinstance(value, dict):
        raise ProgramError(f'{label} must be a JSON object')
    _json(value)
    return value


def _allocation(value):
    _object(value, 'allocation')
    for name, amount in value.items():
        if not isinstance(name, str) or isinstance(amount, bool) or not isinstance(amount, (int, float)):
            raise ProgramError('Allocation entries must be named finite nonnegative numbers')
        if (isinstance(amount, float) and not math.isfinite(amount)) or amount < 0:
            raise ProgramError('Allocation entries must be finite and nonnegative')
    if value.get('wall_minutes', 0) <= 0:
        raise ProgramError('An assignment requires a positive finite wall_minutes allocation')
    return value


def _now():
    return datetime.now(timezone.utc).isoformat()


def _identity(prefix, value=None):
    value = value if value is not None else f'{prefix}-{uuid.uuid4().hex}'
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,127}', value):
        raise ProgramError('IDs must be 1–128 letters, digits, dots, underscores or hyphens')
    return value


def _owned_path(path, directory=False):
    info = path.lstat()
    if stat.S_ISLNK(info.st_mode) or info.st_uid != os.geteuid():
        raise ProgramError(f'Store path must be owned by the manager and not a symlink: {path}')
    if directory:
        if not stat.S_ISDIR(info.st_mode) or info.st_mode & 0o022:
            raise ProgramError(f'Store directory must not be writable by group/others: {path}')
    elif not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or info.st_mode & 0o022:
        raise ProgramError(f'Store file must be a private regular file with one link: {path}')


class ProgramStore:
    """One manager-owned SQLite database for multiple programs.

    The directory is deliberately separate from shared worker output. Refuse
    linked paths, unsafe ownership and writable-by-others store files. This is
    accident prevention, not adversarial isolation from processes with our UID.
    Each mutation takes an immediate transaction and advances program revision.
    """

    def __init__(self, directory, *, create=False):
        self.directory = Path(os.path.abspath(Path(directory).expanduser()))
        for part in reversed((self.directory, *self.directory.parents)):
            if part.is_symlink():
                raise ProgramError(f'Authoritative store may not follow symlinks: {part}')
        if create:
            self.directory.mkdir(mode=0o700, parents=True, exist_ok=True)
        _owned_path(self.directory, directory=True)
        self.path = self.directory / 'programs.sqlite3'
        if create:
            try:
                descriptor = os.open(self.path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
            except FileExistsError:
                pass
            else:
                os.close(descriptor)
        for path in (self.path, *(Path(str(self.path) + suffix) for suffix in ('-journal', '-wal', '-shm'))):
            if path.exists() or path.is_symlink():
                _owned_path(path)
        _owned_path(self.path)
        self.connection = sqlite3.connect(self.path.as_uri() + '?mode=rw', uri=True,
                                          isolation_level=None, timeout=30)
        self.connection.row_factory = sqlite3.Row
        try:
            self.connection.execute('PRAGMA foreign_keys = ON')
            self.connection.execute('PRAGMA synchronous = FULL')
            with self.transaction() as connection:
                version = connection.execute('PRAGMA user_version').fetchone()[0]
                if version == 0 and create and not connection.execute(
                        "SELECT 1 FROM sqlite_master WHERE name NOT LIKE 'sqlite_%'").fetchone():
                    # Individual statements keep schema creation inside this transaction.
                    for statement in (
                        'CREATE TABLE programs (id TEXT PRIMARY KEY, question_id TEXT NOT NULL, '
                        'goal TEXT NOT NULL, constraints_json TEXT NOT NULL, state TEXT NOT NULL, '
                        'revision INTEGER NOT NULL, created_at TEXT NOT NULL, updated_at TEXT NOT NULL)',
                        'CREATE TABLE records (id TEXT PRIMARY KEY, program_id TEXT NOT NULL '
                        'REFERENCES programs(id), kind TEXT NOT NULL, title TEXT NOT NULL, '
                        'details_json TEXT NOT NULL, revision INTEGER NOT NULL, '
                        'created_at TEXT NOT NULL, updated_at TEXT NOT NULL)',
                        'CREATE INDEX program_records ON records(program_id, kind)',
                        'CREATE TRIGGER unique_program_identity BEFORE INSERT ON programs '
                        'WHEN EXISTS (SELECT 1 FROM records WHERE id=NEW.id) BEGIN '
                        "SELECT RAISE(ABORT, 'Duplicate identity'); END",
                        'CREATE TRIGGER unique_record_identity BEFORE INSERT ON records '
                        'WHEN EXISTS (SELECT 1 FROM programs WHERE id=NEW.id) BEGIN '
                        "SELECT RAISE(ABORT, 'Duplicate identity'); END",
                        "CREATE TRIGGER immutable_notes_update BEFORE UPDATE ON records "
                        "WHEN OLD.kind IN ('decision', 'steering') BEGIN "
                        "SELECT RAISE(ABORT, 'Decisions and steering are append-only'); END",
                        "CREATE TRIGGER immutable_notes_delete BEFORE DELETE ON records "
                        "WHEN OLD.kind IN ('decision', 'steering') BEGIN "
                        "SELECT RAISE(ABORT, 'Decisions and steering are append-only'); END",
                        f'PRAGMA user_version = {SCHEMA_VERSION}',
                    ):
                        connection.execute(statement)
                elif version != SCHEMA_VERSION:
                    raise ProgramError(f'Unsupported program schema {version}; expected {SCHEMA_VERSION}')
                # Additive record kinds use the existing version-1 record envelope.
                for action in ('UPDATE', 'DELETE'):
                    connection.execute(
                        f'CREATE TRIGGER IF NOT EXISTS immutable_graph_{action.lower()} BEFORE {action} ON records '
                        "WHEN OLD.kind IN ('claim', 'artifact', 'derivation', 'relation', 'assessment', 'review_event') "
                        "BEGIN SELECT RAISE(ABORT, 'Graph history is append-only'); END")
                    connection.execute(
                        f'CREATE TRIGGER IF NOT EXISTS immutable_review_{action.lower()} BEFORE {action} ON records '
                        "WHEN OLD.kind IN ('submission', 'submission_event', 'review', 'admission') "
                        "BEGIN SELECT RAISE(ABORT, 'Review history is append-only'); END")
                    connection.execute(
                        f'CREATE TRIGGER IF NOT EXISTS immutable_strategy_{action.lower()} BEFORE {action} ON records '
                        "WHEN OLD.kind IN ('route_plan', 'investigation', 'strategy_event', 'human_input') "
                        "BEGIN SELECT RAISE(ABORT, 'Strategy history is append-only'); END")
        except BaseException:
            self.close()
            raise

    def close(self):
        self.connection.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    @contextmanager
    def transaction(self):
        self.connection.execute('BEGIN IMMEDIATE')
        try:
            yield self.connection
            self.connection.execute('COMMIT')
        except BaseException:
            self.connection.execute('ROLLBACK')
            raise

    @staticmethod
    def _decode(row):
        if row is None:
            raise ProgramError('Unknown program or record ID')
        value = dict(row)
        for key in ('constraints', 'details'):
            if key + '_json' in value:
                value[key] = json.loads(value.pop(key + '_json'))
        return value

    def get_program(self, program_id):
        return self._decode(self.connection.execute('SELECT * FROM programs WHERE id=?', (program_id,)).fetchone())

    def get_record(self, program_id, record_id, kind=None):
        result = self._decode(self.connection.execute(
            'SELECT * FROM records WHERE program_id=? AND id=?', (program_id, record_id)).fetchone())
        if kind is not None and result['kind'] != kind:
            raise ProgramError(f'{record_id} must identify a {kind}')
        return result

    def graph(self):
        """Typed graph operations share this store's transactions and revisions."""
        from .program_graph import EvidenceGraph
        return EvidenceGraph(self)

    def workers(self, root, **options):
        """One live dispatcher; saved attempts are inspection records, never PID adoption."""
        from .program_workers import WorkerDispatcher
        return WorkerDispatcher(self, root, **options)

    def reviews(self, root):
        """Controller submission/review/admission API; starts no world or viewer."""
        from .program_review import ReviewController
        return ReviewController(self, root)

    def _touch(self, program_id, expected_revision=None):
        program = self.get_program(program_id)
        if expected_revision is not None and program['revision'] != expected_revision:
            raise ProgramError(f'Stale program revision: expected {expected_revision}, found {program["revision"]}')
        self.connection.execute('UPDATE programs SET revision=revision+1, updated_at=? WHERE id=?',
                                (_now(), program_id))
        return program

    def _insert(self, program_id, kind, title, details, record_id=None):
        identity, timestamp = _identity(kind, record_id), _now()
        self.connection.execute('INSERT INTO records VALUES (?, ?, ?, ?, ?, 1, ?, ?)',
                                (identity, program_id, kind, _text(title, 'title'), _json(details), timestamp, timestamp))
        return self.get_record(program_id, identity)

    def create_program(self, question, constraints=None, *, program_id=None, question_id=None):
        _text(question, 'question')
        constraints = {} if constraints is None else _object(constraints, 'constraints')
        identity, root_id, timestamp = _identity('program', program_id), _identity('question', question_id), _now()
        with self.transaction():
            self.connection.execute('INSERT INTO programs VALUES (?, ?, ?, ?, ?, 1, ?, ?)',
                                    (identity, root_id, question, _json(constraints), 'active', timestamp, timestamp))
            self._insert(identity, 'question', question, {'scope': {}, 'knowledge_state': 'unassessed'}, root_id)
        return self.get_program(identity)

    def add_question(self, program_id, question, *, scope=None, record_id=None, expected_revision=None):
        with self.transaction():
            self._touch(program_id, expected_revision)
            return self._insert(program_id, 'question', question,
                                {'scope': _object({} if scope is None else scope, 'scope'),
                                 'knowledge_state': 'unassessed'}, record_id)

    def add_approach(self, program_id, question_id, title, reason, *, record_id=None, expected_revision=None):
        with self.transaction():
            self._touch(program_id, expected_revision)
            self.get_record(program_id, question_id, 'question')
            return self._insert(program_id, 'approach', title,
                                {'question_id': question_id, 'state': 'live', 'reason': _text(reason, 'reason')}, record_id)

    def add_assignment(self, program_id, approach_id, objective, allocation, *, record_id=None, expected_revision=None):
        _allocation(allocation)
        with self.transaction():
            program = self._touch(program_id, expected_revision)
            approach = self.get_record(program_id, approach_id, 'approach')
            if program['state'] != 'active' or approach['details']['state'] not in ('live', 'promising'):
                raise ProgramError('Assignments require an active program and live/promising approach')
            return self._insert(program_id, 'assignment', objective,
                                {'approach_id': approach_id, 'question_id': approach['details']['question_id'],
                                 'allocation': allocation, 'execution_state': 'planned',
                                 'review_state': 'unreviewed', 'knowledge_state': 'unassessed'}, record_id)

    def add_artifact(self, program_id, reference, sha256, evidence_type, source_revision, *,
                     provenance=None, record_id=None, expected_revision=None):
        if not isinstance(sha256, str) or not re.fullmatch('[0-9a-f]{64}', sha256):
            raise ProgramError('Artifacts require a lowercase SHA-256 hash')
        if evidence_type not in ('simulated', 'synthetic', 'analytic', 'hardware', 'imported'):
            raise ProgramError('Unknown evidence type')
        details = {'reference': _text(reference, 'reference'), 'sha256': sha256,
                   'evidence_type': evidence_type, 'source_revision': _text(source_revision, 'source revision'),
                   'provenance': _object({} if provenance is None else provenance, 'provenance')}
        from .program_graph import validate_artifact
        validate_artifact(details)
        with self.transaction():
            self._touch(program_id, expected_revision)
            return self._insert(program_id, 'artifact', reference, details, record_id)

    def _note(self, program_id, kind, text, subject_id, evidence_ids, actor, record_id=None):
        if subject_id is not None and subject_id != program_id:
            self.get_record(program_id, subject_id)
        if not isinstance(evidence_ids, list):
            raise ProgramError('evidence_ids must be a list')
        for evidence_id in evidence_ids:
            self.get_record(program_id, evidence_id, 'artifact')
        return self._insert(program_id, kind, text,
                            {'subject_id': subject_id, 'evidence_ids': evidence_ids, 'actor': _text(actor, 'actor')}, record_id)

    def record_note(self, program_id, kind, text, *, subject_id=None, evidence_ids=None,
                    actor='manager', record_id=None, expected_revision=None):
        if kind not in ('decision', 'steering'):
            raise ProgramError('Notes must be decisions or steering')
        with self.transaction():
            self._touch(program_id, expected_revision)
            return self._note(program_id, kind, text, subject_id,
                              [] if evidence_ids is None else evidence_ids, actor, record_id)

    def transition(self, program_id, record_id, target, reason, *, expected_revision=None):
        _text(reason, 'reason')
        with self.transaction():
            program = self._touch(program_id, expected_revision)
            record = program if record_id == program_id else self.get_record(program_id, record_id)
            kind = 'program' if record_id == program_id else record['kind']
            if kind not in TRANSITIONS:
                raise ProgramError(f'{kind} records have no mutable execution/route state')
            details = record if kind == 'program' else record['details']
            key = 'execution_state' if kind == 'assignment' else 'state'
            current = details[key]
            if target not in TRANSITIONS[kind].get(current, set()):
                raise ProgramError(f'Invalid {kind} transition: {current} -> {target}')
            if kind == 'program':
                self.connection.execute('UPDATE programs SET state=? WHERE id=?', (target, program_id))
            else:
                if kind == 'assignment' and target in ('ready', 'running'):
                    _allocation(details['allocation'])
                    approach = self.get_record(program_id, details['approach_id'], 'approach')
                    if program['state'] != 'active' or approach['details']['state'] not in ('live', 'promising'):
                        raise ProgramError('Ready/running assignments require an active program and approach')
                details[key] = target
                self.connection.execute('UPDATE records SET details_json=?, revision=revision+1, updated_at=? WHERE id=?',
                                        (_json(details), _now(), record_id))
            self._note(program_id, 'decision', reason, record_id, [], 'manager')
        return self.get_program(program_id) if kind == 'program' else self.get_record(program_id, record_id)

    def snapshot(self, program_id):
        # A single read transaction avoids mixing revisions during concurrent updates.
        owns_transaction = not self.connection.in_transaction
        if owns_transaction:
            self.connection.execute('BEGIN')
        try:
            result = {'format_version': SCHEMA_VERSION, 'program': self.get_program(program_id),
                      'records': [self._decode(row) for row in self.connection.execute(
                          'SELECT * FROM records WHERE program_id=? ORDER BY rowid', (program_id,))]}
            if owns_transaction:
                self.connection.execute('COMMIT')
            return result
        except BaseException:
            if owns_transaction:
                self.connection.execute('ROLLBACK')
            raise

    def overview(self, program_id):
        result = self.snapshot(program_id)
        admissions = [r for r in result['records'] if r['kind'] == 'admission']
        result['scientific_result'] = ('Reviewed findings are recorded with exact submission scope; inspect graph review history.'
                                      if any(r['details']['knowledge_state'] == 'reviewed' for r in admissions) else
                                      'No scientific result is available yet; evidence has not been admitted by review.')
        result['live_alternatives'] = [r['id'] for r in result['records'] if r['kind'] == 'approach'
                                       and r['details']['state'] in ('live', 'promising')]
        result['assignments'] = [r['id'] for r in result['records'] if r['kind'] == 'assignment']
        return result

    def import_snapshot(self, value):
        """Restore one versioned snapshot atomically; never merge/overwrite IDs."""
        if (not isinstance(value, dict) or type(value.get('format_version')) is not int or
                value['format_version'] != SCHEMA_VERSION):
            raise ProgramError(f'Expected program export format_version {SCHEMA_VERSION}')
        program, records = value.get('program'), value.get('records')
        _object(program, 'program')
        if not isinstance(records, list):
            raise ProgramError('records must be a list')
        identity = _identity('program', _text(program.get('id'), 'program ID'))
        required_program = {'id', 'question_id', 'goal', 'constraints', 'state', 'revision', 'created_at', 'updated_at'}
        required_record = {'id', 'program_id', 'kind', 'title', 'details', 'revision', 'created_at', 'updated_at'}
        if set(program) != required_program or program['state'] not in tuple(TRANSITIONS['program']):
            raise ProgramError('Invalid exported program fields/state')
        _text(program['goal'], 'goal')
        _object(program['constraints'], 'constraints')
        by_id = {}
        for record in records:
            if not isinstance(record, dict) or set(record) != required_record:
                raise ProgramError('Invalid exported record fields')
            record_id = _identity('record', _text(record['id'], 'record ID'))
            if record_id in by_id or record_id == identity or record['program_id'] != identity or record['kind'] not in KINDS:
                raise ProgramError('Duplicate identity, foreign program or unknown record kind')
            _text(record['title'], 'title')
            _object(record['details'], 'details')
            by_id[record_id] = record
        for record in [program, *records]:
            if type(record['revision']) is not int or record['revision'] < 1:
                raise ProgramError('Revisions must be positive integers')
            for key in ('created_at', 'updated_at'):
                if not isinstance(record[key], str):
                    raise ProgramError('Timestamps must be ISO strings')
                try:
                    timestamp = datetime.fromisoformat(record[key])
                except ValueError as error:
                    raise ProgramError('Timestamps must be ISO strings') from error
                if timestamp.tzinfo is None:
                    raise ProgramError('Timestamps must include timezone')
        def require(record_id, kind):
            _text(record_id, 'reference ID')
            if record_id not in by_id or by_id[record_id]['kind'] != kind:
                raise ProgramError(f'Missing {kind} reference: {record_id}')
            return by_id[record_id]
        require(program['question_id'], 'question')
        for record in records:
            kind, details = record['kind'], record['details']
            if kind == 'question':
                _object(details.get('scope'), 'question scope')
                if details.get('knowledge_state') != 'unassessed':
                    raise ProgramError('Imported knowledge is unassessed; reviewed admission is not implemented here')
            elif kind == 'approach':
                require(details.get('question_id'), 'question')
                _text(details.get('reason'), 'reason')
                if details.get('state') not in tuple(TRANSITIONS['approach']):
                    raise ProgramError('Invalid approach state')
            elif kind == 'assignment':
                approach = require(details.get('approach_id'), 'approach')
                require(details.get('question_id'), 'question')
                if details['question_id'] != approach['details'].get('question_id'):
                    raise ProgramError('Assignment and approach question differ')
                _allocation(details.get('allocation'))
                if (details.get('execution_state') not in tuple(TRANSITIONS['assignment']) or
                        details.get('review_state') != 'unreviewed' or details.get('knowledge_state') != 'unassessed'):
                    raise ProgramError('Invalid execution/review/knowledge state')
            elif kind == 'artifact':
                if (not re.fullmatch('[0-9a-f]{64}', str(details.get('sha256', ''))) or
                        details.get('evidence_type') not in ('simulated', 'synthetic', 'analytic', 'hardware', 'imported')):
                    raise ProgramError('Invalid artifact hash/type')
                _text(details.get('reference'), 'reference')
                _text(details.get('source_revision'), 'source revision')
                _object(details.get('provenance'), 'provenance')
                from .program_graph import validate_artifact
                validate_artifact(details)
            elif kind in ('decision', 'steering'):
                subject = details.get('subject_id')
                if subject is not None:
                    _text(subject, 'subject ID')
                if subject is not None and subject != identity and subject not in by_id:
                    raise ProgramError('Unknown note subject')
                _text(details.get('actor'), 'actor')
                if not isinstance(details.get('evidence_ids'), list):
                    raise ProgramError('evidence_ids must be a list')
                for evidence_id in details['evidence_ids']:
                    require(evidence_id, 'artifact')
        from .program_graph import validate_graph
        validate_graph(by_id)
        from .program_workers import validate_attempts
        validate_attempts(by_id)
        from .program_review import validate_reviews
        validate_reviews(by_id)
        from .program_strategy import validate_strategy
        validate_strategy(by_id)
        with self.transaction():
            self.connection.execute('INSERT INTO programs VALUES (?, ?, ?, ?, ?, ?, ?, ?)',
                                    (identity, program['question_id'], program['goal'], _json(program['constraints']),
                                     program['state'], program['revision'], program['created_at'], program['updated_at']))
            for record in records:
                self.connection.execute('INSERT INTO records VALUES (?, ?, ?, ?, ?, ?, ?, ?)',
                                        (record['id'], identity, record['kind'], record['title'], _json(record['details']),
                                         record['revision'], record['created_at'], record['updated_at']))
        return self.get_program(identity)


def render_markdown(snapshot):
    """A complete readable export; JSON remains the lossless import format."""
    program = snapshot['program']
    admission_message = ('Reviewed findings are recorded in scoped admission history.'
                         if any(r['kind'] == 'admission' and r['details']['knowledge_state'] == 'reviewed'
                                for r in snapshot['records']) else
                         'No scientific result is available yet; evidence has not been admitted by review.')
    parts = [f'# {program["goal"]}', f'Program: {program["id"]} · revision {program["revision"]} · {program["state"]}',
             admission_message,
             '## Program', '```json\n' + json.dumps(program, indent=2, ensure_ascii=False) + '\n```']
    for record in snapshot['records']:
        parts += [f'## {record["kind"]}: {record["title"]}',
                  '```json\n' + json.dumps(record, indent=2, ensure_ascii=False) + '\n```']
    return '\n\n'.join(parts) + '\n'
