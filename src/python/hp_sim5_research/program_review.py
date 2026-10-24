"""Fixed selected submissions, separate coding-client review and controller admission.

This is a cooperative, same-UID boundary, not protected/adversarial evaluation.
Only controller apply() writes admissions. Neither producer fields nor child exit
status do so. Reviewers read frozen evidence; reproduction is a distinct check.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from . import coding_client
from .program import ProgramError, _identity, _json, _now, _object, _text
from .program_graph import identities
from .program_workers import process_identity, TERMINAL

LEGACY_CONTRACT = 'hp-sim5-evidence-review-v1'
CONTRACT = 'hp-sim5-evidence-review-v2'
CONTRACTS = (LEGACY_CONTRACT, CONTRACT)
KINDS = ('submission', 'submission_event', 'review_run', 'review', 'admission')
FINDINGS = ('supported', 'rejected', 'mixed', 'inconclusive', 'not_tested')
VERDICTS = ('keep', 'revise', 'block')
READINESS = ('report_only', 'ready', 'needs_changes', 'blocked')
MAX_BYTES = 32 * 1024 * 1024


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def json_digest(value):
    return digest(_json(value).encode('utf-8'))


def git(root, *args):
    result = subprocess.run(['git', '-C', str(root), *args], capture_output=True, check=False)
    if result.returncode:
        raise ProgramError('Cannot resolve submission git target: ' + result.stderr.decode(errors='replace').strip())
    return result.stdout


def commit(root, revision):
    _text(revision, 'revision')
    return git(root, 'rev-parse', '--verify', '--end-of-options', revision + '^{commit}').decode().strip()


def physics_changed(paths):
    # Conservative gate coverage; actual review may require it for additional paths.
    return any(any(word in path.lower() for word in ('physics', 'motor', 'cable', 'usd_scenes', 'scene_dynamics',
                                                    'headlessworld', 'simulationsystems'))
               for path in paths)


def validate_response(value, submission):
    required = {'submission_id', 'submission_sha256', 'finding', 'verdict', 'reason',
                'practical_value', 'code_readiness', 'checks', 'limitations', 'discrepancy'}
    allowed = required | ({'required_checks'} if submission['details']['manifest']['contract'] == CONTRACT else set())
    if not isinstance(value, dict) or not required <= set(value) or set(value) - allowed:
        raise ProgramError('Review response fields do not match the review contract')
    requirements = value.get('required_checks', [])
    if (not isinstance(requirements, list) or any(not isinstance(argv, list) or not argv or
            not all(isinstance(arg, str) and arg and '\0' not in arg for arg in argv) for argv in requirements)):
        raise ProgramError('Review required_checks must be explicit argv lists')
    if (value['submission_id'] != submission['id'] or
            value['submission_sha256'] != submission['details']['sha256']):
        raise ProgramError('Review target differs from fixed submission')
    if value['finding'] not in FINDINGS or value['verdict'] not in VERDICTS or value['code_readiness'] not in READINESS:
        raise ProgramError('Invalid finding, verdict or code readiness')
    for name in ('reason', 'practical_value'):
        _text(value[name], name)
    if not isinstance(value['limitations'], list) or not all(isinstance(x, str) for x in value['limitations']):
        raise ProgramError('Review limitations must be a string list')
    if value['discrepancy'] is not None:
        _text(value['discrepancy'], 'discrepancy')
    if not isinstance(value['checks'], list) or not value['checks']:
        raise ProgramError('Review must record actual checks')
    asset_ids = {a['id'] for a in submission['details']['manifest']['assets']}
    for check in value['checks']:
        _object(check, 'review check')
        if check.get('kind') not in ('inspected', 'recomputed', 'reproduced', 'test'):
            raise ProgramError('Review check must distinguish inspection from reproduction')
        _text(check.get('description'), 'check description')
        if not set(identities(check.get('asset_ids'), 'check asset_ids', nonempty=True)) <= asset_ids:
            raise ProgramError('Check cites evidence outside fixed submission')
    return value


def validate_reviews(records):
    """Validate historical references on restore; import is trusted controller restore."""
    def require(identity, kind):
        result = records.get(identity)
        if result is None or result['kind'] != kind:
            raise ProgramError(f'Missing {kind} reference in review history')
        return result
    for record in records.values():
        kind, d = record['kind'], record['details']
        if kind == 'submission':
            manifest = _object(d.get('manifest'), 'submission manifest')
            if d.get('sha256') != json_digest(manifest) or manifest.get('contract') not in CONTRACTS:
                raise ProgramError('Submission manifest hash/contract mismatch')
            claim = require(manifest.get('claim', {}).get('id'), 'claim')
            if manifest['claim'] != claim:
                raise ProgramError('Submission must bind the exact immutable claim')
            attempt = require(manifest.get('attempt_id'), 'attempt')
            if (attempt['details']['assignment_id'] != manifest.get('assignment_id') or
                    attempt['details']['source_revision'] != manifest.get('source_revision')):
                raise ProgramError('Submission source differs from producing attempt')
            if attempt['details']['state'] not in TERMINAL:
                raise ProgramError('Submission requires an ended worker attempt')
            for asset in manifest.get('assets', []):
                _text(asset.get('id'), 'asset ID')
                if asset.get('artifact_id'):
                    artifact = require(asset['artifact_id'], 'artifact')
                    if artifact['details']['sha256'] != asset['sha256']:
                        raise ProgramError('Selected evidence differs from recorded artifact hash')
        elif kind == 'submission_event':
            require(d.get('submission_id'), 'submission')
            _text(d.get('reason'), 'invalidation reason')
        elif kind == 'review_run':
            submission = require(d.get('submission_id'), 'submission')
            if (d.get('submission_sha256') != submission['details']['sha256'] or
                    d.get('contract') != submission['details']['manifest']['contract']):
                raise ProgramError('Review run target/contract mismatch')
            if d.get('state') not in ('running', 'completed', 'failed', 'timed_out', 'cancelling', 'cancelled', 'interrupted'):
                raise ProgramError('Invalid review execution state')
            if d.get('reviewer', {}).get('context_id') != record['id']:
                raise ProgramError('Reviewer must identify its separate context')
        elif kind == 'review':
            run = require(d.get('run_id'), 'review_run')
            submission = require(d.get('submission_id'), 'submission')
            if run['details']['state'] != 'completed' or run['details']['submission_id'] != submission['id']:
                raise ProgramError('Review requires its own completed separate review run')
            if d.get('reviewer') != run['details']['reviewer'] or d.get('contract') != run['details']['contract']:
                raise ProgramError('Review identity/contract differs from dispatch')
            validate_response(d.get('response'), submission)
        elif kind == 'admission':
            review = require(d.get('review_id'), 'review')
            if d.get('submission_id') != review['details']['submission_id'] or d.get('decision') not in VERDICTS:
                raise ProgramError('Admission must bind its reviewed submission')
            submission = require(d['submission_id'], 'submission')
            if (d.get('claim_id') != submission['details']['manifest']['claim']['id'] or
                    d.get('submission_sha256') != submission['details']['sha256'] or
                    d.get('finding') != review['details']['response']['finding'] or
                    d.get('knowledge_state') != ('reviewed' if d['decision'] == 'keep' else 'withheld')):
                raise ProgramError('Admission scope/finding differs from its fixed review')
            response = review['details']['response']
            if (d['decision'] not in (response['verdict'], 'block') or
                    not isinstance(d.get('guard_reasons'), list) or
                    d.get('practical_value') != response['practical_value'] or
                    d.get('code_readiness') not in READINESS or
                    (d['guard_reasons'] and (d['decision'] != 'block' or d['code_readiness'] != 'blocked'))):
                raise ProgramError('Admission decision differs from its review/guard result')
            if d.get('actor') != 'controller':
                raise ProgramError('Only controller admissions are recorded')
            if submission['details']['manifest']['contract'] == CONTRACT:
                if (d.get('contract') != CONTRACT or not isinstance(d.get('engineering_reasons'), list) or
                        d.get('required_checks') != response.get('required_checks', []) or
                        (d['engineering_reasons'] and d['code_readiness'] != 'blocked')):
                    raise ProgramError('Version-2 admission must separate engineering readiness')


class ReviewController:
    def __init__(self, store, root):
        self.store, self.root = store, Path(root).resolve()
        self.owner = {'id': _identity('review-controller'), 'process': process_identity(os.getpid())}
        self.children = {}

    def _insert(self, program_id, kind, title, details, identity=None):
        records = self.store.graph()._records(program_id)
        identity = _identity(kind, identity)
        records[identity] = {'id': identity, 'kind': kind, 'details': details}
        validate_reviews(records)
        return self.store._insert(program_id, kind, title, details, identity)

    def submit(self, program_id, attempt_id, claim_id, *, base_revision, candidate_revision=None,
               config, input_identities, selected, producer_checks, record_id=None):
        """Controller captures explicit selected files; no producer admission fields.

        selected entries: id, role (report/abstract/evidence/check), path and optional
        artifact_id. Missing files are retained as gaps so review can block usefully.
        Candidate code must be committed; the actual base/candidate diff is captured.
        """
        attempt = self.store.get_record(program_id, attempt_id, 'attempt')
        if attempt['details']['state'] not in TERMINAL:
            raise ProgramError('Submission requires an ended worker attempt')
        claim = self.store.get_record(program_id, claim_id, 'claim')
        _object(config, 'configuration')
        identities(input_identities, 'input identities', nonempty=True)
        if not isinstance(selected, list) or not 1 <= len(selected) <= 32:
            raise ProgramError('Select 1–32 explicit files, not an environment or recording tree')
        if not isinstance(producer_checks, list):
            raise ProgramError('producer_checks must be a list')
        source = commit(self.root, attempt['details']['source_revision'])
        base = commit(self.root, base_revision)
        candidate = commit(self.root, candidate_revision) if candidate_revision else None
        diff = git(self.root, 'diff', '--binary', base, candidate, '--') if candidate else b''
        paths = git(self.root, 'diff', '--name-only', '-z', base, candidate, '--').decode().split('\0')[:-1] if candidate else []
        identity = _identity('submission', record_id)
        directory = self.store.directory / 'submissions' / identity
        assets, contents, total = [], {}, len(diff)
        for item in selected:
            if not isinstance(item, dict) or set(item) - {'id', 'role', 'path', 'artifact_id'}:
                raise ProgramError('Unknown selected-file fields')
            asset_id = _identity('asset', item.get('id'))
            if asset_id in ('manifest.json', 'candidate.diff'):
                raise ProgramError('Selected asset ID conflicts with submission metadata')
            if asset_id in contents or any(a['id'] == asset_id for a in assets):
                raise ProgramError('Duplicate selected asset ID')
            if item.get('role') not in ('report', 'abstract', 'evidence', 'check'):
                raise ProgramError('Invalid selected evidence role')
            path = Path(_text(item.get('path'), 'selected path')).resolve()
            if path.name in ('.env', 'auth.json', 'credentials', 'credentials.json', 'connection.json',
                             'manager-connection.json', 'control-token.json') or '.git' in path.parts:
                raise ProgramError('Do not snapshot environments or credentials')
            artifact = self.store.get_record(program_id, item['artifact_id'], 'artifact') if item.get('artifact_id') else None
            try:
                with path.open('rb') as stream:
                    raw = stream.read(MAX_BYTES + 1)
            except FileNotFoundError:
                raw = None
            total += len(raw) if raw is not None else 0
            if total > MAX_BYTES:
                raise ProgramError('Selected submission exceeds 32 MiB; choose smaller explicit evidence')
            sha = digest(raw) if raw is not None else artifact['details']['sha256'] if artifact else None
            if artifact and sha != artifact['details']['sha256']:
                raise ProgramError('Selected evidence differs from recorded artifact hash')
            assets.append({'id': asset_id, 'role': item['role'], 'source': str(path),
                           'artifact_id': artifact['id'] if artifact else None,
                           'sha256': sha, 'missing': raw is None})
            if raw is not None:
                contents[asset_id] = raw
        ids = {a['id'] for a in assets}
        for check in producer_checks:
            if (not isinstance(check, dict) or set(check) != {'command', 'outcome', 'candidate_revision', 'asset_id'} or
                    check['outcome'] not in ('passed', 'failed', 'not_run') or check['asset_id'] not in ids):
                raise ProgramError('Producer checks require command/outcome/candidate_revision/selected asset_id')
            _text(check['command'], 'producer check command')
        manifest = {'contract': CONTRACT, 'claim': claim, 'attempt_id': attempt_id,
                    'assignment_id': attempt['details']['assignment_id'], 'execution_state': attempt['details']['state'],
                    'source_revision': source, 'source_dirty': attempt['details']['source_dirty'],
                    'base_revision': base, 'candidate_revision': candidate,
                    'diff_sha256': digest(diff), 'changed_paths': paths,
                    'physics_gate_required': physics_changed(paths), 'config': config,
                    'input_identities': input_identities, 'assets': assets, 'producer_checks': producer_checks}
        # Failed transactions can leave inspectable orphan files, never an admitted record.
        with self.store.transaction():
            self.store._touch(program_id)
            directory.mkdir(mode=0o700, parents=True)
            (directory / 'manifest.json').write_text(_json(manifest))
            (directory / 'candidate.diff').write_bytes(diff)
            for asset_id, raw in contents.items():
                (directory / asset_id).write_bytes(raw)
            return self._insert(program_id, 'submission', claim['title'],
                                {'manifest': manifest, 'sha256': json_digest(manifest)}, identity)

    def verify(self, program_id, submission_id):
        submission = self.store.get_record(program_id, submission_id, 'submission')
        details, gaps = submission['details'], []
        manifest = details['manifest']
        directory = self.store.directory / 'submissions' / submission_id
        try:
            if (json_digest(manifest) != details['sha256'] or
                    json.loads((directory / 'manifest.json').read_text()) != manifest or
                    digest((directory / 'candidate.diff').read_bytes()) != manifest['diff_sha256']):
                raise ProgramError('Submission integrity mismatch')
            if self.store.get_record(program_id, manifest['claim']['id'], 'claim') != manifest['claim']:
                raise ProgramError('Claim version differs from fixed submission')
            if manifest['candidate_revision'] and digest(git(self.root, 'diff', '--binary', manifest['base_revision'],
                                                            manifest['candidate_revision'], '--')) != manifest['diff_sha256']:
                raise ProgramError('Candidate version differs from fixed submission')
            for asset in manifest['assets']:
                if asset['missing']:
                    gaps.append('Missing selected evidence: ' + asset['id'])
                elif digest((directory / asset['id']).read_bytes()) != asset['sha256']:
                    raise ProgramError('Submission evidence integrity mismatch: ' + asset['id'])
        except (OSError, ValueError) as error:
            if isinstance(error, ProgramError):
                raise
            raise ProgramError(f'Submission integrity check failed: {error}') from error
        for role in ('report', 'abstract', 'evidence'):
            if not any(a['role'] == role and not a['missing'] for a in manifest['assets']):
                gaps.append('Missing required ' + role)
        events = [r for r in self.store.graph()._records(program_id).values()
                  if r['kind'] == 'submission_event' and r['details']['submission_id'] == submission_id]
        if events:
            raise ProgramError('Submission explicitly invalidated; create a new submission')
        return submission, gaps

    def invalidate(self, program_id, submission_id, reason):
        with self.store.transaction():
            self.store._touch(program_id)
            self.store.get_record(program_id, submission_id, 'submission')
            return self._insert(program_id, 'submission_event', _text(reason, 'reason'),
                                {'submission_id': submission_id, 'reason': reason})

    def plan(self, program_id, submission_id, *, backend='codex', model=None, model_provider='openai', wall_minutes=5):
        """Real supported client argv, without starting it or any physics service."""
        submission, gaps = self.verify(program_id, submission_id)
        contract = submission['details']['manifest']['contract']
        if (isinstance(wall_minutes, bool) or not isinstance(wall_minutes, (int, float)) or
                not math.isfinite(wall_minutes) or wall_minutes <= 0):
            raise ProgramError('Review requires positive finite wall_minutes')
        if backend not in ('codex', 'claude') or (backend == 'claude' and model_provider != 'openai'):
            raise ProgramError('Unsupported reviewer backend/provider')
        identity = _identity('review-run')
        directory = self.store.directory / 'reviews' / identity
        target = self.store.directory / 'submissions' / submission_id
        instructions = (self.root / 'research/prompts/reviewer.txt').read_text()
        response = {'submission_id': submission_id, 'submission_sha256': submission['details']['sha256'],
                    'finding': 'supported|rejected|mixed|inconclusive|not_tested', 'verdict': 'keep|revise|block',
                    'reason': 'scoped reason', 'practical_value': 'decision value',
                    'code_readiness': 'report_only|ready|needs_changes|blocked',
                    'checks': [{'kind': 'inspected', 'description': 'actual evidence inspected', 'asset_ids': ['ID']}],
                    'limitations': [], 'discrepancy': None}
        if contract == CONTRACT:
            response['required_checks'] = []
        prompt = (f'{instructions}\n\n# Fixed separate review assignment\nContract: {contract}\n'
                  f'Target directory: {target}\nSubmission: {submission_id}\n'
                  f'SHA-256: {submission["details"]["sha256"]}\nWall allocation: {wall_minutes} minutes.\n'
                  f'Known gaps: {_json(gaps)}\n'
                  'Read manifest.json, candidate.diff and selected asset files by their IDs. '
                  'Do not edit the submission or program store. Do not start physics or duplicate numerical work '
                  'unless a named gap requires an additional bounded assignment. Reading producer results is inspection. '
                  'For v2, scientific finding/verdict is separate from code_readiness. Missing engineering gates '
                  'block code readiness without erasing an otherwise supported scoped finding. '
                  'Use required_checks argv lists for delivery requirements beyond path detection. '
                  'For a sound candidate awaiting checks, ready plus required_checks permits combined evaluation. '
                  'Use blocked or needs_changes for unresolved code defects that require a new fixed review. '
                  'Return ONLY one JSON object with exactly these fields and valid enum values:\n' + _json(response))
        if backend == 'codex':
            argv = coding_client.codex_command(self.root, directory, batch=True, dont_ask=True, model=model,
                                               model_provider=model_provider, sandbox='read-only',
                                               settings=coding_client.disabled_research_mcp_settings(self.root))
        else:
            argv = coding_client.claude_command(directory, batch=True, dont_ask=True, model=model,
                                                allowed_tools='Read,Glob,Grep,Bash')
        return {'id': identity, 'directory': str(directory), 'argv': argv, 'prompt': prompt,
                'reviewer': {'context_id': identity, 'backend': backend, 'model': model, 'model_provider': model_provider},
                'wall_minutes': wall_minutes, 'submission_id': submission_id,
                'submission_sha256': submission['details']['sha256'], 'contract': contract}

    def start(self, program_id, submission_id, *, runtime_reservation=None, **settings):
        plan = self.plan(program_id, submission_id, **settings)
        directory = Path(plan['directory'])
        directory.mkdir(mode=0o700, parents=True)
        (directory / 'prompt.txt').write_text(plan['prompt'])
        (directory / 'instructions.md').write_text('Review saved evidence under the fixed contract. Return the requested JSON.')
        (directory / 'attachment.json').write_text('{"mcpServers":{}}\n')
        details = {k: v for k, v in plan.items() if k not in ('id', 'prompt')}
        details.update(state='running', owner=self.owner, process=None, started_at=_now(),
                       prompt_sha256=digest(plan['prompt'].encode('utf-8')),
                       deadline=time.time() + plan['wall_minutes'] * 60, returncode=None, error=None, review_id=None)
        if runtime_reservation:
            details['runtime_reservation'] = list(runtime_reservation)
        with self.store.transaction():
            from .program_pool import check_review
            check_review(self.store, reservation=runtime_reservation)
            self.store._touch(program_id)
            self._insert(program_id, 'review_run', 'Separate saved-evidence review', details, plan['id'])
        environment = (coding_client.codex_environment(details['reviewer']['model_provider'], root=self.root,
                                                       env_file=self.root / '.env')
                       if details['reviewer']['backend'] == 'codex' else
                       coding_client.claude_environment(env_file=self.root / '.env'))
        try:
            with (directory / 'prompt.txt').open('rb') as stdin, (directory / 'events.jsonl').open('wb') as stdout, (directory / 'stderr.log').open('wb') as stderr:
                guard = [sys.executable, str(self.root / 'scripts/research_program_review_guard.py'),
                         str(details['deadline']), str(directory / 'wall-expired'), *plan['argv']]
                details['guard_argv'] = guard
                process = subprocess.Popen(guard, cwd=self.root, env=environment, stdin=stdin,
                                           stdout=stdout, stderr=stderr, start_new_session=True)
            self.children[plan['id']] = process
            details['process'] = process_identity(process.pid)
        except OSError as error:
            details.update(state='failed', error=str(error))
        self._save(program_id, plan['id'], details)
        return self.status(program_id, plan['id'])

    def _save(self, program_id, run_id, details):
        with self.store.transaction():
            self.store._touch(program_id)
            self.store.connection.execute('UPDATE records SET details_json=?, revision=revision+1, updated_at=? WHERE id=?',
                                          (_json(details), _now(), run_id))

    def status(self, program_id, run_id):
        record = self.store.get_record(program_id, run_id, 'review_run')
        d = record['details']
        process = self.children.get(run_id)
        if process is None or d['state'] not in ('running', 'cancelling'):
            return record
        code = process.poll()
        if d['state'] == 'cancelling':
            if code is not None:
                d.update(state='cancelled', returncode=code, error='Review cancelled; no assessment admitted')
                self._save(program_id, run_id, d)
            return self.store.get_record(program_id, run_id)
        if code is None and time.time() >= d['deadline']:
            if d['owner'] != self.owner or d['process'] != process_identity(process.pid):
                raise ProgramError('Refusing to stop unverified review process')
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
            d.update(state='timed_out', returncode=process.returncode, error='Review wall allocation expired')
        elif code is None:
            return record
        else:
            d['returncode'] = code
            if (Path(d['directory']) / 'wall-expired').exists():
                d.update(state='timed_out', error='Review wall allocation expired')
                self._save(program_id, run_id, d)
                return self.store.get_record(program_id, run_id)
            try:
                if code:
                    raise ProgramError(f'Reviewer exited {code}; no assessment admitted')
                submission, _ = self.verify(program_id, d['submission_id'])
                directory = Path(d['directory'])
                if d['reviewer']['backend'] == 'claude':
                    events = [json.loads(line) for line in (directory / 'events.jsonl').read_text().splitlines() if line.strip()]
                    final = next(e for e in reversed(events) if e.get('type') == 'result')
                    if final.get('is_error'):
                        raise ProgramError('Claude reported a failed review')
                    (directory / 'final-message.md').write_text(final['result'])
                raw = (directory / 'final-message.md').read_bytes()
                if len(raw) > 1024 * 1024:
                    raise ProgramError('Review response exceeds 1 MiB')
                response = validate_response(json.loads(raw), submission)
                d.update(state='completed', review_id=_identity('review'))
                with self.store.transaction():
                    self.store._touch(program_id)
                    self.store.connection.execute('UPDATE records SET details_json=?, revision=revision+1, updated_at=? WHERE id=?',
                                                  (_json(d), _now(), run_id))
                    self._insert(program_id, 'review', response['reason'],
                                 {'run_id': run_id, 'submission_id': d['submission_id'], 'contract': d['contract'],
                                  'reviewer': d['reviewer'], 'response_sha256': digest(raw), 'response': response}, d['review_id'])
                return self.store.get_record(program_id, run_id)
            except (OSError, ValueError, KeyError, StopIteration) as error:
                d.update(state='failed', error=str(error), review_id=None)
        self._save(program_id, run_id, d)
        return self.store.get_record(program_id, run_id)

    def cancel(self, program_id, run_id):
        """Stop only this controller's birth-verified reviewer; status never admits its output."""
        record = self.store.get_record(program_id, run_id, 'review_run')
        d = record['details']
        if d['state'] not in ('running', 'cancelling'):
            return record
        process = self.children.get(run_id)
        if (process is None or d['owner'] != self.owner or
                (process.poll() is None and d['process'] != process_identity(process.pid))):
            raise ProgramError('Refusing to signal historical or unverified reviewer')
        d['state'] = 'cancelling'
        self._save(program_id, run_id, d)
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        return self.status(program_id, run_id)

    def apply(self, program_id, review_id):
        """Only admission write path; all inputs come from the fixed separate review."""
        with self.store.transaction():
            review = self.store.get_record(program_id, review_id, 'review')
            d = review['details']
            submission, gaps = self.verify(program_id, d['submission_id'])
            records = self.store.graph()._records(program_id)
            validate_reviews(records)
            if any(r['kind'] == 'admission' and r['details']['review_id'] == review_id for r in records.values()):
                raise ProgramError('Review already applied; retain its historical decision')
            manifest, response = submission['details']['manifest'], d['response']
            engineering = []
            assets = {a['id']: a for a in manifest['assets']}
            evidence = {manifest['claim']['id'], *(a['artifact_id'] for a in assets.values() if a['artifact_id'])}
            for event in records.values():
                if event['kind'] == 'review_event' and event['details']['event'] == 'withdrawal':
                    affected, _ = self.store.graph()._affected(records, event['details']['subject_id'])
                    if evidence.intersection({event['details']['subject_id'], *affected}):
                        gaps.append('Selected claim/evidence has been withdrawn: ' + event['id'])
            if manifest['physics_gate_required'] and not any(
                    c['command'] == 'npm run test:physics' and c['outcome'] == 'passed' and
                    c['candidate_revision'] == manifest['candidate_revision'] and
                    assets[c['asset_id']]['role'] == 'check' and not assets[c['asset_id']]['missing'] and
                    any(c['asset_id'] in check['asset_ids'] and check['kind'] == 'inspected' for check in response['checks'])
                    for c in manifest['producer_checks']):
                (gaps if manifest['contract'] == LEGACY_CONTRACT else engineering).append(
                    'Candidate physics/motor/cable/dynamics patch lacks inspected passing npm run test:physics gate')
            if manifest['candidate_revision'] is None and response['code_readiness'] == 'ready':
                (gaps if manifest['contract'] == LEGACY_CONTRACT else engineering).append(
                    'Report-only submission has no code candidate to mark ready')
            if response['discrepancy'] and response['verdict'] == 'keep':
                gaps.append('Unresolved reviewer discrepancy requires revision')
            decision = 'block' if gaps else response['verdict']
            self.store._touch(program_id)
            return self._insert(program_id, 'admission', response['reason'],
                                {'review_id': review_id, 'submission_id': submission['id'],
                                 'claim_id': manifest['claim']['id'], 'submission_sha256': submission['details']['sha256'],
                                 'finding': response['finding'], 'decision': decision,
                                 'knowledge_state': 'reviewed' if decision == 'keep' else 'withheld',
                                 'code_readiness': 'blocked' if gaps or engineering else response['code_readiness'],
                                 'practical_value': response['practical_value'], 'guard_reasons': gaps,
                                 **({'contract': CONTRACT, 'engineering_reasons': engineering,
                                     'required_checks': response.get('required_checks', [])}
                                    if manifest['contract'] == CONTRACT else {}),
                                 'actor': 'controller'})
