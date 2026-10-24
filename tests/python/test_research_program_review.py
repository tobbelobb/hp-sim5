"""Controller boundary tests with real child processes and saved synthetic evidence.

The fake reviewer supplies fixture judgements, not live-model or numerical proof.
Same-UID callers can bypass this cooperative API; tests make no isolation claim.
"""
import copy
import hashlib
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import sys
import time

import pytest

from hp_sim5_research.program import ProgramError, ProgramStore
from hp_sim5_research.program_review import CONTRACT, digest

ROOT = Path(__file__).resolve().parents[2]
CASES = json.loads((ROOT / 'tests/fixtures/program-review/cases.json').read_text())


@pytest.fixture
def context(tmp_path, monkeypatch):
    root = tmp_path / 'repo'
    root.mkdir()
    (root / 'scripts').symlink_to(ROOT / 'scripts', target_is_directory=True)
    (root / 'research/prompts').mkdir(parents=True)
    (root / 'research/prompts/reviewer.txt').write_bytes((ROOT / 'research/prompts/reviewer.txt').read_bytes())
    subprocess.run(['git', 'init', '-q', str(root)], check=True)
    subprocess.run(['git', '-C', str(root), 'add', '.'], check=True)
    subprocess.run(['git', '-C', str(root), '-c', 'user.name=fixture', '-c', 'user.email=f@f',
                    'commit', '-qm', 'base'], check=True)
    base = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip()
    store = ProgramStore(tmp_path / 'memory', create=True)
    store.create_program('Which scoped result changes our route?', program_id='p', question_id='q')
    store.add_approach('p', 'q', 'Saved evidence', 'No new physics needed', record_id='route')
    store.add_assignment('p', 'route', 'Inspect saved inputs', {'wall_minutes': 1}, record_id='work')
    store.transition('p', 'work', 'ready', 'Fixture dispatch')
    worker = tmp_path / 'worker'
    worker.write_text(f'#!{sys.executable}\nimport sys\n# Success does not self-admit.\nsys.exit(0)\n')
    worker.chmod(0o755)
    workers = store.workers(root, executable=worker)
    attempt = workers.start('p', 'work')
    workers.children[attempt['id']].wait(timeout=5)
    attempt = workers.status('p', attempt['id'])
    controller = store.reviews(root)
    binaries = tmp_path / 'bin'
    binaries.mkdir()
    monkeypatch.setenv('PATH', str(binaries) + os.pathsep + os.environ['PATH'])
    # A separate executable reads the exact saved submission, then emits the
    # fixture judgement. It never calls the store or writes an admission.
    client = '''import json, sys, re, time
from pathlib import Path
prompt = sys.stdin.read()
target = Path(re.search(r'Target directory: (.+)', prompt).group(1))
manifest = json.loads((target / 'manifest.json').read_text())
for asset in manifest['assets']:
    if not asset['missing']:
        raw = (target / asset['id']).read_bytes()
        import hashlib
        assert hashlib.sha256(raw).hexdigest() == asset['sha256']
response = RESPONSE
response.update(submission_id=re.search(r'Submission: (.+)', prompt).group(1),
                submission_sha256=re.search(r'SHA-256: (.+)', prompt).group(1))
response['checks'] = [{'kind': 'inspected', 'description': 'Read selected frozen files; no independent numerical reproduction',
                       'asset_ids': [a['id'] for a in manifest['assets'] if not a['missing']]}]
MODE
'''
    result = dict(root=root, base=base, store=store, controller=controller, attempt=attempt, tmp=tmp_path,
                  binaries=binaries, client=client)
    yield result
    for process in controller.children.values():
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5)
    store.close()


def submit(c, case='scoped_negative', candidate=None, checks=None, claim='claim'):
    fixture = CASES[case]
    store = c['store']
    store.graph().add_claim('p', fixture['claim'], {'evidence_type': 'synthetic', 'configuration': 'fixture'}, record_id=claim)
    selected = []
    for role, data in (('report', fixture['claim']), ('abstract', fixture['review']['reason']),
                       ('evidence', json.dumps(fixture['measurements'])), ('check', 'Synthetic gate fixture only\n')):
        path = c['tmp'] / f'{claim}-{role}.txt'
        if role != 'evidence' or fixture['measurements'] is not None:
            path.write_text(data)
        artifact_id = None
        if role == 'evidence':
            artifact_id = claim + '-evidence'
            sha = digest(path.read_bytes()) if path.exists() else '0' * 64
            store.add_artifact('p', str(path), sha, 'synthetic', c['base'], record_id=artifact_id,
                               provenance={'input_identities': ['frozen-input'], 'config': {'machine': 'fixture'}})
        selected.append(dict(id=role, role=role, path=str(path), **({'artifact_id': artifact_id} if artifact_id else {})))
    return c['controller'].submit('p', c['attempt']['id'], claim, base_revision=c['base'], candidate_revision=candidate,
                                  config={'machine': 'fixture'}, input_identities=['frozen-input'],
                                  selected=selected, producer_checks=checks or [])


def reviewer(c, submission, case='scoped_negative', backend='codex', response=None, mode='normal', wall_minutes=1):
    response = copy.deepcopy(response or CASES[case]['review'])
    if mode == 'hold':
        action = 'time.sleep(60)'
    elif mode == 'nonzero':
        action = 'sys.exit(9)'
    elif mode == 'wrong-target':
        action = "response['submission_sha256']='0'*64\nPath(sys.argv[sys.argv.index('--output-last-message')+1]).write_text(json.dumps(response))"
    elif mode == 'invalid':
        action = "Path(sys.argv[sys.argv.index('--output-last-message')+1]).write_text('SUCCESS')"
    elif backend == 'claude':
        action = "print(json.dumps({'type':'result','result':json.dumps(response),'is_error':False}))"
    else:
        action = "Path(sys.argv[sys.argv.index('--output-last-message')+1]).write_text(json.dumps(response))"
    if mode == 'omit-check':
        action = "response['checks'][0]['asset_ids'].remove('check')\n" + action
    if mode == 'tamper':
        action = "(target / 'evidence').write_text('changed after reviewer inspection')\n" + action
    script = c['binaries'] / backend
    script.write_text(f'#!{sys.executable}\n' + c['client'].replace('RESPONSE', repr(response)).replace('MODE', action))
    script.chmod(0o755)
    run = c['controller'].start('p', submission['id'], backend=backend, wall_minutes=wall_minutes)
    if mode == 'hold':
        time.sleep(.08)
    else:
        c['controller'].children[run['id']].wait(timeout=5)
    return c['controller'].status('p', run['id'])


def state(c, claim='claim'):
    return c['store'].graph().query('p', 'support', claim)['states'][claim]


def test_report_only_negative_closes_work_after_separate_review(context):
    c = context
    assert c['attempt']['details']['returncode'] == 0
    assert c['store'].overview('p')['scientific_result'].startswith('No scientific result')
    submission = submit(c)
    frozen = copy.deepcopy(submission)
    assert state(c)['assessment_state'] == 'pending_review'
    run = reviewer(c, submission)
    assert run['details']['state'] == 'completed'
    assert state(c)['assessment_state'] == 'pending_review'  # reviewer exit is not admission
    review_id = run['details']['review_id']
    review = c['store'].get_record('p', review_id)
    assert review['details']['reviewer']['context_id'] != c['attempt']['id']
    assert review['details']['contract'] == CONTRACT
    assert all(check['kind'] == 'inspected' for check in review['details']['response']['checks'])
    admission = c['controller'].apply('p', review_id)
    assert admission['details']['decision'] == 'keep'
    assert admission['details']['code_readiness'] == 'report_only'
    assert admission['details']['finding'] == 'supported'
    assert state(c)['assessment_state'] == 'reviewed'
    query = c['store'].graph().query('p', 'support', 'claim')
    assert query['states']['claim-evidence']['assessment_state'] == 'reviewed'
    assert c['store'].get_record('p', submission['id']) == frozen
    assert c['store'].get_record('p', 'work')['details']['execution_state'] == 'completed'
    assert 'No scientific result' not in c['store'].overview('p')['scientific_result']
    with pytest.raises(ProgramError, match='already applied'):
        c['controller'].apply('p', review_id)
    snapshot = c['store'].snapshot('p')
    with ProgramStore(c['tmp'] / 'restored', create=True) as restored:
        restored.import_snapshot(snapshot)
        assert restored.graph().query('p', 'support', 'claim')['states']['claim']['assessment_state'] == 'reviewed'
        # Graph memory restores; admission cannot proceed without selected bytes.
        with pytest.raises(ProgramError, match='integrity check failed'):
            restored.reviews(c['root']).verify('p', submission['id'])


@pytest.mark.parametrize('case,decision,finding', [('lower_fit_overclaim', 'revise', 'mixed'),
                                                  ('missing_evidence', 'block', 'not_tested')])
def test_known_overclaim_and_missing_evidence(context, case, decision, finding):
    c = context
    submission = submit(c, case)
    run = reviewer(c, submission, case)
    admission = c['controller'].apply('p', run['details']['review_id'])
    assert admission['details']['decision'] == decision
    assert admission['details']['finding'] == finding
    assert state(c)['assessment_state'] == 'pending_review'
    if case == 'missing_evidence':
        assert any('Missing' in gap for gap in admission['details']['guard_reasons'])
        # Even an overoptimistic fixture reviewer cannot bypass the missing-byte guard.
        favorable = copy.deepcopy(CASES['scoped_negative']['review'])
        run = reviewer(c, submission, response=favorable)
        assert c['controller'].apply('p', run['details']['review_id'])['details']['decision'] == 'block'


@pytest.mark.parametrize('gate', ['absent', 'wrong-candidate', 'uninspected', 'passed'])
def test_actual_candidate_diff_enforces_matching_inspected_physics_gate(context, gate):
    c = context
    path = c['root'] / 'src/js/cable_joints_3d/physics_fixture.js'
    path.parent.mkdir(parents=True)
    path.write_text('// Synthetic patch: no real dynamics are run.\n')
    subprocess.run(['git', '-C', str(c['root']), 'add', '.'], check=True)
    subprocess.run(['git', '-C', str(c['root']), '-c', 'user.name=fixture', '-c', 'user.email=f@f',
                    'commit', '-qm', 'physics candidate fixture'], check=True)
    candidate = subprocess.check_output(['git', '-C', str(c['root']), 'rev-parse', 'HEAD'], text=True).strip()
    checks = [] if gate == 'absent' else [{'command': 'npm run test:physics', 'outcome': 'passed',
                                         'candidate_revision': c['base'] if gate == 'wrong-candidate' else candidate, 'asset_id': 'check'}]
    submission = submit(c, 'physics_patch', candidate, checks)
    manifest = submission['details']['manifest']
    assert manifest['physics_gate_required']
    assert manifest['changed_paths'] == ['src/js/cable_joints_3d/physics_fixture.js']
    assert '+// Synthetic patch' in (c['store'].directory / 'submissions' / submission['id'] / 'candidate.diff').read_text()
    run = reviewer(c, submission, 'physics_patch', mode='omit-check' if gate == 'uninspected' else 'normal')
    admission = c['controller'].apply('p', run['details']['review_id'])
    assert admission['details']['decision'] == ('keep' if gate == 'passed' else 'block')
    assert admission['details']['code_readiness'] == ('ready' if gate == 'passed' else 'blocked')


def test_changed_bytes_cannot_reuse_favorable_review_and_invalidation_is_historical(context):
    c = context
    submission = submit(c)
    run = reviewer(c, submission)
    target = c['store'].directory / 'submissions' / submission['id'] / 'evidence'
    original = target.read_bytes()
    target.write_text('Changed evidence')
    with pytest.raises(ProgramError, match='integrity mismatch'):
        c['controller'].apply('p', run['details']['review_id'])
    target.write_bytes(original)
    c['controller'].apply('p', run['details']['review_id'])
    c['controller'].invalidate('p', submission['id'], 'Corrected measurement requires a new fixed submission')
    assert state(c)['review_history'][0]['state'] == 'invalidated'
    with pytest.raises(ProgramError, match='explicitly invalidated'):
        c['controller'].plan('p', submission['id'])
    newer = c['store'].graph().add_claim('p', 'Narrower corrected scope', {'machine': 'new-fixture'}, prior_id='claim')
    assert state(c, newer['id'])['assessment_state'] == 'unassessed'
    assert not state(c)['reviewed_findings']


def test_new_submission_needs_own_review_and_later_review_supersedes_favorable_history(context):
    c = context
    first = submit(c)
    run = reviewer(c, first)
    c['controller'].apply('p', run['details']['review_id'])
    manifest = first['details']['manifest']
    # A different explicit configuration is a new fixed submission, even with
    # exactly the same saved measurements and claim version.
    selected = [{'id': a['id'], 'role': a['role'], 'path': a['source'],
                 **({'artifact_id': a['artifact_id']} if a['artifact_id'] else {})} for a in manifest['assets']]
    (c['root'] / 'algorithm.py').write_text('# New explicit candidate, not covered by the preceding review\n')
    subprocess.run(['git', '-C', str(c['root']), 'add', 'algorithm.py'], check=True)
    subprocess.run(['git', '-C', str(c['root']), '-c', 'user.name=fixture', '-c', 'user.email=f@f',
                    'commit', '-qm', 'another candidate'], check=True)
    candidate = subprocess.check_output(['git', '-C', str(c['root']), 'rev-parse', 'HEAD'], text=True).strip()
    second = c['controller'].submit('p', c['attempt']['id'], 'claim', base_revision=c['base'],
                                     candidate_revision=candidate,
                                     config={'machine': 'changed-fixture'}, input_identities=['frozen-input'],
                                     selected=selected, producer_checks=[])
    query = c['store'].graph().query('p', 'support', 'claim')
    assert query['states'][first['id']]['admission_state'] == 'reviewed'
    assert query['states'][second['id']]['assessment_state'] == 'pending_review'
    assert query['states'][second['id']]['candidate_revision'] == candidate
    assert second['id'] in query['states']['claim']['pending_submission_ids']
    with pytest.raises(TypeError):
        c['controller'].apply('p', run['details']['review_id'], candidate_revision='another')
    revision = copy.deepcopy(CASES['scoped_negative']['review'])
    revision.update(verdict='revise', reason='New reviewer found a discrepancy', discrepancy='Attribution needs revision')
    later = reviewer(c, first, response=revision)
    c['controller'].apply('p', later['details']['review_id'])
    assert [h['state'] for h in state(c)['review_history']] == ['superseded', 'withheld']
    assert state(c)['reviewed_findings'] == []


def test_corruption_before_reviewer_receipt_and_malformed_restore_are_rejected(context):
    c = context
    submission = submit(c)
    target = c['store'].directory / 'submissions' / submission['id'] / 'manifest.json'
    target.write_text('{}')
    with pytest.raises(ProgramError, match='integrity mismatch'):
        c['controller'].start('p', submission['id'])
    target.write_text(json.dumps(submission['details']['manifest']))
    run = reviewer(c, submission)
    c['controller'].apply('p', run['details']['review_id'])
    snapshot = c['store'].snapshot('p')
    next(r for r in snapshot['records'] if r['kind'] == 'admission')['details']['finding'] = 'rejected'
    with ProgramStore(c['tmp'] / 'malformed', create=True) as restored:
        with pytest.raises(ProgramError, match='scope/finding differs'):
            restored.import_snapshot(snapshot)
        assert restored.connection.execute('SELECT count(*) FROM programs').fetchone()[0] == 0


def test_original_turnover_does_not_change_frozen_review_scope_and_withdrawal_history(context):
    c = context
    submission = submit(c)
    (c['tmp'] / 'claim-evidence.txt').unlink()
    run = reviewer(c, submission)
    admission = c['controller'].apply('p', run['details']['review_id'])
    c['store'].graph().review_event('p', 'claim-evidence', 'withdrawal', 'Measurement attribution corrected')
    query = c['store'].graph().query('p', 'support', 'claim')
    assert query['states']['claim']['review_history'] == [{'admission_id': admission['id'], 'state': 'withdrawn',
                                                         'finding': 'supported', 'decision': 'keep'}]
    assert query['states']['claim']['reviewed_findings'] == []
    assert query['states']['claim']['review_required']
    assert any(r['kind'] == 'review' for r in query['records'])
    assert any(r['kind'] == 'submission' for r in query['records'])


@pytest.mark.parametrize('mode', ['invalid', 'wrong-target', 'nonzero', 'hold', 'tamper'])
def test_failed_or_mismatched_review_never_supplies_admission(context, mode):
    c = context
    submission = submit(c)
    run = reviewer(c, submission, mode=mode, wall_minutes=.0005 if mode == 'hold' else 1)
    assert run['details']['state'] == ('timed_out' if mode == 'hold' else 'failed')
    assert run['details']['review_id'] is None
    assert state(c)['assessment_state'] == 'pending_review'
    assert not [r for r in c['store'].snapshot('p')['records'] if r['kind'] == 'admission']


def test_codex_claude_plans_reuse_auth_no_physics_and_claude_receipt(context, monkeypatch):
    c = context
    submission = submit(c)
    monkeypatch.setenv('OPENAI_API_KEY', 'must-not-be-exposed')
    monkeypatch.setenv('CODEX_API_KEY', 'must-not-be-exposed')
    for backend in ('codex', 'claude'):
        plan = c['controller'].plan('p', submission['id'], backend=backend)
        assert plan['argv'][0] == backend
        assert 'must-not-be-exposed' not in json.dumps(plan)
        assert not (Path(plan['directory'])).exists()  # dry-run does not mutate
        if backend == 'codex':
            assert plan['argv'][plan['argv'].index('--sandbox')+1] == 'read-only'
            assert 'mcp_servers.hp_sim5.enabled=false' in plan['argv']
            assert not any('research_attach.py' in a or 'hp_sim5_mcp.py' in a for a in plan['argv'])
    run = reviewer(c, submission, backend='claude')
    assert run['details']['state'] == 'completed'
    assert c['controller'].apply('p', run['details']['review_id'])['details']['decision'] == 'keep'


def test_api_and_database_boundary_is_cooperative_not_same_uid_isolation(context):
    c = context
    submission = submit(c)
    with pytest.raises(TypeError):
        c['controller'].submit('p', c['attempt']['id'], 'claim', admitted=True)
    run = reviewer(c, submission)
    admission = c['controller'].apply('p', run['details']['review_id'])
    for identity in (submission['id'], run['details']['review_id'], admission['id']):
        with pytest.raises(sqlite3.IntegrityError, match='append-only'):
            c['store'].connection.execute('UPDATE records SET title=? WHERE id=?', ('forged', identity))
    # Same UID can directly write a new DB record: there is deliberately no claim
    # that Python methods, SQLite triggers or read-only reviewer tools prevent it.
    with c['store'].transaction():
        c['store']._insert('p', 'decision', 'Same-UID direct write is possible',
                          {'actor': 'fixture', 'subject_id': None, 'evidence_ids': []})
    assert c['store'].get_record('p', c['attempt']['id'])['details']['scientific_state'] == 'unassessed'


@pytest.mark.parametrize('filename', ['connection.json', 'manager-connection.json', 'control-token.json'])
def test_control_credentials_are_never_selected_for_submission(context, filename):
    c = context
    c['store'].graph().add_claim('p', 'A bounded saved observation', {}, record_id='claim')
    path = c['tmp'] / filename
    path.write_text('{"runtime_token":"fixture-secret-not-for-review"}')
    with pytest.raises(ProgramError, match='environments or credentials'):
        c['controller'].submit('p', c['attempt']['id'], 'claim', base_revision=c['base'],
                               config={}, input_identities=['input'], producer_checks=[],
                               selected=[{'id': 'secret', 'role': 'evidence', 'path': str(path)}])
    assert not [r for r in c['store'].snapshot('p')['records'] if r['kind'] == 'submission']
