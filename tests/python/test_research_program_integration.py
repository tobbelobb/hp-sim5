"""Real Git branches/worktrees and check children; all scientific judgements synthetic.

No mock physics command is counted as an actual physics delivery gate.
"""
import copy
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
from hp_sim5_research.program_integration import IntegrationController
from hp_sim5_research.program_manager import ProgramManager
from hp_sim5_research.program_pool import check_exclusive, check_review, check_worker
from hp_sim5_research.program_review import LEGACY_CONTRACT, digest, json_digest
from hp_sim5_research.program_workers import process_identity
from hp_sim5_research.program_workers import process_live
from hp_sim5_research.program_host import open_manager, request, live
from .test_research_program_review import context, submit, reviewer, state, CASES
from .test_research_program_controller import local_manager


def git(c, *args):
    return subprocess.check_output(['git', '-C', str(c['root']), *args], stderr=subprocess.PIPE).decode().strip()


def candidate(c, name, files):
    git(c, 'checkout', '-qb', name, c['base'])
    for path, content in files.items():
        target = c['root'] / path
        target.parent.mkdir(parents=True, exist_ok=True)
        if isinstance(content, bytes):
            target.write_bytes(content)
        else:
            target.write_text(content)
    git(c, 'add', '.')
    git(c, '-c', 'user.name=fixture', '-c', 'user.email=f@f', 'commit', '-qm', name)
    result = git(c, 'rev-parse', 'HEAD')
    git(c, 'checkout', '-q', 'master')
    return result


def reviewed(c, name, files=None, *, revision=None, review_fields=None):
    revision = candidate(c, name, files) if files is not None else revision
    submission = submit(c, candidate=revision, claim=name)
    response = copy.deepcopy(CASES['scoped_negative']['review'])
    response['code_readiness'] = 'ready' if revision else 'report_only'
    response.update(review_fields or {})
    run = reviewer(c, submission, response=response)
    admission = c['controller'].apply('p', run['details']['review_id'])
    assert admission['details']['decision'] == 'keep'
    return submission


@pytest.fixture
def integration(context):
    c = context
    # Retain the shared legacy fixture's unknown-cleanup semantics. Only these
    # integration cases explicitly hand off that synthetic no-world attempt.
    local_manager(c)
    c['integration'] = IntegrationController(c['store'], c['root'], c['controller'])
    yield c
    for process in c['integration'].children.values():
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5)


def prepare(c, submissions, **kwargs):
    return c['integration'].prepare('p', [s['id'] for s in submissions], target_base=kwargs.pop('target_base', c['base']),
                                    config={'mode': 'combined-fixture'}, input_identities=['frozen-input'], **kwargs)


def check(c, prepared, code, *, wall_minutes=1):
    return c['integration'].check('p', prepared['job']['id'],
                                  checks=[{'name': 'meaningful fixture', 'argv': [sys.executable, '-c', code]}],
                                  config={'mode': 'combined-fixture'}, input_identities=['frozen-input'], wall_minutes=wall_minutes)


def ended(c, started):
    c['integration'].children[started['job']['id']].wait(timeout=5)
    return until(lambda: (r if (r := c['integration'].status('p', started['job']['id']))['job']['details']['state']
                          != 'checking' else None))


def until(fn):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        result = fn()
        if result:
            return result
        time.sleep(.01)
    pytest.fail('Bounded process observation timed out')


def test_compatible_binary_candidates_exact_combined_checks_and_human_work_preserved(integration):
    c = integration
    first = reviewed(c, 'a', {'a.txt': 'first\n'})
    second = reviewed(c, 'b', {'b.bin': b'\x00\x01\x02'})
    # Mutable producer branches are irrelevant after sealing.
    git(c, 'branch', '-f', 'a', c['base'])
    (c['root'] / 'human-untracked').write_text('keep me')
    (c['root'] / 'human-staged').write_text('staged work')
    git(c, 'add', 'human-staged')
    before = (git(c, 'rev-parse', 'HEAD'), git(c, 'diff', '--cached'), git(c, 'status', '--porcelain'))
    prepared = prepare(c, [first, second])
    m = prepared['preparation']['details']['manifest']
    assert m['outcome'] == 'prepared'
    assert prepared['lease_held']
    assert m['target_base'] == c['base']
    assert m['combined_revision'] not in [s['details']['manifest']['candidate_revision'] for s in (first, second)]
    wt = Path(m['worktree'])
    assert not (wt / 'output').is_symlink()
    assert not (wt / 'RRF/build').is_symlink()
    code = "from pathlib import Path; assert Path('a.txt').read_text() == 'first\\n'; assert Path('b.bin').read_bytes() == bytes([0,1,2])"
    result = ended(c, check(c, prepared, code))
    receipt = result['check']['details']['manifest']
    assert receipt['outcome'] == 'passed'
    assert receipt['pre_identity'] == receipt['post_identity']
    assert receipt['pre_identity']['revision'] == m['combined_revision']
    assert receipt['config_sha256'] == json_digest({'mode': 'combined-fixture'})
    assert receipt['input_identities'] == ['frozen-input']
    assert receipt['plan']['evaluator']['runner_sha256'] == digest(
        (c['root'] / 'scripts/research_program_check_runner.py').read_bytes())
    assert 'owned Linux process group' in receipt['plan']['cleanup_boundary']
    assert 'supplied declarations' in receipt['plan']['configuration_boundary']
    assert receipt['execution'][0]['returncode'] == 0
    assert not result['lease_held']
    assert before == (git(c, 'rev-parse', 'HEAD'), git(c, 'diff', '--cached'), git(c, 'status', '--porcelain'))
    assert (c['root'] / 'human-untracked').read_text() == 'keep me'
    query = c['store'].graph().query('p', 'support', 'a')
    assert result['check']['id'] in query['states'][first['id']]['integration_ids']
    assert query['states']['a']['assessment_state'] == 'reviewed'
    for r in (prepared['preparation'], result['check']):
        with pytest.raises(sqlite3.IntegrityError, match='append-only'):
            c['store'].connection.execute('UPDATE records SET title=? WHERE id=?', ('mutated', r['id']))
    snapshot = c['store'].snapshot('p')
    with ProgramStore(c['tmp'] / 'restored-integration', create=True) as restored:
        restored.import_snapshot(snapshot)
        assert restored.get_record('p', result['check']['id']) == result['check']
    broken = copy.deepcopy(snapshot)
    next(r for r in broken['records'] if r['id'] == result['check']['id'])['details']['manifest']['combined_tree'] = 'forged'
    with ProgramStore(c['tmp'] / 'broken-integration', create=True) as restored:
        with pytest.raises(ProgramError, match='hash/contract'):
            restored.import_snapshot(broken)


@pytest.mark.parametrize('scenario', ['conflict', 'stale', 'mutated', 'unrelated'])
def test_rejected_preparations_preserve_accepted_findings(integration, scenario):
    c = integration
    first = reviewed(c, 'a', {'same.txt': 'first\n'})
    second = reviewed(c, 'b', {'same.txt': 'second\n'} if scenario == 'conflict' else {'other.txt': 'second\n'})
    target = c['base']
    if scenario == 'stale':
        target = first['details']['manifest']['candidate_revision']
    elif scenario == 'mutated':
        (c['store'].directory / 'submissions' / first['id'] / 'candidate.diff').write_text('changed after review')
    elif scenario == 'unrelated':
        # A sealed base must actually be an ancestor; tree similarity is insufficient.
        tree = git(c, 'rev-parse', second['details']['manifest']['candidate_revision'] + '^{tree}')
        env = dict(os.environ, GIT_AUTHOR_NAME='fixture', GIT_AUTHOR_EMAIL='f@f',
                   GIT_COMMITTER_NAME='fixture', GIT_COMMITTER_EMAIL='f@f')
        other = subprocess.check_output(['git', '-C', str(c['root']), 'commit-tree', tree, '-m', 'unrelated'], env=env).decode().strip()
        second = reviewed(c, 'unrelated', revision=other)
    result = prepare(c, [first, second], target_base=target)
    assert result['preparation']['details']['manifest']['outcome'] == {
        'conflict': 'conflict', 'stale': 'stale_base', 'mutated': 'changed_or_unreviewed', 'unrelated': 'stale_base'}[scenario]
    assert not result['lease_held']
    assert state(c, 'a')['assessment_state'] == 'reviewed'
    assert state(c, 'b')['assessment_state'] == 'reviewed'
    assert c['store'].get_record('p', first['id']) == first


def test_individual_passing_candidates_combination_fails_without_retracting_science(integration):
    c = integration
    a = reviewed(c, 'a', {'a.txt': 'route a'})
    b = reviewed(c, 'b', {'b.txt': 'route b'})
    code = "from pathlib import Path; assert not (Path('a.txt').exists() and Path('b.txt').exists()), 'incompatible routes'"
    for sub in (a, b):
        assert ended(c, check(c, prepare(c, [sub]), code))['patch_readiness'] == 'passed'
    combined = prepare(c, [a, b])
    result = ended(c, check(c, combined, code))
    assert result['patch_readiness'] == 'check_failed'
    m = result['check']['details']['manifest']
    assert m['combined_revision'] == combined['preparation']['details']['manifest']['combined_revision']
    assert 'incompatible routes' in Path(m['logs'][0]['path']).read_text()
    assert all(state(c, identity)['assessment_state'] == 'reviewed' for identity in ('a', 'b'))


@pytest.mark.parametrize('when', ['before', 'during', 'receipt', 'source'])
def test_changed_candidates_cannot_reuse_checks(integration, when):
    c = integration
    a = reviewed(c, 'a', {'a.txt': 'route a'})
    prepared = prepare(c, [a])
    if when == 'before':
        Path(prepared['job']['details']['worktree'], 'a.txt').write_text('changed')
    elif when == 'receipt':
        Path(prepared['preparation']['details']['path']).write_text('{}')
        with pytest.raises(ProgramError, match='receipt integrity'):
            check(c, prepared, 'pass')
        assert c['integration'].status('p', prepared['job']['id'])['lease_held']
        c['integration'].release('p', prepared['job']['id'], 'Discard corrupted fixture receipt')
        return
    elif when == 'source':
        (c['store'].directory / 'submissions' / a['id'] / 'evidence').write_text('mutated reviewed bytes')
    started = check(c, prepared, "from pathlib import Path; Path('a.txt').write_text('changed')" if when == 'during' else 'pass')
    result = ended(c, started) if when == 'during' else started
    assert result['patch_readiness'] == 'changed_candidate'
    assert state(c, 'a')['assessment_state'] == 'reviewed'


def test_required_physics_gate_is_missing_even_when_reviewed_finding_is_accepted(integration):
    c = integration
    sub = reviewed(c, 'physics-claim', {'src/physics/synthetic.txt': 'fixture only, no production physics edit'})
    admission = [r for r in c['store'].snapshot('p')['records'] if r['kind'] == 'admission'][0]
    assert admission['details']['knowledge_state'] == 'reviewed'
    assert admission['details']['code_readiness'] == 'blocked'
    prepared = prepare(c, [sub])
    assert prepared['preparation']['details']['manifest']['physics_gate_required']
    result = check(c, prepared, 'pass')
    assert result['patch_readiness'] == 'missing_physics_gate'
    assert not c['integration'].children
    assert state(c, 'physics-claim')['assessment_state'] == 'reviewed'


def test_no_patch_negative_needs_no_integration(integration):
    c = integration
    result = prepare(c, [reviewed(c, 'negative')])
    assert result['preparation']['details']['manifest']['outcome'] == 'report_only'
    assert not result['lease_held']
    assert state(c, 'negative')['assessment_state'] == 'reviewed'


def test_durable_exclusion_in_startup_prepared_running_and_recovery(integration, monkeypatch):
    c = integration
    sub = reviewed(c, 'a', {'a.txt': 'a'})
    original = c['integration']._sources
    def blocked(*args):
        with ProgramStore(c['store'].directory) as other:
            for fn in (lambda: check_worker(other, 'p', 'x'), lambda: check_review(other), lambda: check_exclusive(other)):
                with pytest.raises(ProgramError, match='integration/evaluation lease'):
                    with other.transaction():
                        fn()
        return original(*args)
    monkeypatch.setattr(c['integration'], '_sources', blocked)
    prepared = prepare(c, [sub])
    monkeypatch.setattr(c['integration'], '_sources', original)
    for fn in (lambda: check_worker(c['store'], 'p', 'x'), lambda: check_review(c['store'])):
        with pytest.raises(ProgramError, match='integration/evaluation lease'):
            fn()
    started = check(c, prepared, 'import time; time.sleep(30)')
    later = IntegrationController(c['store'], c['root'], c['controller'])
    result = later.recover('p', started['job']['id'])
    assert result['job']['details']['state'] == 'ownership_lost'
    with pytest.raises(ProgramError, match='remains live'):
        later.recover('p', started['job']['id'], reason='Inspection cannot release a live child')
    with pytest.raises(ProgramError, match='integration/evaluation lease'):
        check_review(c['store'])
    os.killpg(c['integration'].children[started['job']['id']].pid, signal.SIGKILL)
    c['integration'].children[started['job']['id']].wait(timeout=5)
    result = later.recover('p', started['job']['id'], reason='Child exit observed; discard unowned checks')
    assert not result['lease_held']
    assert result['job']['details']['release']['checks_admitted'] is False
    assert result['check'] is None


def test_manager_polled_check_stop_and_wall_deadline(integration):
    c = integration
    sub = reviewed(c, 'a', {'a.txt': 'a'})
    manager = ProgramManager(c['store'], c['root'], 'p', 'manager', c['tmp'] / 'manager-session', reviews=c['controller'])
    c['integration'] = manager.integrations
    prepared = manager.call('integration_prepare', {'submission_ids': [sub['id']], 'target_base': c['base'],
                                                   'config': {'mode': 'combined-fixture'}, 'input_identities': ['frozen-input']})
    assert prepared['ok'], prepared
    before = time.monotonic()
    started = check(c, prepared['result'], 'import time; time.sleep(30)')
    assert time.monotonic() - before < 2
    assert not manager.drained()
    stopped = manager.call('stop', {'reason': 'Bounded fixture stop'})
    assert stopped['ok'], stopped
    c['integration'].children[started['job']['id']].wait(timeout=5)
    manager.poll()
    assert manager.drained()
    result = manager.call('integration_status', {'integration_id': started['job']['id']})
    assert result['ok'] and result['result']['patch_readiness'] == 'cancelled'
    assert not result['result']['lease_held']
    # A separate active program is unnecessary; resume only applies to paused programs.
    c['store'].create_program('Finite wall checks', program_id='wall', question_id='wall-q')


def test_check_runner_deadline_and_configuration_binding(integration):
    c = integration
    prepared = prepare(c, [reviewed(c, 'a', {'a.txt': 'a'})])
    with pytest.raises(ProgramError, match='configuration/input'):
        c['integration'].check('p', prepared['job']['id'], checks=[{'name': 'x', 'argv': [sys.executable, '-c', 'pass']}],
                               config={}, input_identities=['different'])
    result = ended(c, check(c, prepared, 'import time; time.sleep(30)', wall_minutes=.003))
    assert result['patch_readiness'] == 'timed_out'
    assert not result['lease_held']


@pytest.mark.parametrize('mode', ['physics', 'required', 'blocked'])
def test_explicit_reviewer_engineering_requirements_survive_scientific_admission(integration, mode):
    c = integration
    fields = ({'required_checks': [['npm', 'run', 'test:physics']]} if mode == 'physics' else
              {'required_checks': [[sys.executable, '-c', 'assert 2 + 2 == 4']]} if mode == 'required' else
              {'code_readiness': 'needs_changes', 'reason': 'Code defect requires correction'})
    sub = reviewed(c, 'a', {'tooling/algorithm.txt': 'candidate outside physics path heuristics'}, review_fields=fields)
    assert not sub['details']['manifest']['physics_gate_required']
    result = check(c, prepare(c, [sub]), 'pass')
    assert result['patch_readiness'] == {'physics': 'missing_physics_gate', 'required': 'missing_required_check',
                                         'blocked': 'engineering_blocked'}[mode]
    assert not c['integration'].children
    assert state(c, 'a')['assessment_state'] == 'reviewed'


def test_explicit_required_nonphysics_check_can_pass_on_combined_revision(integration):
    c = integration
    argv = [sys.executable, '-c', "from pathlib import Path; assert Path('a.txt').read_text() == 'a'"]
    prepared = prepare(c, [reviewed(c, 'a', {'a.txt': 'a'}, review_fields={'required_checks': [argv]})])
    started = c['integration'].check('p', prepared['job']['id'], checks=[{'name': 'reviewer required', 'argv': argv}],
                                     config={'mode': 'combined-fixture'}, input_identities=['frozen-input'])
    assert ended(c, started)['patch_readiness'] == 'passed'


def test_check_imports_candidate_source_from_combined_worktree(integration):
    c = integration
    sub = reviewed(c, 'a', {'src/python/task015_candidate.py': 'VALUE = "combined source"\n'})
    prepared = prepare(c, [sub])
    wt = prepared['job']['details']['worktree']
    code = ('import task015_candidate; '
            f'assert task015_candidate.__file__.startswith({wt!r}); '
            'assert task015_candidate.VALUE == "combined source"')
    assert ended(c, check(c, prepared, code))['patch_readiness'] == 'passed'


def test_runner_loss_cannot_release_lease_while_descendant_is_live(integration):
    c = integration
    prepared = prepare(c, [reviewed(c, 'a', {'a.txt': 'a'})])
    marker = c['tmp'] / 'descendant.json'
    code = ('import json,os,time; from pathlib import Path; '
            f'Path({str(marker)!r}).write_text(json.dumps({{"pid":os.getpid()}})); time.sleep(30)')
    started = check(c, prepared, code)
    until(marker.exists)
    child = process_identity(json.loads(marker.read_text())['pid'])
    runner = c['integration'].children[started['job']['id']]
    os.kill(runner.pid, signal.SIGKILL)
    runner.wait(timeout=5)
    assert process_live(child)
    result = c['integration'].status('p', started['job']['id'])
    assert result['lease_held']  # This poll requests group cleanup, then waits for observation.
    with pytest.raises(ProgramError, match='integration/evaluation lease'):
        check_review(c['store'])
    result = ended(c, started)
    assert not process_live(child)
    assert result['patch_readiness'] == 'check_failed'
    assert not result['lease_held']


def test_runner_handshake_closes_popen_receipt_gap_and_unknown_startup_stays_charged(integration):
    c = integration
    prepared = prepare(c, [reviewed(c, 'a', {'a.txt': 'a'})])
    started = check(c, prepared, 'import time; time.sleep(30)')
    identity_file = Path(started['job']['details']['directory']) / 'runner-identity.json'
    until(identity_file.exists)
    d = c['store'].get_record('p', started['job']['id'])['details']
    d['process'] = None  # Simulate owner death after Popen, before saving its birth receipt.
    c['integration']._save('p', started['job']['id'], d)
    later = IntegrationController(c['store'], c['root'], c['controller'])
    with pytest.raises(ProgramError, match='group remains live'):
        later.recover('p', started['job']['id'], reason='Cannot release startup child')
    runner = c['integration'].children[started['job']['id']]
    os.killpg(runner.pid, signal.SIGKILL)
    runner.wait(timeout=5)
    result = later.recover('p', started['job']['id'], reason='Observed whole group shutdown')
    assert not result['lease_held']
    assert result['check'] is None
    # A genuinely unknown startup has no identity receipt to establish cleanup.
    prepared = prepare(c, [c['store'].get_record('p', prepared['preparation']['details']['manifest']['submission_ids'][0])])
    d = prepared['job']['details']
    d.update(state='checking', check_plan={'deadline': time.time() - 10}, process=None)
    c['integration']._save('p', prepared['job']['id'], d)
    with pytest.raises(ProgramError, match='identity unknown'):
        later.recover('p', prepared['job']['id'], reason='A deadline alone cannot establish unknown cleanup')
    assert later.status('p', prepared['job']['id'])['lease_held']


def test_actual_file_host_returns_before_long_check_and_keeps_ownership(integration, monkeypatch):
    c = integration
    sub = reviewed(c, 'a', {'a.txt': 'fixture'})
    monkeypatch.delenv('PYTHONPATH', raising=False)
    opened = open_manager(c['store'].directory, c['root'], 'p')
    assert opened['ok'], opened
    attachment = opened['result']['attachment']
    try:
        prepared = request(attachment, 'integration_prepare', {'submission_ids': [sub['id']], 'target_base': c['base'],
                           'config': {'mode': 'combined-fixture'}, 'input_identities': ['frozen-input']})
        assert prepared['ok'], prepared
        started_at = time.monotonic()
        started = request(attachment, 'integration_check', {'integration_id': prepared['result']['job']['id'],
                          'checks': [{'name': 'fresh process', 'argv': [sys.executable, '-c',
                                                                      'import time; time.sleep(.3); assert 2 + 2 == 4']}],
                          'config': {'mode': 'combined-fixture'}, 'input_identities': ['frozen-input'], 'wall_minutes': 1})
        assert started['ok'], started
        assert time.monotonic() - started_at < 2
        def finished():
            result = request(attachment, 'integration_status', {'integration_id': prepared['result']['job']['id']})
            assert result['ok'], result
            return result if result['result']['patch_readiness'] == 'passed' else None
        result = until(finished)
        assert not result['result']['lease_held']
        assert request(attachment, 'overview')['result']['integrations'][0]['patch_readiness'] == 'passed'
    finally:
        request(attachment, 'shutdown')
        until(lambda: not live(attachment))
        try:
            os.waitpid(attachment['process']['pid'], os.WNOHANG)
        except ChildProcessError:
            pass


def test_detected_detached_child_holds_lease_without_claiming_shutdown(integration):
    c = integration
    prepared = prepare(c, [reviewed(c, 'a', {'a.txt': 'a'})])
    marker = c['tmp'] / 'detached-pid.json'
    code = ('import subprocess,sys,json; from pathlib import Path; '
            'child=subprocess.Popen([sys.executable,"-c","import time; time.sleep(30)"],start_new_session=True); '
            f'Path({str(marker)!r}).write_text(json.dumps({{"pid":child.pid}}))')
    started = check(c, prepared, code)
    child = None
    try:
        until(marker.exists)
        child = process_identity(json.loads(marker.read_text())['pid'])
        result = ended(c, started)
        assert result['job']['details']['state'] == 'ownership_lost'
        assert result['lease_held'] and result['check'] is None
        assert 'escaped services' in result['cleanup_boundary']
        with pytest.raises(ProgramError, match='integration/evaluation lease'):
            check_review(c['store'])
        with pytest.raises(ProgramError, match='detached process remains live'):
            c['integration'].recover('p', started['job']['id'], reason='Cannot claim detached cleanup')
    finally:
        if process_live(child):
            os.kill(child['pid'], signal.SIGKILL)
        until(lambda: not process_live(child))
    result = c['integration'].recover('p', started['job']['id'], reason='Fixture explicitly stopped detected detached child')
    assert not result['lease_held'] and result['check'] is None
