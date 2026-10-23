import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

from hp_sim5_research.archive import export_handoff, read_handoff, saved_autocal_status

ROOT = Path(__file__).resolve().parents[2]
CLI = ROOT / 'scripts/research_handoff.py'


def fixture(tmp_path):
    worktree = tmp_path / 'old-worktree'
    session = worktree / 'output/session'
    session.mkdir(parents=True)
    (session / 'abstract.md').write_text('# Synthetic fixture\n\n## Decision handoff\nKeep current policy.\n')
    (session / 'report.md').write_text('[Measurements](metrics.json); [input](inputs/history.json)\n')
    (session / 'research.md').write_text('Synthetic lifecycle/evidence test; no accuracy claim.\n')
    (session / 'metrics.json').write_text('{"fixture_only": true}\n')
    external = worktree / 'history.json'
    external.write_text('{"saved_input": [1, 2, 3]}\n')
    (session / 'job.json').write_text(json.dumps(dict(
        job_id='ended-job', kind='full_autocal', status='accepted', exit_code=0,
        stop_reason='patience', ended_unix_s=123, pid=12345,
        directory=str(session), artifacts={'history': str(external)},
        supervisor_source_sha256={'producer.py': 'saved-source-hash'})))
    selection = dict(claim='Synthetic portability fixture', scope='Saved bytes and process outcome only',
                     decision='No calibration decision follows from an accepted CLI',
                     source_revision='fixture-revision', files=[
                         dict(path='metrics.json'),
                         dict(path='inputs/history.json', source=str(external), role='input',
                              sha256=hashlib.sha256(external.read_bytes()).hexdigest()),
                         dict(path='job.json', role='job')])
    return worktree, session, selection


def test_handoff_cli_survives_relocation_without_original_tree(tmp_path):
    worktree, session, selection = fixture(tmp_path)
    (session / 'connection.json').write_text('private attachment, never select')
    (session / 'large.rrd').write_text('unneeded recording')
    selected = tmp_path / 'selection.json'
    selected.write_text(json.dumps(selection))
    original = tmp_path / 'export'
    run = subprocess.run([sys.executable, str(CLI), 'export', str(session), str(selected), str(original)],
                         capture_output=True, text=True)
    assert run.returncode == 0, run.stderr
    relocated = tmp_path / 'new-host/handoff'
    relocated.parent.mkdir()
    shutil.move(original, relocated)
    shutil.rmtree(worktree)
    selected.unlink()
    run = subprocess.run([sys.executable, str(CLI), 'show', str(relocated)], capture_output=True, text=True)
    assert run.returncode == 0, run.stderr
    recovered = json.loads(run.stdout)
    assert recovered['integrity'] == 'verified_selected_files'
    assert recovered['decision'] == selection['decision']
    assert recovered['scope'] == selection['scope']
    assert recovered['source_revision'] == 'fixture-revision'
    assert recovered['jobs'][0]['observation'] == 'terminal'
    assert recovered['jobs'][0]['scientific_success'] == 'not_assessed'
    assert recovered['jobs'][0]['live_status'] == 'unknown'
    assert recovered['jobs'][0]['supervisor_source_sha256'] == {'producer.py': 'saved-source-hash'}
    files = {entry['path']: entry for entry in recovered['files']}
    assert not Path(files['inputs/history.json']['source']).exists()
    for entry in files.values():
        assert entry['integrity'] == 'ok'
        assert Path(entry['current_path']).is_relative_to(relocated)
    assert (relocated / 'inputs/history.json').read_text() == '{"saved_input": [1, 2, 3]}\n'
    assert not (relocated / 'connection.json').exists()
    assert not (relocated / 'large.rrd').exists()


@pytest.mark.parametrize('status,observation', [
    ('accepted', 'terminal'), ('failed', 'terminal'), ('cancelled', 'terminal'),
    ('budget_exhausted', 'terminal'), ('running', 'historical_nonterminal'),
    ('cancelling', 'historical_nonterminal'), ('surprise', 'unknown_status')])
def test_saved_job_is_historical_and_never_probes_or_signals_pid(tmp_path, monkeypatch, status, observation):
    def forbidden(*args, **kwargs):
        pytest.fail('Offline recovery must not inspect or signal a process')
    monkeypatch.setattr('os.kill', forbidden)
    monkeypatch.setattr('os.getpgid', forbidden)
    path = tmp_path / 'job.json'
    path.write_text(json.dumps(dict(status=status, pid=1, exit_code=0)))
    record = saved_autocal_status(path)
    assert record['observation'] == observation
    assert record['saved_status'] == status
    assert record['live_status'] == 'unknown'
    assert record['scientific_success'] == 'not_assessed'


def test_missing_and_corrupt_evidence_keeps_partial_handoff(tmp_path):
    _worktree, session, selection = fixture(tmp_path)
    selection['files'][0]['sha256'] = '0' * 64  # a known identity mismatch at export
    selection['files'].append(dict(path='lost-input.json', role='input'))
    (session / 'job.json').write_text('{interrupted write')
    bundle = tmp_path / 'partial'
    exported = export_handoff(session, selection, bundle)
    assert exported['integrity'] == 'partial'
    assert any('Source differs' in message for message in exported['diagnostics'])
    assert any('lost-input.json' in message for message in exported['diagnostics'])
    assert exported['jobs'][0]['observation'] == 'unavailable'
    assert (bundle / 'report.md').is_file()
    (bundle / 'inputs/history.json').write_text('corruption after transfer')
    recovered = read_handoff(bundle)
    assert any('inputs/history.json: SHA-256 mismatch' in message for message in recovered['diagnostics'])
    run = subprocess.run([sys.executable, str(CLI), 'show', str(bundle)], capture_output=True, text=True)
    assert run.returncode == 1
    assert json.loads(run.stdout)['integrity'] == 'partial'


@pytest.mark.parametrize('path', ['../escape', '/absolute/file', 'handoff.json', 'report.md', 'a/../../escape'])
def test_export_rejects_unsafe_or_duplicate_destinations_before_writing(tmp_path, path):
    _worktree, session, selection = fixture(tmp_path)
    selection['files'].append(dict(path=path))
    with pytest.raises(ValueError):
        export_handoff(session, selection, tmp_path / 'bundle')
    assert not (tmp_path / 'bundle').exists()


def test_handoff_does_not_overwrite_or_follow_relocated_symlink(tmp_path):
    _worktree, session, selection = fixture(tmp_path)
    bundle = tmp_path / 'bundle'
    export_handoff(session, selection, bundle)
    with pytest.raises(FileExistsError):
        export_handoff(session, selection, bundle)
    (bundle / 'metrics.json').unlink()
    (bundle / 'metrics.json').symlink_to(session / 'metrics.json')
    assert any('outside the handoff' in message for message in read_handoff(bundle)['diagnostics'])


@pytest.mark.parametrize('content', [None, '{bad json', '[]'])
def test_missing_or_corrupt_manifests_and_jobs_have_diagnostics(tmp_path, content):
    if content is not None:
        (tmp_path / 'handoff.json').write_text(content)
        (tmp_path / 'job.json').write_text(content)
    assert read_handoff(tmp_path)['integrity'] == 'partial'
    assert read_handoff(tmp_path)['diagnostics']
    assert saved_autocal_status(tmp_path / 'job.json')['observation'] == 'unavailable'
