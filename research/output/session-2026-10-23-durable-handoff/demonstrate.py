"""Package saved CM-E1 evidence once; delete its source tree before recovery."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent
SAVED = HERE.parent / '2026-10-23-consensus-membership'
CLI = ROOT / 'scripts/research_handoff.py'
REVISION = '166f0352e032266711887d0abd01d194cbd4acf2'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


commands = []


def cli(*args, expected=0):
    argv = [sys.executable, str(CLI), *map(str, args)]
    commands.append(argv)
    run = subprocess.run(argv, capture_output=True, text=True, cwd=ROOT)
    assert run.returncode == expected, (run.returncode, run.stderr, run.stdout)
    return json.loads(run.stdout)


with tempfile.TemporaryDirectory(prefix='hp-sim5-handoff-') as temporary:
    temporary = Path(temporary)
    old_tree = temporary / 'old-worktree'
    source = old_tree / 'session'
    shutil.copytree(SAVED, source)
    inputs = json.loads((source / 'inputs.json').read_text())
    entries = [dict(path=name, sha256=sha(source / name)) for name in
               ('results.json', 'inputs.json', 'audit.json', 'probe.py', 'audit.py')]
    for entry in inputs:
        entries.append(dict(path=entry['input'], role='input', sha256=entry['gzip_sha256']))
    # Exercise an explicit external-worktree reference as well as session-local files.
    external = old_tree / 'external/history.json.gz'
    external.parent.mkdir()
    shutil.move(source / entries[5]['path'], external)
    entries[5]['source'] = str(external)
    selection = dict(
        claim='CM-E1 rejects a blanket finite-prediction consensus restriction',
        scope='Eleven frozen simulated histories; final selection, no trajectory replay',
        decision='Preserve consensus defaults; common-unseen predictive gain remains unestablished',
        source_revision=REVISION, files=entries, limitations=[
            'Portability and byte identity check only; CM-E1 numerical work is not rerun.',
            'Prior-study links are retained as context, not selected; the initial-screen report is absent in this checkout.',
            'Analysis requires the recorded source revision and its dependencies/layout; runtime is not bundled.'
        ])
    selection_path = temporary / 'selection.json'
    selection_path.write_text(json.dumps(selection))
    exported = temporary / 'export'
    result = cli('export', source, selection_path, exported)
    assert result['integrity'] == 'verified_selected_files'
    relocated = temporary / 'new-host/handoff'
    relocated.parent.mkdir()
    shutil.move(exported, relocated)
    shutil.rmtree(old_tree)
    selection_path.unlink()
    recovered = cli('show', relocated)
    assert recovered['integrity'] == 'verified_selected_files'
    assert len(recovered['files']) == 19
    assert len([row for row in recovered['files'] if row['role'] == 'input']) == 11
    assert all(not Path(row['source']).exists() for row in recovered['files'])
    assert all(row['integrity'] == 'ok' for row in recovered['files'])
    assert recovered['decision'] == selection['decision']
    assert not recovered['jobs']  # CM-E1 was a saved selection probe, not a supervisor job.
    evidence = HERE / 'evidence'
    evidence.mkdir(exist_ok=True)
    (evidence / 'export-manifest.json').write_bytes((relocated / 'handoff.json').read_bytes())
    (evidence / 'recovered.json').write_text(json.dumps(recovered, indent=2) + '\n')
    summary = dict(
        experiment_id='DH-E1', source_revision=REVISION,
        source_session=str(SAVED.relative_to(ROOT)), selected_files=len(recovered['files']),
        selected_input_files=11, copied_bytes=sum(Path(row['current_path']).stat().st_size for row in recovered['files']),
        original_source_paths_unavailable=True, scoped_decision_preserved=True,
        selected_hashes_verified=True, commands=commands,
        limitations=selection['limitations'], physics_runs=0, fresh_fits=0, collections=0)
    (HERE / 'results.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps({key: value for key, value in summary.items() if key != 'commands'}, indent=2))
