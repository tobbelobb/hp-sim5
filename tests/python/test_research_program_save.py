"""Git result checkpoints exclude live state and preserve unrelated staging."""
import hashlib
import json
from pathlib import Path
import subprocess

import pytest

from hp_sim5_research.program import ProgramError, ProgramStore
from hp_sim5_research.program_save import MAX_FILE, save_program


def git(root, *args):
    return subprocess.run(['git', '-C', str(root), *args], capture_output=True, text=True, check=True).stdout.strip()


@pytest.fixture
def results(tmp_path):
    git(tmp_path, 'init', '-q')
    git(tmp_path, 'config', 'user.name', 'Research fixture')
    git(tmp_path, 'config', 'user.email', 'fixture@example.invalid')
    (tmp_path / 'code.py').write_text('original\n')
    git(tmp_path, 'add', 'code.py')
    git(tmp_path, 'commit', '-qm', 'Initial fixture')
    with ProgramStore(tmp_path / '.research-programs', create=True) as store:
        store.create_program('A scoped research question', program_id='p')
        yield tmp_path, store


def artifact(store, root, name, content):
    path = root / name
    path.write_bytes(content)
    store.add_artifact('p', str(path), hashlib.sha256(content).hexdigest(), 'analytic', 'fixture-revision')
    return path


def test_commit_saves_evidence_and_leaves_unrelated_staging_alone(results):
    root, store = results
    artifact(store, root, 'measurement.csv', b'measured,error\n1,0.2\n')
    (root / 'analysis.py').write_text('print("recorded analysis")\n')
    (root / 'code.py').write_text('unrelated staged work\n')
    git(root, 'add', 'code.py')
    result = save_program(store, root, 'p', files=['analysis.py'], commit=True)
    directory = Path(result['directory'])
    assert json.loads((directory / 'program.json').read_text())['program']['goal'] == 'A scoped research question'
    assert list(directory.glob('evidence/*/measurement.csv'))
    assert (directory / 'selected/analysis.py').read_text() == (root / 'analysis.py').read_text()
    assert git(root, 'show', 'HEAD:code.py') == 'original'
    assert git(root, 'diff', '--cached', '--name-only') == 'code.py'
    assert all(p.startswith('research/programs/p/') for p in git(root, 'show', '--pretty=', '--name-only', 'HEAD').splitlines())
    assert result['commit'] == git(root, 'rev-parse', 'HEAD')


def test_private_large_and_missing_artifacts_are_reported_and_not_copied(results):
    root, store = results
    artifact(store, root, 'connection.json', b'{"runtime_token":"never-save"}')
    artifact(store, root, 'large.rrd', b'0' * (MAX_FILE + 1))
    result = save_program(store, root, 'p')
    assert len(result['skipped']) == 2
    assert result['copied'] == 0
    assert not list(Path(result['directory']).rglob('connection.json'))
    assert not list(Path(result['directory']).rglob('large.rrd'))
    assert result['commit'] is None


@pytest.mark.parametrize('name', ['connection.json', '.env', '../outside.txt'])
def test_explicit_private_or_outside_selection_fails_before_writing(results, name):
    root, store = results
    with pytest.raises(ProgramError):
        save_program(store, root, 'p', files=[name])
    assert not (root / 'research/programs/p').exists()
