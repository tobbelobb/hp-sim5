import importlib.util
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/python'))
sys.path.insert(0, str(ROOT / 'research-program'))


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def repo(tmp_path):
    root = tmp_path / 'repo'
    root.mkdir()
    subprocess.run(['git', 'init', '-q', '-b', 'main', str(root)], check=True)
    (root / '.gitignore').write_text('.research-state/\n.venv\noutput\nnode_modules\nRRF/build\nsrc/wasm/cable_solver/target\n')
    (root / 'research/output/session-prior').mkdir(parents=True)
    (root / 'research/output/session-prior/report.md').write_text('Prior evidence\n')
    (root / 'research-program/roles').mkdir(parents=True)
    for role in ('manager','researcher','reviewer'):
        shutil.copyfile(ROOT / f'research-program/roles/{role}.md', root / f'research-program/roles/{role}.md')
    subprocess.run(['git', '-C', str(root), 'add', '-f', '.gitignore', 'research', 'research-program'], check=True)
    subprocess.run(['git', '-C', str(root), '-c', 'user.name=Test', '-c', 'user.email=test@localhost', 'commit', '-qm', 'fixture'], check=True)
    return root


@pytest.fixture
def cli(repo, monkeypatch):
    module = load('research_program_cli', ROOT / 'research-program/cli.py')
    monkeypatch.setattr(module, 'ROOT', repo)
    return module


@pytest.fixture
def launcher(repo, monkeypatch):
    module = load('research_agent_launcher', ROOT / 'scripts/research_agent.py')
    monkeypatch.setattr(module, 'ROOT', repo)
    return module
