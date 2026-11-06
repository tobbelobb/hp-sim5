"""Cloud launcher setup regressions; no model calls or physics worlds."""
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

from program.workers import runtime_check

ROOT = Path(__file__).resolve().parents[4]


@pytest.mark.parametrize('launcher,arguments,cloud', [
    ('hp-sim5-research-program', ['--cloud', '--question', 'fixture', '--dry-run'], True),
    ('hp-sim5-research-program', ['--codex', '--question', 'fixture', '--dry-run'], False),
    ('hp-sim5-research-agent', ['--codex', '--batch', '--machine', 'hp3', '--prompt', 'fixture', '--dry-run'], True),
])
def test_entry_loads_cloud_environment_without_parent_sourcing(tmp_path, launcher, arguments, cloud):
    scripts = tmp_path / '.agents/skills/cloud-research-client/scripts'
    scripts.mkdir(parents=True)
    marker = tmp_path / 'loaded'
    (scripts / 'env.sh').write_text(
        f'printf loaded > "{marker}"\n' + (ROOT / '.agents/skills/cloud-research-client/scripts/env.sh').read_text())
    shutil.copy2(ROOT / launcher, tmp_path / launcher)
    for directory in ('scripts', 'research-program-feature/scripts'):
        (tmp_path / directory).mkdir(parents=True, exist_ok=True)
    for script in ('scripts/research_agent.py', 'research-program-feature/scripts/research_program.py'):
        shutil.copy2(ROOT / script, tmp_path / script)
    for directory in ('src/python', 'research-program-feature/program', 'research-program-feature/role-instructions',
                      'research', 'hp-sim-3d'):
        (tmp_path / directory).parent.mkdir(parents=True, exist_ok=True)
        (tmp_path / directory).symlink_to(ROOT / directory, target_is_directory=True)
    (tmp_path / '.venv/bin').mkdir(parents=True)
    (tmp_path / '.venv/bin/python').symlink_to(sys.executable)
    result = subprocess.run([str(tmp_path / launcher), *arguments], capture_output=True, text=True,
                            env=dict(os.environ, HP_SIM5_RESEARCH_CLIENT='cloud'), timeout=10)
    assert result.returncode == 0, result.stderr
    assert marker.exists() is cloud


def test_runtime_check_reloads_environment_of_existing_host(tmp_path, monkeypatch):
    (tmp_path / '.agents/skills/cloud-research-client/scripts').mkdir(parents=True)
    (tmp_path / 'scripts').mkdir(exist_ok=True)
    (tmp_path / '.agents/skills/cloud-research-client/scripts/env.sh').write_text('export HP_SIM5_SETUP_FIX=loaded\n')
    (tmp_path / 'scripts/research_runtime_check.py').write_text(
        'import json, os; print(json.dumps({"ok": os.environ.get("HP_SIM5_SETUP_FIX") == "loaded"}))')
    monkeypatch.setenv('HP_SIM5_RESEARCH_CLIENT', 'cloud')
    monkeypatch.delenv('HP_SIM5_SETUP_FIX', raising=False)
    assert runtime_check(tmp_path)['ok']
    assert 'HP_SIM5_SETUP_FIX' not in os.environ


@pytest.mark.parametrize('failure', ['sockets', 'python', None])
def test_setup_diagnostics_locate_failure_without_movement(tmp_path, monkeypatch, failure):
    spec = importlib.util.spec_from_file_location('runtime_setup_check', ROOT / 'scripts/research_runtime_check.py')
    checker = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(checker)
    monkeypatch.setattr(checker, 'ROOT', tmp_path)
    for name in ('toolchain/bin/llvm-profdata', 'RRF/build/rrf_simulator'):
        path = tmp_path / name
        path.parent.mkdir(parents=True)
        path.touch(mode=0o755)
    calls = []

    def run(argv, **kwargs):
        calls.append(argv)
        if '--print' in argv:
            return subprocess.CompletedProcess(argv, 0, stdout=str(tmp_path / 'toolchain/lib'), stderr='')
        code = int(failure == 'python' and argv[0].endswith('/.venv/bin/python'))
        return subprocess.CompletedProcess(argv, code, stdout='', stderr='No module named pxr' if code else '')

    monkeypatch.setattr(checker.subprocess, 'run', run)
    monkeypatch.setattr(checker, 'probe', lambda: {
        'outcome': 'blocked' if failure == 'sockets' else 'passed',
        'checks': {'socket_creation': {'outcome': 'blocked', 'error': '[Errno 1] Operation not permitted',
                                     'next_step': 'Use supervised jobs'}} if failure == 'sockets' else {}})
    result = checker.check()
    assert result['ok'] is (failure is None)
    assert result['physics_started'] is False
    assert all('build:native' not in argv for argv in calls)
    if failure == 'python':
        assert 'pxr' in result['checks']['python_dependencies']['error']
    if failure == 'sockets':
        sockets = result['checks']['launcher_sockets']
        assert sockets['probe']['context'] == 'worker_launcher'
        assert 'same context cannot grant permission' in sockets['next_step']
        assert 'next_step' not in sockets['probe']['checks']['socket_creation']
