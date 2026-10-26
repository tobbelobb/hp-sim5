"""Cloud launcher setup regressions; no model calls or physics worlds."""
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

from hp_sim5_research.program_workers import runtime_check

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize('launcher,arguments,cloud', [
    ('hp-sim5-research-program', ['--cloud', '--question', 'fixture'], True),
    ('hp-sim5-research-program', ['--codex', '--question', 'fixture'], False),
    ('hp-sim5-research-agent', ['--codex', '--batch'], True),
])
def test_entry_loads_cloud_environment_without_parent_sourcing(tmp_path, launcher, arguments, cloud):
    (tmp_path / 'scripts/cloud').mkdir(parents=True)
    shutil.copy2(ROOT / launcher, tmp_path / launcher)
    shutil.copy2(ROOT / 'scripts/cloud/env.sh', tmp_path / 'scripts/cloud/env.sh')
    script = 'research_program.py' if launcher.endswith('program') else 'research_agent.py'
    (tmp_path / 'scripts' / script).write_text('import json, os; print(json.dumps(dict(path=os.environ["PATH"])))')
    if launcher.endswith('agent'):
        (tmp_path / '.venv/bin').mkdir(parents=True)
        (tmp_path / '.venv/bin/python').symlink_to(sys.executable)
    tools = tmp_path / 'cloud-tools'
    # Simulate a host that started before env.sh was sourced. A local manager
    # explicitly overrides the inherited mode and must not inject cloud paths.
    result = subprocess.run([str(tmp_path / launcher), *arguments], capture_output=True, text=True,
                            env=dict(os.environ, HP_SIM5_RESEARCH_CLIENT='cloud', HP_SIM5_CLOUD_TOOLS=str(tools)),
                            check=True, timeout=5)
    assert (str(tools / 'node') in json.loads(result.stdout)['path']) is cloud


def test_runtime_check_reloads_environment_of_existing_host(tmp_path, monkeypatch):
    (tmp_path / 'scripts/cloud').mkdir(parents=True)
    (tmp_path / 'scripts/cloud/env.sh').write_text('export HP_SIM5_SETUP_FIX=loaded\n')
    (tmp_path / 'scripts/cloud/check_research.py').write_text(
        'import json, os; print(json.dumps({"ok": os.environ.get("HP_SIM5_SETUP_FIX") == "loaded"}))')
    monkeypatch.setenv('HP_SIM5_RESEARCH_CLIENT', 'cloud')
    monkeypatch.delenv('HP_SIM5_SETUP_FIX', raising=False)
    assert runtime_check(tmp_path)['ok']
    assert 'HP_SIM5_SETUP_FIX' not in os.environ


@pytest.mark.parametrize('failure', ['sockets', 'python', None])
def test_setup_diagnostics_locate_failure_without_movement(tmp_path, monkeypatch, failure):
    spec = importlib.util.spec_from_file_location('cloud_setup_check', ROOT / 'scripts/cloud/check_research.py')
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
