"""Direct Python children must not depend on a parent having sourced env.sh."""
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

from hp_sim5_research.cloud_environment import SETTINGS, activate

ROOT = Path(__file__).resolve().parents[2]


def test_activation_preserves_auth_and_local_mode(tmp_path, monkeypatch):
    monkeypatch.setenv('HP_SIM5_RESEARCH_CLIENT', 'local')
    before = dict(os.environ)
    activate(tmp_path)  # Local installs do not require any cloud setup files.
    assert dict(os.environ) == before
    script = tmp_path / 'scripts/cloud/env.sh'
    script.parent.mkdir(parents=True)
    shutil.copy2(ROOT / 'scripts/cloud/env.sh', script)
    monkeypatch.setenv('HP_SIM5_RESEARCH_CLIENT', 'cloud')
    monkeypatch.setenv('HP_SIM5_CLOUD_TOOLS', str(tmp_path / 'tools with spaces'))
    monkeypatch.setenv('PATH', '/usr/bin:/bin')
    monkeypatch.setenv('OPENAI_API_KEY', 'fixture-value')
    monkeypatch.setenv('HTTPS_PROXY', 'http://fixture.invalid:1234')
    monkeypatch.setenv('VK_ICD_FILENAMES', '/fixture/custom-vulkan.json')
    for key in ('CARGO_HOME', 'RUSTUP_HOME'):
        monkeypatch.delenv(key, raising=False)
    # Restore every changed export after this test.
    for key in SETTINGS:
        if key in os.environ:
            monkeypatch.setenv(key, os.environ[key])
        else:
            monkeypatch.delenv(key, raising=False)
    activate(tmp_path)
    assert os.environ['PATH'].startswith(str(tmp_path / 'tools with spaces/node'))
    assert os.environ['CARGO_HOME'] == str(tmp_path / 'tools with spaces/cargo')
    assert os.environ['OPENAI_API_KEY'] == 'fixture-value'
    assert os.environ['HTTPS_PROXY'] == 'http://fixture.invalid:1234'
    assert os.environ['VK_ICD_FILENAMES'] == '/fixture/custom-vulkan.json'
    assert os.environ['HP_SIM5_RESEARCH_CLIENT'] == 'cloud'


@pytest.mark.parametrize('script,arguments', [
    ('research_agent.py', ['--help']),
    ('research_program.py', ['--cloud', '--question', 'fixture', '--dry-run']),
])
def test_direct_python_from_stale_parent_loads_setup(tmp_path, script, arguments):
    # The real dispatcher invokes Python directly, bypassing the shell wrapper.
    (tmp_path / 'scripts/cloud').mkdir(parents=True)
    shutil.copy2(ROOT / 'scripts' / script, tmp_path / 'scripts' / script)
    (tmp_path / 'src').mkdir()
    (tmp_path / 'src/python').symlink_to(ROOT / 'src/python', target_is_directory=True)
    marker = tmp_path / 'loaded'
    (tmp_path / 'scripts/cloud/env.sh').write_text(
        f'printf loaded > "{marker}"\n' + (ROOT / 'scripts/cloud/env.sh').read_text())
    env = dict(os.environ, PATH='/usr/bin:/bin', HP_SIM5_RESEARCH_CLIENT='cloud')
    result = subprocess.run([sys.executable, str(tmp_path / 'scripts' / script), *arguments],
                            env=env, capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stderr
    assert marker.read_text() == 'loaded'
    assert result.stdout
