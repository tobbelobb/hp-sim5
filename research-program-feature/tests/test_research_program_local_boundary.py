"""The local program can run with the entire cloud adapter absent."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

from program.host import request
from test_research_program_manager import until, live

ROOT = Path(__file__).resolve().parents[2]


def test_local_core_opens_and_plans_without_cloud_files(tmp_path):
    for package in ('research-program-feature/program', 'research-program-feature/scripts',
                    'research-program-feature/role-instructions', 'src/python/hp_sim5_research', 'machines'):
        shutil.copytree(ROOT / package, tmp_path / package,
                        ignore=shutil.ignore_patterns('__pycache__'))
    assert not (tmp_path / '.agents/skills/cloud-research-client').exists()
    environment = dict(os.environ, HP_SIM5_RESEARCH_CLIENT='local')
    command = [sys.executable, str(tmp_path / 'research-program-feature/scripts/research_program.py')]

    def run(*arguments):
        result = subprocess.run([*command, *arguments], env=environment,
                                capture_output=True, text=True, timeout=10, check=True)
        return json.loads(result.stdout)

    opened = run('--question', 'Local core without a cloud installation')['result']
    attachment = opened['attachment']
    try:
        assert opened['client_transport'] == 'local'
        assert request(attachment, 'capabilities')['ok']
        for backend in ('codex', 'claude'):
            plan = run('--' + backend, '--program', attachment['program_id'], '--dry-run')['result']['client']
            assert plan['argv'][0] == backend
            assert '.agents/skills/cloud-research-client' not in plan['instructions']
        cloud = subprocess.run([*command, '--cloud', '--program', attachment['program_id'], '--dry-run'],
                               env=environment, capture_output=True, text=True, timeout=10)
        assert cloud.returncode == 2
        assert 'requires the cloud-research-client skill' in cloud.stdout
    finally:
        request(attachment, 'shutdown')
        until(lambda: not live(attachment))
