"""The local program can run with the entire cloud adapter absent."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

from hp_sim5_research.program_host import request
from .test_research_program_manager import until, live

ROOT = Path(__file__).resolve().parents[2]


def test_local_core_opens_and_plans_without_cloud_files(tmp_path):
    for name in ('research_program.py', 'research_program_host.py'):
        target = tmp_path / 'scripts' / name
        target.parent.mkdir(exist_ok=True)
        shutil.copy2(ROOT / 'scripts' / name, target)
    package = Path('src/python/hp_sim5_research')
    shutil.copytree(ROOT / package, tmp_path / package,
                    ignore=shutil.ignore_patterns('__pycache__'))
    prompt = Path('research/prompts/research-manager-workflow.txt')
    (tmp_path / prompt).parent.mkdir(parents=True)
    shutil.copy2(ROOT / prompt, tmp_path / prompt)
    assert not (tmp_path / '.agents/skills/cloud-research-client').exists()
    environment = dict(os.environ, HP_SIM5_RESEARCH_CLIENT='local')
    command = [sys.executable, str(tmp_path / 'scripts/research_program.py')]

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
