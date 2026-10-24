"""Run finite integration checks outside the ten-second manager request path."""
import json
import ctypes
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/python'))
from hp_sim5_research.program_workers import process_identity


def finish_group(directory, subreaper):
    detached = []
    # Linux reparents ordinary orphan descendants to this runner. This detects
    # unsupported detachment at clean completion, not all external services.
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            fields = (proc / 'stat').read_text().rsplit(') ', 1)[1].split()
            if int(fields[1]) == os.getpid() and int(fields[2]) != os.getpgrp() and fields[0] not in ('Z', 'X'):
                detached.append(process_identity(int(proc.name)))
        except (OSError, ValueError, IndexError):
            continue
    if detached:
        (directory / 'detached-observation.json').write_text(json.dumps({'subreaper': subreaper, 'processes': detached}))
    os.killpg(os.getpgrp(), signal.SIGKILL)


def main():
    directory, worktree = map(Path, sys.argv[1:])
    try:
        subreaper = ctypes.CDLL(None, use_errno=True).prctl(36, 1, 0, 0, 0) == 0
    except (OSError, AttributeError):
        subreaper = False
    temporary_identity = directory / 'runner-identity.json.tmp'
    temporary_identity.write_text(json.dumps(process_identity(os.getpid())))
    temporary_identity.replace(directory / 'runner-identity.json')
    plan = json.loads((directory / 'check-plan.json').read_text())
    environment = dict(os.environ, PYTHONPATH=str(worktree / 'src/python'), PYTHONDONTWRITEBYTECODE='1')
    results = []
    for index, check in enumerate(plan['checks']):
        remaining = plan['deadline'] - time.time()
        if remaining <= 0:
            (directory / 'wall-expired').write_text('Check wall allocation expired\n')
            finish_group(directory, subreaper)
        with (directory / f'check-{index:02d}.log').open('wb') as output:
            try:
                process = subprocess.Popen(check['argv'], cwd=worktree, stdin=subprocess.DEVNULL,
                                           stdout=output, stderr=output, env=environment)
                code = process.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                (directory / 'wall-expired').write_text('Check wall allocation expired\n')
                finish_group(directory, subreaper)
            except OSError as error:
                output.write((str(error) + '\n').encode())
                code = 127
        results.append({'check': check, 'returncode': code})
    temporary = directory / 'execution.json.tmp'
    temporary.write_text(json.dumps(results))
    temporary.replace(directory / 'execution.json')
    # A check may leave background descendants in this session. Kill the whole
    # owned group, including this runner, before the controller releases its lease.
    # execution.json records completed commands; exit status alone never passes.
    finish_group(directory, subreaper)


if __name__ == '__main__':
    sys.exit(main())
