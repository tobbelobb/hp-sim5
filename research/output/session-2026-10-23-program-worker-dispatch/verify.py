"""Verify saved task009 check records, source hashes, links and queue; no science."""
import hashlib
import json
from pathlib import Path
import re

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]

hashes = json.loads((HERE / 'source-hashes.json').read_text())
for name, expected in hashes.items():
    assert hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == expected, name

queue = json.loads((ROOT / 'research/program-tasks/tasks.json').read_text())['tasks']
tasks = {task['id']: task for task in queue}
assert tasks['009']['status'] == 'done'
assert all(tasks[dependency]['status'] == 'done' for dependency in tasks['009']['depends_on'])
ready = sorted((task for task in queue if task['status'] == 'ready' and
                all(tasks[d]['status'] == 'done' for d in task['depends_on'])), key=lambda t: t['priority'])
assert ready[0]['id'] == '010'

links = 0
for path in [ROOT / 'research/program-workers.md', ROOT / 'research/program-tasks/009-existing-worker-dispatch.md',
             *(HERE / name for name in ('abstract.md', 'report.md', 'research.md'))]:
    for target in re.findall(r'\[[^\]]+\]\(([^)]+)\)', path.read_text()):
        if '://' in target or target.startswith('#'):
            continue
        assert (path.parent / target.split('#', 1)[0]).exists(), (path, target)
        links += 1

dry_runs = json.loads((HERE / 'dry-run-records.json').read_text())
assert len(dry_runs) == 4
assert {(run['backend'], run['worktree']) for run in dry_runs} == {
    ('codex', False), ('codex', True), ('claude', False), ('claude', True)}
for run in dry_runs:
    assert run['returncode'] == 0 and not run['session_created']
    assert run['stdout']['mode'] == ('worktree' if run['worktree'] else 'batch')
    assert '--batch' in run['argv'] and '--prompt-file' in run['argv'] and '--dry-run' in run['argv']
    if not run['worktree']:
        assert run['stdout']['prompt'].encode() == (HERE / 'dry-run-prompt.txt').read_bytes()

checks = json.loads((HERE / 'checks.json').read_text())
assert checks['final_adapter_launcher']['passed'] == 57
assert checks['final_adapter_after_import_validation']['passed'] == 10
assert checks['reused_store_graph_tests'] == 17
print(f'Verified {len(hashes)} source hashes, {links} links, four real dry-runs and queue: next 010.')
