"""Build the offline architecture report from source prompts and archived evidence."""
import ast
from collections import Counter
from datetime import datetime
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
ARCHIVE = Path('research/programs/program-7aa80aa18f10499e85cb24ffeae712cb')
AUDIT = ARCHIVE / 'selected/output/research/sessions/2026-10-26-cloud-platform-audit'
PRODUCTION = ARCHIVE / 'selected/output/research/sessions/attempt-e13302e362f545e39608027dcb85f963'
OUTPUT = ROOT / 'research/reports/research-program-cloud.html'


def read(path):
    return (ROOT / path).read_text()


def prompt(title, path, *, assignments=(), function=None, excerpt=None):
    source = read(path)
    text = source
    if assignments or function:
        nodes = ast.walk(ast.parse(source))
        if function:
            text = ast.get_source_segment(source, next(n for n in nodes
                if isinstance(n, ast.FunctionDef) and n.name == function))
        else:
            text = '\n\n'.join(ast.get_source_segment(source, n) for n in nodes
                if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and
                t.id in assignments for t in n.targets))
    if excerpt:
        start = source.index(excerpt[0])
        text = source[start:source.index(excerpt[1], start)]
    return {'title': title, 'path': str(path), 'text': text,
            'sha256': hashlib.sha256(source.encode()).hexdigest()}


def build():
    snapshot = json.loads(read(ARCHIVE / 'program.json'))
    coverage = json.loads(read(PRODUCTION / 'coverage.json'))
    records = snapshot['records']
    counts = Counter(r['kind'] for r in records)
    timeline = []
    for record in records:
        kind, details = record['kind'], record['details']
        if kind not in ('attempt', 'review_run', 'submission', 'admission', 'decision', 'human_input'):
            continue
        item = {'id': record['id'], 'kind': kind, 'at': record['created_at'],
                'title': record['title'], 'state': details.get('state', details.get('decision', 'recorded'))}
        if kind == 'attempt':
            start = datetime.fromisoformat(details['started_at'])
            exits = [o for o in details['observations'] if o['event'] == 'exited']
            item.update(assignment=details['assignment_id'], cleanup=details['cleanup'],
                        source=details['source_revision'][:8], allocation=details['allocation'],
                        returncode=details['returncode'])
            if exits:
                item['seconds'] = (datetime.fromisoformat(exits[-1]['at']) - start).total_seconds()
        if kind == 'review_run':
            item.update(submission=details['submission_id'], returncode=details['returncode'],
                        wall_minutes=details['wall_minutes'], error=details.get('error'))
        if kind == 'admission':
            item.update(finding=details['finding'], code_readiness=details['code_readiness'],
                        submission=details['submission_id'])
        timeline.append(item)
    prompts = [
        prompt('All roles · repository instructions', 'AGENTS.md'),
        prompt('Manager · cloud entry', 'scripts/cloud/prompts/research-manager.txt'),
        prompt('Manager · shared workflow', 'research/prompts/research-manager-workflow.txt'),
        prompt('Manager · cloud supplement', 'scripts/cloud/prompts/manager-workflow.txt'),
        prompt('Manager / broker · discoverable cloud skill', '.agents/skills/cloud-research-client/SKILL.md'),
        prompt('Worker · research instructions', 'research/AGENTS.md'),
        prompt('Worker · assignment envelope (source template)', 'src/python/hp_sim5_research/program_workers.py', assignments=('contract', 'prompt')),
        prompt('Worker · shared runtime boundary (source template)', 'src/python/hp_sim5_research/program_pool.py', assignments=('SHARED_BOUNDARY',)),
        prompt('Worker · injected strategy contract (source template)', 'src/python/hp_sim5_research/program_manager.py', function='_worker_contract'),
        prompt('Worker · session suffix (source template)', 'scripts/research_agent.py',
               excerpt=("        instructions = (ROOT / 'research/AGENTS.md').read_text()", "        full_prompt = prompt or ''")),
        prompt('Reviewer · shared review instructions', 'research/prompts/reviewer.txt'),
        prompt('Reviewer · fixed assignment and response schema (source template)', 'src/python/hp_sim5_research/program_review.py',
               excerpt=("        response = {'submission_id':", "        if backend == 'codex':")),
        prompt('Reviewer · instructions.md (source template)', 'src/python/hp_sim5_research/program_review.py',
               excerpt=("        (directory / 'instructions.md').write_text", "        (directory / 'attachment.json').write_text")),
    ]
    data = {'program': snapshot['program'], 'counts': counts, 'timeline': timeline,
            'coverage': coverage, 'prompts': prompts, 'archive': str(ARCHIVE),
            'audit': str(AUDIT), 'production': str(PRODUCTION),
            'source_revision': subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT,
                capture_output=True, text=True, check=True).stdout.strip(),
            'snapshot_sha256': hashlib.sha256(read(ARCHIVE / 'program.json').encode()).hexdigest()}
    template = read('scripts/cloud/report_template.html')
    # Safe even when future prompts contain HTML/script tags.
    encoded = json.dumps(data, ensure_ascii=False).replace('<', '\\u003c').replace('&', '\\u0026')
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(template.replace('REPORT_DATA', encoded))
    print(OUTPUT.relative_to(ROOT))


if __name__ == '__main__':
    build()
