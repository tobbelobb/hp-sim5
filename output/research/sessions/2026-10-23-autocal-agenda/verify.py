"""Check this desk exercise's links, saved claims and source identities; no fits."""
import hashlib
import json
from pathlib import Path
import re
import subprocess
from urllib.parse import unquote

ROOT = Path(__file__).resolve().parents[4]
SESSION = Path(__file__).resolve().parent
CM = ROOT / 'output/research/sessions/2026-10-23-consensus-membership'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


results = json.loads((CM / 'results.json').read_text())
summary = results['summary']['additional_dataset']
assert [summary[k] for k in ('datasets', 'improved', 'worsened', 'unchanged')] == [8, 1, 2, 5]
changed = [r for r in results['rows'] if abs(r['delta_sum_anchor_error_mm']) > 1e-8]
assert {r['fixture'] for r in changed} == {
    'flexible_lines_2000', 'ten_points_w_buildup', 'ten_points_w_buildup2'}
assert all(not r['common_unseen_fold_ids'] for r in changed)
assert all(r['single_fit_prediction_matches_saved'] for r in results['rows'])

sources = ['autocal/history_selection.py', 'autocal/predictive_validation.py',
           'autocal/autocal.py', 'autocal/stopping.py',
           'autocal/spool_model.py', 'autocal/ellipse_cost.py']
hashes = {name: sha(ROOT / name) for name in sources}
comparable = {name: value for name, value in hashes.items()
              if name in results['current_source_sha256']}
assert len(comparable) == 5
assert all(value == results['current_source_sha256'][name]
           for name, value in comparable.items())

docs = [ROOT / 'research/autocal-agenda.md',
        ROOT / 'research/program-tasks/005-follow-explicit-dependencies.md',
        SESSION / 'abstract.md', SESSION / 'report.md']
# checks.json is this command's output, so create it before checking its links.
checks_path = SESSION / 'checks.json'
checks_path.write_text('{}\n')
links = []
for doc in docs:
    for match in re.finditer(r'\]\(([^)]+)\)', doc.read_text()):
        target = unquote(match[1])
        if '://' in target:
            continue
        name, _, anchor = target.partition('#')
        path = (doc.parent / name).resolve() if name else doc
        assert path.is_file() or path.is_dir(), (doc, target)
        if anchor:
            headings = re.findall(r'^#+\s+(.+)$', path.read_text(), re.M)
            slugs = [re.sub(r'[^\w\- ]', '', h.lower()).replace(' ', '-') for h in headings]
            assert anchor in slugs, (doc, target, slugs)
        links.append({'document': str(doc.relative_to(ROOT)), 'target': target})

queue = json.loads((ROOT / 'research/program-tasks/tasks.json').read_text())
states = {task['id']: task['status'] for task in queue['tasks']}
assert all(state == 'done' for state in states.values())
assert all(states[dep] == 'done' for task in queue['tasks'] for dep in task['depends_on'])
assert 'autocal-agenda.md' in (ROOT / 'research/README.md').read_text()
assert 'autocal-agenda.md' in (ROOT / 'research/catalog.md').read_text()
assert 'research/autocal-agenda.md' in (ROOT / 'research/prompts/next-assignment.txt').read_text()
index = (ROOT / 'output/research/index.md').read_text()
assert '## [2026-10-23-autocal-agenda]' in index
assert 'AG-D1' in index and 'AG-D2' in index
abstract = (SESSION / 'abstract.md').read_text()
report = (SESSION / 'report.md').read_text()
assert abstract.split('## Abstract\n\n')[1].split('\n## Topics')[0] == report.split('## Abstract\n\n')[1].split('\n## AG-D1')[0]

missing = [
    'output/research/sessions/2026-10-23-program-task-sanity/report.md',
    'output/research/sessions/b7deb1bcc9cb418388273e2b7b9adc8c/paired_regressions.json',
    'output/research/reference-refresh-october-7-2026/comparison.json',
    'output/research/sessions/2026-10-22-autocal-filtering-integration/summary-comparison.json',
    'output/research/sessions/2026-10-22-autocal-solver-integration/truth-verification.json',
    'output/research/2026-10-09-slip-recovery-and-encoder-model/encoder-objective-audit.json',
]
evidence = [CM / name for name in ('report.md', 'results.json', 'inputs.json', 'audit.json')]
evidence += [ROOT / name for name in (
    'output/research/sessions/b7deb1bcc9cb418388273e2b7b9adc8c/report.md',
    'output/research/reference-refresh-october-7-2026/report.md',
    'output/research/2026-10-09-slip-recovery-and-encoder-model/report.md',
    'research/autocal-filtering-lane02-integration.md',
    'research/autocal-solver-lane01-integration.md',
    'research/autocal-ranking-lane04-integration.md')]
subprocess.run(['git', 'diff', '--check'], cwd=ROOT, check=True)
checks = {
    'command': '.venv/bin/python output/research/sessions/2026-10-23-autocal-agenda/verify.py',
    'base_revision': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
    'result': 'passed',
    'kind': 'saved-evidence and documentation inspection; no numerical reproduction',
    'inspected_current_source_sha256': hashes,
    'source_sha256_matching_cm_e1_analysis': comparable,
    'source_without_saved_cm_e1_hash': [name for name in hashes if name not in comparable],
    'inspected_evidence_sha256': {str(p.relative_to(ROOT)): sha(p) for p in evidence},
    'saved_summary_counts': {k: summary[k] for k in ('datasets', 'improved', 'worsened', 'unchanged')},
    'saved_changed_cases_without_common_unseen_fold': [r['fixture'] for r in changed],
    'missing_artifacts': [name for name in missing if not (ROOT / name).exists()],
    'local_links_and_anchors_checked': len(links),
    'queue_statuses': states,
    'catalog': 'generated via scripts/research_index.py; new entry retained with historical entries whose local artifacts are absent',
    'abstract_report_match': True,
    'guide_prompt_discovery': True,
    'workflow_exercises': {'AG-D1': 'four routes compared, one next brief',
                           'AG-D2': 'counterfactual withdrawal changes allocation'},
    'actual_experiments': {'fresh_fits': 0, 'selection_evaluations': 0,
                           'prediction_evaluations': 0, 'physics_steps': 0,
                           'collections': 0, 'hardware_measurements': 0},
}
checks_path.write_text(json.dumps(checks, indent=2) + '\n')
print(json.dumps({'result': 'passed', 'links': len(links), 'sources': len(hashes),
                  'changed_cases': len(changed), 'checks': str(checks_path.relative_to(ROOT))}))
