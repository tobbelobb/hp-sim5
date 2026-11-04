"""Inspect saved research outcomes; run no models, fits, simulations or services."""
from collections import Counter
from datetime import datetime
import hashlib
import json
from pathlib import Path
import statistics
import subprocess

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).with_name('data.json')


def load(path):
    return json.loads((ROOT / path).read_text())


def source(path):
    return {'path': str(path), 'sha256': hashlib.sha256((ROOT / path).read_bytes()).hexdigest()}


def main():
    snapshots = sorted(ROOT.glob('research/programs/*/program.json')) + [
        ROOT / 'research/cloud-client-pilot/2026-10-24/program-export.json',
        ROOT / 'output/research/sessions/2026-10-24-live-manager-release/program-export.json',
    ]
    programs = []
    for path in snapshots:
        snapshot = load(path)
        attempts, admissions = [], []
        for record in snapshot['records']:
            details = record['details']
            if record['kind'] == 'attempt':
                seconds = (datetime.fromisoformat(details['exited_at']) -
                           datetime.fromisoformat(details['started_at'])).total_seconds()
                attempts.append({'id': record['id'], 'title': record['title'], 'state': details['state'],
                    'wall_minutes_allocated': details['allocation']['wall_minutes'],
                    'observed_wall_seconds': seconds, 'allocation': details['allocation'],
                    'goal_mode_setting': details['settings'].get('goal_mode'),
                    'worktree': details['settings'].get('worktree'),
                    'wall_expired': any(o['event'] == 'wall_expired' for o in details['observations'])})
            elif record['kind'] == 'admission':
                admissions.append({k: details[k] for k in ('decision', 'finding', 'code_readiness')})
        programs.append({'source': source(path.relative_to(ROOT)), 'program': snapshot['program'],
                         'record_counts': dict(Counter(r['kind'] for r in snapshot['records'])),
                         'attempts': attempts, 'admissions': admissions})
    attempts = [a for p in programs for a in p['attempts']]
    admissions = [a for p in programs for a in p['admissions']]
    direct = []
    # Manual classification after reading these sessions' abstracts/reports.
    # These labels describe historical producer claims, not fresh verification.
    retained = {'130caf5a': 'exact spool-model transformation cache',
        '2c84dbc6': 'persistent JAX compilation cache',
        'c812e7b9': 'guarded recent encoder settling window',
        'df7f5fd7': 'independently testable stages and metadata-copy optimization',
        '51aa157a': 'settling reuse and endpoint/geometry corrections',
        '2f03cb59': 'centered encoder averaging',
        'b65aa69c': 'shorter quiet window and geometric consensus',
        '267d719f': 'history-seeded fit variant',
        '98170696': 'warm-started anchor solve',
        '9b850a37': 'constant-objective compilation shortcut'}
    for path in sorted(ROOT.glob('output/research/sessions/2026-*-improve-encoder*')):
        # Require an actual launcher record. Three similar directories lack one;
        # hash-name aliases and interactive sessions are outside this cohort.
        if not (path / 'launch.json').is_file():
            continue
        abstract = (path / 'abstract.md').read_text()
        status = next(line for line in abstract.splitlines() if line.startswith('Status:'))
        outcome = ('retained implementation reported' if path.name[-8:] in retained else
                   'tooling repair only' if path.name.endswith('76802a07') else
                   'no source change justified' if path.name.endswith('1687b8ed') else 'unsummarized')
        direct.append({'session': path.name, 'abstract': source(path.relative_to(ROOT) / 'abstract.md'),
                       'title': abstract.splitlines()[0].removeprefix('# '), 'status': status,
                       'outcome': outcome, 'retained_change': retained.get(path.name[-8:]),
                       'summarized_complete': status.startswith('Status: complete'),
                       'batch': 'exec' in load(path / 'launch.json')['argv']})

    cache_path = Path('output/research/sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-130caf5a/E06-isolated-summary.json')
    cache = load(cache_path)
    cache_results = []
    for entry in cache:
        baseline, candidate = map(statistics.median, (entry['baseline_seconds'], entry['cache_seconds']))
        saving = 100 * (baseline - candidate) / baseline
        assert abs(saving - entry['median_saving_percent']) < 1e-9
        cache_results.append({'dataset': entry['dataset'], 'baseline_median_seconds': baseline,
                              'candidate_median_seconds': candidate, 'saving_percent': saving})
    constant_path = Path('output/research/sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/E26-regression-timing/summary.json')
    timing = load(constant_path)
    means = {label: {metric: statistics.mean(x[metric] for x in timing['trials'] if x['label'] == label)
                     for metric in ('wall_s', 'child_cpu_s')} for label in ('baseline', 'patched')}
    savings = {metric: 100 * (means['baseline'][metric] - means['patched'][metric]) /
               means['baseline'][metric] for metric in means['baseline']}

    commits = ['54e88492', '854430c6', '9422620c', '294e24c8', '79e53cbb']
    ancestry = {c: subprocess.run(['git', 'merge-base', '--is-ancestor', c, 'HEAD'],
                                  cwd=ROOT, check=False).returncode == 0 for c in commits}
    result = {'source_revision': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'method': 'Saved-source inspection and arithmetic only. No scientific re-execution or matched agent trial.',
        'programs': programs, 'direct_prompt_cohort': direct,
        'summary': {'program_snapshots': len(programs), 'worker_attempts': len(attempts),
            'worker_states': dict(Counter(a['state'] for a in attempts)),
            'worker_wall_caps_minutes': sorted(a['wall_minutes_allocated'] for a in attempts),
            'median_worker_wall_cap_minutes': statistics.median(a['wall_minutes_allocated'] for a in attempts),
            'observed_worker_wall_minutes_total': sum(a['observed_wall_seconds'] for a in attempts) / 60,
            'observed_worker_wall_minutes_max': max(a['observed_wall_seconds'] for a in attempts) / 60,
            'admissions': len(admissions),
            'admission_readiness': dict(Counter(a['code_readiness'] for a in admissions)),
            'integration_records': sum(v for p in programs for k, v in p['record_counts'].items() if k.startswith('integration')),
            'direct_cohort_sessions': len(direct),
            'direct_cohort_complete_summaries': sum(x['summarized_complete'] for x in direct),
            'direct_cohort_unsummarized': sum(not x['summarized_complete'] for x in direct),
            'direct_producer_outcomes': dict(Counter(x['outcome'] for x in direct))},
        'arithmetic_checks': {'cache_source': source(cache_path), 'cache': cache_results,
            'constant_objective_source': source(constant_path), 'constant_objective_means': means,
            'constant_objective_saving_percent': savings},
        'historical_commits_are_ancestors_of_current_head': ancestry,
        'limitations': ['Task, platform, dates and budgets differ; no causal success-rate estimate.',
            'Direct completed labels and retained changes are producer reports, not fresh verification.',
            'Several direct sessions overlap; they are not independent improvements.',
            'No reliable complete direct-session elapsed-time or token-cost records were found.',
            'Controlled cancellation and report-only assignments are intentional, not failed science.',
            'These snapshots cover available saved runs, not every conversation the human has had.']}
    assert len(programs) == 5 and len(attempts) == 9 and len(admissions) == 6
    assert len(direct) == 16 and sum(x['summarized_complete'] for x in direct) == 12
    OUT.write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps(result['summary'], indent=2))


if __name__ == '__main__':
    main()
