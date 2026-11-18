#!/usr/bin/env python3
"""Plan benchmark jobs and publish their measurements using only the stdlib."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from autocal.tools.benchmark_history import (  # noqa: E402
    BACKFILL_DIR, HISTORY_REF, MACHINES, METRICS, ROOT, execution_fingerprint, fingerprint, git,
    markdown_summary, measurement,
    merge_results, metric_medians, protocol, read_results, regressions, results_text, verified_results,
)


def remote_history():
    ref = subprocess.run(['git', 'ls-remote', '--exit-code', 'origin', HISTORY_REF],
                         cwd=ROOT, capture_output=True, text=True)
    if ref.returncode == 2:
        return None, []
    ref.check_returncode()
    git('fetch', '--no-tags', 'origin', HISTORY_REF)
    head = git('rev-parse', 'FETCH_HEAD')
    return head, [json.loads(line) for line in git('show', f'{head}:results.jsonl').splitlines()]


def should_run(candidate, rows, parent):
    if (parent is None or fingerprint(parent, candidate['machine']) != candidate['fingerprint']
            or execution_fingerprint(parent) != candidate['execution_fingerprint']):
        return True
    # Installed dependencies are recorded after setup. The fingerprint includes the
    # dependency declarations so a change to those declarations still triggers a run.
    matching = [row for row in rows if row.get('github_run_id') and row['machine'] == candidate['machine']
                and row.get('fingerprint_version') == candidate['fingerprint_version']
                and row['fingerprint'] == candidate['fingerprint']
                and row.get('execution_fingerprint') == candidate['execution_fingerprint']
                and row.get('settings') == candidate['settings'] and row.get('scorer') == candidate['scorer']]
    last = max(matching, key=lambda row: row['recorded_at'], default={})
    return last.get('status') != 'ok'


def metrics_table(current, baseline=None):
    def value(row, key):
        reading = row.get(key)
        return f'{reading:.3f}' if reading is not None else 'Unavailable'

    lines = ['| Metric | Current | Baseline median |', '|---|---:|---:|']
    for key in METRICS:
        prior = value(baseline, key) if baseline is not None else '—'
        lines.append(f'| `{key}` | {value(current, key)} | {prior} |')
    return lines


def review_measurement(current, rows):
    machine, commit = current['machine'], current['commit']
    heading = f'### Autocal {machine} at {commit[:12]}'
    if current['status'] != 'ok':
        return [heading, '', f'Benchmark status: **{current["status"]}**. See the benchmark artifact.']
    candidates = [row for row in rows if row['machine'] == machine and row['status'] == 'ok']
    if not candidates:
        return [heading, '', 'No successful historical measurements are available.', '', *metrics_table(current)]
    # The matrix jobs use a shallow checkout. Only the commit graph is needed here.
    if git('rev-parse', '--is-shallow-repository') == 'true':
        git('fetch', '--unshallow', '--filter=tree:0', '--no-tags', 'origin', commit)
    ancestors = git('rev-list', '--first-parent', commit).splitlines()[1:]
    by_commit = {}
    for row in candidates:
        by_commit.setdefault(row['commit'], []).append(row)
    current_protocol = protocol(current)
    compatible = next((row for ancestor in ancestors for row in by_commit.get(ancestor, [])
                       if protocol(row) == current_protocol), None)
    prior = compatible or next(
        (by_commit[ancestor][-1] for ancestor in ancestors if ancestor in by_commit), None)
    if prior is None:
        return [heading, '', 'No successful first-parent baseline is available.', '', *metrics_table(current)]
    prior_protocol = protocol(prior)
    baseline = [row for row in by_commit[prior['commit']] if protocol(row) == prior_protocol]
    medians = metric_medians(baseline)
    findings = regressions(medians, current)
    reference = f'Baseline: {prior["commit"][:12]} ({len(baseline)} successful measurements).'
    caveat = '' if compatible else 'The benchmark protocols differ or are unknown. Treat this comparison as exploratory.'
    lines = [heading, '', reference, caveat, '', *metrics_table(current, medians), '']
    for finding in findings.values():
        print(f'::warning::{machine} {commit[:12]} vs {prior["commit"][:12]}: {finding}' +
              (' (mixed or unknown protocol)' if caveat else ''))
        lines.append(f'- {finding}')
    return lines if findings else lines + ['No review thresholds crossed.']


def history_commit(parent, rows):
    blobs = []
    for name, content in [('results.jsonl', results_text(rows)), ('summary.md', markdown_summary(rows)),
                          ('by-fingerprint.md', markdown_summary(rows, by_fingerprint=True))]:
        blob = subprocess.run(['git', 'hash-object', '-w', '--stdin'], cwd=ROOT, text=True,
                              input=content, capture_output=True, check=True).stdout.strip()
        blobs.append(f'100644 blob {blob}\t{name}\n')
    tree = subprocess.run(['git', 'mktree'], cwd=ROOT, input=''.join(blobs),
                          text=True, capture_output=True, check=True).stdout.strip()
    parents = ['-p', parent] if parent else []
    return git('commit-tree', tree, *parents, '-m', 'Update autocal benchmark history')


def collect(artifacts):
    incoming = [row for path in sorted(artifacts.glob('*/*.jsonl')) for row in read_results(path)]
    contributed = read_results(BACKFILL_DIR / 'results.jsonl') + read_results(BACKFILL_DIR / 'retry.jsonl')
    if not incoming and not contributed:
        return
    parent, previous = remote_history()
    # Archived rows have full SHAs. Legacy backfill files can retain abbreviated SHAs.
    widths = {len(row['commit']) for row in contributed + incoming}
    archived = {json.dumps(dict(row, commit=row['commit'][:width]), sort_keys=True)
                for row in previous for width in widths}
    if all(json.dumps(row, sort_keys=True) in archived for row in contributed + incoming):
        return
    if git('rev-parse', '--is-shallow-repository') == 'true':
        git('fetch', '--unshallow', '--no-tags', 'origin', git('rev-parse', 'HEAD'))
    contributed = list(verified_results(contributed))
    incoming = list(verified_results(incoming))
    for attempt in range(5):
        rows = merge_results(previous, contributed, incoming)
        if rows == previous:
            break
        commit = history_commit(parent, rows)
        push = subprocess.run(['git', 'push', 'origin', f'{commit}:{HISTORY_REF}'], cwd=ROOT)
        if push.returncode == 0:
            break
        # Another collector may have published since our fetch. Retry from its tip.
        next_parent, previous = remote_history()
        if attempt == 4 or next_parent == parent:
            push.check_returncode()
        parent = next_parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    actions = parser.add_subparsers(dest='action', required=True)
    check = actions.add_parser('check')
    check.add_argument('machine', choices=MACHINES)
    missing = actions.add_parser('missing')
    missing.add_argument('machine', choices=MACHINES)
    missing.add_argument('output', type=Path)
    review = actions.add_parser('review')
    review.add_argument('result', type=Path)
    merge = actions.add_parser('collect')
    merge.add_argument('artifacts', type=Path)
    args = parser.parse_args()
    if args.action == 'collect':
        collect(args.artifacts)
        return
    if args.action == 'review':
        current, = read_results(args.result)
        rows = remote_history()[1] if current['status'] == 'ok' else []
        summary = review_measurement(current, rows)
        with open(os.environ['GITHUB_STEP_SUMMARY'], 'a') as output:
            output.write('\n'.join(summary) + '\n')
        return
    commit = git('rev-parse', 'HEAD')
    row = measurement(commit, args.machine, ci=True)
    if args.action == 'check':
        parent = git('rev-list', '--parents', '-n', '1', commit).split()[1:2]
        run = (os.environ.get('GITHUB_RUN_ATTEMPT', '1') != '1'
               or should_run(row, remote_history()[1], parent[0] if parent else None))
        with open(os.environ['GITHUB_OUTPUT'], 'a') as output:
            output.write(f'run={str(run).lower()}\n')
        if not run:
            message = f'{args.machine}: an earlier CI run measured the unchanged inputs.'
            print(message)
            if summary_path := os.environ.get('GITHUB_STEP_SUMMARY'):
                with open(summary_path, 'a') as output:
                    output.write(message + '\n')
    else:
        path = args.output / f'{args.machine}.jsonl'
        if not path.exists():
            path.parent.mkdir(parents=True, exist_ok=True)
            row.update(status='setup-failed', reason='Benchmark job failed before recording a result')
            path.write_text(results_text([row]))


if __name__ == '__main__':
    main()
