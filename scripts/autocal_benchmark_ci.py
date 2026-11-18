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
    CONTRIBUTIONS, HISTORY_REF, MACHINES, ROOT, fingerprint, git, markdown_summary, measurement,
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
    if parent is None or fingerprint(parent, candidate['machine']) != candidate['fingerprint']:
        return True
    matching = [row for row in rows if row.get('github_run_id') and row['machine'] == candidate['machine']
                and row['fingerprint'] == candidate['fingerprint'] and protocol(row) == protocol(candidate)]
    last = max(matching, key=lambda row: row['recorded_at'], default={})
    return last.get('status') != 'ok'


def report_regressions(incoming, rows):
    for commit in sorted({row['commit'] for row in incoming}):
        ancestors = git('rev-list', '--first-parent', commit).splitlines()[1:]
        for current in (row for row in incoming if row['commit'] == commit and row['status'] == 'ok'):
            comparable = [row for row in rows if row['machine'] == current['machine'] and row['status'] == 'ok'
                          and protocol(row) == protocol(current)]
            same = [row for row in comparable if row['commit'] == commit]
            baseline = {}
            for row in comparable:
                if row['fingerprint'] != current['fingerprint']:
                    baseline.setdefault(row['commit'], []).append(row)
            previous = next((old for old in ancestors if old in baseline), None)
            if previous is None:
                continue
            for finding in regressions(metric_medians(baseline[previous]), metric_medians(same)).values():
                print(f'::warning::{current["machine"]} {commit[:12]} vs {previous[:12]}: {finding}')


def history_commit(parent, rows):
    blobs = []
    for name, content in [('results.jsonl', results_text(rows)), ('summary.md', markdown_summary(rows))]:
        blob = subprocess.run(['git', 'hash-object', '-w', '--stdin'], cwd=ROOT, text=True,
                              input=content, capture_output=True, check=True).stdout.strip()
        blobs.append(f'100644 blob {blob}\t{name}\n')
    tree = subprocess.run(['git', 'mktree'], cwd=ROOT, input=''.join(blobs),
                          text=True, capture_output=True, check=True).stdout.strip()
    parents = ['-p', parent] if parent else []
    return git('commit-tree', tree, *parents, '-m', 'Update autocal benchmark history')


def collect(artifacts):
    incoming = [row for path in sorted(artifacts.rglob('*.jsonl')) for row in read_results(path)]
    contributed = read_results(CONTRIBUTIONS)
    if not incoming and not contributed:
        return
    parent, previous = remote_history()
    archived = {json.dumps(row, sort_keys=True) for row in previous}
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
    report_regressions(incoming, rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    actions = parser.add_subparsers(dest='action', required=True)
    check = actions.add_parser('check')
    check.add_argument('machine', choices=MACHINES)
    missing = actions.add_parser('missing')
    missing.add_argument('machine', choices=MACHINES)
    missing.add_argument('output', type=Path)
    merge = actions.add_parser('collect')
    merge.add_argument('artifacts', type=Path)
    args = parser.parse_args()
    if args.action == 'collect':
        collect(args.artifacts)
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
            print(f'{args.machine}: an earlier CI run measured the unchanged inputs.')
    else:
        path = args.output / f'{args.machine}.jsonl'
        if not path.exists():
            path.parent.mkdir(parents=True, exist_ok=True)
            row.update(status='setup-failed', reason='Benchmark job failed before recording a result')
            path.write_text(results_text([row]))


if __name__ == '__main__':
    main()
