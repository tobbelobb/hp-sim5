import json
import subprocess

import pytest

from autocal.tests import test_autocal_benchmark as benchmark
from autocal.tools import benchmark_history as history
from scripts import autocal_benchmark_ci as ci


def test_regression_thresholds_and_median():
    old = dict(accuracy_mm=0.99, convergence_sim_s=100, total_sim_s=100, precision_mm=4)
    current = dict(accuracy_mm=1.01, convergence_sim_s=None, total_sim_s=105, precision_mm=4.3)
    findings = history.regressions(old, current)
    assert set(findings) == {'accuracy_mm', 'convergence_sim_s'}
    assert '1 mm target' in findings['accuracy_mm']
    assert set(history.regressions(old, dict(current, total_sim_s=106, precision_mm=4.5))) == set(history.METRICS)
    assert history.metric_medians([old, current, current]) == current


def test_collector_bootstraps_and_preserves_repeats(tmp_path, monkeypatch, capsys):
    monkeypatch.delenv('GITHUB_RUN_ID', raising=False)
    monkeypatch.delenv('GITHUB_RUN_ATTEMPT', raising=False)
    root = tmp_path / 'source'
    root.mkdir()
    monkeypatch.setattr(history, 'ROOT', root)
    monkeypatch.setattr(ci, 'ROOT', root)
    contributions = root / 'backfill' / 'results.jsonl'
    monkeypatch.setattr(ci, 'BACKFILL_DIR', contributions.parent)

    def git(*args):
        return history.git(*args, cwd=root)

    git('init', '--quiet')
    git('config', 'user.name', 'Test')
    git('config', 'user.email', 'test@example.com')
    remote = tmp_path / 'remote.git'
    git('init', '--quiet', '--bare', str(remote))
    history.git('config', 'uploadpack.allowFilter', 'true', cwd=remote)
    git('remote', 'add', 'origin', str(remote))
    scorer = root / 'autocal/tools/autocal_benchmark.py'
    scorer.parent.mkdir(parents=True)
    scorer.write_text('# scorer\n')
    entry = root / 'autocal/autocal.py'
    entry.write_text('pass\n')
    catalog = root / history.CATALOG
    catalog.parent.mkdir()
    catalog.write_text(json.dumps({'machines': [], 'scenes': {'hp3': {'autocal_solver_iterations': 20}}}))
    git('add', '.')
    git('commit', '-qm', 'Baseline')
    first = git('rev-parse', 'HEAD')
    machine = history.MACHINES[0]
    old = dict(history.measurement(first, machine), status='ok', accuracy_mm=0.95, total_sim_s=100)
    history.write_results(contributions, [dict(old, commit=first[:12])])
    artifacts = tmp_path / 'artifacts'
    ci.collect(artifacts)
    parent, archived = ci.remote_history()
    assert archived == [old]
    assert git('ls-tree', '--name-only', parent).splitlines() == ['by-fingerprint.md', 'results.jsonl', 'summary.md']

    catalog.write_text(json.dumps({'machines': [], 'scenes': {'hp3': {'autocal_solver_iterations': 40}}}))
    git('commit', '-qam', 'Changed inputs')
    second = git('rev-parse', 'HEAD')
    monkeypatch.setenv('GITHUB_RUN_ID', '42')
    monkeypatch.setenv('GITHUB_RUN_ATTEMPT', '1')
    new = dict(history.measurement(second, machine, ci=True), status='ok', accuracy_mm=1.1, total_sim_s=108)
    assert ci.should_run(new, archived, first)
    legacy = history.fingerprint(first, machine, 1)
    assert legacy == history.fingerprint(second, machine, 1)
    assert old['fingerprint'] != new['fingerprint']
    legacy_row = dict(old, fingerprint=legacy)
    del legacy_row['fingerprint_version']
    list(history.verified_results([legacy_row]))
    history.write_results(artifacts / 'original' / 'result.jsonl', [new])
    diagnostic = artifacts / 'original' / 'diagnostics' / 'sweeps.full_auto_log.jsonl'
    diagnostic.parent.mkdir()
    diagnostic.write_text('{"event": "summary"}\n')
    failure = dict(new, measurement_id='ci:42:2:hangprinter_3', status='timeout', recorded_at='9999')
    history.write_results(artifacts / 'rerun' / 'result.jsonl', [failure])

    monkeypatch.delenv('GITHUB_RUN_ID')
    repeat = dict(history.measurement(second, machine, repeat=2), status='ok', accuracy_mm=1.15, total_sim_s=112)
    git('push', '-q', 'origin', 'HEAD:refs/heads/main')
    shallow = tmp_path / 'shallow'
    git('clone', '-q', '--depth=1', '--branch=main', remote.as_uri(), str(shallow))
    root = shallow
    monkeypatch.setattr(history, 'ROOT', root)
    monkeypatch.setattr(ci, 'ROOT', root)
    monkeypatch.setattr(ci, 'BACKFILL_DIR', root / 'backfill')
    git('config', 'user.name', 'Test')
    git('config', 'user.email', 'test@example.com')
    history.write_results(ci.BACKFILL_DIR / 'results.jsonl', [dict(old, commit=first[:12]), repeat])
    retry = dict(old, measurement_id='legacy-retry', status='retry')
    history.write_results(ci.BACKFILL_DIR / 'retry.jsonl', [dict(retry, commit=first[:12])])
    assert git('rev-parse', '--is-shallow-repository') == 'true'
    assert '1 mm target' in '\n'.join(ci.review_measurement(new, ci.remote_history()[1]))
    ci.collect(artifacts)
    assert git('rev-parse', '--is-shallow-repository') == 'false'
    tip, archived = ci.remote_history()
    assert {row['measurement_id'] for row in archived} == {r['measurement_id'] for r in (old, new, repeat, failure, retry)}
    assert '2/3' in git('show', f'{tip}:summary.md')
    assert '1 mm target' in '\n'.join(ci.review_measurement(new, archived))
    assert '::warning::' in capsys.readouterr().out
    mixed = dict(new, environment={'python': '3.12.1'})
    assert 'exploratory' in '\n'.join(ci.review_measurement(mixed, archived))
    ci.collect(artifacts)
    assert ci.remote_history()[0] == tip
    assert not ci.should_run(new, [row for row in archived if row != failure], second)
    assert ci.should_run(new, archived, second)
    with pytest.raises(ValueError, match='Fingerprint mismatch'):
        list(history.verified_results([dict(new, fingerprint=old['fingerprint'])]))
    with pytest.raises(subprocess.CalledProcessError):
        list(history.verified_results([dict(new, commit='0' * 40)]))

    parent = second
    for path in ('autocal/tests/test_autocal_benchmark.py', 'package.json', '.github/workflows/nodejs.yml'):
        changed = root / path
        changed.parent.mkdir(parents=True, exist_ok=True)
        changed.write_text('changed execution input\n')
        git('add', path)
        git('commit', '-qm', f'Change {path}')
        commit = git('rev-parse', 'HEAD')
        candidate = history.measurement(commit, machine, ci=True)
        assert candidate['fingerprint'] == new['fingerprint']
        assert ci.should_run(candidate, archived, parent)
        parent = commit


def test_summary_keeps_a_reverted_input_at_its_commit():
    rows = [dict(commit=sha * 40, date=f'2026-11-0{i}', machine=history.MACHINES[0], fingerprint=fp,
                 status='ok', accuracy_mm=error, subject=subject)
            for i, (sha, fp, error, subject) in enumerate(
                [('a', 'X', 1, 'Initial'), ('b', 'Y', 2, 'Regression'), ('c', 'X', 1, 'Revert')], 1)]
    summary = history.markdown_summary(rows)
    assert summary.index('Revert') < summary.index('Regression') < summary.index('Initial')
    aggregates = history.markdown_summary(rows, by_fingerprint=True)
    assert '2/2' in aggregates
    assert 'Revert' not in aggregates
    for text in (summary, aggregates):
        assert {line.count('|') for line in text.splitlines() if line.startswith('|')} == {13}


def test_scorer_fingerprint_includes_imported_source(tmp_path):
    entry = tmp_path / history.SCORER_ENTRY
    entry.parent.mkdir(parents=True)
    entry.write_text('from autocal.pose_fit import score\n')
    imported = tmp_path / 'autocal/pose_fit.py'
    imported.write_text('score = 1\n')
    original = history.scorer_fingerprint(tmp_path)
    imported.write_text('score = 2\n')
    assert history.scorer_fingerprint(tmp_path) != original


def test_failed_benchmark_retains_available_diagnostics(tmp_path, monkeypatch):
    output = tmp_path / 'artifacts'
    monkeypatch.setenv('AUTOCAL_BENCHMARK_OUTPUT', str(output))
    monkeypatch.setattr(benchmark, 'measurement', lambda *args, **kwargs: {
        'measurement_id': 'simulation', 'github_run_id': '42',
    })
    monkeypatch.setattr(benchmark, 'dependency_environment', lambda _: {})
    files = ['manifest.json', 'physics.log', 'rrf.log', 'sweeps.full_auto.log', 'sweeps.full_auto_log.jsonl']

    def timed_out(machine, dataset, repo, log, timeout_s):
        headless = dataset.with_suffix('.headless')
        headless.mkdir(parents=True)
        for name in files[:3]:
            (headless / name).write_text(name)
        for name in files[3:]:
            (dataset.parent / name).write_text(name)
        raise subprocess.TimeoutExpired('autocal', timeout_s)

    monkeypatch.setattr(benchmark, 'run_autocal', timed_out)
    machine = history.MACHINES[0]
    with pytest.raises(subprocess.TimeoutExpired):
        benchmark.run_benchmark(tmp_path, machine)
    assert history.read_results(output / f'{machine}.jsonl')[0]['status'] == 'timeout'
    assert {path.name for path in (output / 'diagnostics').rglob('*') if path.is_file()} == set(files)
