import subprocess

import pytest

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
    contributions = root / 'contributions.jsonl'
    monkeypatch.setattr(ci, 'CONTRIBUTIONS', contributions)

    def git(*args):
        return history.git(*args, cwd=root)

    git('init', '--quiet')
    git('config', 'user.name', 'Test')
    git('config', 'user.email', 'test@example.com')
    remote = tmp_path / 'remote.git'
    git('init', '--quiet', '--bare', str(remote))
    git('remote', 'add', 'origin', str(remote))
    scorer = root / 'autocal/tools/autocal_benchmark.py'
    scorer.parent.mkdir(parents=True)
    scorer.write_text('# scorer\n')
    entry = root / 'autocal/autocal.py'
    entry.write_text('pass\n')
    git('add', '.')
    git('commit', '-qm', 'Baseline')
    first = git('rev-parse', 'HEAD')
    machine = history.MACHINES[0]
    old = dict(history.measurement(first, machine), status='ok', accuracy_mm=0.95, total_sim_s=100)
    history.write_results(contributions, [old])
    artifacts = tmp_path / 'artifacts'
    ci.collect(artifacts)
    parent, archived = ci.remote_history()
    assert archived == [old]
    assert git('ls-tree', '--name-only', parent).splitlines() == ['results.jsonl', 'summary.md']

    entry.write_text('pass  # changed inputs\n')
    git('commit', '-qam', 'Changed inputs')
    second = git('rev-parse', 'HEAD')
    monkeypatch.setenv('GITHUB_RUN_ID', '42')
    monkeypatch.setenv('GITHUB_RUN_ATTEMPT', '1')
    new = dict(history.measurement(second, machine, ci=True), status='ok', accuracy_mm=1.1, total_sim_s=108)
    assert ci.should_run(new, archived, first)
    history.write_results(artifacts / 'original' / 'result.jsonl', [new])
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
    monkeypatch.setattr(ci, 'CONTRIBUTIONS', root / 'contributions.jsonl')
    git('config', 'user.name', 'Test')
    git('config', 'user.email', 'test@example.com')
    history.write_results(ci.CONTRIBUTIONS, [old, repeat])
    ci.collect(artifacts)
    assert git('rev-parse', '--is-shallow-repository') == 'false'
    tip, archived = ci.remote_history()
    assert {row['measurement_id'] for row in archived} == {r['measurement_id'] for r in (old, new, repeat, failure)}
    assert '2/3' in git('show', f'{tip}:summary.md')
    assert '1 mm target' in capsys.readouterr().out
    ci.collect(artifacts)
    assert ci.remote_history()[0] == tip
    assert not ci.should_run(new, [row for row in archived if row != failure], second)
    assert ci.should_run(new, archived, second)
    with pytest.raises(ValueError, match='Fingerprint mismatch'):
        list(history.verified_results([dict(new, fingerprint=old['fingerprint'])]))
    with pytest.raises(subprocess.CalledProcessError):
        list(history.verified_results([dict(new, commit='0' * 40)]))
