"""Full headless autocal benchmarks. Opt in with pytest -m heavy."""
import json
import os
from pathlib import Path
import shutil
import subprocess

import numpy as np
import pytest

from autocal.pose_fit import dataset_line_lengths, dataset_stretch
from autocal.tools.autocal_benchmark import benchmark_metrics
from autocal.tools.benchmark_history import ROOT, dependency_environment, git, measurement, results_text, run_autocal


def verify_extra_sensor_run(dataset):
    manifest = verify_applied_calibration(dataset)
    assert manifest['stop_reason'] in ('good-fit', 'stable-estimate', 'patience')
    assert set(manifest['firmware_final']) == {'M669', 'M666'}
    data = json.loads(dataset.read_text())
    assert data['config']['extra_sensors_enabled']
    points = [point for sweep in data['sweeps'] for point in sweep['data_points']]
    extras = [extra for point in points for extra in point.get('extra_sensor_ranges', [])]
    assert len(extras) == len(points)
    assert any(extra['accepted'] for extra in extras)
    assert all(extra['force_n'] > data['config']['force_tuning']['force_low_n'] for extra in extras)
    assert all(np.isfinite(extra['length_mu_mm']) for extra in extras)
    lengths = dataset_line_lengths(data)['lengths_mm']
    assert np.isfinite(lengths).sum() == 4 * len(points) + sum(extra['accepted'] for extra in extras)
    assert dataset_stretch(data) is not None
    assert all(point.get('extra_sensor_ranges') for point in data['sweeps'][0]['data_points'])
    assert all(sweep['metadata']['sensor_force_profile'] for sweep in data['sweeps'])
    for sweep in data['sweeps']:
        for point in sweep['data_points']:
            for extra in point['extra_sensor_ranges']:
                assert extra['anchor'] not in [*sweep['fixed_anchors'], sweep['drive_anchor'], sweep['sensor_anchor']]
    for service in manifest['owned_services']:
        with pytest.raises(ProcessLookupError):
            os.kill(service['pid'], 0)
    return dict(points=len(points), accepted=sum(extra['accepted'] for extra in extras),
                stop_reason=manifest['stop_reason'], simulated_s=manifest['simulated_s'])


def verify_applied_calibration(dataset):
    manifest = json.loads(dataset.with_suffix('.headless').joinpath('manifest.json').read_text())
    assert manifest['normal_completion'] and manifest['status'] == 'complete'
    assert any(command.startswith('M669 ') for command in manifest['applied_parameters'])
    return manifest


def run_benchmark(tmp_path, machine, verify=verify_applied_calibration):
    dataset = tmp_path / machine / 'sweeps.json'
    output = Path(os.environ.get('AUTOCAL_BENCHMARK_OUTPUT', ROOT / 'output/autocal-benchmark'))
    output.mkdir(parents=True, exist_ok=True)
    row = measurement(git('rev-parse', 'HEAD'), machine, ci=True)
    log = output / f'{machine}-{row["measurement_id"].replace(":", "-")}.log'
    try:
        row['environment'] = dependency_environment((ROOT / 'node_modules').resolve())
        result = run_autocal(machine, dataset, ROOT, log, timeout_s=55 * 60)
        assert result == 0, log.read_text(errors='replace')[-4000:]
        verify(dataset)
        row.update(benchmark_metrics(dataset, ROOT), status='ok')
    except Exception as error:
        status = 'timeout' if isinstance(error, subprocess.TimeoutExpired) else 'failed'
        details = log.read_text(errors='replace')[-4000:] if log.exists() else ''
        if 'physics step budget exhausted' in details.lower():
            status = 'step-cap'
        row.update(status=status, reason=str(error).splitlines()[0][:400] if str(error) else type(error).__name__)
        raise
    finally:
        if 'status' in row:
            name = f'{machine}.jsonl' if row['github_run_id'] else f'{row["measurement_id"]}.jsonl'
            (output / name).write_text(results_text([row]))
            print(machine, row['status'], {key: row[key] for key in ('accuracy_mm', 'total_sim_s') if key in row})
        diagnostics = output / 'diagnostics' / log.stem
        headless = dataset.with_suffix('.headless')
        for source in (headless / 'manifest.json', headless / 'physics.log', headless / 'rrf.log',
                       dataset.with_suffix('.full_auto.log'), dataset.with_suffix('.full_auto_log.jsonl')):
            if source.is_file():
                diagnostics.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source, diagnostics / source.name)


@pytest.mark.slow
@pytest.mark.heavy
def test_benchmark_hangprinter_3(tmp_path):
    run_benchmark(tmp_path, 'hangprinter_3')


@pytest.mark.slow
@pytest.mark.heavy
def test_benchmark_hangprinter_4(tmp_path):
    run_benchmark(tmp_path, 'hangprinter_4')


@pytest.mark.slow
@pytest.mark.heavy
def test_benchmark_hangprinter_5(tmp_path):
    run_benchmark(tmp_path, 'hangprinter_5', verify_extra_sensor_run)


@pytest.mark.slow
@pytest.mark.heavy
def test_benchmark_hangprinter_5_pulley_reduced(tmp_path):
    run_benchmark(tmp_path, 'hangprinter_5_pulley_reduced', verify_extra_sensor_run)
