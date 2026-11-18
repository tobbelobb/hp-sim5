"""Full headless autocal benchmark per machine; opt in with pytest -m heavy.

Each run appends its metrics to autocal_benchmark_history.jsonl. Commit that file
so total time, convergence time, precision and accuracy can be tracked from commit
to commit. A metric that got worse than the previous commit's entry for the same
machine raises a warning; only a broken calibration run fails the test.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import warnings

import numpy as np
import pytest

from autocal.dataset_roles import normalize_dataset_point_roles
from autocal.pose_fit import (dataset_line_lengths, dataset_stretch, fit_pose_least_squares,
                              shift_dataset_home_offsets, HOME_OFFSET_PRIOR_SIGMA_MM)
from autocal.tools.sanity_battery import anchor_errors

ROOT = Path(__file__).resolve().parents[2]
HISTORY = Path(__file__).with_name('autocal_benchmark_history.jsonl')
CONVERGED_MM = 2.0
# Worsening beyond these margins warns: relative for times, absolute mm for errors.
TIME_MARGIN = 0.02
ERROR_MARGIN_MM = 0.1


def verify_extra_sensor_run(dataset):
    manifest = json.loads(dataset.with_suffix('.headless').joinpath('manifest.json').read_text())
    assert manifest['normal_completion'] and manifest['status'] == 'complete'
    assert manifest['stop_reason'] in ('good-fit', 'stable-estimate', 'patience')
    assert any(command.startswith('M669 ') for command in manifest['applied_parameters'])
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


def split_fit_difference_mm(data, seed):
    """Refit disjoint alternating points from a common seed; largest anchor disagreement."""
    normalize_dataset_point_roles(data)
    estimates = []
    options = dict(home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM)
    for parity in (0, 1):
        subset = dict(data, sweeps=[dict(s, data_points=[p for p in s['data_points']
                              if p['step_index'] % 2 == parity]) for s in data['sweeps']])
        first = fit_pose_least_squares(subset, seed, fit_home_offsets=True, **options)
        shifted = shift_dataset_home_offsets(subset, first['home_offsets_mm'])
        fit = fit_pose_least_squares(shifted, first['anchors'], fit_home_offsets=False, **options)
        assert first['success'] and fit['success']
        estimates.append(fit['anchors'])
    return float(np.max(np.abs(estimates[0] - estimates[1])))


def benchmark_metrics(dataset):
    """Truth from the simulator's firmware config is used only for scoring."""
    data = json.loads(dataset.read_text())
    manifest = json.loads(dataset.with_suffix('.headless').joinpath('manifest.json').read_text())
    error = anchor_errors(manifest['firmware_initial']['M669'], manifest['firmware_final']['M669'])
    truth = np.asarray(error['reference_mm'])
    rows = [json.loads(line) for line in
            dataset.with_suffix('.full_auto_log.jsonl').read_text().splitlines()]
    convergence_s = None
    for row in rows:
        if 'selected_run' not in row:
            continue
        run = next(r for r in row['runs'] if r['id'] == row['selected_run'])
        if np.max(np.abs(np.asarray(run['metrics']['pose_fit_anchors']) - truth)) <= CONVERGED_MM:
            points = [p for s in data['sweeps'][:row['num_sweeps']] for p in s['data_points']]
            convergence_s = max(p['clock_time_ms'] for p in points) / 1000
            break
    return dict(total_sim_s=manifest['simulated_s'], convergence_sim_s=convergence_s,
                precision_mm=split_fit_difference_mm(data, np.asarray(error['fitted_mm'])),
                accuracy_mm=error['max_abs_coordinate_mm'], sweeps=len(data['sweeps']),
                stop_reason=manifest['stop_reason'], wall_s=manifest['wall_s'])


def git_revision():
    rev = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT, capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(['git', 'status', '--porcelain', '--untracked-files=no', '--', '.',
                            f':!{HISTORY.relative_to(ROOT)}'], cwd=ROOT, capture_output=True, text=True).stdout
    return rev + ('-dirty' if dirty.strip() else '')


def regressions(previous, metrics):
    found = []
    for key in ('total_sim_s', 'convergence_sim_s'):
        old, new = previous['metrics'].get(key), metrics[key]
        if new is None and old is not None:
            found.append(f'{key}: no longer converges within {CONVERGED_MM} mm (was {old:.0f} s)')
        elif None not in (old, new) and new > old * (1 + TIME_MARGIN):
            found.append(f'{key}: {old:.0f} s -> {new:.0f} s')
    for key in ('precision_mm', 'accuracy_mm'):
        old, new = previous['metrics'][key], metrics[key]
        if new > old + ERROR_MARGIN_MM:
            found.append(f'{key}: {old:.3f} mm -> {new:.3f} mm')
    return found


def record_and_compare(machine, metrics):
    history = [json.loads(line) for line in HISTORY.read_text().splitlines()] if HISTORY.exists() else []
    revision = git_revision()
    previous = next((h for h in reversed(history)
                     if h['machine'] == machine and h['revision'] != revision), None)
    with HISTORY.open('a') as f:
        f.write(json.dumps(dict(machine=machine, revision=revision, metrics=metrics)) + '\n')
    print(machine, revision, metrics)
    if previous is None:
        return
    for message in regressions(previous, metrics):
        warnings.warn(f'AUTOCAL REGRESSION {machine} since {previous["revision"][:12]}: {message}')


def run_benchmark(tmp_path, machine):
    dataset = tmp_path / machine / 'sweeps.json'
    env = {**os.environ, 'OMP_NUM_THREADS': '1', 'OPENBLAS_NUM_THREADS': '1',
           'HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS': '6000000'}
    with (tmp_path / 'autocal.log').open('w') as log:
        result = subprocess.run([sys.executable, 'autocal/autocal.py', '--headless-sim',
            '--machine-type', machine, '--dataset', str(dataset), '--recording-mode', 'none',
            '--patience', '8'], cwd=ROOT, env=env, stdout=log, stderr=log, timeout=3600)
    assert result.returncode == 0, (tmp_path / 'autocal.log').read_text()[-4000:]
    record_and_compare(machine, benchmark_metrics(dataset))
    return dataset


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
    verify_extra_sensor_run(run_benchmark(tmp_path, 'hangprinter_5'))


@pytest.mark.slow
@pytest.mark.heavy
def test_benchmark_hangprinter_5_pulley_reduced(tmp_path):
    verify_extra_sensor_run(run_benchmark(tmp_path, 'hangprinter_5_pulley_reduced'))
