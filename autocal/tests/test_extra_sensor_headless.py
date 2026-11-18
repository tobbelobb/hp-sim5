"""Full production-path regression; opt in with pytest -m heavy."""
from datetime import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import pytest

from autocal.dataset_roles import normalize_dataset_point_roles
from autocal.pose_fit import (dataset_line_lengths, dataset_stretch, fit_pose_least_squares,
                              shift_dataset_home_offsets, HOME_OFFSET_PRIOR_SIGMA_MM)
from autocal.rough_bootstrap import assess_rough_geometry
from autocal.tools.sanity_battery import anchor_errors

ROOT = Path(__file__).resolve().parents[2]


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


def calibration_benchmark(dataset, started_at):
    """Score a fresh stock run including tuning and bootstrap. Truth is used only for scoring."""
    data = json.loads(dataset.read_text())
    rough = json.loads(dataset.with_suffix('.rough_bootstrap.json').read_text())
    manifest = json.loads(dataset.with_suffix('.headless').joinpath('manifest.json').read_text())
    error = anchor_errors(manifest['firmware_initial']['M669'], manifest['firmware_final']['M669'])
    truth = np.asarray(error['reference_mm'])
    rows = map(json.loads, dataset.with_suffix('.full_auto_log.jsonl').read_text().splitlines())
    first_good_s = float('inf')
    for row in rows:
        if 'selected_run' not in row:
            continue
        run = next(r for r in row['runs'] if r['id'] == row['selected_run'])
        if np.max(np.abs(run['metrics']['pose_fit_anchors'] - truth)) <= 2.0:
            first_good_s = datetime.fromisoformat(row['timestamp']).timestamp() - started_at
            break

    spans = []
    for sweep in rough['sweeps'][:3]:
        for sensor in (sweep['sensor_anchor'], sweep['drive_anchor']):
            points = [p for p in sweep['data_points']
                      if p.get('source_sensor_anchor', sweep['sensor_anchor']) == sensor]
            assert len(points) == 10
            spans.append((points[-1]['clock_time_ms'] - points[0]['clock_time_ms']) / 1000)
    # Fit disjoint alternating points in both directions, keeping the checked
    # initialization common. This measures subsample agreement, including the
    # effect of the slightly different poses. It is independent of ground truth.
    normalize_dataset_point_roles(data)
    normalize_dataset_point_roles(rough)
    seed, check = assess_rough_geometry(rough)
    assert check['ready']
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
    sigma = [np.asarray(p['sigma']) * data['config']['mm_per_degree']
             for s in data['sweeps'] for p in s['data_points']]
    return dict(slowest_initial_arc_s=max(spans), collection_sim_s=manifest['simulated_s'],
                first_good_wall_s=first_good_s, encoder_sigma_p95_mm=float(np.percentile(sigma, 95)),
                split_fit_difference_mm=float(np.max(np.abs(estimates[0] - estimates[1]))),
                applied_error_mm=error['max_abs_coordinate_mm'])


@pytest.mark.slow
@pytest.mark.heavy
@pytest.mark.parametrize('machine', ['hangprinter_5', 'hangprinter_5_pulley_reduced'])
def test_fresh_hp5_extra_sensor_full_calibration(tmp_path, machine):
    dataset = tmp_path / machine / 'sweeps.json'
    env = {**os.environ, 'OMP_NUM_THREADS': '1', 'OPENBLAS_NUM_THREADS': '1',
           'HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS': '6000000'}
    started_at = time.time()
    with (tmp_path / 'autocal.log').open('w') as log:
        result = subprocess.run([sys.executable, 'autocal/autocal.py', '--headless-sim',
            '--machine-type', machine, '--dataset', str(dataset), '--recording-mode', 'none',
            '--patience', '8'], cwd=ROOT, env=env, stdout=log, stderr=log, timeout=1800)
    assert result.returncode == 0, (tmp_path / 'autocal.log').read_text()[-4000:]
    verified = verify_extra_sensor_run(dataset)
    assert verified['stop_reason'] == 'good-fit'
    assert len(json.loads(dataset.read_text())['sweeps']) <= 6
    metrics = calibration_benchmark(dataset, started_at)
    print(machine, metrics)
    limits = dict(slowest_initial_arc_s=45, collection_sim_s=5000, first_good_wall_s=900,
                  encoder_sigma_p95_mm=0.01, split_fit_difference_mm=3, applied_error_mm=3)
    for key, limit in limits.items():
        assert 0 <= metrics[key] <= limit, (key, limit, metrics)
