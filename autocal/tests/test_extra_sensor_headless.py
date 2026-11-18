"""Full production-path regression; opt in with pytest -m heavy."""
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from autocal.pose_fit import dataset_line_lengths, dataset_stretch

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


@pytest.mark.slow
@pytest.mark.heavy
@pytest.mark.parametrize('machine', ['hangprinter_5', 'hangprinter_5_pulley_reduced'])
def test_fresh_hp5_extra_sensor_full_calibration(tmp_path, machine):
    dataset = tmp_path / machine / 'sweeps.json'
    env = {**os.environ, 'OMP_NUM_THREADS': '1', 'OPENBLAS_NUM_THREADS': '1',
           'HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS': '6000000'}
    with (tmp_path / 'autocal.log').open('w') as log:
        result = subprocess.run([sys.executable, 'autocal/autocal.py', '--headless-sim',
            '--machine-type', machine, '--dataset', str(dataset), '--recording-mode', 'none',
            '--patience', '8'], cwd=ROOT, env=env, stdout=log, stderr=log, timeout=1800)
    assert result.returncode == 0, (tmp_path / 'autocal.log').read_text()[-4000:]
    verified = verify_extra_sensor_run(dataset)
    assert verified['stop_reason'] == 'good-fit'
    assert len(json.loads(dataset.read_text())['sweeps']) <= 6
