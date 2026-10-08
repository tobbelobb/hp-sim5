import json
import os
from pathlib import Path
import subprocess
import sys
import time
import signal

import pytest

from autocal import autocal as ac
from autocal.headless_sim import HeadlessSimulation

ROOT = Path(__file__).resolve().parents[2]


def test_headless_implies_sim_and_keeps_fitting_options(tmp_path, monkeypatch):
    captured = {}

    def run(args, spool_opts, machine_type, full_auto_runs, collector_args, headless):
        captured.update(sim=args.sim, machine=machine_type, radii=spool_opts['find_radii'],
                        bounds=spool_opts['r0_bounds'], no_collect=args.no_collect, headless=headless)
        return 0

    monkeypatch.setattr(ac, '_run_full_auto', run)
    assert ac.main(['--headless-sim', '--machine-type', 'hangprinter_3', '--no-collect',
                    '--dataset', str(tmp_path / 'sweeps.json'), '--find-radii', 'global',
                    '--base-radii', '30', '--r0-bounds', '39,40']) == 0
    assert captured == dict(sim=True, machine='hangprinter_4', radii='global', bounds=(39., 40.),
                            no_collect=True, headless=None)


@pytest.mark.parametrize('flags', [ ['--firmware', 'klipper'], ['--machine-type', 'hangprinter_5'] ])
def test_headless_rejects_unavailable_backends(flags):
    with pytest.raises(SystemExit):
        ac.main(['--headless-sim', *flags])


def test_failed_lifecycle_retains_clock_error_and_partial_points(tmp_path):
    from argparse import Namespace
    args = Namespace(machine_type='hangprinter_3', find_radii='global', find_buildup_factor='off',
                     rrf_config=None, config=None, dataset=tmp_path / 'sweeps.json', keep_sim_alive=False)
    service = HeadlessSimulation(args, [])
    partial = tmp_path / 'sweeps.json.partial-points.jsonl'
    partial.write_text('{"point":{"l_drive":1}}\n')
    (service.directory / 'clock.json').write_text(json.dumps(dict(wall_s=2., simulated_s=5., error=None)))
    service.__exit__(RuntimeError, RuntimeError('deliberate failure'), None)
    manifest = json.loads((service.directory / 'manifest.json').read_text())
    assert manifest['status'] == 'failed' and manifest['error'] == 'deliberate failure'
    assert manifest['simulated_s'] == 5. and manifest['service_wall_s'] == 2.
    assert not manifest['normal_completion']
    assert partial.read_text() == '{"point":{"l_drive":1}}\n'


@pytest.mark.slow
def test_fresh_hp3_full_auto_global_radii_and_matched_browser_collection(tmp_path):
    """No seed dataset, manual stopping, shortened collection or acceptance threshold."""
    dataset = tmp_path / 'hp3' / 'sweeps.json'
    log = tmp_path / 'full-auto.stdout.log'
    with log.open('w') as output:
        result = subprocess.run([sys.executable, 'autocal/autocal.py', '--headless-sim',
                                 '--machine-type', 'hangprinter_3', '--dataset', str(dataset),
                                 '--find-radii', 'global', '--base-radii', '30',
                                 '--buildup-factor', '0.636619', '--r0-bounds', '39,40'],
                                cwd=ROOT, stdout=output, stderr=output, timeout=1800)
    assert result.returncode == 0, log.read_text()[-4000:]
    manifest = json.loads(dataset.with_suffix('.headless').joinpath('manifest.json').read_text())
    assert manifest['normal_completion'] and manifest['status'] == 'complete'
    assert manifest['stop_reason'] == 'patience-or-threshold'
    assert manifest['epoch'] == 1  # reset once before bootstrap, same world for later sweeps
    assert manifest['simulated_s'] > 0 and manifest['wall_s'] > 0
    assert manifest['firmware_config'] == 'sys/config_hp3_w_line_layers.g'
    assert [command.split()[0] for command in manifest['applied_parameters']] == ['M669', 'M666']
    assert '39.' in manifest['firmware_final']['M666']
    assert all(len(manifest[key]) == 64 for key in ('source_sha256', 'autocal_source_sha256',
                'scene_sha256', 'baked_scene_sha256', 'firmware_config_sha256', 'rrf_sha256'))
    data = json.loads(dataset.read_text())
    assert len(data['sweeps']) > 3
    assert all(len(sweep['data_points']) == 20 for sweep in data['sweeps'][:3])
    tuning = data['config']['force_tuning']
    assert tuning['auto_tuned']
    for pid in manifest['services']:
        with pytest.raises(ProcessLookupError):
            os.kill(pid, 0)
    iterations = [json.loads(line) for line in dataset.with_name('sweeps.full_auto_log.jsonl').read_text().splitlines()]
    decisions = [row['decision'] for row in iterations if 'decision' in row]
    assert 'collect' in decisions and decisions[-1] == 'accept'
    assert any(row.get('runs', [{}])[0].get('settings', {}).get('find_radii') == 'global' for row in iterations)
    parity = tmp_path / 'browser'
    result = subprocess.run(['node', 'tests/parity3d/browser_collection.mjs', str(dataset), str(parity)],
                            cwd=ROOT, capture_output=True, text=True, timeout=900)
    assert result.returncode == 0, result.stdout + result.stderr
    comparison = json.loads((parity / 'parity.json').read_text())
    assert comparison['passed'] and comparison['points'] == 60 and comparison['sweeps'] == 3


@pytest.mark.slow
def test_interrupted_bootstrap_preserves_points_and_stops_owned_services(tmp_path):
    dataset = tmp_path / 'sweeps.json'
    output = (tmp_path / 'stdout.log').open('w')
    process = subprocess.Popen([sys.executable, 'autocal/autocal.py', '--headless-sim',
        '--machine-type', 'hangprinter_3', '--dataset', str(dataset), '--find-radii', 'global',
        '--base-radii', '30', '--buildup-factor', '0.636619', '--r0-bounds', '39,40', '--collector-args',
        '--no-auto-tune-force', '--force-low', '0.01', '--force-mid', '0.23', '--force-max', '9.2',
        '--max-travel-mm', '300'], cwd=ROOT, stdout=output, stderr=output)
    try:
        journal = Path(f'{dataset}.partial-points.jsonl')
        deadline = time.monotonic() + 180
        while not journal.exists() or not journal.stat().st_size:
            assert process.poll() is None, (tmp_path / 'stdout.log').read_text()
            assert time.monotonic() < deadline
            time.sleep(.1)
        process.send_signal(signal.SIGTERM)
        assert process.wait(timeout=15) == 130
        manifest = json.loads(dataset.with_suffix('.headless').joinpath('manifest.json').read_text())
        assert manifest['status'] == 'interrupted' and not manifest['normal_completion']
        assert manifest['simulated_s'] > 0
        records = [json.loads(line) for line in journal.read_text().splitlines()]
        assert records and all('config' in record and 'point' in record for record in records)
        for pid in manifest['services']:
            with pytest.raises(ProcessLookupError):
                os.kill(pid, 0)
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGTERM)
            process.wait(timeout=15)
        output.close()
