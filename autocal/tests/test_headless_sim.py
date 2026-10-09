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
from rerun.chunk import RrdReader
from websockets.sync.client import connect

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


@pytest.mark.parametrize('flags', [[], ['--headless-sim', '--no-collect']])
def test_owned_reference_requires_headless_collection(flags, monkeypatch):
    monkeypatch.delenv('AUTOCAL_REFERENCE_WS', raising=False)
    with pytest.raises(SystemExit):
        ac.main(['--extended-reference', *flags])


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


def recorded_events(path, source):
    chunks = RrdReader(path).stream().filter(content=f'/autocal/{source}', components='event_json').to_chunks()
    events = [json.loads(value[0]) for chunk in chunks for value in chunk.to_record_batch().column('event_json').to_pylist()]
    return sorted(events, key=lambda event: (event['wall_time_ms'], event.get('monotonic_ms', 0)))


def test_extended_headless_records_resets_clocks_and_finalizes(tmp_path, monkeypatch):
    from argparse import Namespace
    from autocal import extended_reference
    monkeypatch.delenv('AUTOCAL_REFERENCE_WS', raising=False)
    args = Namespace(machine_type='hangprinter_3', find_radii='off', find_buildup_factor='off',
                     rrf_config=None, config=None, dataset=tmp_path / 'sweeps.json',
                     keep_sim_alive=False, extended_reference=True)
    with HeadlessSimulation(args, []) as service:
        extended_reference.emit('run_start', test=True)
        service.request('advance', seconds=.05)
        service.request('gcode', line='M569.3 P40.0')
        service.request('payload', payload={'type': 'reset'})
        service.request('payload', payload={'type': 'set_speed_scale', 'value': 25})
        state = service.request('advance', seconds=.01)
        assert state['step'] == 125 and state['collector_time_s'] == pytest.approx(.06)
        extended_reference.emit('run_complete', returncode=0)
    assert 'AUTOCAL_REFERENCE_WS' not in os.environ
    recording, = service.manifest['recordings']
    assert recording['backend'] == 'headless-js' and recording['status'] == 'finalized'
    assert recording['autocal_complete'] and not recording['rejected_messages']
    assert [segment['generation'] for segment in recording['headless_segments']] == [0, 1]
    assert recording['sample_stride'] == 10 and recording['geometry_detail'] == 'compact'
    path = recording['rrd']
    assert len(RrdReader(path).recordings()) == 1
    events = recorded_events(path, 'headless')
    assert sum(event['kind'] == 'scene_context' for event in events) == 2
    assert events[-1]['kind'] == 'service_stopped'
    assert events[-1]['sim_time_s'] == pytest.approx(.25)
    assert all(event['sim_time_source'] == 'headless.researchClock' for event in events)
    encoder_events = [event for event in events if event['kind'] == 'encoder_response_sent']
    assert encoder_events and all(event['payload']['simulationClock']['source'] == 'headless.researchClock' for event in encoder_events)
    python_events = recorded_events(path, 'python')
    assert {event['kind'] for event in python_events} == {'headless_start', 'artifact', 'run_start', 'run_complete'}
    artifacts = {Path(event['payload']['path']).name: event['payload']['content'] for event in python_events if event['kind'] == 'artifact'}
    assert set(artifacts) == {'scene.usda', 'baked-scene.usda', 'firmware-config.g'}
    assert all(artifacts.values())
    clocks = RrdReader(path).stream().filter(content='/clocks/headless/flight_recorder', components='Scalars:scalars').to_chunks()
    assert clocks and all('wall_time' in chunk.timeline_names for chunk in clocks)
    for pid in service.manifest['services']:
        with pytest.raises(ProcessLookupError):
            os.kill(pid, 0)


def test_extended_headless_fails_when_recorder_disconnects(tmp_path, monkeypatch):
    from argparse import Namespace
    from urllib.error import HTTPError
    monkeypatch.delenv('AUTOCAL_REFERENCE_WS', raising=False)
    args = Namespace(machine_type='hangprinter_3', find_radii='off', find_buildup_factor='off',
                     rrf_config=None, config=None, dataset=tmp_path / 'sweeps.json',
                     keep_sim_alive=False, extended_reference=True)
    with pytest.raises(HTTPError):
        with HeadlessSimulation(args, []) as service:
            service.recorder.terminate()
            service.recorder.wait(timeout=15)
            service.request('advance', seconds=.1)
    assert service.manifest['status'] == 'failed'
    assert service.manifest['recording_error'] == 'Flight recorder disconnected'
    assert not service.manifest['normal_completion']


def test_headless_uses_external_recorder_and_preserves_collector_clock(tmp_path, monkeypatch):
    from argparse import Namespace
    from autocal.headless_sim import free_port
    port = free_port()
    url = f'ws://127.0.0.1:{port}'
    with (tmp_path / 'recorder.log').open('w') as output:
        recorder = subprocess.Popen([sys.executable, 'scripts/hangprinter_flight_recorder.py',
            '--extended-reference', '--no-viewer', '--sample-stride', '1', '--geometry-detail', 'full',
            '--port', str(port), '--output', str(tmp_path)], cwd=ROOT, stdout=output, stderr=output)
    try:
        deadline = time.monotonic() + 15
        while True:
            try:
                with connect(url, open_timeout=1, close_timeout=1):
                    break
            except OSError:
                assert recorder.poll() is None, (tmp_path / 'recorder.log').read_text()
                assert time.monotonic() < deadline
                time.sleep(.05)
        monkeypatch.setenv('AUTOCAL_REFERENCE_WS', url)
        args = Namespace(machine_type='hangprinter_3', find_radii='off', find_buildup_factor='off',
                         rrf_config=None, config=None, dataset=tmp_path / 'sweeps.json',
                         keep_sim_alive=False, extended_reference=False)
        with HeadlessSimulation(args, []) as service:
            service.request('payload', payload={'type': 'set_speed_scale', 'value': 25})
            subprocess.run(['node', '--input-type=module', '-e', """
                import {createHeadlessBridge} from './autocal/control/primitives/headless_bridge.mjs';
                import {createReferenceLogger} from './autocal/control/primitives/extended_reference.mjs';
                const bridge = await createHeadlessBridge(process.argv[1]);
                const log = createReferenceLogger();
                await bridge.simulationClock.sleep(10);
                log.emit('gcode_send', {commandId: 'live-test', line: 'M115'}, bridge.simulationClock);
                log.emit('gcode_reply', {commandId: 'live-test', result: await bridge.sendGcodeLine('M115')}, bridge.simulationClock);
                await log.close();
            """, service.url], cwd=ROOT, check=True, timeout=15)
        assert recorder.poll() is None
        assert not service.manifest['owned_recorder']
    finally:
        recorder.terminate()
        recorder.wait(timeout=15)
    manifest, = [json.loads(path.read_text()) for path in tmp_path.glob('hangprinter-*.json')]
    assert manifest['status'] == 'finalized' and not manifest['rejected_messages']
    assert manifest['sample_stride'] == 1 and manifest['geometry_detail'] == 'full'
    assert manifest['end_step'] == 125 and manifest['end_time_s'] == pytest.approx(.25)
    events = recorded_events(manifest['rrd'], 'collector')
    assert [event['kind'] for event in events] == ['gcode_send', 'gcode_reply']
    assert all(event['sim_time_s'] == pytest.approx(.01) for event in events)
    assert all(event['sim_time_source'] == 'collector.headless.collectorClock' for event in events)
    assert all(event['sim_time_observed_wall_ms'] is not None for event in events)


@pytest.mark.slow
def test_fresh_hp3_full_auto_global_radii_and_matched_browser_collection(tmp_path):
    """No seed dataset, manual stopping, shortened collection or acceptance threshold."""
    dataset = tmp_path / 'hp3' / 'sweeps.json'
    log = tmp_path / 'full-auto.stdout.log'
    with log.open('w') as output:
        result = subprocess.run([sys.executable, 'autocal/autocal.py', '--headless-sim', '--extended-reference',
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
    recording, = manifest['recordings']
    assert recording['status'] == 'finalized' and recording['autocal_complete']
    assert recording['backend'] == 'headless-js' and not recording['rejected_messages']
    assert recording['physics_samples'] > 0 and recording['event_count'] > 0
    assert len(RrdReader(recording['rrd']).recordings()) == 1
    events = recorded_events(recording['rrd'], 'collector')
    assert sum(event['kind'] == 'measurement' for event in events) >= sum(len(sweep['data_points']) for sweep in data['sweeps'])
    assert sum(event['kind'] == 'collection_complete' for event in events) > 1
    python_events = recorded_events(recording['rrd'], 'python')
    artifacts = {event['payload']['path']: event['payload']['content'] for event in python_events if event['kind'] == 'artifact'}
    for path in (dataset, dataset.with_name('sweeps.full_auto.log'), dataset.with_name('sweeps.full_auto_log.jsonl')):
        assert artifacts[str(path)] == path.read_text()
    assert any(Path(path).is_relative_to(dataset.with_name('sweeps.stages')) for path in artifacts)
    assert any(event['kind'] == 'run_complete' for event in python_events)
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
    process = subprocess.Popen([sys.executable, 'autocal/autocal.py', '--headless-sim', '--extended-reference',
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
        recording, = manifest['recordings']
        assert recording['status'] == 'finalized' and not recording['autocal_complete']
        assert recording['physics_samples'] > 0 and not recording['rejected_messages']
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
