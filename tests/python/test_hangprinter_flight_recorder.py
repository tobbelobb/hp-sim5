from copy import deepcopy
import json
import os
import signal
import socket
import subprocess
import sys
import time

import numpy as np
import pytest
from rerun.chunk import RrdReader

from scripts.hangprinter_flight_recorder import FlightRecording
from websockets.sync.client import connect


def sample(step=0):
    return {
        "version": 1, "session": "test", "generation": 1,
        "step": step, "time": step * 0.01, "dt": 0.01 if step else 0,
        "frames": [{
            "path": "world/machines/test/anchors/A", "name": "A", "kind": "anchor",
            "position": [step, 0, 0], "quaternion": [0, 0, 0, 1],
        }],
        "cables": [{
            "machine": "test", "name": "A", "color": "#ffaa00", "radius": 0.001,
            "segments": [{
                "name": "span", "points": [[0, 0, 0], [1, 0, 0]],
                "force_n": step + 2, "force_vector_n": [step + 2, 0, 0], "origin": [0, 0, 0],
            }],
            "wraps": [],
            "lengths": {"commanded": 1, "actual": 1.1, "geometric": 1.12, "error": 0.1, "stretch": 0.02},
        }],
    }


def rows(recording, path, component):
    chunks = RrdReader(recording.path).stream().filter(content=path, components=component).to_chunks()
    result = []
    for chunk in chunks:
        batch = chunk.to_record_batch()
        result.extend(zip(batch.column("sim_step").to_pylist(), batch.column(component).to_pylist()))
    return sorted(result, key=lambda row: row[0])


def test_saved_rrd_retains_every_timestep_and_requested_archetypes(tmp_path):
    recording = FlightRecording(tmp_path, None, sample(), 0.01)
    try:
        for step in range(3):
            recording.log_sample(sample(step))
    finally:
        recording.close()

    lengths = rows(recording, "/line_lengths/test/A", "Scalars:scalars")
    assert [step for step, _ in lengths] == [0, 1, 2]
    assert lengths[-1][1] == [1, 1.1, 1.12]
    assert rows(recording, "/cable_forces/test/A", "Scalars:scalars") == [(0, [2]), (1, [3]), (2, [4])]
    positions = rows(recording, "/world/machines/test/anchors/A", "Transform3D:translation")
    assert [step for step, _ in positions] == [0, 1, 2]
    assert np.allclose(positions[-1][1], [[2, 0, 0]])
    strips = rows(recording, "/world/machines/test/cables/A/segments", "LineStrips3D:strips")
    assert [step for step, _ in strips] == [0, 1, 2]
    assert np.allclose(strips[-1][1], [[[0, 0, 0], [1, 0, 0]]])
    assert RrdReader(recording.path).blueprints()
    manifest = json.loads(recording.path.with_suffix('.json').read_text())
    assert manifest['backend'] == 'browser-js' and manifest['browser_session'] == 'test'
    assert manifest['scene_generation'] == 1 and manifest['status'] == 'finalized'
    assert manifest['start_step'] == 0 and manifest['end_step'] == 2
    assert manifest['end_time_s'] == .02 and manifest['recording_id']
    assert manifest['rrd'] == str(recording.path.resolve())


def test_timestep_gaps_are_rejected_before_logging(tmp_path):
    recording = FlightRecording(tmp_path, None, sample(), 0.01)
    try:
        recording.log_sample(sample())
        with pytest.raises(ValueError, match="Nonconsecutive timestep"):
            recording.log_sample(sample(2))
        invalid_clock = sample(1)
        invalid_clock["time"] = 0.02
        with pytest.raises(ValueError, match="clock"):
            recording.log_sample(invalid_clock)
    finally:
        recording.close()
    assert rows(recording, "/line_lengths/test/A", "Scalars:scalars") == [(0, [1, 1.1, 1.12])]


def test_sampled_steps_keep_true_clocks_and_detect_unexpected_gaps(tmp_path):
    recording = FlightRecording(tmp_path, None, sample(), .01)
    try:
        recording.log_sample(dict(sample(), sample_stride=10, geometry_detail="compact"))
        recording.log_sample(dict(sample(10), sample_stride=10, geometry_detail="compact"))
        with pytest.raises(ValueError, match="Nonconsecutive timestep"):
            recording.log_sample(dict(sample(30), sample_stride=10))
    finally:
        recording.close()
    assert [step for step, _ in rows(recording, "/line_lengths/test/A", "Scalars:scalars")] == [0, 10]
    manifest = json.loads(recording.path.with_suffix('.json').read_text())
    assert manifest['end_time_s'] == .1
    assert manifest['sample_stride'] == 10 and manifest['geometry_detail'] == 'compact'


def test_unchanged_poses_are_not_duplicated_but_changes_are_recorded(tmp_path):
    recording = FlightRecording(tmp_path, None, sample(), .01)
    try:
        recording.log_sample(sample())
        stationary = sample(1)
        stationary['frames'][0]['position'] = [0, 0, 0]
        recording.log_sample(stationary)
        recording.log_sample(sample(2))
    finally:
        recording.close()
    assert [step for step, _ in rows(recording, "/world/machines/test/anchors/A", "Transform3D:translation")] == [0, 2]


def test_removing_cables_clears_old_geometry_and_telemetry(tmp_path):
    recording = FlightRecording(tmp_path, None, sample(), 0.01)
    try:
        recording.log_sample(sample())
        without_cable = deepcopy(sample(1))
        without_cable["cables"] = []
        recording.log_sample(without_cable)
    finally:
        recording.close()
    assert rows(recording, "/world/machines/test/cables/A/segments", "Clear:is_recursive") == [(1, [True])]
    assert rows(recording, "/line_lengths/test/A", "Clear:is_recursive") == [(1, [True])]


def test_extended_events_keep_source_wall_time_without_inheriting_physics_clock(tmp_path):
    recording = FlightRecording(tmp_path, None, sample(), .01, extended=True)
    stamp = 1_800_000_000_123
    try:
        physics = dict(sample(), wall_time_ms=stamp, speed_scale=25)
        recording.log_sample(physics)
        recording.log_events([dict(source='collector', wall_time_ms=stamp - 2,
                                 kind='gcode_send', payload={'line': 'G1 A1', 'source': {'file': 'test.mjs', 'line': 42}},
                                 sim_time_s=None, sim_time_source='unavailable'),
                             dict(source='browser', wall_time_ms=stamp + 2,
                                 kind='encoder_response_sent', sim_time_s=4.2,
                                 sim_time_source='browser.researchClock'),
                             dict(source='collector', wall_time_ms=stamp + 2,
                                 kind='gcode_reply', payload={'result': None}, sim_time_s=4.2,
                                 sim_time_source='collector.browser.researchClock', sim_time_observed_wall_ms=stamp)])
        reset = dict(sample(), session='second-page', generation=2, wall_time_ms=stamp + 3)
        recording.log_sample(reset)
    finally:
        recording.close()
    reader = RrdReader(recording.path)
    assert len(reader.recordings()) == 1
    events = reader.stream().filter(content='/autocal/collector', components='TextLog:text').to_chunks()
    assert events
    batch = next(chunk.to_record_batch() for chunk in events if 'sim_time' not in chunk.timeline_names)
    assert (stamp - 2) * 1_000_000 in batch.column('wall_time').cast('int64').to_pylist()
    assert 'sim_time' not in batch.schema.names
    raw = reader.stream().filter(content='/autocal/collector', components='event_json').to_chunks()[0]
    body = json.loads(raw.to_record_batch().column('event_json').to_pylist()[0][0])
    assert body['payload']['source']['line'] == 42
    observed = reader.stream().filter(content='/clocks/collector', components='sim_time_observed_wall_ms').to_chunks()
    assert observed[0].to_record_batch().column('sim_time_observed_wall_ms').to_pylist() == [[stamp]]
    physics_chunks = reader.stream().filter(content='/line_lengths/test/A', components='Scalars:scalars').to_chunks()
    assert sum(chunk.num_rows for chunk in physics_chunks) == 2


def test_event_batches_preserve_envelopes_order_clocks_and_completion(tmp_path):
    recording = FlightRecording(tmp_path, None, sample(), .01, extended=True)
    stamp = 1_800_000_000_123
    events = [
        dict(source='python', wall_time_ms=stamp, kind='run_start'),
        dict(source='collector', wall_time_ms=stamp + 1, kind='gcode_send', payload={'line': 'G1 X1'},
             sim_time_s=None, sim_time_source='unavailable'),
        dict(source='browser', wall_time_ms=stamp - 1, kind='command', payload={'axes': {'A': .2}},
             sim_time_s=4.2, sim_time_source='browser.researchClock'),
        dict(source='collector', wall_time_ms=stamp, kind='gcode_reply', sim_time_s=4.2,
             sim_time_source='collector.browser.researchClock', sim_time_observed_wall_ms=stamp - 3),
        dict(source='collector', wall_time_ms=stamp, kind='command', sim_time_s=4.3,
             sim_time_source='collector.browser.researchClock'),
        dict(source='collector', wall_time_ms=stamp, kind='command', sim_time_s=4.4,
             sim_time_source='collector.browser.researchClock', sim_time_observed_wall_ms=stamp - 1),
        dict(source='python', wall_time_ms=stamp, kind='run_complete', payload={'returncode': 0}),
    ]
    try:
        with pytest.raises(ValueError, match='Unknown event source'):
            recording.log_events([events[0], dict(events[0], source='unknown')])
        assert recording.event_count == 0
        recording.log_sample(dict(sample(), wall_time_ms=stamp))
        recording.log_events(events)
        assert recording.event_count == len(events)
        assert recording.manifest['autocal_complete']
    finally:
        recording.close()
    chunks = RrdReader(recording.path).stream().filter(content='/autocal/**', components='event_json').to_chunks()
    stored = []
    for chunk in chunks:
        batch = chunk.to_record_batch()
        assert 'sim_time' not in batch.column_names
        for order, wall, raw in zip(batch.column('event_order').to_pylist(),
                                    batch.column('wall_time').cast('int64').to_pylist(),
                                    batch.column('event_json').to_pylist()):
            event = json.loads(raw[0])
            assert wall == event['wall_time_ms'] * 1_000_000
            stored.append((order, event))
    assert sorted(stored) == list(enumerate(events, 1))
    clock_chunks = RrdReader(recording.path).stream().filter(content='/clocks/collector', components='Scalars:scalars').to_chunks()
    clock_rows = [row for chunk in clock_chunks for row in chunk.to_record_batch().column('Scalars:scalars').to_pylist()]
    assert clock_rows == [[4.2], [4.3], [4.4]]


def test_live_recorder_negotiates_sampling_and_drains_collector_events(tmp_path):
    with socket.socket() as port_socket:
        port_socket.bind(('127.0.0.1', 0))
        port = port_socket.getsockname()[1]
    url = f'ws://127.0.0.1:{port}'
    process = subprocess.Popen([sys.executable, 'scripts/hangprinter_flight_recorder.py',
                               '--extended-reference', '--no-viewer', '--port', str(port),
                               '--output', str(tmp_path)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        deadline = time.monotonic() + 10
        while True:
            try:
                browser = connect(url, open_timeout=1)
                break
            except OSError:
                if process.poll() is not None:
                    raise AssertionError(process.stdout.read())
                if time.monotonic() >= deadline:
                    raise AssertionError('Recorder startup timed out')
                time.sleep(.05)
        with browser:
            # Autocal metadata is emitted before its collector starts the browser recorder.
            browser.send(json.dumps(dict(version=1, type='autocal_event', source='python',
                                         kind='run_start', wall_time_ms=1_800_000_000_100,
                                         sim_time_s=None, sim_time_source='unavailable')))
            assert json.loads(browser.recv())['type'] == 'event_ack'
            browser.send(json.dumps(dict(sample(), wall_time_ms=1_800_000_000_123)))
            assert json.loads(browser.recv()) == {'type': 'recording_config', 'sample_stride': 10, 'geometry_detail': 'compact'}
            assert json.loads(browser.recv())['type'] == 'ack'
            browser.send(json.dumps(dict(sample(10), sample_stride=10, geometry_detail='compact', wall_time_ms=1_800_000_000_456)))
            assert json.loads(browser.recv())['step'] == 10
            # Reproduce the force-return burst, with physics interleaved on the
            # same socket. No event or sample may be acknowledged away/lost.
            burst = [dict(version=1, type='autocal_event', source='browser', kind='external_payload_received',
                          wall_time_ms=1_800_000_000_500 + index // 100,
                          sim_time_s=4.2 + index / 500, sim_time_source='browser.researchClock',
                          payload={'type': 'command', 'command': {'type': 'Move', 'A': index / 500}})
                     for index in range(16162)]
            expected_acks = []
            for batch_index, start in enumerate(range(0, len(burst), 256)):
                events = burst[start:start + 256]
                browser.send(json.dumps(dict(version=1, type='autocal_event_batch', events=events)))
                expected_acks.append({'type': 'event_ack', 'count': len(events)})
                if batch_index % 8 == 7:
                    step = 20 + (batch_index // 8) * 10
                    browser.send(json.dumps(dict(sample(step), sample_stride=10, geometry_detail='compact',
                                                 wall_time_ms=1_800_000_000_700 + step)))
                    expected_acks.append({'type': 'ack', 'step': step})
            for expected in expected_acks:
                assert json.loads(browser.recv(timeout=10)) == expected
            subprocess.run(['node', '--input-type=module', '-e', """
                import {createReferenceLogger} from './autocal/control/primitives/extended_reference.mjs';
                const log = createReferenceLogger();
                for (let i = 0; i < 150; i++) log.emit('gcode_send', {line: `G1 X${i}`, source: {file: 'verification.mjs', line: i}});
                log.emit('collection_complete', {});
                await log.close();
            """], check=True, env={**os.environ, 'AUTOCAL_REFERENCE_WS': url}, timeout=15)
    finally:
        process.send_signal(signal.SIGINT)
        output, _ = process.communicate(timeout=15)
    assert process.returncode == 0, output
    path = next(tmp_path.glob('*.rrd'))
    reader = RrdReader(path)
    assert len(reader.recordings()) == 1
    events = reader.stream().filter(content='/autocal/collector', components='event_json').to_chunks()
    assert sum(chunk.num_rows for chunk in events) == 151
    startup = reader.stream().filter(content='/autocal/python', components='event_json').to_chunks()
    assert sum(chunk.num_rows for chunk in startup) == 1
    assert all('wall_time' in chunk.timeline_names for chunk in events)
    physics = reader.stream().filter(content='/line_lengths/test/A', components='Scalars:scalars').to_chunks()
    steps = sorted(step for chunk in physics for step in chunk.to_record_batch().column('sim_step').to_pylist())
    assert steps == list(range(0, 100, 10))
    captured = reader.stream().filter(content='/autocal/browser', components='event_json').to_chunks()
    stored = []
    for chunk in captured:
        batch = chunk.to_record_batch()
        stored.extend(zip(batch.column('event_order').to_pylist(),
                          [json.loads(row[0]) for row in batch.column('event_json').to_pylist()]))
    assert [event for _, event in sorted(stored)] == burst


@pytest.mark.parametrize('events', [[], [None], [dict(version=1, type='autocal_event', source='unknown', wall_time_ms=1)],
                                  [dict(version=1, type='autocal_event')] * 257])
def test_live_recorder_rejects_invalid_batches_without_acknowledging(tmp_path, events):
    with socket.socket() as port_socket:
        port_socket.bind(('127.0.0.1', 0))
        port = port_socket.getsockname()[1]
    process = subprocess.Popen([sys.executable, 'scripts/hangprinter_flight_recorder.py', '--extended-reference',
                                '--no-viewer', '--port', str(port), '--output', str(tmp_path)],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        deadline = time.monotonic() + 10
        while True:
            try:
                browser = connect(f'ws://127.0.0.1:{port}', open_timeout=1)
                break
            except OSError:
                if process.poll() is not None or time.monotonic() >= deadline:
                    raise AssertionError('Recorder startup failed')
                time.sleep(.05)
        with browser:
            browser.send(json.dumps(dict(version=1, type='autocal_event_batch', events=events)))
            from websockets.exceptions import ConnectionClosedError
            with pytest.raises(ConnectionClosedError):
                browser.recv(timeout=10)
    finally:
        process.send_signal(signal.SIGINT)
        output, _ = process.communicate(timeout=15)
    assert process.returncode == 0, output
    manifest = json.loads(next(tmp_path.glob('*.json')).read_text())
    assert manifest['rejected_messages']
    assert manifest['event_count'] == 0
