import json
import math
from pathlib import Path

import pytest

from cable_joints_3d.ecs import EncoderComponent
from cable_joints_3d.stepper_motor import StepperMotorComponent
from hp_sim5_research.session import NativeSession
from hp_sim5_research.services import RuntimeService
from hp_sim5_research.validation import validate_collection
from hp_sim5_research.experiments import sample

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.asyncio
async def test_bridge_encoder_order_unwrapping_and_diagnostic_offset(tmp_path):
    session = NativeSession(ROOT, tmp_path)
    try:
        for index, axis in enumerate('ABCD'):
            entity = session.remote.axis_to_entity[axis][0]
            session.world.get_component(entity, EncoderComponent).angle = (index + 2) * math.tau + .1
            session.world.get_component(entity, StepperMotorComponent).missed_step_encoder_offset = .5
        response = await session.handle({'type': 'encoder_request', 'requestId': 7, 'axes': ['D', 'A', 'Q', 'B']})
        assert response['requestId'] == 7
        assert response['anglesDeg'] == pytest.approx([math.degrees(5 * math.tau + .1),
                                                       math.degrees(2 * math.tau + .1), None,
                                                       math.degrees(3 * math.tau + .1)])
        assert session.step == 0
        row = sample(session.world, 0, session.dt)['motors'][0]
        assert row['encoder_angle_rad'] - row['diagnostic_encoder_angle_rad'] == pytest.approx(.5)
    finally:
        session.close()


@pytest.mark.asyncio
async def test_persistent_world_batch_motion_barrier_and_advance(tmp_path):
    session = NativeSession(ROOT, tmp_path)
    try:
        world = session.world
        await session.handle({'commands': [{'type': 'Move', 'A': .001}, {},
                                          {'type': 'SetTorqueMode', 'axis': 'B', 'torqueNm': -.001,
                                           'driver': 41, 'timestamp': 123}]})
        ack = await session.handle({'type': 'encoder_request', 'requestId': 1, 'axes': []})
        assert ack['anglesDeg'] == []
        assert session.step == 0 and session.remote.get_queue_length() == 3
        result = await session.handle({'type': 'encoder_request', 'requestId': 2, 'axes': list('ABCD')})
        assert len(result['anglesDeg']) == 4
        assert session.remote.get_queue_length() == 0 and session.step == 3
        await session.advance(.01)
        assert session.world is world and session.step == 8
        assert session.world.get_component(session.remote.axis_to_entity['B'][0], StepperMotorComponent).torque_mode
        await session.handle({'type': 'set_speed_scale', 'value': 2})
        clock = session.collector_time_s
        await session.advance(.004)
        assert session.collector_time_s - clock == pytest.approx(.002)
        await session.handle({'type': 'reset'})
        assert session.world is not world and session.step == 0 and session.epoch == 1
        assert session.remote.get_queue_length() == 0
        with pytest.raises(ValueError, match='Klipper'):
            await session.handle({'type': 'klipper_api_session_start'})
        with pytest.raises(ValueError, match='finite'):
            await session.advance(float('nan'))
        with pytest.raises(ValueError, match='Unknown command'):
            await session.handle({'commands': [{'type': 'Move', 'Q': 1}]})
    finally:
        session.close()
    events = [json.loads(line) for line in (tmp_path / 'events.jsonl').read_text().splitlines()]
    assert any(row['type'] == 'encoder_response' and row['step'] == 3 for row in events)


@pytest.mark.asyncio
async def test_collection_rrd_rotation_finalizes_files_and_preserves_physics(tmp_path):
    from rerun.chunk import RrdReader
    session = NativeSession(ROOT, tmp_path)
    try:
        world = session.world
        session.start_recording(tmp_path / 'collection.rrd')
        await session.advance(.004)
        session.observe()
        session.start_recording(tmp_path / 'next.rrd')
        assert session.world is world and session.step == 2
        reader = RrdReader(tmp_path / 'collection.rrd')
        assert reader.blueprints()
        assert reader.store()  # requires the finalized footer, unlike a live file scan
    finally:
        session.close()


@pytest.mark.slow
@pytest.mark.asyncio
async def test_live_viewer_receives_collector_advances_and_can_seek(tmp_path):
    import asyncio
    import importlib.util
    from mcp import Client, StdioServerParameters
    spec = importlib.util.spec_from_file_location('research_agent', ROOT / 'scripts/research_agent.py')
    launcher = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launcher)
    viewer = service = None
    try:
        viewer, endpoint = launcher.start_viewer('headless', tmp_path)
        service = RuntimeService(ROOT, tmp_path / 'runtime', endpoint).start()
        # Advance like the collector, without the step tool's explicit flush.
        service.call('advance', seconds=.1)
        await asyncio.sleep(5)
        service.call('advance', seconds=.1)
        parameters = StdioServerParameters(command=str(ROOT / '.venv/bin/rerun'),
            args=['viewer-mcp', '--endpoint', endpoint])
        # Rerun 0.38.1 supports initialize; SDK 2.3 discovery needs legacy mode.
        async with Client(parameters, read_timeout_seconds=30, mode='legacy') as client:
            state = await client.call_tool('rerun_get_viewer_state', {})
            assert not state.is_error
            data = json.loads(state.content[0].text)
            timelines = data['recordings'][0]['timelines']
            assert any(t['timeline']['name'] == 'sim_step' and t['time_range']['end'] == 100 for t in timelines)
            assert any(view['class'] == '3D' and view['visible'] for view in data['views'])
            seek = await client.call_tool('rerun_set_time_cursor',
                {'timeline': {'name': 'sim_step'}, 'time': {'time': 100}})
            assert not seek.is_error
    finally:
        if service is not None:
            service.close()
        launcher.stop_process(viewer)


@pytest.mark.slow
@pytest.mark.asyncio
async def test_supervised_services_stdio_gcode_and_browser(tmp_path):
    from mcp import Client, StdioServerParameters
    import sys
    service = RuntimeService(ROOT, tmp_path)
    try:
        service.start()
        parameters = StdioServerParameters(command=sys.executable, args=[str(ROOT / 'scripts/hp_sim5_mcp.py')],
            env={'HP_SIM5_RUNTIME_URL': service.endpoint, 'HP_SIM5_RUNTIME_TOKEN': service.token})
        async with Client(parameters, read_timeout_seconds=120) as client:
            assert not (await client.call_tool('capabilities', {})).is_error
            before = (await client.call_tool('runtime_status', {})).structured_content
            assert before['connected'] and before['step'] == 0
            assert all(item['exit_code'] is None for item in before['services'].values())
            reply = await client.call_tool('send_gcode', {'line': 'G1 H2 X1 F600'})
            assert not reply.is_error, reply.content
            read = await client.call_tool('send_gcode', {'line': 'M569.3 P40.0:41.0:42.0:43.0'})
            assert not read.is_error, read.content
            assert 'reply' in read.structured_content
            step = await client.call_tool('step_physics', {'steps': 10})
            assert step.structured_content['step'] > 10
            browser = await client.call_tool('start_browser_service', {})
            assert not browser.is_error, browser.content
            assert browser.structured_content['url'].startswith('http://127.0.0.1:')
            # Optional browser failure cannot poison the independent native backend.
            import asyncio
            import os
            import signal
            state = service.call('status')
            os.kill(state['services']['vite']['pid'], signal.SIGTERM)
            for _ in range(50):
                if service.call('status')['services']['vite']['exit_code'] is not None:
                    break
                await asyncio.sleep(.1)
            assert service.call('step', steps=1)['step'] > 0
            restarted = service.call('browser')
            assert restarted['url'] != browser.structured_content['url']
            assert service.call('status')['session_id'] == before['session_id']
            reset = await client.call_tool('reset_session', {})
            assert not reset.is_error, reset.content
            assert reset.structured_content['session_id'] != before['session_id']
            assert reset.structured_content['step'] == 0
    finally:
        service.close()
    assert service.process.poll() == 0


@pytest.mark.slow
def test_real_rrf_native_collection_autocal_and_process_cleanup(tmp_path):
    service = RuntimeService(ROOT, tmp_path)
    try:
        service.start()
        before = service.call('status')
        import time
        job = service.call('start_collection', options={'sweepPoints': 3, 'noiseSamples': 4})
        deadline = time.monotonic() + 920
        while True:
            status = service.call('job_status', job_id=job['job_id'])
            if status['status'] not in ('running', 'cancelling'):
                break
            assert time.monotonic() < deadline
            time.sleep(1)
        assert status['status'] == 'complete', status
        result = status['result']
        assert result['session_id'] == before['session_id']
        assert result['status'] == 'complete' and result['point_count'] == 6
        assert result['backend'] == 'native-python' and result['partial_point_count'] == 6
        points = [json.loads(line) for line in Path(result['artifacts']['partial_points']).read_text().splitlines()]
        assert all('raw_angles_deg' in point['point'] for point in points)
        validation = validate_collection(result['artifacts']['dataset'])
        assert validation['autocal_residual_count'] == 6
        after = service.call('status')
        assert after['step'] == result['end_step'] > before['step']
        reply = service.call('gcode', line='M569.3 P40.0:41.0:42.0:43.0')
        assert 'reply' in reply
        reset = service.call('reset')
        assert reset['step'] == 0 and reset['session_id'] != before['session_id']
        assert Path(result['artifacts']['rrd']).stat().st_size > 0
    finally:
        service.close()
    assert service.process.poll() == 0


@pytest.mark.asyncio
async def test_cancel_freezes_at_fixed_step_boundary_and_rejects_later_payloads(tmp_path):
    import asyncio
    session = NativeSession(ROOT, tmp_path)
    try:
        await session.handle({'commands': [{'type': 'Move', 'A': .001}] * 100})
        advance = asyncio.create_task(session.advance(.2))
        await asyncio.sleep(0)
        boundary = session.stop_execution()
        with pytest.raises(RuntimeError, match='cancelled'):
            await advance
        assert session.step == boundary['step'] < 100
        assert session.remote.get_queue_length() == 0
        with pytest.raises(RuntimeError, match='cancelled'):
            await session.handle({'commands': [{'type': 'Move', 'A': 1}]})
        with pytest.raises(RuntimeError, match='cancelled'):
            await session.advance(.002)
        assert session.step == boundary['step']
    finally:
        session.close()


@pytest.mark.slow
def test_live_collection_job_cancel_preserves_boundary_and_requires_reset(tmp_path):
    import time
    from rerun.chunk import RrdReader
    service = RuntimeService(ROOT, tmp_path).start()
    try:
        job = service.call('start_collection', options={'sweepPoints': 3, 'noiseSamples': 4})
        deadline = time.monotonic() + 30
        while service.call('status')['step'] < 20:
            assert time.monotonic() < deadline
            time.sleep(.1)
        cancelled = service.call('cancel_job', job_id=job['job_id'])
        assert cancelled['status'] == 'cancelled', cancelled
        result = cancelled['result']
        boundary = result['cancel_boundary']
        assert boundary['reset_required']
        assert result['end_step'] == boundary['step']
        assert result['collector_boundary']['cancelled']
        assert result['steps_executed'] > 0
        assert Path(result['artifacts']['events']).stat().st_size > 0
        assert RrdReader(result['artifacts']['rrd']).store()
        before = service.call('status')
        time.sleep(.2)
        after = service.call('status')
        assert before['step'] == after['step'] == boundary['step']
        assert after['queue_length'] == 0 and after['reset_required']
        assert all(after['services'][name]['exit_code'] is not None for name in ('rrf', 'collector'))
        assert service.call('job_status', job_id=job['job_id'])['status'] == 'cancelled'
        with pytest.raises(RuntimeError, match='reset_session'):
            service.call('step', steps=1)
        reset = service.call('reset')
        assert reset['step'] == 0 and not reset['reset_required']
        assert reset['session_id'] != job['session_id']
    finally:
        service.close()


@pytest.mark.slow
@pytest.mark.asyncio
async def test_attachment_reconnect_isolation_reset_and_dead_supervisor(tmp_path):
    import subprocess
    import sys
    from mcp import Client, StdioServerParameters
    service = RuntimeService(ROOT, tmp_path / 'runtime').start()
    descriptor = tmp_path / 'connection.json'
    descriptor.touch(mode=0o600)
    descriptor.write_text(json.dumps({'repo': str(ROOT), 'session_id': service.call('status')['session_id'],
        'runtime_endpoint': service.endpoint, 'runtime_token': service.token, 'viewer_endpoint': None}))
    command = [sys.executable, str(ROOT / 'scripts/research_attach.py'), str(descriptor)]
    parameters = StdioServerParameters(command=command[0], args=command[1:])
    try:
        async with Client(parameters, read_timeout_seconds=30) as client:
            before = (await client.call_tool('runtime_status', {})).structured_content
            await client.call_tool('step_physics', {'steps': 3})
            competing = subprocess.run(command, stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=5)
            assert competing.returncode != 0 and 'another chat' in competing.stderr
            assert service.token not in competing.stdout + competing.stderr
        async with Client(parameters, read_timeout_seconds=30) as client:
            reconnected = (await client.call_tool('runtime_status', {})).structured_content
            assert reconnected['session_id'] == before['session_id'] and reconnected['step'] == 3
            captured = await client.call_tool('capture_native_context', {'message': 'Inspect the continuing world', 'step': 3})
            assert captured.structured_content['sim_step'] == 3
            assert captured.structured_content['session_id'] == before['session_id']
            reset = (await client.call_tool('reset_session', {})).structured_content
            assert reset['session_id'] != before['session_id'] and reset['step'] == 0
        async with Client(parameters, read_timeout_seconds=30) as client:
            after = (await client.call_tool('runtime_status', {})).structured_content
            assert after['session_id'] == reset['session_id'] and after['step'] == 0
    finally:
        service.close()
    stale = subprocess.run(command, stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=5)
    assert stale.returncode != 0 and 'restart --serve' in stale.stderr
    assert service.token not in stale.stdout + stale.stderr
