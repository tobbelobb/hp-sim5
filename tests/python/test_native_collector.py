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
        result = service.call('collect', options={'sweepPoints': 3, 'noiseSamples': 4})
        assert result['session_id'] == before['session_id']
        assert result['status'] == 'complete' and result['point_count'] == 6
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
