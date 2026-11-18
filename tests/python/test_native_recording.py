from pathlib import Path

import numpy as np
import pytest
import rerun as rr
from rerun.chunk import RrdReader
from scipy.spatial.transform import Rotation

from hp_sim5_research import mirror
from hp_sim5_research.js_physics import JSPhysics
from hp_sim5_research.recording import RerunSystem, blueprint

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.slow
def test_hp4_native_recording_frames_and_telemetry(tmp_path):
    rrd = tmp_path / 'recording.rrd'
    physics = JSPhysics(ROOT, tmp_path, ROOT / 'public/usd_scenes/hp4.usda', dt=.002)
    stream = rr.RecordingStream('HP4 native recording test')
    stream.set_sinks(rr.FileSink(rrd), default_blueprint=blueprint())
    recorder = RerunSystem(stream)
    commands = [{'type': 'Move', 'D': .005, 'E': .00001},
                {'type': 'Move', 'D': .01, 'E': .00002},
                {'type': 'SetTorqueMode', 'axis': 'D', 'torqueNm': -.01}]
    try:
        states = [physics.request()]
        states.extend(physics.request(steps=1, commands=[command]) for command in commands)
        world = mirror.World()
        for step, state in enumerate(states):
            mirror.apply_state(world, state)
            recorder.update(world, .002 if step else 0.)
        recorder.flush()
    finally:
        try:
            stream.disconnect()
        finally:
            physics.close()

    reader = RrdReader(rrd)
    assert reader.blueprints()
    samples = {}
    static = {}
    for chunk in reader.stream().to_chunks():
        batch = chunk.to_record_batch()
        if chunk.is_static:
            for name in batch.schema.names:
                if ':' in name:
                    static[(chunk.entity_path, name)] = batch.column(name).to_pylist()[0]
        if 'sim_step' in batch.schema.names:
            steps = batch.column('sim_step').to_pylist()
            for name in batch.schema.names:
                if ':' in name:
                    samples.setdefault((chunk.entity_path, name), {}).update(zip(steps, batch.column(name).to_pylist()))

    def value(path, component, step=3):
        return samples[(path, component)][step][0]

    def scalar(path, step=3):
        return value(path, 'Scalars:scalars', step)

    machine = '/world/machines/default'
    state = dict(states[-1]['components'])
    names = {entity: info['name'] for entity, info in state['SceneEntityInfoComponent']}
    translation = 'Transform3D:translation'
    quaternion = 'Transform3D:quaternion'
    positions = dict(state['PositionComponent'])
    orientations = dict(state['OrientationComponent'])
    assert len(state['RigidBodyMemberComponent']) == 33
    for entity, member in state['RigidBodyMemberComponent']:
        parent = member['body_entity']
        parent_path = f'{machine}/bodies/{names[parent]}'
        path = f'{parent_path}/members/{names[entity]}'
        recorded_rotation = Rotation.from_quat(value(parent_path, quaternion))
        recorded_position = value(parent_path, translation) + recorded_rotation.apply(value(path, translation))
        body_rotation = Rotation.from_quat(orientations[parent]['quaternion'])
        expected_position = positions[parent]['pos'] + body_rotation.apply(member['local_position'])
        assert np.linalg.norm(recorded_position - expected_position) < 1e-6
        recorded_member_rotation = recorded_rotation * Rotation.from_quat(value(path, quaternion))
        expected_member_rotation = body_rotation * Rotation.from_quat(member['local_orientation'])
        assert (recorded_member_rotation.inv() * expected_member_rotation).magnitude() < 1e-6
        assert static[(f'{path}/shape', 'Points3D:positions')] == [[0., 0., 0.]]
        assert (f'{machine}/bodies/{names[entity]}', translation) not in samples

    anchors = [entity for entity, info in state['SceneEntityInfoComponent'] if 'Anchor' in info['tags']]
    assert anchors
    for entity in anchors:
        assert value(f'{machine}/anchors/{names[entity]}', translation) == pytest.approx(positions[entity]['pos'])
        assert (f'{machine}/bodies/{names[entity]}', translation) not in samples

    motor_entity, = states[-1]['axes']['D']
    motor = f'/motors/default/{names[motor_entity]}'
    assert scalar(f'{motor}/torque_mode', 2) == 0
    assert scalar(f'{motor}/torque_mode') == 1
    assert scalar(f'{motor}/target_angle_rad', 2) == pytest.approx(.01)
    assert 3 not in samples[(f'{motor}/target_angle_rad', 'Scalars:scalars')]
    assert scalar(f'{motor}/target_torque_nm') == pytest.approx(-.01)
    motor_state = dict(state['StepperMotorComponent'])[motor_entity]
    for name in ('missed_steps', 'current_missed_steps'):
        assert scalar(f'{motor}/{name}') == motor_state[name]

    encoder = f'/encoders/default/{names[motor_entity]}'
    angle = dict(state['EncoderComponent'])[motor_entity]['angle']
    assert abs(angle) > 1e-6
    assert scalar(f'{encoder}/angle_rad') == pytest.approx(angle)
    assert [scalar(f'{encoder}/axis_{axis}') for axis in 'xyz'] == pytest.approx(
        dict(state['EncoderComponent'])[motor_entity]['axis'])

    for component, prefix, field in (('VelocityComponent', 'linear', 'vel'),
                                      ('AngularVelocityComponent', 'angular', 'omega')):
        entity, motion = max(state[component], key=lambda row: np.linalg.norm(row[1][field]))
        assert np.linalg.norm(motion[field]) > 1e-6
        path = '/velocities/' + mirror.entity_key(world, entity)
        assert [scalar(f'{path}/{prefix}_{axis}') for axis in 'xyz'] == pytest.approx(motion[field])

    (_, extruder), = state['ExtruderComponent']
    assert value(f'{machine}/effector', translation) == pytest.approx(extruder['machine_effector_centers']['default'])
    assert value(f'{machine}/effector', quaternion) == pytest.approx(
        value(f'{machine}/bodies/Effector', quaternion), abs=1e-10)
    for label, field in (('effector_center', 'machine_effector_centers'), ('root', 'machine_centers'),
                         ('tip', 'machine_tips'), ('cold_end', 'machine_cold_ends')):
        assert value(f'{machine}/tool_points/{label}', 'Points3D:positions') == pytest.approx(extruder[field]['default'])

    deposits = samples[(f'{machine}/extrusions/deposits', 'Points3D:positions')]
    assert 0 not in deposits
    assert len(deposits[1]) == 1 and len(deposits[2]) == len(deposits[3]) == 2
    assert np.linalg.norm(np.subtract(*deposits[2])) > 1e-6
    assert deposits[3] == deposits[2]
    np.testing.assert_allclose(deposits[3], [record['pos'] for record in extruder['extrusions']], atol=1e-6)
    assert scalar('/extrusion_lengths/default', 1) == pytest.approx(.00001)
    assert scalar('/extrusion_lengths/default', 2) == pytest.approx(.00003)
