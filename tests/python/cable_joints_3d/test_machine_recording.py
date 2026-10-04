import json
import os
import pickle
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest
import rerun as rr
from rerun.chunk import RrdReader

from cable_joints_3d.ecs import OrientationComponent, PositionComponent, RadiusComponent, RenderableComponent, World
from cable_joints_3d.extruder import ExtruderComponent
from cable_joints_3d.machine_simulation import load_machine_world, register_machine_systems
from cable_joints_3d.machine_snapshot import capture_machine_snapshot
from cable_joints_3d.remote_spool_system import RemoteSpoolSystem
from cable_joints_3d.rerun_system import RerunSystem
from parity_harness import ROOT
from test_rerun_system import Recording, _fake_rerun


def rows(path, content, component):
    chunks = RrdReader(path).stream().filter(content=content, components=component).to_chunks()
    result = []
    for chunk in chunks:
        batch = chunk.to_record_batch()
        if 'sim_step' not in batch.schema.names:  # static styles/clears have no timeline
            continue
        result.extend(zip(batch.column('sim_step').to_pylist(), batch.column(component).to_pylist()))
    return sorted(result, key=lambda row: row[0])


def test_saved_native_rrd_contains_initial_and_every_step_with_real_telemetry(tmp_path):
    path = tmp_path / 'native.rrd'
    stream = rr.RecordingStream('native recording test')
    stream.set_sinks(rr.FileSink(path))
    try:
        world = load_machine_world(ROOT / 'public/usd_scenes/hp4_rigid_body.usda', recording=stream)
        initial = capture_machine_snapshot(world)
        for _ in range(3):
            world.update(.002)
        final = capture_machine_snapshot(world)
        # Switch a real authored endpoint to torque mode: commanded/error series
        # disappear, while actual/geometric lengths and forces remain observable.
        world.get_system(RemoteSpoolSystem).commands = [{'type': 'SetTorqueMode', 'axis': 'A', 'torqueNm': .003}]
        world.update(.002)
        stream.flush(timeout_sec=5)
    finally:
        stream.disconnect()
    cable = next(cable for cable in final['cables'] if cable['name'].startswith('CablePathAL_'))
    key = f"{cable['machine']}/{cable['name']}"
    lengths = rows(path, '/line_lengths/' + key + '/actual', 'Scalars:scalars')
    assert [step for step, _ in lengths] == [0, 1, 2, 3, 4]
    assert lengths[3][1] == [cable['lengths']['actual']]
    targets = rows(path, '/line_lengths/' + key + '/commanded', 'Scalars:scalars')
    assert [step for step, _ in targets] == [0, 1, 2, 3]
    assert rows(path, '/line_lengths/' + key + '/commanded', 'Clear:is_recursive') == [(4, [True])]
    forces = rows(path, '/cable_forces/' + key + '/' + cable['segments'][0]['name'], 'Scalars:scalars')
    assert forces[3][1] == [cable['segments'][0]['force_n']]
    loaded = next(cable for cable in final['cables'] if any(segment['force_n'] > 0 for segment in cable['segments']))
    loaded_forces = rows(path, f"/cable_forces/{loaded['machine']}/{loaded['name']}/{loaded['segments'][0]['name']}", 'Scalars:scalars')
    assert any(value > 0 for _, values in loaded_forces for value in values)
    strips = rows(path, f"/world/machines/{cable['machine']}/cables/{cable['name']}/segments", 'LineStrips3D:strips')
    assert [step for step, _ in strips] == [0, 1, 2, 3, 4]
    assert np.allclose(strips[3][1], [segment['points'] for segment in cable['segments']], atol=2e-7)
    member = next(frame for frame in initial['frames'] if frame['name'] == 'WheelAL_top')
    assert '/members/' in member['path']
    transforms = rows(path, '/' + member['path'], 'Transform3D:translation')
    assert [step for step, _ in transforms] == [0, 1, 2, 3, 4]
    assert np.allclose(transforms[0][1], [member['position']], atol=1e-8)
    entity_name = member['path'].split('/')[-1]
    assert rows(path, '/encoders/default/' + entity_name + '/angle', 'Scalars:scalars')
    assert rows(path, '/velocities/default/' + entity_name + '/linear_x', 'Scalars:scalars')
    motor_name = next(frame['path'].split('/')[-1] for frame in initial['frames'] if frame['name'] == 'SpoolA')
    assert rows(path, '/motors/default/' + motor_name + '/commanded_angle', 'Scalars:scalars')


def test_recording_is_read_only_and_copied_snapshot_does_not_alias_components(monkeypatch):
    monkeypatch.setitem(sys.modules, 'rerun', _fake_rerun())
    world = load_machine_world(ROOT / 'public/usd_scenes/hp4_rigid_body.usda')
    world.update(.002)
    state = pickle.dumps((world.components, world.resources))
    snapshot = capture_machine_snapshot(world)
    RerunSystem(Recording()).update(world, .002)
    assert pickle.dumps((world.components, world.resources)) == state
    snapshot['frames'][0]['position'][0] += 100
    snapshot['cables'][0]['segments'][0]['points'][0][0] += 100
    assert pickle.dumps((world.components, world.resources)) == state


def test_clock_resets_and_static_shapes_clear_when_style_or_scene_changes(monkeypatch):
    monkeypatch.setitem(sys.modules, 'rerun', _fake_rerun())
    world = World()
    world.set_resource('sceneGeneration', 1)
    entity = world.create_entity()
    world.add_component(entity, PositionComponent())
    world.add_component(entity, OrientationComponent(0, 0, .5, .5))
    world.add_component(entity, RadiusComponent(.1))
    world.add_component(entity, RenderableComponent('circle', '#abc'))
    stream = Recording()
    system = RerunSystem(stream)
    system.update(world, .1)
    world.remove_component(entity, RadiusComponent)
    world.remove_component(entity, OrientationComponent)
    system.update(world, 0)
    assert set(stream.static) == {'world'}
    transforms = [value for _, value, _ in stream.logs if value[0] == 'transform']
    assert transforms[-1][1]['rotation'][1]['xyzw'] == [0, 0, 0, 1]
    world.set_resource('pauseState', SimpleNamespace(paused=True))
    system.update(world, .1)
    assert (system.step, system.elapsed) == (1, .1)
    world.set_resource('sceneGeneration', 2)
    system.update(world, 0)
    assert (system.step, system.elapsed) == (0, 0)
    assert stream.times[-3:] == [('scene_generation', {'sequence': 2}), ('sim_step', {'sequence': 0}),
                                ('sim_time', {'duration': 0})]


def test_native_recording_registration_is_idempotent_and_rejects_a_second_stream(monkeypatch):
    monkeypatch.setitem(sys.modules, 'rerun', _fake_rerun())
    world = load_machine_world(ROOT / 'public/usd_scenes/hp4_rigid_body.usda')
    stream = Recording()
    register_machine_systems(world, stream)
    systems = list(world.systems)
    register_machine_systems(world, stream)
    assert world.systems == systems
    assert world.systems[-1].recording is stream
    with pytest.raises(ValueError, match='different recording stream'):
        register_machine_systems(world, Recording())
    assert world.systems == systems


def test_removing_a_native_path_clears_its_geometry_and_scalar_traces(monkeypatch):
    monkeypatch.setitem(sys.modules, 'rerun', _fake_rerun())
    world = load_machine_world(ROOT / 'public/usd_scenes/hp4_rigid_body.usda')
    stream = Recording()
    system = RerunSystem(stream)
    system.update(world, 0)
    from cable_joints_3d.cable_joints_components import CablePathComponent
    path_entity = world.query([CablePathComponent])[0]
    cable = capture_machine_snapshot(world)['cables'][0]
    world.destroy_entity(path_entity)
    system.update(world, 0)
    clears = {path for path, value, options in stream.logs if value[0] == 'clear' and not options.get('static')}
    key = f"{cable['machine']}/{cable['name']}"
    assert {f"world/machines/{cable['machine']}/cables/{cable['name']}/segments",
            f"world/machines/{cable['machine']}/cables/{cable['name']}/forces",
            'line_lengths/' + key + '/actual', 'line_errors/' + key + '/stretch',
            'cable_forces/' + key + '/' + cable['segments'][0]['name']} <= clears


def test_native_cli_records_authored_machine_commands_and_final_json(tmp_path):
    commands = tmp_path / 'commands.json'
    commands.write_text(json.dumps([{'type': 'Move', 'A': .0003, 'E': .001}]))
    output, snapshot = tmp_path / 'cli.rrd', tmp_path / 'final.json'
    result = subprocess.run([str(ROOT / '.venv/bin/python'), '-m', 'cable_joints_3d',
                             'public/usd_scenes/hp4_rigid_body.usda', '--steps', '3', '--dt', '.001',
                             '--commands', str(commands), '--output', str(output), '--snapshot', str(snapshot)],
                            cwd=ROOT, env={**os.environ, 'PYTHONPATH': str(ROOT / 'src/python')},
                            capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stderr
    assert '3 steps (0.003 s)' in result.stdout
    assert RrdReader(output).blueprints()
    data = json.loads(snapshot.read_text())
    assert data['frames'] and data['cables']
    assert any('/members/' in frame['path'] for frame in data['frames'])
    extruder = load_machine_world(ROOT / 'public/usd_scenes/hp4_rigid_body.usda').query([ExtruderComponent])[0]
    assert any(value > 0 for _, values in rows(output, f'/extrusion_lengths/default/{extruder}/deposited_length', 'Scalars:scalars') for value in values)


def test_native_rrd_keeps_joint_series_identity_through_split_merge(tmp_path):
    from cable_joints_3d.cable_joints_components import CablePathComponent
    from cable_joints_3d.cable_topology import merge_joints, split_joints
    from test_cable_topology_parity import make_topology_world

    output = tmp_path / 'topology.rrd'
    stream = rr.RecordingStream('topology recording test')
    stream.set_sinks(rr.FileSink(output))
    world, joint_id, path_id = make_topology_world()
    path = world.get_component(path_id, CablePathComponent)
    recording = RerunSystem(stream)
    try:
        recording.update(world, 0)
        split_joints(world)
        removed_id = path.joint_entities[1]
        recording.update(world, .002)
        path.stored[1] = -.01
        merge_joints(world)
        recording.update(world, .002)
        split_joints(world)
        created_id = path.joint_entities[1]
        recording.update(world, .002)
        stream.flush(timeout_sec=5)
    finally:
        stream.disconnect()
    cable = capture_machine_snapshot(world)['cables'][0]
    key = f"{cable['machine']}/{cable['name']}"
    force_path = lambda entity: f'/cable_forces/{key}/entity_{entity}'
    assert [step for step, _ in rows(output, force_path(joint_id), 'Scalars:scalars')] == [0, 1, 2, 3]
    assert [step for step, _ in rows(output, force_path(removed_id), 'Scalars:scalars')] == [1]
    assert rows(output, force_path(removed_id), 'Clear:is_recursive') == [(2, [True])]
    assert [step for step, _ in rows(output, force_path(created_id), 'Scalars:scalars')] == [3]
    strips = rows(output, f"/world/machines/{cable['machine']}/cables/{cable['name']}/segments", 'LineStrips3D:strips')
    assert [(step, len(value)) for step, value in strips] == [(0, 1), (1, 2), (2, 1), (3, 2)]
