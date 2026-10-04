"""Fixture plumbing and strict numerical comparison; physics stays in engines."""
import copy
import json
import math
import numbers
from pathlib import Path
import subprocess
from types import SimpleNamespace

import numpy as np
from cable_joints_3d import ecs, common_systems
from cable_joints_3d import rigid_bodies as rigid
from cable_joints_3d import geometry3 as geometry
from cable_joints_3d.quaternion import Quaternion
from cable_joints_3d.spools import SpoolStateComponent, SpoolTagComponent
from cable_joints_3d import cable_joints_components as cable
from cable_joints_3d.create_cable_paths import create_cable_paths
from cable_joints_3d.cable_attachment_cache_system import CableAttachmentCacheSystem
from cable_joints_3d.cable_friction_system import CableFrictionSystem
from cable_joints_3d.cable_attachment_update_system import CableAttachmentUpdateSystem
from cable_joints_3d.cable_layering import cable_stored_length_after_rotation
from cable_joints_3d.stepper_motor import StepperMotorComponent, StepperMotorSystem
from cable_joints_3d.pbd_cable_constraint_solver import PBDCableConstraintSolver
from cable_joints_3d.pbd_resolve_cable_over_corrections import PBDResolveCableOverCorrections
from cable_joints_3d.torque_mode_system import TorqueModeSystem
from cable_joints_3d.motor_diagnostics import MissedStepTrackingSystem, get_machine_motor_diagnostics, reset_machine_motor_diagnostics
from cable_joints_3d.extruder import ExtruderComponent, ExtruderSystem, estimate_effector_rotation
from cable_joints_3d.remote_spool_system import RemoteSpoolSystem

ROOT = Path(__file__).resolve().parents[3]
CONTRACT = json.loads((ROOT / 'tests/parity3d/contract.json').read_text())
GEOMETRY_CONTRACT = json.loads((ROOT / 'tests/parity3d/geometry_contract.json').read_text())
FIXTURES = ROOT / 'tests/fixtures/python_3d_parity'
QUATERNION_FIELDS = tuple(js for fields in CONTRACT.values() for js, _, kind in fields if kind == 'quaternion')


def run_python(fixture):
    fixture = copy.deepcopy(fixture)
    world = ecs.World()
    ids = {entity['name']: world.create_entity() for entity in fixture['entities']}
    for definition in fixture.get('scenes', []):
        from usd.cable_scene_loader import open_cable_scene
        from cable_joints_3d.machine_scene import populate_machine_scene

        bake_options = definition.get('bakeOptions', {})
        stage = open_cable_scene(definition.get('source') or ROOT / definition['path'],
            derive_all=bake_options.get('deriveAll', False),
            cable_path_half_width_override=bake_options.get('cablePathHalfWidthOverride'))
        options = definition.get('options', {})
        populate_machine_scene(world, stage, definition.get('scenePrimPath', '/World/SlideprinterScene'),
            namespace=options.get('namespace'), append=options.get('append', False), palette=options.get('palette'),
            tint_color=options.get('tintColor'), extrusion_color=options.get('extrusionColor'))
    if 'scenes' in fixture:
        for entity in world.entities:
            info = world.get_component(entity, ecs.SceneEntityInfoComponent)
            tag = world.get_component(entity, ecs.MachineTagComponent)
            name = f'{tag.id}::{info.name}' if info else f'@{entity}'
            assert name not in ids, f'Duplicate scene entity name {name}'
            ids[name] = entity
    names = {value: key for key, value in ids.items()}
    components = {name: getattr(ecs, name) for name in CONTRACT if hasattr(ecs, name)}
    components['SpoolStateComponent'] = SpoolStateComponent
    components['SpoolTagComponent'] = SpoolTagComponent
    components['ExtruderComponent'] = ExtruderComponent
    components['StepperMotorComponent'] = StepperMotorComponent
    components.update({name: getattr(cable, name) for name in CONTRACT if hasattr(cable, name)})

    def decode(value, kind):
        if value is None:
            return None
        if kind == 'vector':
            return np.array(value, dtype=float)
        if kind == 'quaternion':
            return Quaternion(*value)
        if kind == 'entity':
            return ids[value]
        if kind == 'entities':
            return [ids[name] for name in value]
        if kind == 'vectors':
            return [decode(point, 'vector') for point in value]
        if kind.endswith('Map') and kind != 'entityMap':
            return {key: decode(item, kind[:-3]) for key, item in value.items()}
        if kind == 'entityMap':
            return {name if name == '__default__' else str(ids[name]): angle for name, angle in value.items()}
        if kind == 'parameter' and value == 'Infinity':
            return math.inf
        return value

    def add(entity, name, args):
        component_type = components[name]
        if name == 'RigidBodyComponent':
            component = component_type([ids[name] for name in args[0]])
        elif name == 'RigidBodyMemberComponent':
            component = component_type(ids[args[0]], np.array(args[1]), Quaternion(*args[2]), args[3])
        elif name == 'DistanceConstraintComponent':
            component = component_type(ids[args[0]], ids[args[1]], *args[2:])
        elif name == 'SpoolStateComponent':
            component = component_type(args[0], decode(args[1], 'vector'), decode(args[2], 'quaternion'))
        elif name == 'MomentOfInertiaComponent':
            component = component_type(args[0], *[np.array(a) for a in args[1:]])
        elif name == 'EncoderComponent':
            component = component_type(args[0], np.array(args[1:4]))
        elif name == 'CableLinkComponent':
            component = component_type(*args[:3], decode(args[3], 'quaternion'), decode(args[4], 'vector'), decode(args[5], 'vector'))
        elif name == 'CableJointComponent':
            values = [ids[args[0]], ids[args[1]], args[2], np.array(args[3]), np.array(args[4])]
            component = component_type.from_local(world, *values) if args[5] == 'local' else component_type.from_world(*values)
        elif name == 'CablePathComponent':
            values = list(args[1:])
            values[2] = decode(values[2], 'parameter')
            component = cable.create_cable_path_component(world, [ids[name] for name in args[0]], *values)
        else:
            component = component_type(*args)
        world.add_component(ids[entity], component)

    def resources(values, entity_values=None, map_values=None):
        for key, value in (values or {}).items():
            if key in ('gravity', 'defaultPlaneNormal'):
                value = np.array(value, dtype=float)
            elif key == 'grabbedBall' and value is not None:
                value = ids[value]
            elif isinstance(value, dict):
                value = SimpleNamespace(**value)
            world.set_resource(key, value)
        for key, definition in (entity_values or {}).items():
            world.set_resource(key, {ids[name]: value for name, value in definition['values'].items()})
        for key, value in (map_values or {}).items():
            world.set_resource(key, value)

    def mutate(values):
        for entity, type_name, field, value in values or []:
            _, py, kind = next(f for f in CONTRACT[type_name] if f[0] == field)
            component = world.get_component(ids[entity], components[type_name])
            decoded = decode(value, kind)
            if kind == 'vector' and getattr(component, py) is not None and decoded is not None:
                getattr(component, py)[:] = decoded
            elif kind == 'quaternion' and getattr(component, py) is not None and decoded is not None:
                getattr(component, py).set(decoded)
            else:
                setattr(component, py, decoded)

    resources(fixture.get('resources'), fixture.get('entityResources'), fixture.get('mapResources'))
    for entity in fixture['entities']:
        for name, args in entity['components'].items():
            add(entity['name'], name, args)
    for addition in fixture.get('addComponents', []):
        add(*addition)
    for definition in fixture.get('createPaths', []):
        args = definition['args']
        values = list(args[1:])
        values[2] = decode(values[2], 'parameter')
        created = create_cable_paths(world, [ids[name] for name in args[0]], *values)
        assert len(created) == len(definition['names']), 'Unexpected number of split paths'
        for name, entity in zip(definition['names'], created):
            ids[name], names[entity] = entity, name
    mutate(fixture.get('initialSet'))
    for name in fixture.get('initializeRigidBodies', []):
        rigid.initialize_rigid_body_sync_state(world, ids[name])
    systems = {'CableAttachmentCacheSystem': CableAttachmentCacheSystem, 'CableFrictionSystem': CableFrictionSystem,
               'CableAttachmentUpdateSystem': CableAttachmentUpdateSystem, 'PBDCableConstraintSolver': PBDCableConstraintSolver,
               'PBDResolveCableOverCorrections': PBDResolveCableOverCorrections, 'StepperMotorSystem': StepperMotorSystem,
               'TorqueModeSystem': TorqueModeSystem, 'MissedStepTrackingSystem': MissedStepTrackingSystem,
               'ExtruderSystem': ExtruderSystem, 'RemoteSpoolSystem': RemoteSpoolSystem}
    for definition in fixture['systems']:
        name = definition if isinstance(definition, str) else definition['name']
        args = [] if isinstance(definition, str) else definition.get('args', [])
        system = systems.get(name)
        world.register_system((system if system is not None else getattr(common_systems, name))(*args))

    remote = world.get_system(RemoteSpoolSystem)
    if fixture.get('initializeExtruder'):
        world.get_system(ExtruderSystem).update(world, 0)
    events = []
    if 'commands' in fixture:
        remote.commands = fixture['commands']
    if fixture.get('observeCommands'):
        remote.set_command_executed_listener(lambda value: events.append({'kind': 'command', 'value': copy.deepcopy(value)}))
        remote.set_extrusion_listener(lambda value: events.append({'kind': 'extrusion', 'value': copy.deepcopy(value)}))

    def command_actions(actions):
        methods = {'clearCommandQueue': remote.clear_command_queue, 'clearPlaybackState': remote.clear_playback_state,
                   'resetAxisMapping': remote.reset_axis_mapping} if remote is not None else {}
        for action in actions or []:
            method = action['method']
            if method == 'processCommand':
                remote.process_command(world, action['command'], record_history=action.get('recordHistory', True), emit_events=action.get('emitEvents', True))
            elif method == 'setCommands':
                remote.commands = action['commands']
            elif method == 'addCommand':
                remote.add_command(action['command'])
            elif method == 'setPlaybackState':
                remote.set_playback_state(action['state'])
            else:
                methods[method]()

    def encode(value, kind):
        if value is None:
            return None
        if kind == 'entity':
            return names[value]
        if kind == 'entities':
            return [names[entity] for entity in value]
        if kind == 'booleans':
            return [bool(item) for item in value]
        if kind == 'vectors':
            return [encode(point, 'vector') for point in value]
        if kind.endswith('Map') and kind != 'entityMap':
            return {key: encode(item, kind[:-3]) for key, item in value.items()}
        if kind == 'entityMap':
            return {key if key == '__default__' else names[int(key)]: angle for key, angle in value.items()}
        if kind == 'quaternion':
            return value.as_xyzw().tolist()
        if kind == 'parameter' and value == math.inf:
            return 'Infinity'
        if isinstance(value, np.ndarray):
            return value.tolist()
        return copy.deepcopy(value)

    def snapshot(step):
        diagnostics = [get_machine_motor_diagnostics(world, machine) for machine in fixture.get('motorDiagnostics', [])]
        entities = {}
        for name, entity in ids.items():
            state = entities[name] = {}
            for type_name, fields in CONTRACT.items():
                if type_name not in components:
                    continue
                component = world.get_component(entity, components[type_name])
                if component is not None:
                    state[type_name] = {js: encode(getattr(component, py), kind)
                                        for js, py, kind in fields}
                    if type_name == 'CableJointComponent':
                        state[type_name]['geometricLength'] = float(np.linalg.norm(
                            component.attachment_point_a_world - component.attachment_point_b_world))
        queries = [[names[e] for e in world.query([components[t] for t in types])]
                   for types in fixture.get('queries', [])]
        attachments = []
        for probe in fixture.get('attachments', []):
            entity = ids[probe['entity']]
            point = rigid.compute_world_attachment(world, entity, np.array(probe['localPoint']))
            endpoint = rigid.resolve_rigid_body_solver_endpoint(world, entity, ids[probe['counterpart']], point)
            attachments.append({
                'worldPoint': encode(point, 'vector'),
                'localPoint': encode(rigid.compute_local_attachment(world, entity, point), 'vector'),
                'solverEntity': encode(endpoint.entity_id, 'entity'),
                'solverLocalPoint': encode(endpoint.local_point, 'vector'),
                'internalToBody': bool(endpoint.internal_to_body),
            })
        state = {'step': step, 'entities': entities, 'queries': queries, 'attachments': attachments}
        if 'motorDiagnostics' in fixture:
            state['motorDiagnostics'] = diagnostics
        if fixture.get('commandState'):
            state['commandState'] = copy.deepcopy(remote.get_playback_state())
            state['commandState']['queueLength'] = remote.get_queue_length()
            state['commandState']['axisToEntity'] = {axis: encode(value, 'entities' if isinstance(value, list) else 'entity')
                                                    for axis, value in remote.axis_to_entity.items()}
        if fixture.get('observeCommands'):
            state['commandEvents'] = copy.deepcopy(events)
        if 'effectorRotations' in fixture:
            state['effectorRotations'] = []
            for probe in fixture['effectorRotations']:
                component = world.get_component(ids[probe['extruder']], ExtruderComponent)
                machine = probe['machine']
                rotation = estimate_effector_rotation(component.center_source_offsets.get(machine),
                    component.machine_effector_centers.get(machine), component.center_sources.get(machine), world)
                state['effectorRotations'].append({'quaternion': encode(rotation, 'quaternion')})
        if 'snapshotResources' in fixture:
            state['resources'] = {key: encode(world.get_resource(key), 'vector') if key in ('gravity', 'defaultPlaneNormal')
                                  else world.get_resource(key) for key in fixture['snapshotResources']}
        if 'snapshotMapResources' in fixture:
            state['mapResources'] = {key: copy.deepcopy(world.get_resource(key) or {})
                                     for key in fixture['snapshotMapResources']}
        if 'snapshotEntityMaps' in fixture:
            state['entityMaps'] = {}
            for key in fixture['snapshotEntityMaps']:
                value = world.get_resource(key)
                state['entityMaps'][key] = None if value is None else {names[entity]: number for entity, number in value.items()}
        if 'cableRotations' in fixture:
            state['cableRotations'] = [cable_stored_length_after_rotation(
                world, world.get_component(ids[p['path']], cable.CablePathComponent),
                p['index'], ids[p['entity']], p['delta']) for p in fixture['cableRotations']]
        return state

    snapshots = [snapshot(0)]
    for index, step in enumerate(fixture['steps'], 1):
        resources(step.get('resources'), step.get('entityResources'), step.get('mapResources'))
        mutate(step.get('set'))
        for entity, type_name in step.get('removeComponents', []):
            world.remove_component(ids[entity], components[type_name])
        for machine in step.get('resetMotorDiagnostics', []):
            reset_machine_motor_diagnostics(world, machine)
        command_actions(step.get('commandActions'))
        world.update(step['dt'])
        snapshots.append(snapshot(index))
    result = {'schema': 1, 'snapshots': snapshots}
    if 'usdBake' in fixture:
        from usd.cable_scene_loader import bake_cable_stage, open_stage

        definition = fixture['usdBake']
        options = definition.get('options', {})
        stage = open_stage(definition.get('source') or ROOT / definition['path'])
        result['usdBake'] = bake_cable_stage(stage, derive_all=options.get('deriveAll', False),
            cable_path_half_width_override=options.get('cablePathHalfWidthOverride'))
    if 'geometry' in fixture:
        def plain(value):
            if isinstance(value, np.ndarray):
                return value.tolist()
            if isinstance(value, np.generic):
                return value.item()
            if isinstance(value, dict):
                return {key: plain(item) for key, item in value.items()}
            return value
        result['geometry'] = []
        for probe in fixture['geometry']:
            method, kinds = GEOMETRY_CONTRACT[probe['method']]
            args = [decode(arg, kind) for arg, kind in zip(probe['args'], kinds)]
            result['geometry'].append(plain(getattr(geometry, method)(*args)))
    json.dumps(result, allow_nan=False)
    return result


def run_js(fixture):
    result = subprocess.run(
        ['node', str(ROOT / 'tests/parity3d/oracle.mjs')],
        input=json.dumps(fixture, allow_nan=False), text=True,
        capture_output=True, cwd=ROOT, timeout=30,
    )
    assert result.returncode == 0, f'JS oracle failed:\n{result.stderr}'
    return json.loads(result.stdout)


def assert_equivalent(actual, expected, *, atol, rtol, path='state'):
    """Report a precise state path, rejecting shape differences and nonfinites."""
    pending = [(actual, expected, path)]
    while pending:
        actual, expected, path = pending.pop()
        if isinstance(expected, dict):
            assert isinstance(actual, dict) and actual.keys() == expected.keys(), f'{path}: keys differ'
            pending.extend((actual[key], expected[key], f'{path}.{key}') for key in reversed(expected))
        elif isinstance(expected, list):
            assert isinstance(actual, list) and len(actual) == len(expected), f'{path}: lengths differ'
            if path.endswith(QUATERNION_FIELDS):
                assert np.isfinite(actual).all() and np.isfinite(expected).all(), f'{path}: nonfinite'
                if np.dot(actual, expected) < 0:
                    actual = [-v for v in actual]
            pending.extend((actual[i], expected[i], f'{path}[{i}]') for i in reversed(range(len(expected))))
        elif isinstance(expected, numbers.Real) and not isinstance(expected, bool):
            assert isinstance(actual, numbers.Real) and not isinstance(actual, bool), f'{path}: numeric type differs'
            assert math.isfinite(actual) and math.isfinite(expected), f'{path}: nonfinite'
            assert math.isclose(actual, expected, abs_tol=atol, rel_tol=rtol), f'{path}: Python={actual}, JS={expected}, atol={atol}, rtol={rtol}'
        else:
            assert type(actual) is type(expected) and actual == expected, f'{path}: Python={actual!r}, JS={expected!r}'


if __name__ == '__main__':
    import sys
    fixture = json.loads(Path(sys.argv[1]).read_text())
    print(json.dumps(run_python(fixture), allow_nan=False))
