"""Fixture plumbing and strict numerical comparison; physics stays in engines."""
import json
import math
import numbers
from pathlib import Path
import subprocess
from types import SimpleNamespace

import numpy as np
from cable_joints_3d import ecs, common_systems
from cable_joints_3d.quaternion import Quaternion

ROOT = Path(__file__).resolve().parents[3]
CONTRACT = json.loads((ROOT / 'tests/parity3d/contract.json').read_text())
FIXTURES = ROOT / 'tests/fixtures/python_3d_parity'


def run_python(fixture):
    world = ecs.World()
    ids = {entity['name']: world.create_entity() for entity in fixture['entities']}
    names = {value: key for key, value in ids.items()}
    components = {name: getattr(ecs, name) for name in CONTRACT if hasattr(ecs, name)}
    # Extend this registry as simulation layers land; unknown systems fail loudly.
    if 'SpoolStateComponent' not in components:
        try:
            from cable_joints_3d.spools import SpoolStateComponent
        except ModuleNotFoundError:
            pass
        else:
            components['SpoolStateComponent'] = SpoolStateComponent

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
        else:
            component = component_type(*args)
        world.add_component(ids[entity], component)

    def resources(values):
        for key, value in (values or {}).items():
            if key in ('gravity', 'defaultPlaneNormal'):
                value = np.array(value, dtype=float)
            elif key == 'grabbedBall' and value is not None:
                value = ids[value]
            elif isinstance(value, dict):
                value = SimpleNamespace(**value)
            world.set_resource(key, value)

    resources(fixture.get('resources'))
    for entity in fixture['entities']:
        for name, args in entity['components'].items():
            add(entity['name'], name, args)
    for addition in fixture.get('addComponents', []):
        add(*addition)
    if fixture.get('initializeRigidBodies') or fixture.get('attachments'):
        from cable_joints_3d import rigid_bodies as rigid
        for name in fixture.get('initializeRigidBodies', []):
            rigid.initialize_rigid_body_sync_state(world, ids[name])
    for name in fixture['systems']:
        world.register_system(getattr(common_systems, name)())

    def encode(value, kind):
        if value is None:
            return None
        if kind == 'entity':
            return names[value]
        if kind == 'entities':
            return [names[entity] for entity in value]
        if kind == 'quaternion':
            return value.as_xyzw().tolist()
        if isinstance(value, np.ndarray):
            return value.tolist()
        return value

    def snapshot(step):
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
        return {'step': step, 'entities': entities, 'queries': queries, 'attachments': attachments}

    snapshots = [snapshot(0)]
    for index, step in enumerate(fixture['steps'], 1):
        resources(step.get('resources'))
        for entity, type_name, field, value in step.get('set', []):
            _, py, kind = next(f for f in CONTRACT[type_name] if f[0] == field)
            component = world.get_component(ids[entity], components[type_name])
            decoded = decode(value, kind)
            if kind == 'vector':
                getattr(component, py)[:] = decoded
            elif kind == 'quaternion':
                getattr(component, py).set(decoded)
            else:
                setattr(component, py, decoded)
        world.update(step['dt'])
        snapshots.append(snapshot(index))
    result = {'schema': 1, 'snapshots': snapshots}
    json.dumps(result, allow_nan=False)
    return result


def run_js(fixture):
    result = subprocess.run(
        ['node', str(ROOT / 'tests/parity3d/oracle.mjs')],
        input=json.dumps(fixture, allow_nan=False), text=True,
        capture_output=True, cwd=ROOT, timeout=30, check=True,
    )
    return json.loads(result.stdout)


def assert_equivalent(actual, expected, *, atol, rtol, path='state'):
    """Report a precise state path, rejecting shape differences and nonfinites."""
    if isinstance(expected, dict):
        assert isinstance(actual, dict) and actual.keys() == expected.keys(), f'{path}: keys differ'
        for key in expected:
            assert_equivalent(actual[key], expected[key], atol=atol, rtol=rtol, path=f'{path}.{key}')
    elif isinstance(expected, list):
        assert isinstance(actual, list) and len(actual) == len(expected), f'{path}: lengths differ'
        if path.endswith(('quaternion', 'localOrientation', 'syncedOrientation', 'referenceOrientation')):
            assert np.isfinite(actual).all() and np.isfinite(expected).all(), f'{path}: nonfinite'
            if np.dot(actual, expected) < 0:
                actual = [-v for v in actual]
        for index, value in enumerate(expected):
            assert_equivalent(actual[index], value, atol=atol, rtol=rtol, path=f'{path}[{index}]')
    elif isinstance(expected, numbers.Real) and not isinstance(expected, bool):
        assert isinstance(actual, numbers.Real) and not isinstance(actual, bool), f'{path}: numeric type differs'
        assert math.isfinite(actual) and math.isfinite(expected), f'{path}: nonfinite'
        assert math.isclose(actual, expected, abs_tol=atol, rel_tol=rtol), f'{path}: Python={actual}, JS={expected}, atol={atol}, rtol={rtol}'
    else:
        assert type(actual) is type(expected) and actual == expected, f'{path}: Python={actual!r}, JS={expected!r}'
