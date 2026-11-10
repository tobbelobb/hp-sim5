"""Detached observation model for the Rust physics worker.

Python never advances physics. This module only stores the latest worker state so
telemetry, recording, encoder reads and tests share one small data model.
"""
from __future__ import annotations

import math
import numpy as np


def vec(values) -> np.ndarray:
    return np.asarray(values, dtype=float)


class Quaternion:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x, self.y, self.z, self.w = float(x), float(y), float(z), float(w)

    def as_xyzw(self):
        return [self.x, self.y, self.z, self.w]


class _Component:
    def __init__(self, **values):
        for key, value in values.items():
            setattr(self, key, value)


class PositionComponent(_Component): pass
class PrevFinalPosComponent(_Component): pass
class VelocityComponent(_Component): pass
class AngularVelocityComponent(_Component): pass
class OrientationComponent(_Component): pass
class PrevFinalOrientationComponent(_Component): pass
class EncoderComponent(_Component): pass
class RigidBodyMemberComponent(_Component): pass
class StepperMotorComponent(_Component): pass
class CableJointComponent(_Component): pass
class CablePathComponent(_Component): pass
class ExtruderComponent(_Component): pass
class RadiusComponent(_Component): pass
class MachineTagComponent(_Component): pass
class SceneEntityInfoComponent(_Component): pass
class RenderableComponent(_Component): pass
class SpoolStateComponent(_Component): pass
class RigidBodyComponent(_Component): pass


COMPONENTS = {component.__name__: component for component in (
    PositionComponent, PrevFinalPosComponent, VelocityComponent, AngularVelocityComponent,
    OrientationComponent, PrevFinalOrientationComponent, EncoderComponent, RigidBodyMemberComponent,
    StepperMotorComponent, CableJointComponent, CablePathComponent, ExtruderComponent,
    RadiusComponent, MachineTagComponent, SceneEntityInfoComponent, RenderableComponent,
    SpoolStateComponent, RigidBodyComponent,
)}
VECTOR_FIELDS = {'pos', 'vel', 'omega', 'local_position', 'attachment_point_a_world',
                 'attachment_point_b_world', 'constraint_force'}
QUATERNION_FIELDS = {'quaternion', 'local_orientation'}


def _decode(key, value):
    if value is None:
        return None
    if key in VECTOR_FIELDS:
        return vec(value)
    if key in QUATERNION_FIELDS:
        return Quaternion(*value)
    if key in ('machine_effector_centers', 'machine_centers', 'machine_tips', 'machine_cold_ends'):
        return {machine: vec(position) for machine, position in value.items()}
    if key == 'extrusions':
        return [{**record, 'pos': vec(record['pos'])} for record in value]
    return value


class World:
    def __init__(self):
        self.entities = {}
        self.components = {}
        self.resources = {}

    def query(self, component_classes):
        stores = [self.components.get(component) for component in component_classes]
        if any(store is None for store in stores):
            return []
        stores.sort(key=len)
        result = []
        for entity in stores[0]:
            if all(entity in store for store in stores[1:]):
                result.append(entity)
        return result

    def get_component(self, entity, component_class):
        store = self.components.get(component_class)
        return None if store is None else store.get(entity)

    def has_component(self, entity, component_class):
        store = self.components.get(component_class)
        return bool(store and entity in store)

    def add_component(self, entity, component):
        self.components.setdefault(type(component), {})[entity] = component
        self.entities.setdefault(entity, set()).add(type(component))

    def get_resource(self, key):
        return self.resources.get(key)

    def set_resource(self, key, value):
        self.resources[key] = value

    def clear(self):
        self.entities.clear()
        self.components.clear()


def apply_state(world: World, state: dict):
    """Replace detached observations with one worker state snapshot."""
    seen = set()
    for name, rows in state.get('components', []):
        component_class = COMPONENTS.get(name)
        if component_class is None:
            continue
        store = world.components.setdefault(component_class, {})
        for entity, values in rows:
            seen.add((entity, component_class))
            component = store.get(entity)
            if component is None:
                component = component_class()
                store[entity] = component
                world.entities.setdefault(entity, set()).add(component_class)
            for key, value in values.items():
                setattr(component, key, _decode(key, value))
    for component_class, store in world.components.items():
        for entity in list(store):
            if (entity, component_class) not in seen:
                del store[entity]
                world.entities.get(entity, set()).discard(component_class)
    for entity in list(world.entities):
        if not world.entities[entity]:
            del world.entities[entity]
    for key in ('enableLayering', 'layeringFrictionEffectiveRadius', 'layeringRenderWraps'):
        if key in state.get('layering_flags', {}):
            world.set_resource(key, state['layering_flags'][key])
    if state.get('scene_generation') is not None:
        world.set_resource('sceneGeneration', state['scene_generation'])
    return world


class RemoteSpoolSystem:
    """Command queue mirror. The Rust worker owns execution."""

    def __init__(self):
        self.commands = []
        self.axis_to_entity = {}
        self._queue_length = 0

    def add_command(self, command):
        self.commands.append(command)
        self._queue_length += 1

    def clear_command_queue(self):
        self.commands = []
        self._queue_length = 0

    def set_queue_length(self, value):
        self._queue_length = int(value)

    def get_queue_length(self):
        return self._queue_length


def entity_name(world, entity):
    info = world.get_component(entity, SceneEntityInfoComponent)
    return info.name if info is not None and info.name else f'entity {entity}'


def machine_id(world, entity):
    tag = world.get_component(entity, MachineTagComponent)
    return tag.id if tag is not None and tag.id else 'default'


def world_position(world, entity):
    position = world.get_component(entity, PositionComponent)
    if position is None:
        return None
    return np.asarray(position.pos, dtype=float)


def world_orientation(world, entity):
    orientation = world.get_component(entity, OrientationComponent)
    return Quaternion(*orientation.quaternion.as_xyzw()) if orientation is not None else None


def _path_part(value):
    import re
    return re.sub(r'[^a-zA-Z0-9_.-]', '_', str(value))


def capture_snapshot(world):
    frames = []
    for entity in world.query([PositionComponent]):
        info = world.get_component(entity, SceneEntityInfoComponent)
        member = world.get_component(entity, RigidBodyMemberComponent)
        position = world_position(world, member.body_entity if member is not None else entity)
        orientation = world_orientation(world, member.body_entity if member is not None else entity)
        frames.append({'path': f'world/machines/{_path_part(machine_id(world, entity))}/bodies/{_path_part(entity_name(world, entity))}',
                       'name': info.name if info is not None and info.name else f'entity {entity}',
                       'position': position.tolist(), 'quaternion': (orientation or Quaternion()).as_xyzw(),
                       'kind': 'anchor' if info is not None and 'Anchor' in (info.tags or []) else 'body'})
    for entity in world.query([ExtruderComponent]):
        extruder = world.get_component(entity, ExtruderComponent)
        for machine, center in getattr(extruder, 'machine_effector_centers', {}).items():
            frames.append({'path': f'world/machines/{_path_part(machine)}/effector', 'name': 'Effector',
                           'position': np.asarray(center, dtype=float).tolist(),
                           'quaternion': Quaternion().as_xyzw(), 'kind': 'effector'})
    cables = []
    for entity in world.query([CablePathComponent]):
        path = world.get_component(entity, CablePathComponent)
        joints = [world.get_component(joint, CableJointComponent) for joint in path.joint_entities]
        segments = []
        for joint_id, joint in zip(path.joint_entities, joints):
            if joint is None:
                continue
            a, b = np.asarray(joint.attachment_point_a_world), np.asarray(joint.attachment_point_b_world)
            delta = b - a
            length = float(np.linalg.norm(delta))
            force = max(0., joint.constraint_force_magnitude or 0., joint.transferred_constraint_force_magnitude or 0.)
            segments.append({'name': _path_part(entity_name(world, joint_id)),
                             'points': [a.tolist(), b.tolist()], 'rest_length': joint.rest_length,
                             'geometric_length': length, 'force_n': force,
                             'force_vector_n': (delta * (force / length) if length > 0 else np.zeros(3)).tolist(),
                             'origin': a.tolist()})
        guides = sum(path.stored[1:-1]) if path.stored else 0.
        actual = sum(segment['rest_length'] for segment in segments) + guides
        geometric = sum(segment['geometric_length'] for segment in segments) + guides
        render = world.get_component(path.joint_entities[0], RenderableComponent) if path.joint_entities else None
        cables.append({'machine': _path_part(machine_id(world, entity)), 'name': _path_part(entity_name(world, entity)),
                       'color': render.color if render is not None and render.color else '#ffff00',
                       'segments': segments, 'lengths': {'commanded': actual, 'actual': actual,
                       'geometric': geometric, 'error': actual - actual, 'stretch': geometric - actual}})
    return {'frames': frames, 'cables': cables}


def sample(world, step, dt):
    snapshot = capture_snapshot(world)
    motors = []
    for entity in world.query([StepperMotorComponent]):
        motor = world.get_component(entity, StepperMotorComponent)
        encoder = world.get_component(entity, EncoderComponent)
        axis = world.get_component(entity, SpoolStateComponent)
        measured = None if encoder is None else encoder.angle - (motor.missed_step_encoder_offset or 0.)
        target = motor.commanded_angle - motor.delta_angle
        motors.append({'name': entity_name(world, entity), 'axis': getattr(axis, 'axis', None),
                       'mode': 'torque' if motor.torque_mode else 'position',
                       'target_angle_rad': None if motor.torque_mode else target,
                       'encoder_angle_rad': None if encoder is None else encoder.angle,
                       'diagnostic_encoder_angle_rad': measured,
                       'diagnostic_encoder_offset_rad': motor.missed_step_encoder_offset or 0.,
                       'tracking_error_rad': None if motor.torque_mode or measured is None else measured - target,
                       'peak_missed_steps': motor.missed_steps})
    return {'step': step, 'sim_time_s': step * dt,
            'effectors': [frame for frame in snapshot['frames'] if frame['kind'] == 'effector'],
            'motors': motors,
            'cables': [{'name': f"{cable['machine']}/{cable['name']}", 'lengths_m': cable['lengths'],
                        'segment_forces_n': [segment['force_n'] for segment in cable['segments']]}
                       for cable in snapshot['cables']]}
