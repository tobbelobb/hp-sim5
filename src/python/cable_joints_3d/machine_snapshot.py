"""Detached recording data from the live specialized Hangprinter ECS."""
import math
import re

import numpy as np

from .cable_joints_components import CableJointComponent, CablePathComponent
from .cable_layering import cable_stored_length_after_rotation
from .ecs import EncoderComponent, MachineTagComponent, PositionComponent, RenderableComponent, RigidBodyMemberComponent, SceneEntityInfoComponent
from .extruder import ExtruderComponent, estimate_effector_rotation
from .machine_runtime import object_keys
from .quaternion import Quaternion
from .rigid_bodies import get_entity_world_orientation, get_entity_world_position
from .stepper_motor import StepperMotorComponent


def path_part(value):
    return re.sub(r'[^a-zA-Z0-9_.-]', '_', str(value))


def entity_name(world, entity):
    info = world.get_component(entity, SceneEntityInfoComponent)
    return f'{path_part(info.name if info and info.name else "entity")}_{entity}'


def machine_id(world, entity):
    tag = world.get_component(entity, MachineTagComponent)
    return tag.id if tag is not None and tag.id else 'default'


def frame_path(world, entity):
    children, visited = [], set()
    while (member := world.get_component(entity, RigidBodyMemberComponent)) is not None:
        if entity in visited:
            raise ValueError('Cyclic rigid-member frame hierarchy')
        visited.add(entity)
        children.insert(0, 'members/' + entity_name(world, entity))
        entity = member.body_entity
    info = world.get_component(entity, SceneEntityInfoComponent)
    category = 'anchors' if info is not None and 'Anchor' in info.tags else 'bodies'
    return '/'.join(['world/machines', path_part(machine_id(world, entity)), category,
                     entity_name(world, entity), *children])


def _frames(world):
    frames = []
    for entity in world.query([PositionComponent]):
        member = world.get_component(entity, RigidBodyMemberComponent)
        info = world.get_component(entity, SceneEntityInfoComponent)
        position = member.local_position if member is not None else get_entity_world_position(world, entity)
        orientation = member.local_orientation if member is not None else get_entity_world_orientation(world, entity)
        frames.append({'path': frame_path(world, entity), 'name': info.name if info and info.name else f'entity {entity}',
                       'position': position.tolist(), 'quaternion': (orientation or Quaternion()).as_xyzw().tolist(),
                       'kind': 'anchor' if info is not None and 'Anchor' in info.tags else 'body'})
    for entity in world.query([ExtruderComponent]):
        extruder = world.get_component(entity, ExtruderComponent)
        for machine in object_keys(extruder.center_sources):
            sources = extruder.center_sources[machine]
            positions = [get_entity_world_position(world, source) for source in sources]
            positions = [position for position in positions if position is not None]
            if not positions:
                continue
            center = sum(positions, np.zeros(3)) / len(positions)
            members = [world.get_component(source, RigidBodyMemberComponent) for source in sources]
            parent = members[0].body_entity if members[0] is not None else None
            common_body = parent is not None and all(member is not None and member.body_entity == parent for member in members)
            orientation = (get_entity_world_orientation(world, parent) if common_body else
                           estimate_effector_rotation(extruder.center_source_offsets.get(machine), center, sources, world))
            frames.append({'path': f'world/machines/{path_part(machine)}/effector', 'name': 'Effector',
                           'position': center.tolist(), 'quaternion': (orientation or Quaternion()).as_xyzw().tolist(), 'kind': 'effector'})
    return frames


def _cables(world):
    cables = []
    for entity in world.query([CablePathComponent]):
        path = world.get_component(entity, CablePathComponent)
        joints = [world.get_component(joint, CableJointComponent) for joint in path.joint_entities]
        segments = []
        for joint_id, joint in zip(path.joint_entities, joints):
            delta = joint.attachment_point_b_world - joint.attachment_point_a_world
            length = float(np.linalg.norm(delta))
            force = max(0., joint.constraint_force_magnitude or 0., joint.transferred_constraint_force_magnitude or 0.)
            segments.append({'name': entity_name(world, joint_id),
                             'points': [joint.attachment_point_a_world.tolist(), joint.attachment_point_b_world.tolist()],
                             'rest_length': joint.rest_length, 'geometric_length': length, 'force_n': force,
                             'force_vector_n': (delta * (force / length) if length > 0 else np.zeros(3)).tolist(),
                             'origin': joint.attachment_point_a_world.tolist()})
        guides = sum(path.stored[1:-1])
        actual = sum((segment['rest_length'] for segment in segments), guides)
        geometric = sum((segment['geometric_length'] for segment in segments), guides)
        commanded = actual
        if joints:
            for index, endpoint in [(0, joints[0].entity_a), (len(path.link_types) - 1, joints[-1].entity_b)]:
                motor = world.get_component(endpoint, StepperMotorComponent)
                if motor is None:
                    continue
                encoder = world.get_component(endpoint, EncoderComponent)
                if motor.torque_mode or encoder is None or not math.isfinite(encoder.angle):
                    commanded = None
                    break
                measured = encoder.angle - (motor.missed_step_encoder_offset or 0.)
                target = motor.commanded_angle - motor.delta_angle
                target_stored = cable_stored_length_after_rotation(world, path, index, endpoint, target - measured)
                commanded -= target_stored - (path.stored[index] or 0.)
        render = world.get_component(path.joint_entities[0], RenderableComponent) if joints else None
        cables.append({'machine': path_part(machine_id(world, entity)), 'name': entity_name(world, entity),
                       'color': render.color if render is not None and render.color else '#ffff00', 'segments': segments,
                       'lengths': {'commanded': commanded, 'actual': actual, 'geometric': geometric,
                                   'error': None if commanded is None else actual - commanded, 'stretch': geometric - actual}})
    return cables


def capture_machine_snapshot(world):
    """Read without sync/mutation. Native visuals use straight constraint spans.

    Lengths include stored intermediate guide wraps; browser sag/wrap tessellation
    is intentionally presentation-only. Cable endpoints retain solver sample time.
    """
    return {'frames': _frames(world), 'cables': _cables(world)}
