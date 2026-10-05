"""Ordered split/merge of specialized cable spans in live 3D guide frames."""
import math
import warnings

import numpy as np
from cable_joints.util import effective_cw, is_attachment, is_rolling

from .cable_attachment_update_system import effective_rolling_radius
from .cable_frames import get_plane_normal
from .cable_joints_components import CableJointComponent, CableLinkComponent, CablePathComponent
from .ecs import MachineTagComponent, PositionComponent, RadiusComponent, layering_enabled
from .geometry3 import line_segment_sphere_intersection, right_of_plane, signed_arc_length_on_wheel, tangent_from_point_to_sphere, tangent_from_sphere_to_point, tangent_from_sphere_to_sphere
from .rigid_bodies import get_entity_world_position

EPSILON = 1e-9


def _machine_id(world, entity):
    tag = world.get_component(entity, MachineTagComponent)
    return tag.id if tag is not None else ''


def effective_path_radius(world, path, entity, preferred_index=None):
    radius = world.get_component(entity, RadiusComponent)
    base = radius.radius if radius is not None else None
    if base is None or not math.isfinite(base) or base <= EPSILON:
        return base
    indices = []
    if preferred_index is not None and 0 <= preferred_index < len(path.link_types):
        indices.append(preferred_index)
    if path.joint_entities:
        first = world.get_component(path.joint_entities[0], CableJointComponent)
        last = world.get_component(path.joint_entities[-1], CableJointComponent)
        if first is not None and first.entity_a == entity:
            indices.append(0)
        for index in range(1, len(path.link_types) - 1):
            left = world.get_component(path.joint_entities[index - 1], CableJointComponent)
            right = world.get_component(path.joint_entities[index], CableJointComponent)
            if left is not None and right is not None and left.entity_b == entity and right.entity_a == entity:
                indices.append(index)
        if last is not None and last.entity_b == entity:
            indices.append(len(path.link_types) - 1)
    # A guide not yet on this path retains its raw radius, as in the reference.
    return max([base, *[effective_rolling_radius(world, path, index, base)[0] for index in indices]])


def merge_joints(world):
    for path_id in world.query([CablePathComponent]):
        path = world.get_component(path_id, CablePathComponent)
        rerun = True
        while rerun:
            rerun = False
            index = 0
            while index < len(path.joint_entities) - 1:
                first = world.get_component(path.joint_entities[index], CableJointComponent)
                second_id = path.joint_entities[index + 1]
                second = world.get_component(second_id, CableJointComponent)
                if path.link_types[index + 1] != 'rolling':
                    index += 1
                    continue
                if first.entity_b != second.entity_a:
                    warnings.warn('Merge loop saw disconnected cable path', stacklevel=2)
                    index += 1
                    continue
                if (not layering_enabled(world) and first.entity_a == second.entity_b) or not path.stored[index + 1] < 0:
                    index += 1
                    continue
                point_a, point_b = first.attachment_point_a_world, second.attachment_point_b_world
                pos_a, pos_b = get_entity_world_position(world, first.entity_a), get_entity_world_position(world, second.entity_b)
                radius_a = effective_path_radius(world, path, first.entity_a, index)
                radius_b = effective_path_radius(world, path, second.entity_b, index + 2)
                normal_a, normal_b = get_plane_normal(world, first.entity_a), get_plane_normal(world, second.entity_b)
                cw_a, cw_b = effective_cw(path, index, True), path.cw[index + 2]
                rolling_a, rolling_b = is_rolling(path.link_types[index]), is_rolling(path.link_types[index + 2])
                attachment_a, attachment_b = is_attachment(path.link_types[index]), is_attachment(path.link_types[index + 2])
                first.rest_length += second.rest_length + path.stored[index + 1]
                first.entity_b = second.entity_b
                new_a, new_b = point_a, point_b
                if rolling_a and rolling_b:
                    tangent = tangent_from_sphere_to_sphere(pos_a, radius_a, cw_a, pos_b, radius_b, cw_b, normal_a)
                    new_a, new_b = tangent['a_sphere'], tangent['b_sphere']
                elif rolling_a and attachment_b:
                    new_a = tangent_from_sphere_to_point(point_b, pos_a, radius_a, normal_a, cw_a)['a_sphere']
                elif attachment_a and rolling_b:
                    new_b = tangent_from_point_to_sphere(point_a, pos_b, radius_b, normal_b, cw_b)['a_sphere']
                shift_a = signed_arc_length_on_wheel(point_a, new_a, pos_a, radius_a, cw_a, normal_a) if rolling_a else 0.
                shift_b = signed_arc_length_on_wheel(point_b, new_b, pos_b, radius_b, cw_b, normal_b) if rolling_b else 0.
                path.stored[index] += shift_a
                first.rest_length -= shift_a
                path.stored[index + 2] -= shift_b
                first.rest_length += shift_b
                rerun = path.stored[index] < 0 or path.stored[index + 2] < 0
                first.attachment_point_a_world[:] = new_a
                first.attachment_point_b_world[:] = new_b
                for values in (path.joint_entities, path.stored, path.cw, path.link_types):
                    del values[index + 1]
                world.destroy_entity(second_id)
                # Preserve the reference's traversal of the now shorter list.
                index += 1


def split_joints(world):
    splitters = world.query([PositionComponent, RadiusComponent, CableLinkComponent])
    for path_id in world.query([CablePathComponent]):
        path = world.get_component(path_id, CablePathComponent)
        machine = _machine_id(world, path_id)
        index = 0
        while index < len(path.joint_entities):
            joint = world.get_component(path.joint_entities[index], CableJointComponent)
            # These are live array references: another splitter in this same
            # inner loop must see the kept span after an earlier split.
            point_a, point_b = joint.attachment_point_a_world, joint.attachment_point_b_world
            for splitter in splitters:
                if splitter in (joint.entity_a, joint.entity_b) or _machine_id(world, splitter) != machine:
                    continue
                center = get_entity_world_position(world, splitter)
                radius = effective_path_radius(world, path, splitter)
                if not line_segment_sphere_intersection(point_a, point_b, center, radius):
                    continue
                entity_a, entity_b = joint.entity_a, joint.entity_b
                pos_a, pos_b = get_entity_world_position(world, entity_a), get_entity_world_position(world, entity_b)
                radius_a = effective_path_radius(world, path, entity_a, index)
                radius_b = effective_path_radius(world, path, entity_b, index + 1)
                normal_a, normal_b = get_plane_normal(world, entity_a), get_plane_normal(world, entity_b)
                normal = get_plane_normal(world, splitter)
                previous = world.get_component(splitter, CableLinkComponent).prev_cable_attachment_time_pos
                cw = right_of_plane(previous, point_a, point_b, normal)
                cw_a, cw_b = effective_cw(path, index, True), path.cw[index + 1]
                rolling_a, rolling_b = is_rolling(path.link_types[index]), is_rolling(path.link_types[index + 1])
                if rolling_a:
                    tangent = tangent_from_sphere_to_sphere(pos_a, radius_a, cw_a, center, radius, cw, normal_a)
                    new_a, inlet = tangent['a_sphere'], tangent['b_sphere']
                elif is_attachment(path.link_types[index]):
                    tangent = tangent_from_point_to_sphere(point_a, center, radius, normal, cw)
                    new_a, inlet = tangent['a_attach'], tangent['a_sphere']
                else:
                    warnings.warn('Unsupported split inlet link type', stacklevel=2)
                    continue
                if rolling_b:
                    tangent = tangent_from_sphere_to_sphere(center, radius, cw, pos_b, radius_b, cw_b, normal)
                    outlet, new_b = tangent['a_sphere'], tangent['b_sphere']
                elif is_attachment(path.link_types[index + 1]):
                    tangent = tangent_from_sphere_to_point(point_b, center, radius, normal, cw)
                    outlet, new_b = tangent['a_sphere'], tangent['a_attach']
                else:
                    raise ValueError('Unsupported split outlet link type')
                shift_a = signed_arc_length_on_wheel(point_a, new_a, pos_a, radius_a, cw_a, normal_a) if rolling_a else 0.
                shift_b = signed_arc_length_on_wheel(point_b, new_b, pos_b, radius_b, cw_b, normal_b) if rolling_b else 0.
                wrap = signed_arc_length_on_wheel(inlet, outlet, center, radius, cw, normal)
                if wrap <= 0 or wrap + EPSILON >= 2 * math.pi * radius:
                    warnings.warn('Nonpositive or full guide wrap; split aborted', stacklevel=2)
                    continue
                distance_a, distance_b = np.linalg.norm(new_a - inlet), np.linalg.norm(outlet - new_b)
                distance = distance_a + distance_b
                available = joint.rest_length + shift_b - shift_a - wrap
                rest_a, rest_b = 0., 0.
                if distance > EPSILON:
                    if available < EPSILON:
                        warnings.warn('Insufficient available rest length; split aborted', stacklevel=2)
                        continue
                    rest_a, rest_b = available * distance_a / distance, available * distance_b / distance
                else:
                    warnings.warn('Near-zero split segment distances', stacklevel=2)
                new_id = world.create_entity()
                world.add_component(new_id, MachineTagComponent(machine))
                path.stored[index + 1] -= shift_b
                path.stored[index] += shift_a
                path.joint_entities.insert(index + 1, new_id)
                path.cw.insert(index + 1, bool(cw))
                path.link_types.insert(index + 1, 'rolling')
                path.stored.insert(index + 1, wrap)
                joint.entity_b, joint.rest_length = splitter, rest_a
                joint.attachment_point_a_world[:] = new_a
                joint.attachment_point_b_world[:] = inlet
                world.add_component(new_id, CableJointComponent(splitter, entity_b, rest_b, outlet, new_b))
            index += 1
