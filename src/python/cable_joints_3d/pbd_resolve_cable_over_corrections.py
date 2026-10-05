"""Average shared cable over-corrections with the reference's member reactions."""
from dataclasses import dataclass

import numpy as np
from .vector3 import cross, length as norm3
from cable_joints.util import is_hybrid

from .cable_attachment_update_system import calculate_attachment_points
from .cable_joints_components import CableJointComponent, CableLinkComponent, CablePathComponent
from .ecs import MassComponent, MomentOfInertiaComponent, OrientationComponent, PositionComponent
from .inertia_tensor import apply_world_inverse_inertia, has_any_inverse_inertia, inverse_inertia_quadratic_form
from .pbd_cable_constraint_solver import has_axis_only_cable_spin_dof
from .rigid_bodies import (
    apply_world_angular_correction, compute_world_attachment, get_entity_world_position,
    get_rigid_body_entity_for_member, resolve_rigid_body_solver_endpoint, update_rigid_body_member_local_orientation,
)
from .stepper_motor import finite_number

EPSILON = 1e-9


@dataclass
class AngularContribution:
    entity: int
    moment: object
    quaternion: object
    gradient: np.ndarray
    denominator: float


def _angular_contribution(world, entity, gradient):
    moment = world.get_component(entity, MomentOfInertiaComponent)
    orientation = world.get_component(entity, OrientationComponent)
    if moment is None or orientation is None:
        return None
    denominator = inverse_inertia_quadratic_form(moment, orientation.quaternion, gradient)
    return AngularContribution(entity, moment, orientation.quaternion, gradient, denominator)


def _member_angular_contribution(world, entity, point, gradient):
    link = world.get_component(entity, CableLinkComponent)
    moment = world.get_component(entity, MomentOfInertiaComponent)
    center = get_entity_world_position(world, entity)
    if link is None or link.cable_plane_normal_local is None or center is None or not has_any_inverse_inertia(moment):
        return None
    return _angular_contribution(world, entity, cross(point - center, gradient))


def _external_member_contribution(world, path, index, first, entity, mapped, point, gradient):
    if mapped.internal_to_body or mapped.entity_id == entity:
        return None
    direct = _member_angular_contribution(world, entity, point, gradient)
    if direct is not None:
        return direct
    link_index = index if first else index + 1
    if len(path.joint_entities) < 2 or path.link_types[link_index] != 'pinhole':
        return None
    if first and link_index > 0 and is_hybrid(path.link_types[link_index - 1]):
        internal_index, spin_first = index - 1, True
    elif not first and link_index < len(path.link_types) - 1 and is_hybrid(path.link_types[link_index + 1]):
        internal_index, spin_first = index + 1, False
    else:
        return None
    if not 0 <= internal_index < len(path.joint_entities):
        return None
    internal = world.get_component(path.joint_entities[internal_index], CableJointComponent)
    if internal is None:
        return None
    spin_entity = internal.entity_a if spin_first else internal.entity_b
    spin_point = internal.attachment_point_a_world if spin_first else internal.attachment_point_b_world
    gradient = point - spin_point
    if np.dot(gradient, gradient) <= EPSILON:
        return None
    gradient = gradient / norm3(gradient)
    return _member_angular_contribution(world, spin_entity, spin_point, gradient)


@dataclass
class CorrectionEnd:
    entity: int
    gradient: np.ndarray
    inv_mass: float
    angular: AngularContribution | None
    reaction: AngularContribution | None
    member: AngularContribution | None

    def denominator(self):
        result = self.inv_mass * np.dot(self.gradient, self.gradient)
        for contribution in (self.angular, self.reaction, self.member):
            if contribution is not None and contribution.denominator > 0:
                result += contribution.denominator
        return result


def _correction_end(world, path, index, first, entity, counterpart, point, direction):
    mapped = resolve_rigid_body_solver_endpoint(world, entity, counterpart, point)
    solver_point = compute_world_attachment(world, mapped.entity_id, mapped.local_point)
    position = world.get_component(mapped.entity_id, PositionComponent)
    if solver_point is None or position is None:
        return None
    mass = world.get_component(mapped.entity_id, MassComponent)
    inv_mass = 1 / mass.mass if mass is not None and finite_number(mass.mass) and mass.mass > 0 else 0.
    angular_gradient = cross(solver_point - position.pos, direction)
    angular = None if has_axis_only_cable_spin_dof(world, mapped.entity_id) else _angular_contribution(world, mapped.entity_id, angular_gradient)
    body = get_rigid_body_entity_for_member(world, entity)
    reaction = _angular_contribution(world, body, angular_gradient) if body is not None else None
    gradient = -direction
    member = _external_member_contribution(world, path, index, first, entity, mapped, point, gradient)
    return CorrectionEnd(mapped.entity_id, gradient, inv_mass, angular, reaction, member)


def _calculate_joint_correction(world, joint, path, index, position_corrections, angular_corrections):
    points = calculate_attachment_points(world, joint, path, index)
    if any(point is None for point in points):
        return
    length = norm3(points[1] - points[0])
    error = length - joint.rest_length
    if error >= -EPSILON or length <= EPSILON:
        return
    direction = (points[1] - points[0]) / length
    first = _correction_end(world, path, index, True, joint.entity_a, joint.entity_b, points[0], direction)
    second = _correction_end(world, path, index, False, joint.entity_b, joint.entity_a, points[1], -direction)
    if first is None or second is None:
        return
    denominator = first.denominator() + second.denominator()
    dt = world.get_resource('dt')
    if dt is not None and dt > 0:
        denominator += path.compliance / (dt * dt)
    if denominator <= EPSILON:
        return
    multiplier = -error / denominator
    for end in (first, second):
        if end.inv_mass > 0:
            position_corrections.setdefault(end.entity, []).append(end.gradient * (end.inv_mass * multiplier))
        # Preserve JS ordering and opposite host/member angular signs. These
        # contributions may share an entity and cancel when averaged.
        for contribution, sign in ((end.angular, -1), (end.member, -1), (end.reaction, 1)):
            if contribution is not None and contribution.denominator > 0:
                delta = apply_world_inverse_inertia(contribution.moment, contribution.quaternion, contribution.gradient) * (sign * multiplier)
                angular_corrections.setdefault(contribution.entity, []).append(delta)


class PBDResolveCableOverCorrections:
    def update(self, world, dt_unused):
        joint_paths = {}
        for path_entity in world.query([CablePathComponent]):
            path = world.get_component(path_entity, CablePathComponent)
            for index, joint in enumerate(path.joint_entities):
                # First insertion orders the scan; last path wins metadata.
                joint_paths[joint] = path, index
        over_corrected = []
        for joint_entity, (path, index) in joint_paths.items():
            joint = world.get_component(joint_entity, CableJointComponent)
            before = norm3(joint.attachment_point_b_world - joint.attachment_point_a_world)
            if before >= joint.rest_length:
                points = calculate_attachment_points(world, joint, path, index)
                if all(point is not None for point in points) and norm3(points[1] - points[0]) < joint.rest_length:
                    over_corrected.append(joint_entity)
        if len(over_corrected) < 2:
            return
        positions, angles = {}, {}
        for entity in over_corrected:
            path, index = joint_paths[entity]
            _calculate_joint_correction(world, world.get_component(entity, CableJointComponent), path, index, positions, angles)
        for entity, deltas in positions.items():
            position = world.get_component(entity, PositionComponent)
            if len(deltas) >= 2 and position is not None:
                position.pos += sum(deltas, np.zeros(3)) / len(deltas)
        for entity, deltas in angles.items():
            if len(deltas) >= 2:
                # JS refreshes member-local orientation even for zero averages.
                delta = sum(deltas, np.zeros(3)) / len(deltas)
                apply_world_angular_correction(world, entity, delta)
                if norm3(delta) <= EPSILON:
                    update_rigid_body_member_local_orientation(world, entity)
