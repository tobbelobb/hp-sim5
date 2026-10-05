"""Cable wrap planes and authored hybrid-knot frames."""
import math

import numpy as np

from .cable_joints_components import CableJointComponent, CableLinkComponent
from .cable_layering import KNOT_SPAN, stored_to_theta_signed
from .ecs import HybridKnotAngleComponent, RadiusComponent, RigidBodyMemberComponent, layering_enabled
from .geometry3 import build_plane_basis
from .quaternion import Quaternion
from .rigid_bodies import get_entity_world_orientation, get_entity_world_position
from .spools import normalize_angle

EPSILON = 1e-9
DEFAULT_PLANE_NORMAL = np.array([0., 0., 1.])


def same_rigid_body_member_pair(world, entity, counterpart):
    member = world.get_component(entity, RigidBodyMemberComponent)
    other = world.get_component(counterpart, RigidBodyMemberComponent)
    return member is not None and other is not None and member.body_entity == other.body_entity


def orientation_frame_for_endpoint(world, entity, counterpart, current_world, previous_world, link):
    if same_rigid_body_member_pair(world, entity, counterpart):
        member = world.get_component(entity, RigidBodyMemberComponent)
        return member.local_orientation, link.prev_cable_attachment_time_local_orientation if link else previous_world
    return current_world, previous_world


def arc_frame_for_endpoint(world, entity, counterpart, previous, current, normal,
                           previous_world, current_world, previous_frame, current_frame):
    if not same_rigid_body_member_pair(world, entity, counterpart) or any(
            q is None for q in (previous_world, current_world, previous_frame, current_frame)):
        return previous, current, normal
    previous_body_inverse = previous_world.copy().multiply(previous_frame.copy().conjugate().normalize()).normalize().conjugate().normalize()
    current_body_inverse = current_world.copy().multiply(current_frame.copy().conjugate().normalize()).normalize().conjugate().normalize()
    frame_normal = current_body_inverse.transform_vector(normal)
    if np.dot(frame_normal, frame_normal) > EPSILON:
        frame_normal /= np.linalg.norm(frame_normal)
    return previous_body_inverse.transform_vector(previous), current_body_inverse.transform_vector(current), frame_normal


def delta_angle_for_entity(world, entity, previous, current, previous_local=None, current_local=None):
    link = world.get_component(entity, CableLinkComponent)
    local_axis = link.cable_plane_normal_local if link is not None else None
    if local_axis is not None:
        previous = previous_local if previous_local is not None else previous
        current = current_local if current_local is not None else current
    if previous is None or current is None:
        return 0.
    inverse_previous = previous.copy().conjugate().normalize()
    if local_axis is not None:
        relative = inverse_previous.multiply(current).normalize()
        return orientation_angle_for_entity(world, entity, relative, relative)
    relative = current.copy().multiply(inverse_previous).normalize()
    w = float(np.clip(relative.w, -1., 1.))
    angle = 2 * math.acos(w)
    if angle < EPSILON:
        return 0.
    sin_half = math.sqrt(max(0., 1 - w * w))
    axis = get_plane_normal(world, entity)
    axis_relative = axis / np.linalg.norm(axis) if sin_half < EPSILON and np.linalg.norm(axis) > 0 else (
        axis.copy() if sin_half < EPSILON else relative.as_xyzw()[:3] / sin_half)
    sign = np.sign(np.dot(axis_relative, axis))
    if sign == 0:
        return angle
    return (angle - 2 * math.pi if angle > math.pi else angle) * sign


def get_plane_normal(world, entity):
    link = world.get_component(entity, CableLinkComponent)
    if link is not None and link.cable_plane_normal_local is not None:
        local_axis = link.cable_plane_normal_local.copy()
        orientation = get_entity_world_orientation(world, entity)
        if orientation is not None:
            axis = orientation.transform_vector(local_axis)
            if np.dot(axis, axis) > EPSILON:
                return axis / np.linalg.norm(axis)
        if np.dot(local_axis, local_axis) > EPSILON:
            return local_axis / np.linalg.norm(local_axis)
    return link.cable_plane_normal if link is not None and link.cable_plane_normal is not None else DEFAULT_PLANE_NORMAL


def orientation_angle_for_entity(world, entity, quaternion, local_quaternion=None):
    link = world.get_component(entity, CableLinkComponent)
    if link is not None and link.cable_plane_normal_local is not None:
        q = local_quaternion if local_quaternion is not None else quaternion
        if q is None:
            return 0.
        axis = link.cable_plane_normal_local.copy()
        norm = np.linalg.norm(axis)
        if norm > 0:
            axis /= norm
        projection = np.dot(q.as_xyzw()[:3], axis)
        twist = Quaternion(*(axis * projection), q.w)
        if np.dot(twist.as_xyzw(), twist.as_xyzw()) <= EPSILON:
            return 0.
        twist.normalize()
        return normalize_angle(2 * math.atan2(np.dot(twist.as_xyzw()[:3], axis), twist.w))
    if quaternion is None:
        return 0.
    normal, u, v = build_plane_basis(get_plane_normal(world, entity))
    rotated = quaternion.transform_vector(u)
    projected = rotated - normal * np.dot(rotated, normal)
    if np.dot(projected, projected) <= EPSILON:
        return 0.
    projected /= np.linalg.norm(projected)
    return math.atan2(np.dot(projected, v), np.dot(projected, u))


def attachment_relative_orientation(world, entity, center, point, quaternion, orientation_angle):
    link = world.get_component(entity, CableLinkComponent)
    if link is not None and link.cable_plane_normal_local is not None:
        if quaternion is None:
            return None
        relative = quaternion.copy().conjugate().normalize().transform_vector(point - center)
        _, u, v = build_plane_basis(link.cable_plane_normal_local)
        return math.atan2(np.dot(relative, v), np.dot(relative, u))
    _, u, v = build_plane_basis(get_plane_normal(world, entity))
    relative = point - center
    return normalize_angle(math.atan2(np.dot(relative, v), np.dot(relative, u)) - orientation_angle)


def hybrid_knot_angle_for_path(knot, path_id):
    if knot is None:
        return None
    angle = knot.angle if path_id is None else knot.path_angles.get(str(path_id))
    return angle if angle is not None and np.isfinite(angle) else None


def ensure_hybrid_knot_angle_for_endpoint(world, path, index, path_id=None, *,
                                         attachment_point=None, center=None, quaternion=None,
                                         local_quaternion=None, orientation_angle=None, create_if_missing=True):
    if not path.joint_entities or index not in (0, len(path.link_types) - 1) or path.link_types[index] != 'hybrid':
        return
    first = index == 0
    joint = world.get_component(path.joint_entities[0 if first else -1], CableJointComponent)
    if joint is None:
        return
    entity = joint.entity_a if first else joint.entity_b
    point = attachment_point if attachment_point is not None else (joint.attachment_point_a_world if first else joint.attachment_point_b_world)
    knot = world.get_component(entity, HybridKnotAngleComponent)
    key = '__default__' if path_id is None else str(path_id)
    if knot is not None:
        if hybrid_knot_angle_for_path(knot, path_id) is not None:
            return
        default = knot.path_angles.get('__default__')
        others = [v for k, v in knot.path_angles.items() if k != '__default__' and np.isfinite(v)]
        if path_id is not None and default is not None and np.isfinite(default) and not others:
            knot.path_angles[key] = knot.angle = default
            return
    if knot is None and not create_if_missing:
        return
    center = center if center is not None else get_entity_world_position(world, entity)
    if center is None:
        return
    radius_component = world.get_component(entity, RadiusComponent)
    radius = radius_component.radius if radius_component is not None else 0.
    half_width = path.cable_half_width if layering_enabled(world) else 0.
    theta = abs(stored_to_theta_signed(path.stored[index], radius, half_width, max(0., radius) * KNOT_SPAN))
    effective_cw = not path.cw[index] if first else path.cw[index]
    sign = (-1 if effective_cw else 1) if first else (1 if effective_cw else -1)
    orientation = quaternion if quaternion is not None else get_entity_world_orientation(world, entity)
    member = world.get_component(entity, RigidBodyMemberComponent)
    local_orientation = local_quaternion if local_quaternion is not None else (member.local_orientation if member is not None else orientation)
    angle = orientation_angle if orientation_angle is not None and np.isfinite(orientation_angle) else orientation_angle_for_entity(world, entity, orientation, local_orientation)
    relative = attachment_relative_orientation(world, entity, center, point, orientation, angle)
    if relative is None or not np.isfinite(relative):
        return
    knot_angle = normalize_angle(relative - sign * theta)
    if knot is None:
        knot = HybridKnotAngleComponent(knot_angle)
        world.add_component(entity, knot)
    knot.path_angles[key] = knot.angle = knot_angle
