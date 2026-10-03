"""Cable wrap planes and authored hybrid-knot frames."""
import math

import numpy as np

from .cable_joints_components import CableJointComponent, CableLinkComponent
from .cable_layering import stored_to_theta_signed
from .ecs import HybridKnotAngleComponent, RadiusComponent, RigidBodyMemberComponent, layering_enabled
from .geometry3 import build_plane_basis
from .quaternion import Quaternion
from .rigid_bodies import get_entity_world_orientation, get_entity_world_position
from .spools import normalize_angle

EPSILON = 1e-9
KNOT_SPAN = math.pi / 30
DEFAULT_PLANE_NORMAL = np.array([0., 0., 1.])


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


def ensure_hybrid_knot_angle_for_endpoint(world, path, index, path_id=None):
    if not path.joint_entities or index not in (0, len(path.link_types) - 1) or path.link_types[index] != 'hybrid':
        return
    first = index == 0
    joint = world.get_component(path.joint_entities[0 if first else -1], CableJointComponent)
    if joint is None:
        return
    entity = joint.entity_a if first else joint.entity_b
    point = joint.attachment_point_a_world if first else joint.attachment_point_b_world
    knot = world.get_component(entity, HybridKnotAngleComponent)
    key = '__default__' if path_id is None else str(path_id)
    if knot is not None:
        known = knot.angle if path_id is None else knot.path_angles.get(key)
        if known is not None and np.isfinite(known):
            return
        default = knot.path_angles.get('__default__')
        others = [v for k, v in knot.path_angles.items() if k != '__default__' and np.isfinite(v)]
        if path_id is not None and default is not None and np.isfinite(default) and not others:
            knot.path_angles[key] = knot.angle = default
            return
    center = get_entity_world_position(world, entity)
    if center is None:
        return
    radius_component = world.get_component(entity, RadiusComponent)
    radius = radius_component.radius if radius_component is not None else 0.
    half_width = path.cable_half_width if layering_enabled(world) else 0.
    theta = abs(stored_to_theta_signed(path.stored[index], radius, half_width, max(0., radius) * KNOT_SPAN))
    effective_cw = not path.cw[index] if first else path.cw[index]
    sign = (-1 if effective_cw else 1) if first else (1 if effective_cw else -1)
    orientation = get_entity_world_orientation(world, entity)
    member = world.get_component(entity, RigidBodyMemberComponent)
    local_orientation = member.local_orientation if member is not None else orientation
    angle = orientation_angle_for_entity(world, entity, orientation, local_orientation)
    relative = attachment_relative_orientation(world, entity, center, point, orientation, angle)
    if relative is None or not np.isfinite(relative):
        return
    knot_angle = normalize_angle(relative - sign * theta)
    if knot is None:
        knot = HybridKnotAngleComponent(knot_angle)
        world.add_component(entity, knot)
    knot.path_angles[key] = knot.angle = knot_angle
