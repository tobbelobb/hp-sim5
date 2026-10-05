"""Moving cable attachments and hybrid endpoints in the Hangprinter step order."""
from dataclasses import dataclass
import math

import numpy as np
from cable_joints.util import effective_cw, is_hybrid, is_rolling

from .cable_frames import (
    DEFAULT_PLANE_NORMAL,
    arc_frame_for_endpoint, attachment_relative_orientation,
    delta_angle_for_entity, ensure_hybrid_knot_angle_for_endpoint,
    get_plane_normal, hybrid_knot_angle_for_path, orientation_angle_for_entity,
    orientation_frame_for_endpoint,
)
from .cable_joints_components import CableJointComponent, CableLinkComponent, CablePathComponent
from .cable_layering import (
    KNOT_SPAN, hybrid_angle_correction, hybrid_stored_delta, stored_to_radius_and_theta,
    stored_to_theta_signed, theta_to_stored_length,
)
from .ecs import HybridKnotAngleComponent, OrientationComponent, RadiusComponent, layering_enabled
from .geometry3 import (
    signed_arc_length_on_wheel, tangent_from_point_to_sphere,
    tangent_from_sphere_to_point, tangent_from_sphere_to_sphere,
)
from .quaternion import Quaternion
from .rigid_bodies import (
    get_entity_world_orientation, get_entity_world_position,
    update_rigid_body_member_local_orientation,
)
from .spools import normalize_angle
from .vector3 import cross, length as norm3, normalize

EPSILON = 1e-9
MIN_JOINT_REST_LENGTH = 1e-6


def feature_flag(world, key, fallback=True):
    value = world.get_resource(key)
    return value if isinstance(value, bool) else fallback


def effective_rolling_radius(world, path, index, base_radius):
    if base_radius is None or not math.isfinite(base_radius) or not layering_enabled(world) or path.cable_half_width <= EPSILON:
        return base_radius, 0.
    radius = base_radius + path.cable_half_width
    if index not in (0, len(path.link_types) - 1) or not is_hybrid(path.link_types[index]) or path.stored[index] <= EPSILON:
        return radius, 0.
    state = stored_to_radius_and_theta(path.stored[index], base_radius, path.cable_half_width, base_radius * KNOT_SPAN)
    return max(radius, state.radius), state.theta


def _rotate(vector, axis, angle):
    # Match Vector3's Rodrigues rotation, including its zero-axis convention.
    axis = normalize(axis)
    cosine, sine = math.cos(angle), math.sin(angle)
    return vector * cosine + cross(axis, vector) * sine + axis * np.dot(vector, axis) * (1 - cosine)


@dataclass
class EndpointFrame:
    entity: int
    counterpart: int
    index: int
    position: np.ndarray | None
    previous_position: np.ndarray | None
    quaternion: Quaternion | None
    previous_quaternion: Quaternion | None
    frame_quaternion: Quaternion | None
    previous_frame_quaternion: Quaternion | None
    normal: np.ndarray
    base_radius: float | None
    radius: float | None
    theta: float
    angle: float
    previous_angle: float
    delta: float
    cw: bool
    rolling: bool
    hybrid: bool
    previous_attachment: np.ndarray


def _endpoint_frame(world, path, index, entity, counterpart, attachment, *, geometry_only=False):
    link = world.get_component(entity, CableLinkComponent)
    position = get_entity_world_position(world, entity)
    rolling, hybrid = is_rolling(path.link_types[index]), is_hybrid(path.link_types[index])
    winding = rolling or hybrid
    quaternion = get_entity_world_orientation(world, entity) if winding else None
    previous_quaternion = link.prev_cable_attachment_time_orientation if link else None
    current_frame = previous_frame = None
    if winding:
        current_frame, previous_frame = orientation_frame_for_endpoint(
            world, entity, counterpart, quaternion, previous_quaternion, link)
    radius = world.get_component(entity, RadiusComponent)
    base_radius = radius.radius if radius else None
    effective_radius, theta = effective_rolling_radius(world, path, index, base_radius)
    # Angles only affect winding. Geometry-only callers need delta solely for
    # rotating hybrid attachments; fixed attachments and pinholes need neither.
    needs_delta = winding if not geometry_only else path.link_types[index] == 'hybrid-attachment'
    needs_angles = hybrid and not geometry_only
    return EndpointFrame(
        entity, counterpart, index, position, link.prev_cable_attachment_time_pos if link else None,
        quaternion, previous_quaternion, current_frame, previous_frame,
        get_plane_normal(world, entity) if winding else DEFAULT_PLANE_NORMAL,
        base_radius, effective_radius, theta,
        orientation_angle_for_entity(world, entity, quaternion, current_frame) if needs_angles else 0.,
        orientation_angle_for_entity(world, entity, previous_quaternion, previous_frame) if needs_angles else 0.,
        delta_angle_for_entity(world, entity, previous_quaternion, quaternion, previous_frame, current_frame) if needs_delta else 0.,
        effective_cw(path, index, index == 0), rolling, hybrid, attachment.copy(),
    )


def _moving_hybrid_attachment(state):
    if state.position is None or state.previous_position is None:
        return state.position.copy() if state.position is not None else None
    rotated = _rotate(state.previous_attachment - state.previous_position, state.normal, state.delta)
    return state.position + rotated


def _rolling_side_attachment(state, counterpart, point_is_first):
    if state.position is None or counterpart is None or state.radius is None:
        return state.position.copy() if state.position is not None else None
    normal = normalize(state.normal)
    projected = counterpart - normal * np.dot(counterpart - state.position, normal) if np.dot(state.normal, state.normal) > EPSILON else counterpart
    tangent = tangent_from_sphere_to_point if point_is_first else tangent_from_point_to_sphere
    return tangent(projected, state.position, state.radius, state.normal, state.cw)['a_sphere']


def _calculate_attachments(path, first, second):
    a = first.position.copy() if first.position is not None else None
    b = second.position.copy() if second.position is not None else None
    rotated_a = path.link_types[first.index] == 'hybrid-attachment' and abs(first.delta) > EPSILON
    rotated_b = path.link_types[second.index] == 'hybrid-attachment' and abs(second.delta) > EPSILON
    parallel = (first.rolling and second.rolling
                and np.dot(first.normal, first.normal) > EPSILON and np.dot(second.normal, second.normal) > EPSILON
                and abs(abs(np.dot(normalize(first.normal), normalize(second.normal))) - 1) <= 1e-6)
    if first.rolling and second.rolling and parallel and not rotated_a and not rotated_b:
        if a is not None and b is not None and first.radius is not None and second.radius is not None:
            tangent = tangent_from_sphere_to_sphere(a, first.radius, first.cw, b, second.radius, second.cw, first.normal)
            a, b = tangent['a_sphere'], tangent['b_sphere']
    else:
        if (rotated_a or not first.rolling) and first.hybrid:
            a = _moving_hybrid_attachment(first)
        if (rotated_b or not second.rolling) and second.hybrid:
            b = _moving_hybrid_attachment(second)
        if first.rolling and not rotated_a:
            a = _rolling_side_attachment(first, b, True)
        if second.rolling and not rotated_b:
            b = _rolling_side_attachment(second, a, False)
    return a, b


def calculate_attachment_points(world, joint, path, index):
    first = _endpoint_frame(world, path, index, joint.entity_a, joint.entity_b, joint.attachment_point_a_world, geometry_only=True)
    second = _endpoint_frame(world, path, index + 1, joint.entity_b, joint.entity_a, joint.attachment_point_b_world, geometry_only=True)
    return _calculate_attachments(path, first, second)


def _stored_delta(world, state, attachment, half_width):
    if not state.rolling or state.radius is None or any(p is None for p in (state.position, state.previous_position, attachment)):
        return 0.
    if norm3(state.previous_attachment - state.previous_position) < 1e-4:
        return 0.
    previous, current, normal = arc_frame_for_endpoint(
        world, state.entity, state.counterpart, state.previous_attachment - state.previous_position,
        attachment - state.position, state.normal, state.previous_quaternion, state.quaternion,
        state.previous_frame_quaternion, state.frame_quaternion)
    stored = signed_arc_length_on_wheel(previous, current, np.zeros(3), state.radius, state.cw, normal)
    if state.hybrid:
        stored += hybrid_stored_delta(state.theta, state.delta, state.cw, state.base_radius, half_width, state.radius)
    else:
        stored += (1 if state.cw else -1) * state.delta * state.radius
    return stored


def _apply_clamp_shift(world, state, stored_shift, half_width):
    if not (state.hybrid or state.rolling) or state.quaternion is None:
        return
    direction = 1 if state.cw else -1
    if state.hybrid:
        angle = hybrid_angle_correction(state.theta, state.delta, state.cw, state.base_radius, half_width, stored_shift, state.radius)
    elif state.radius is not None and abs(state.radius) > EPSILON:
        angle = stored_shift / (direction * state.radius)
    else:
        return
    if not math.isfinite(angle) or abs(angle) <= EPSILON or np.dot(state.normal, state.normal) <= EPSILON:
        return
    delta = Quaternion().set_from_axis_angle(state.normal, angle)
    state.quaternion.premultiply(delta).normalize()
    component = world.get_component(state.entity, OrientationComponent)
    if component is not None:
        component.quaternion.premultiply(delta).normalize()
        update_rigid_body_member_local_orientation(world, state.entity)


def _project_knot_phase(world, path, path_id, joint, state, attachment, half_width, first):
    if not state.hybrid or state.base_radius is None or not math.isfinite(state.base_radius) or state.position is None or attachment is None:
        return
    knot = hybrid_knot_angle_for_path(world.get_component(state.entity, HybridKnotAngleComponent), path_id)
    if knot is None:
        return
    ramp = max(0., state.base_radius) * KNOT_SPAN
    theta = abs(stored_to_theta_signed(path.stored[state.index], state.base_radius, half_width, ramp))
    if not math.isfinite(theta):
        theta = effective_rolling_radius(world, path, state.index, state.base_radius)[1]
    relative = attachment_relative_orientation(world, state.entity, state.position, attachment, state.quaternion, state.angle)
    if relative is None:
        return
    direction = (-1 if state.cw else 1) if first else (1 if state.cw else -1)
    target = normalize_angle(relative - knot)
    reference = direction * theta
    while target - reference > math.pi:
        target -= 2 * math.pi
    while target - reference < -math.pi:
        target += 2 * math.pi
    stored_target = theta_to_stored_length(direction * target, state.base_radius, half_width, ramp)
    shift = stored_target - path.stored[state.index]
    if math.isfinite(stored_target) and abs(shift) > 1e-3:
        path.stored[state.index] = stored_target
        joint.rest_length -= shift


def update_attachment_points(world):
    for path_id in world.query([CablePathComponent]):
        path = world.get_component(path_id, CablePathComponent)
        half_width = path.cable_half_width if layering_enabled(world) else 0.
        for index, joint_id in enumerate(path.joint_entities):
            joint = world.get_component(joint_id, CableJointComponent)
            first = _endpoint_frame(world, path, index, joint.entity_a, joint.entity_b, joint.attachment_point_a_world)
            second = _endpoint_frame(world, path, index + 1, joint.entity_b, joint.entity_a, joint.attachment_point_b_world)
            a, b = _calculate_attachments(path, first, second)
            for state, endpoint in ((first, index == 0), (second, index == len(path.joint_entities) - 1)):
                if state.hybrid and endpoint:
                    ensure_hybrid_knot_angle_for_endpoint(
                        world, path, state.index, path_id, attachment_point=state.previous_attachment,
                        center=state.previous_position, quaternion=state.previous_quaternion,
                        orientation_angle=state.previous_angle, create_if_missing=False)
            s_a, s_b = _stored_delta(world, first, a, half_width), _stored_delta(world, second, b, half_width)
            if feature_flag(world, 'layeringClampJointRestLength') and math.isfinite(joint.rest_length):
                unclamped = joint.rest_length - s_a + s_b
                if unclamped < MIN_JOINT_REST_LENGTH:
                    lift = MIN_JOINT_REST_LENGTH - unclamped
                    decrease_a, decrease_b = max(0., s_a), max(0., -s_b)
                    total = decrease_a + decrease_b
                    shift_a = lift * decrease_a / total if total > EPSILON else 0.
                    shift_b = lift - shift_a
                    s_a -= shift_a
                    s_b += shift_b
                    # JS corrects B before A; shared members and later joints can observe it.
                    _apply_clamp_shift(world, second, shift_b, half_width)
                    if total > EPSILON:
                        _apply_clamp_shift(world, first, -shift_a, half_width)
            path.stored[index] += s_a
            joint.rest_length -= s_a
            path.stored[index + 1] -= s_b
            joint.rest_length += s_b
            if index == 0:
                _project_knot_phase(world, path, path_id, joint, first, a, half_width, True)
            if index == len(path.joint_entities) - 1:
                _project_knot_phase(world, path, path_id, joint, second, b, half_width, False)
            if a is not None:
                joint.attachment_point_a_world[:] = a
            if b is not None:
                joint.attachment_point_b_world[:] = b


def update_hybrid_link_states(world):
    for path_id in world.query([CablePathComponent]):
        path = world.get_component(path_id, CablePathComponent)
        if not path.joint_entities:
            continue
        threshold = max(1e-6, .25 * path.cable_half_width) if path.cable_half_width > EPSILON else 1e-6
        for index in (0, len(path.link_types) - 1):
            joint = world.get_component(path.joint_entities[0 if index == 0 else -1], CableJointComponent)
            entity = joint.entity_a if index == 0 else joint.entity_b
            point = joint.attachment_point_a_world if index == 0 else joint.attachment_point_b_world
            neighbor = joint.attachment_point_b_world if index == 0 else joint.attachment_point_a_world
            center = get_entity_world_position(world, entity)
            radius_component = world.get_component(entity, RadiusComponent)
            radius = effective_rolling_radius(world, path, index, radius_component.radius if radius_component else None)[0]
            normal = get_plane_normal(world, entity)
            if path.link_types[index] == 'hybrid' and path.stored[index] < -threshold:
                old_stored = path.stored[index]
                path.link_types[index] = 'hybrid-attachment'
                # Recompute after changing link type, as the reference does.
                radius = effective_rolling_radius(world, path, index, radius_component.radius if radius_component else None)[0]
                joint.rest_length += old_stored
                path.stored[index] = 0.
                if center is not None and radius is not None and math.isfinite(radius) and radius > EPSILON:
                    angle = -old_stored / radius * (1 if path.cw[index] else -1)
                    point[:] = center + _rotate(point - center, normal, angle)
            elif path.link_types[index] == 'hybrid-attachment':
                if center is None or radius is None or not math.isfinite(radius) or radius <= EPSILON:
                    continue
                if norm3(point - neighbor) <= max(1e-6, 2 * path.cable_half_width + 1e-6):
                    continue
                cw_point = tangent_from_sphere_to_point(neighbor, center, radius, normal, True)['a_sphere']
                ccw_point = tangent_from_sphere_to_point(neighbor, center, radius, normal, False)['a_sphere']
                arc_cw = signed_arc_length_on_wheel(point, cw_point, center, radius, True, normal)
                arc_ccw = signed_arc_length_on_wheel(point, ccw_point, center, radius, False, normal)
                distance_cw, distance_ccw = np.dot(point - cw_point, point - cw_point), np.dot(point - ccw_point, point - ccw_point)
                if arc_ccw > 0 and distance_ccw < distance_cw:
                    cw, tangent, stored = True, ccw_point, arc_ccw
                elif arc_cw > 0 and distance_cw < distance_ccw:
                    cw, tangent, stored = False, cw_point, arc_cw
                else:
                    continue
                if stored <= threshold:
                    continue
                path.link_types[index], path.cw[index] = 'hybrid', cw
                joint.rest_length -= stored - path.stored[index]
                path.stored[index] = stored
                point[:] = tangent
                ensure_hybrid_knot_angle_for_endpoint(world, path, index, path_id, attachment_point=point)


class CableAttachmentUpdateSystem:
    def __init__(self, merge_and_split_feature=True):
        self.merge_and_split_feature = merge_and_split_feature

    def update(self, world, dt):
        step = world.get_resource('cableHybridTransitionStep')
        step = math.floor(step) if isinstance(step, (int, float)) and not isinstance(step, bool) and math.isfinite(step) else 0
        world.set_resource('cableHybridTransitionStep', step + 1)
        debug_points = world.get_resource('debugRenderPoints')
        if debug_points is not None:
            debug_points.clear()
        if feature_flag(world, 'layeringAttachmentUpdatePoints'):
            update_attachment_points(world)
        if feature_flag(world, 'layeringMergeJoints', self.merge_and_split_feature):
            from .cable_topology import merge_joints
            merge_joints(world)
        if feature_flag(world, 'layeringSplitJoints', self.merge_and_split_feature):
            from .cable_topology import split_joints
            split_joints(world)
        if feature_flag(world, 'layeringHybridLinkStates'):
            update_hybrid_link_states(world)
