"""The Hangprinter rotor's specialized one-axis member model, without UI."""
from dataclasses import dataclass
import math

import numpy as np

from .ecs import OrientationComponent, RigidBodyMemberComponent
from .quaternion import Quaternion

EPSILON = 1e-12


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def normalize_spool_axis_local(axis=None):
    axis = np.asarray(axis, dtype=float).copy() if axis is not None else np.array([0., 0., 1.])
    norm_squared = np.dot(axis, axis)
    return axis / np.sqrt(norm_squared) if norm_squared > EPSILON else np.array([0., 0., 1.])


def _quaternion_or_identity(value):
    return value.copy().normalize() if value is not None and np.isfinite(value.as_xyzw()).all() else Quaternion()


class SpoolTagComponent:
    pass


class SpoolStateComponent:
    def __init__(self, axis=None, axis_local=None, reference_orientation=None):
        self.axis = axis
        self.axis_local = normalize_spool_axis_local(axis_local)
        self.reference_orientation = _quaternion_or_identity(reference_orientation)


@dataclass
class SpoolOrientation:
    axis_local: np.ndarray
    swing: Quaternion
    twist: Quaternion
    angle: float
    reference_orientation: Quaternion


def decompose_spool_orientation(state, orientation):
    axis = normalize_spool_axis_local(state.axis_local)
    reference = _quaternion_or_identity(state.reference_orientation)
    relative = reference.copy().conjugate().normalize().multiply(_quaternion_or_identity(orientation)).normalize()
    projection = axis * np.dot(relative.as_xyzw()[:3], axis)
    twist = Quaternion(*projection, relative.w)
    twist = twist.normalize() if np.dot(twist.as_xyzw(), twist.as_xyzw()) > EPSILON else Quaternion()
    swing = relative.copy().multiply(twist.copy().conjugate().normalize()).normalize()
    angle = normalize_angle(2 * math.atan2(np.dot(twist.as_xyzw()[:3], axis), twist.w))
    return SpoolOrientation(axis, swing, twist, angle, reference)


def get_spool_rotation_angle(state, orientation):
    return decompose_spool_orientation(state, orientation).angle


def get_spool_world_axis(state, orientation=None):
    axis = normalize_spool_axis_local(state.axis_local)
    orientation = orientation if orientation is not None else state.reference_orientation
    if orientation is not None:
        world_axis = orientation.transform_vector(axis)
        norm_squared = np.dot(world_axis, world_axis)
        if norm_squared > EPSILON:
            return world_axis / np.sqrt(norm_squared)
    return axis


def compose_spool_orientation(state, swing, angle):
    relative = _quaternion_or_identity(swing).multiply(
        Quaternion().set_from_axis_angle(normalize_spool_axis_local(state.axis_local), angle)
    ).normalize()
    return _quaternion_or_identity(state.reference_orientation).multiply(relative).normalize()


def constrain_spool_orientation(state, orientation):
    # Keep twist and discard swing, exactly as the specialized JS model does.
    return compose_spool_orientation(state, None, get_spool_rotation_angle(state, orientation))


def rotate_spool_reference_orientation(state, delta):
    state.reference_orientation.premultiply(delta).normalize()


def constrain_spool_angular_velocity(state, orientation, velocity):
    axis = get_spool_world_axis(state, orientation)
    return axis * np.dot(velocity, axis)


@dataclass
class MemberSpoolFrame:
    member: RigidBodyMemberComponent
    body_orientation: Quaternion
    world_orientation: Quaternion
    local_spool_state: SpoolStateComponent


def get_rigid_body_member_spool_frame(world, entity, state):
    member = world.get_component(entity, RigidBodyMemberComponent)
    if member is None:
        return None
    body = world.get_component(member.body_entity, OrientationComponent)
    if body is None:
        return None
    local_reference = body.quaternion.copy().conjugate().normalize().multiply(state.reference_orientation).normalize()
    return MemberSpoolFrame(
        member, body.quaternion, body.quaternion.copy().multiply(member.local_orientation).normalize(),
        SpoolStateComponent(axis_local=state.axis_local, reference_orientation=local_reference),
    )
