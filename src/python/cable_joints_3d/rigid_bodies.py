"""Rigid-member attachment frames and constraint reaction redirection."""
from dataclasses import dataclass

import numpy as np

from .ecs import PositionComponent, OrientationComponent, RigidBodyComponent, RigidBodyMemberComponent
from .quaternion import Quaternion


def _finite_quaternion(value):
    return value is not None and np.isfinite(value.as_xyzw()).all()


def get_rigid_body_entity_for_member(world, entity):
    member = world.get_component(entity, RigidBodyMemberComponent)
    return member.body_entity if member is not None else None


def get_entity_world_position(world, entity):
    member = world.get_component(entity, RigidBodyMemberComponent)
    if member is not None:
        body_position = world.get_component(member.body_entity, PositionComponent)
        body_orientation = world.get_component(member.body_entity, OrientationComponent)
        if body_position is not None and body_orientation is not None:
            return body_position.pos + body_orientation.quaternion.transform_vector(member.local_position)
    position = world.get_component(entity, PositionComponent)
    return position.pos.copy() if position is not None else None


def get_entity_world_orientation(world, entity):
    member = world.get_component(entity, RigidBodyMemberComponent)
    if member is not None:
        body_orientation = world.get_component(member.body_entity, OrientationComponent)
        if body_orientation is not None and _finite_quaternion(member.local_orientation):
            return body_orientation.quaternion.copy().multiply(member.local_orientation).normalize()
    orientation = world.get_component(entity, OrientationComponent)
    if orientation is not None and _finite_quaternion(orientation.quaternion):
        return orientation.quaternion.copy().normalize()
    return None


def compute_world_attachment(world, entity, local_point):
    if local_point is None:
        return None
    position = get_entity_world_position(world, entity)
    if position is None:
        return np.asarray(local_point, dtype=float).copy()
    orientation = get_entity_world_orientation(world, entity)
    return position + (orientation.transform_vector(local_point) if orientation else local_point)


def compute_local_attachment(world, entity, world_point):
    if world_point is None:
        return None
    position = get_entity_world_position(world, entity)
    if position is None:
        return np.asarray(world_point, dtype=float).copy()
    relative = world_point - position
    orientation = get_entity_world_orientation(world, entity)
    return orientation.conjugate().normalize().transform_vector(relative) if orientation else relative


@dataclass
class SolverEndpoint:
    entity_id: int
    local_point: np.ndarray | None
    world_point: np.ndarray | None
    internal_to_body: bool


def resolve_rigid_body_solver_endpoint(world, entity, counterpart, world_point):
    member = world.get_component(entity, RigidBodyMemberComponent)
    other = world.get_component(counterpart, RigidBodyMemberComponent)
    internal = member is not None and other is not None and member.body_entity == other.body_entity
    solver_entity = member.body_entity if member is not None and not internal else entity
    return SolverEndpoint(
        solver_entity, compute_local_attachment(world, solver_entity, world_point),
        np.asarray(world_point, dtype=float).copy() if world_point is not None else None, internal,
    )


def update_rigid_body_member_local_orientation(world, entity):
    member = world.get_component(entity, RigidBodyMemberComponent)
    if member is None:
        return
    body = world.get_component(member.body_entity, OrientationComponent)
    orientation = world.get_component(entity, OrientationComponent)
    if body is None or orientation is None:
        return
    if _finite_quaternion(body.quaternion) and _finite_quaternion(orientation.quaternion):
        member.local_orientation.set(
            body.quaternion.copy().conjugate().normalize().multiply(orientation.quaternion).normalize()
        )


def initialize_rigid_body_sync_state(world, entity):
    body = world.get_component(entity, RigidBodyComponent)
    if body is None:
        return
    position = world.get_component(entity, PositionComponent)
    orientation = world.get_component(entity, OrientationComponent)
    body.synced_position = position.pos.copy() if position is not None else np.zeros(3)
    body.synced_orientation = (
        orientation.quaternion.copy().normalize()
        if orientation is not None and _finite_quaternion(orientation.quaternion) else Quaternion()
    )
