"""Integration systems for the Python 3D engine."""
import math
import numpy as np
from cable_joints.ecs import GravityAffectedComponent, MassComponent
from .ecs import (AngularVelocityComponent, OrientationComponent, PositionComponent,
    PrevFinalOrientationComponent, PrevFinalPosComponent,
    RigidBodyComponent, RigidBodyMemberComponent, VelocityComponent,
    DistanceConstraintComponent, EncoderComponent)
from .inertia_tensor import (MomentOfInertiaComponent, has_any_inverse_inertia,
    apply_world_inverse_inertia, inverse_inertia_quadratic_form)
from .quaternion import Quaternion
from .rigid_bodies import (compute_world_attachment, get_entity_world_position,
    get_entity_world_orientation, initialize_rigid_body_sync_state,
    resolve_rigid_body_solver_endpoint, update_rigid_body_member_local_orientation)
from .spools import (SpoolStateComponent, constrain_spool_orientation,
    constrain_spool_angular_velocity, rotate_spool_reference_orientation,
    get_rigid_body_member_spool_frame, get_spool_rotation_angle, get_spool_world_axis)

class GravitySystem:
    def update(self, world, dt):
        gravity = world.get_resource("gravity")
        if gravity is None: return
        for entity in world.query([VelocityComponent, GravityAffectedComponent]):
            if (entity != world.get_resource("grabbedBall")
                    and world.get_component(entity, RigidBodyMemberComponent) is None):
                world.get_component(entity, VelocityComponent).vel += np.asarray(gravity) * dt
class MovementSystem:
    def update(self, world, dt):
        for entity in world.query([PositionComponent, VelocityComponent]):
            if entity == world.get_resource("grabbedBall"):
                continue
            if world.get_component(entity, RigidBodyMemberComponent) is not None:
                continue
            position = world.get_component(entity, PositionComponent)
            velocity = world.get_component(entity, VelocityComponent)
            position.pos += velocity.vel * dt
class AngularMovementSystem:
    def update(self, world, dt):
        for entity in world.query([OrientationComponent, AngularVelocityComponent]):
            if (world.get_component(entity, RigidBodyMemberComponent) is not None
                    and world.get_component(entity, SpoolStateComponent) is not None):
                continue
            omega = world.get_component(entity, AngularVelocityComponent).omega; speed = np.linalg.norm(omega)
            if speed > 1e-12:
                delta = Quaternion().set_from_axis_angle(omega / speed, speed * dt)
                orientation = world.get_component(entity, OrientationComponent)
                orientation.quaternion.premultiply(delta).normalize()
class PrevFinalPosSystem:
    def update(self, world, dt):
        for entity in world.query([PositionComponent, PrevFinalPosComponent]):
            world.get_component(entity, PrevFinalPosComponent).pos[:] = world.get_component(entity, PositionComponent).pos
class PrevFinalOrientationSystem:
    def update(self, world, dt):
        for entity in world.query([OrientationComponent, PrevFinalOrientationComponent]):
            world.get_component(entity, PrevFinalOrientationComponent).quaternion.set(world.get_component(entity, OrientationComponent).quaternion)
class PBDVelocityUpdateSystem:
    def update(self, world, dt):
        if dt <= 1e-9: return
        for entity in world.query([PositionComponent, VelocityComponent, PrevFinalPosComponent, MassComponent]):
            if world.get_component(entity, RigidBodyMemberComponent) is not None:
                continue
            if world.get_component(entity, MassComponent).mass > 0 and entity != world.get_resource("grabbedBall"):
                current = world.get_component(entity, PositionComponent).pos
                previous = world.get_component(entity, PrevFinalPosComponent).pos
                world.get_component(entity, VelocityComponent).vel[:] = (current - previous) / dt


class PBDAngularVelocityUpdateSystem:
    """Reconstruct world angular velocity after positional constraints run."""

    def update(self, world, dt):
        if dt <= 1e-9:
            return
        components = [OrientationComponent, AngularVelocityComponent,
                      PrevFinalOrientationComponent, MomentOfInertiaComponent]
        for entity in world.query(components):
            if entity == world.get_resource("grabbedBall"):
                continue
            if world.get_component(entity, RigidBodyMemberComponent) is not None:
                continue
            moment = world.get_component(entity, MomentOfInertiaComponent)
            if not has_any_inverse_inertia(moment):
                continue

            current = world.get_component(entity, OrientationComponent).quaternion
            previous = world.get_component(
                entity, PrevFinalOrientationComponent
            ).quaternion
            delta = current.copy().multiply(previous.copy().conjugate().normalize())
            delta.normalize()
            if delta.w < 0.:
                delta.x *= -1.; delta.y *= -1.; delta.z *= -1.; delta.w *= -1.

            w = float(np.clip(delta.w, -1., 1.))
            sin_half = math.hypot(delta.x, delta.y, delta.z)
            angle = 2. * math.atan2(sin_half, w)
            angular_velocity = world.get_component(
                entity, AngularVelocityComponent
            ).omega
            if sin_half <= 1e-12 or angle <= 1e-9:
                angular_velocity[:] = 0.
            else:
                scale = angle / (dt * sin_half)
                angular_velocity[:] = np.array(
                    [delta.x, delta.y, delta.z]
                ) * scale


class RigidBodySyncSystem:
    def update(self, world, dt):
        for entity in world.query([SpoolStateComponent, OrientationComponent]):
            if world.get_component(entity, RigidBodyMemberComponent) is not None:
                continue
            state = world.get_component(entity, SpoolStateComponent)
            orientation = world.get_component(entity, OrientationComponent).quaternion
            orientation.set(constrain_spool_orientation(state, orientation))
            velocity = world.get_component(entity, AngularVelocityComponent)
            if velocity is not None:
                velocity.omega[:] = constrain_spool_angular_velocity(state, orientation, velocity.omega)

        for entity in world.query([RigidBodyComponent, PositionComponent, OrientationComponent]):
            body = world.get_component(entity, RigidBodyComponent)
            if not body.members:
                continue
            position = world.get_component(entity, PositionComponent).pos
            orientation = world.get_component(entity, OrientationComponent).quaternion.copy().normalize()
            velocity = world.get_component(entity, VelocityComponent)
            angular_velocity = world.get_component(entity, AngularVelocityComponent)
            if body.synced_position is None or body.synced_orientation is None:
                initialize_rigid_body_sync_state(world, entity)
            delta = orientation.copy().multiply(body.synced_orientation.copy().normalize().conjugate().normalize()).normalize()

            for member_entity in body.members:
                member = world.get_component(member_entity, RigidBodyMemberComponent)
                if member is None:
                    continue
                member_orientation = world.get_component(member_entity, OrientationComponent)
                spool = world.get_component(member_entity, SpoolStateComponent)
                if member_orientation is not None:
                    member_orientation.quaternion.premultiply(delta).normalize()
                    if spool is not None:
                        rotate_spool_reference_orientation(spool, delta)
                    update_rigid_body_member_local_orientation(world, member_entity)
                    member_orientation.quaternion.set(orientation.copy().multiply(member.local_orientation).normalize())
                    if spool is not None:
                        member_orientation.quaternion.set(constrain_spool_orientation(spool, member_orientation.quaternion))
                        update_rigid_body_member_local_orientation(world, member_entity)
                member_position = world.get_component(member_entity, PositionComponent)
                offset = orientation.transform_vector(member.local_position)
                if member_position is not None:
                    member_position.pos[:] = position + offset
                member_velocity = world.get_component(member_entity, VelocityComponent)
                if member_velocity is not None:
                    member_velocity.vel[:] = velocity.vel if velocity is not None else 0.
                    if angular_velocity is not None:
                        member_velocity.vel += np.cross(angular_velocity.omega, offset)
                member_angular_velocity = world.get_component(member_entity, AngularVelocityComponent)
                if spool is not None and member_angular_velocity is not None and member_orientation is not None:
                    member_angular_velocity.omega[:] = constrain_spool_angular_velocity(
                        spool, member_orientation.quaternion, member_angular_velocity.omega
                    )
            body.synced_position[:] = position
            body.synced_orientation.set(orientation)


class XPBDDistanceConstraintSystem:
    """Distance constraints with rigid-member reactions and full tensor inertia."""
    def update(self, world, dt):
        for entity in world.query([DistanceConstraintComponent]):
            constraint = world.get_component(entity, DistanceConstraintComponent)
            point_a = get_entity_world_position(world, constraint.entity_a)
            point_b = get_entity_world_position(world, constraint.entity_b)
            if point_a is None or point_b is None:
                continue
            endpoint_a = resolve_rigid_body_solver_endpoint(world, constraint.entity_a, constraint.entity_b, point_a)
            endpoint_b = resolve_rigid_body_solver_endpoint(world, constraint.entity_b, constraint.entity_a, point_b)
            if endpoint_a.entity_id == endpoint_b.entity_id:
                continue
            point_a = compute_world_attachment(world, endpoint_a.entity_id, endpoint_a.local_point)
            point_b = compute_world_attachment(world, endpoint_b.entity_id, endpoint_b.local_point)
            if point_a is None or point_b is None:
                continue
            positions = [world.get_component(e.entity_id, PositionComponent) for e in (endpoint_a, endpoint_b)]
            if any(p is None for p in positions):
                continue
            difference = point_b - point_a
            length = np.linalg.norm(difference)
            if length <= 1e-9:
                continue
            direction = difference / length
            endpoints = []
            for endpoint, point, position, gradient in zip(
                    (endpoint_a, endpoint_b), (point_a, point_b), positions, (direction, -direction)):
                mass = world.get_component(endpoint.entity_id, MassComponent)
                inverse_mass = 1 / mass.mass if mass is not None and mass.mass > 0 else 0.
                moment = world.get_component(endpoint.entity_id, MomentOfInertiaComponent)
                orientation = world.get_component(endpoint.entity_id, OrientationComponent)
                angular_gradient = np.cross(point - position.pos, gradient)
                angular_denominator = inverse_inertia_quadratic_form(moment, orientation.quaternion, angular_gradient) if orientation else 0.
                endpoints.append((endpoint, position, gradient, inverse_mass, moment, orientation, angular_gradient, angular_denominator))
            alpha = constraint.compliance / (dt * dt)
            denominator = alpha + sum(e[3] * np.dot(e[2], e[2]) + e[7] for e in endpoints)
            if denominator <= 1e-9:
                continue
            delta_lambda = (-(length - constraint.rest_length) - alpha * constraint.lambda_val) / denominator
            constraint.lambda_val += delta_lambda
            for endpoint, position, gradient, inverse_mass, moment, orientation, angular_gradient, angular_denominator in endpoints:
                if inverse_mass > 0:
                    position.pos += gradient * (-inverse_mass * delta_lambda)
                if angular_denominator > 0 and orientation is not None:
                    correction = apply_world_inverse_inertia(moment, orientation.quaternion, angular_gradient) * -delta_lambda
                    angle = np.linalg.norm(correction)
                    if angle > 1e-9:
                        orientation.quaternion.premultiply(Quaternion().set_from_axis_angle(correction / angle, angle)).normalize()
                        update_rigid_body_member_local_orientation(world, endpoint.entity_id)


class EncoderUpdateSystem:
    def update(self, world, dt):
        for entity in world.query([OrientationComponent, EncoderComponent]):
            encoder = world.get_component(entity, EncoderComponent)
            orientation = get_entity_world_orientation(world, entity)
            spool = world.get_component(entity, SpoolStateComponent)
            if spool is not None:
                frame = get_rigid_body_member_spool_frame(world, entity, spool)
                if frame is not None:
                    orientation = frame.world_orientation
                axis = get_spool_world_axis(spool, orientation)
                angle = get_spool_rotation_angle(
                    frame.local_spool_state if frame else spool,
                    frame.member.local_orientation if frame else orientation,
                )
            else:
                axis = np.asarray(encoder.axis, dtype=float).copy()
                if np.dot(axis, axis) <= 1e-12:
                    default = world.get_resource('defaultPlaneNormal')
                    axis = np.asarray(default, dtype=float).copy() if default is not None else np.array([0., 0., 1.])
                    if np.dot(axis, axis) <= 1e-12:
                        axis = np.array([0., 0., 1.])
                axis /= np.linalg.norm(axis)
                reference = np.array([1., 0., 0.]) if abs(axis[0]) < .9 else np.array([0., 1., 0.])
                u = reference - axis * np.dot(axis, reference)
                if np.dot(u, u) <= 1e-12:
                    reference = np.array([0., 0., 1.])
                    u = reference - axis * np.dot(axis, reference)
                u /= np.linalg.norm(u)
                rotated = orientation.transform_vector(u) if orientation else u
                projected = rotated - axis * np.dot(rotated, axis)
                if np.dot(projected, projected) <= 1e-12:
                    angle = 0.
                else:
                    projected /= np.linalg.norm(projected)
                    angle = np.arctan2(np.dot(projected, np.cross(axis, u)), np.dot(projected, u))
            if not np.isfinite(angle):
                continue
            if np.isfinite(encoder.angle):
                while angle - encoder.angle > np.pi:
                    angle -= 2 * np.pi
                while angle - encoder.angle < -np.pi:
                    angle += 2 * np.pi
            encoder.angle = float(angle)
            encoder.axis = axis
