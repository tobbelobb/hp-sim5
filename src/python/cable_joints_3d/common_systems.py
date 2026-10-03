"""Integration systems for the Python 3D engine."""
import numpy as np
from cable_joints.ecs import GravityAffectedComponent, MassComponent
from .ecs import (AngularVelocityComponent, OrientationComponent, PositionComponent,
    PrevFinalOrientationComponent, PrevFinalPosComponent,
    RigidBodyMemberComponent, VelocityComponent)
from .inertia_tensor import MomentOfInertiaComponent, has_any_inverse_inertia
from .quaternion import Quaternion

class GravitySystem:
    def update(self, world, dt):
        gravity = world.get_resource("gravity")
        if gravity is None: return
        for entity in world.query([VelocityComponent, GravityAffectedComponent]):
            if entity != world.get_resource("grabbedBall"):
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
        for entity in world.query([PositionComponent, PrevFinalPosComponent, VelocityComponent, MassComponent]):
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
            angle = 2. * np.arccos(w)
            sin_half = np.sqrt(max(0., 1. - w*w))
            angular_velocity = world.get_component(
                entity, AngularVelocityComponent
            ).omega
            if sin_half <= 1e-12 or angle <= 1e-9:
                angular_velocity[:] = 0.
            else:
                angular_velocity[:] = np.array(
                    [delta.x, delta.y, delta.z]
                ) * angle / (dt * sin_half)
