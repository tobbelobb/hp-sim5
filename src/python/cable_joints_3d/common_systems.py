"""Integration systems for the Python 3D engine."""
import numpy as np
from cable_joints.ecs import GravityAffectedComponent, MassComponent
from .ecs import (AngularVelocityComponent, OrientationComponent, PositionComponent,
    PrevFinalOrientationComponent, PrevFinalPosComponent, VelocityComponent)
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
            mass = world.get_component(entity, MassComponent)
            if entity != world.get_resource("grabbedBall") and (mass is None or mass.mass > 0):
                world.get_component(entity, PositionComponent).pos += world.get_component(entity, VelocityComponent).vel * dt
class AngularMovementSystem:
    def update(self, world, dt):
        for entity in world.query([OrientationComponent, AngularVelocityComponent]):
            omega = world.get_component(entity, AngularVelocityComponent).omega; speed = np.linalg.norm(omega)
            if speed > 1e-12:
                delta = Quaternion().set_from_axis_angle(omega / speed, speed * dt)
                world.get_component(entity, OrientationComponent).quaternion.multiply(delta).normalize()
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
