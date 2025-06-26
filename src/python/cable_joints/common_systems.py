import math
import numpy as np
from . import vector2 as v2_helpers
from .ecs import (
    PositionComponent,
    VelocityComponent,
    GravityAffectedComponent,
    MassComponent,
    PrevFinalPosComponent,
    OrientationComponent,
    MomentOfInertiaComponent,
    AngularVelocityComponent,
    DistanceConstraintComponent,
    PrevFinalOrientationComponent
)


class PrevFinalOrientationSystem:
    run_in_pause = False
    def update(self, world, dt):
        entities = world.query([OrientationComponent, PrevFinalOrientationComponent])
        for entity_id in entities:
            orientation_comp = world.get_component(entity_id, OrientationComponent)
            prev_final_orientation_comp = world.get_component(entity_id, PrevFinalOrientationComponent)
            prev_final_orientation_comp.angle = orientation_comp.angle

class PBDAngularVelocityUpdateSystem:
    run_in_pause = False

    def _normalize_angle(self, angle): # Helper to find the shortest angle difference
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def update(self, world, dt):
        grabbed = world.get_resource('grabbedBall')
        entities = world.query([
            OrientationComponent,
            AngularVelocityComponent,
            PrevFinalOrientationComponent,
            MomentOfInertiaComponent
        ])
        epsilon = 1e-9

        if dt <= epsilon:
            return

        for entity_id in entities:
            if entity_id == grabbed:
                continue

            moi_comp = world.get_component(entity_id, MomentOfInertiaComponent)
            if moi_comp and moi_comp.inv_inertia <= 0: # Skip static or non-rotational objects
                continue

            orientation_comp = world.get_component(entity_id, OrientationComponent)
            angular_vel_comp = world.get_component(entity_id, AngularVelocityComponent)
            prev_final_orientation_comp = world.get_component(entity_id, PrevFinalOrientationComponent)

            current_angle = orientation_comp.angle
            prev_angle = prev_final_orientation_comp.angle

            delta_angle = self._normalize_angle(current_angle - prev_angle)

            angular_vel_comp.angular_velocity = delta_angle / dt

class PBDVelocityUpdateSystem:
    run_in_pause = False
    def update(self, world, dt):
        grabbed = world.get_resource('grabbedBall')
        entities = world.query([PositionComponent, VelocityComponent, PrevFinalPosComponent, MassComponent])
        epsilon = 1e-9

        if dt <= epsilon: # Avoid division by zero or very small dt
            return

        for entity_id in entities:
            if entity_id == grabbed: # Don't update velocity of grabbed ball this way
                continue

            mass_comp = world.get_component(entity_id, MassComponent)
            # Only update velocities for dynamic objects (mass > 0)
            if mass_comp and mass_comp.mass <= 0:
                continue

            pos_comp = world.get_component(entity_id, PositionComponent)
            vel_comp = world.get_component(entity_id, VelocityComponent)
            prev_final_pos_comp = world.get_component(entity_id, PrevFinalPosComponent)

            # v = (x_new - x_old) / dt
            np.subtract(pos_comp.pos, prev_final_pos_comp.pos, out=vel_comp.vel)
            np.divide(vel_comp.vel, dt, out=vel_comp.vel)


class GravitySystem:
    run_in_pause = False
    def update(self, world, dt):
        grabbed = world.get_resource('grabbedBall')
        gravity = world.get_resource('gravity')
        if gravity is None:
            return

        entities = world.query([VelocityComponent, GravityAffectedComponent])
        for entity_id in entities:
            if entity_id == grabbed:
                continue
            vel_comp = world.get_component(entity_id, VelocityComponent)
            vel_comp.vel += gravity * dt

class XPBDDistanceConstraintSystem:
    run_in_pause = False
    def update(self, world, dt):
        constraint_entities = world.query([DistanceConstraintComponent])
        epsilon = 1e-9 # Small value to avoid division by zero

        for entity_id in constraint_entities:
            constraint = world.get_component(entity_id, DistanceConstraintComponent)

            entity_a = constraint.entityA
            entity_b = constraint.entityB

            pos_a_comp = world.get_component(entity_a, PositionComponent)
            pos_b_comp = world.get_component(entity_b, PositionComponent)

            if not pos_a_comp or not pos_b_comp:
                continue

            p_a = pos_a_comp.pos
            p_b = pos_b_comp.pos

            mass_a_comp = world.get_component(entity_a, MassComponent)
            inv_mass_a = 1.0 / mass_a_comp.mass if mass_a_comp and mass_a_comp.mass > 0 else 0.0
            moi_a_comp = world.get_component(entity_a, MomentOfInertiaComponent)
            inv_inertia_a = moi_a_comp.inv_inertia if moi_a_comp else 0.0

            mass_b_comp = world.get_component(entity_b, MassComponent)
            inv_mass_b = 1.0 / mass_b_comp.mass if mass_b_comp and mass_b_comp.mass > 0 else 0.0
            moi_b_comp = world.get_component(entity_b, MomentOfInertiaComponent)
            inv_inertia_b = moi_b_comp.inv_inertia if moi_b_comp else 0.0

            if inv_mass_a + inv_mass_b + inv_inertia_a + inv_inertia_b <= epsilon:
                continue

            diff = p_b - p_a
            current_length = np.linalg.norm(diff)
            if current_length <= epsilon:
                continue

            direction = diff / current_length

            C = current_length - constraint.rest_length

            alpha_tilde = constraint.compliance / (dt * dt)

            w_sum = inv_mass_a + inv_mass_b

            denominator = w_sum + alpha_tilde
            if denominator <= epsilon:
                continue

            delta_lambda = (-C - alpha_tilde * constraint.lambda_val) / denominator
            constraint.lambda_val += delta_lambda

            correction = direction * delta_lambda

            if inv_mass_a > 0:
                dp_a = correction * -inv_mass_a
                p_a += dp_a
            if inv_mass_b > 0:
                dp_b = correction * inv_mass_b
                p_b += dp_b

class MovementSystem:
    run_in_pause = False
    def update(self, world, dt):
        grabbed = world.get_resource('grabbedBall')
        linear_entities = world.query([PositionComponent, VelocityComponent])
        for entity_id in linear_entities:
            if entity_id == grabbed:
                continue
            pos_comp = world.get_component(entity_id, PositionComponent)
            vel_comp = world.get_component(entity_id, VelocityComponent)
            pos_comp.pos += vel_comp.vel * dt

class PrevFinalPosSystem:
    run_in_pause = False
    def update(self, world, dt):
        entities = world.query([PositionComponent, PrevFinalPosComponent])
        for entity_id in entities:
            pos_component = world.get_component(entity_id, PositionComponent)
            prev_final_pos_component = world.get_component(entity_id, PrevFinalPosComponent)
            prev_final_pos_component.pos[:] = pos_component.pos

class AngularMovementSystem:
    run_in_pause = False
    def update(self, world, dt):
        entities = world.query([OrientationComponent, AngularVelocityComponent])
        for entity_id in entities:
            orientation = world.get_component(entity_id, OrientationComponent)
            angular_vel = world.get_component(entity_id, AngularVelocityComponent)
            orientation.angle += angular_vel.angular_velocity * dt

