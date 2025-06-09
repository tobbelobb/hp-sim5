import math
import numpy as np
from . import vector2 as v2_helpers
from .ecs import (
    PositionComponent,
    VelocityComponent,
    GravityAffectedComponent,
    BallTagComponent,
    RadiusComponent,
    MassComponent,
    RestitutionComponent,
    PrevFinalPosComponent,
    OrientationComponent,
    MomentOfInertiaComponent,
    AngularVelocityComponent,
    CoefficientOfFrictionComponent,
    ObstacleTagComponent,
    ObstaclePushComponent,
    ScoredTagComponent,
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
            if moi_comp and moi_comp.invInertia <= 0: # Skip static or non-rotational objects
                continue

            orientation_comp = world.get_component(entity_id, OrientationComponent)
            angular_vel_comp = world.get_component(entity_id, AngularVelocityComponent)
            prev_final_orientation_comp = world.get_component(entity_id, PrevFinalOrientationComponent)

            current_angle = orientation_comp.angle
            prev_angle = prev_final_orientation_comp.angle

            delta_angle = self._normalize_angle(current_angle - prev_angle)

            angular_vel_comp.angularVelocity = delta_angle / dt

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
            vel_comp.vel = (pos_comp.pos - prev_final_pos_comp.pos) / dt

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
            inv_inertia_a = moi_a_comp.invInertia if moi_a_comp else 0.0

            mass_b_comp = world.get_component(entity_b, MassComponent)
            inv_mass_b = 1.0 / mass_b_comp.mass if mass_b_comp and mass_b_comp.mass > 0 else 0.0
            moi_b_comp = world.get_component(entity_b, MomentOfInertiaComponent)
            inv_inertia_b = moi_b_comp.invInertia if moi_b_comp else 0.0

            if inv_mass_a + inv_mass_b + inv_inertia_a + inv_inertia_b <= epsilon:
                continue

            diff = p_b - p_a
            current_length = np.linalg.norm(diff)
            if current_length <= epsilon:
                continue

            direction = diff / current_length

            C = current_length - constraint.restLength

            alpha_tilde = constraint.compliance / (dt * dt)

            w_sum = inv_mass_a + inv_mass_b

            denominator = w_sum + alpha_tilde
            if denominator <= epsilon:
                continue
            
            # Note: `constraint.lambda` from JS is renamed to `constraint.lambda_val`
            # to avoid conflict with Python's `lambda` keyword.
            # The `DistanceConstraintComponent` dataclass should reflect this.
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
            orientation.angle += angular_vel.angularVelocity * dt

class PBDBallBallCollisions:
    run_in_pause = False
    def update(self, world, dt):
        ball_entities = world.query([BallTagComponent, PositionComponent, VelocityComponent, RadiusComponent, MassComponent, RestitutionComponent])
        for i in range(len(ball_entities)):
            for j in range(i + 1, len(ball_entities)):
                e1 = ball_entities[i]
                e2 = ball_entities[j]

                p1 = world.get_component(e1, PositionComponent).pos
                v1 = world.get_component(e1, VelocityComponent).vel
                r1 = world.get_component(e1, RadiusComponent).radius
                m1 = world.get_component(e1, MassComponent).mass
                res1 = world.get_component(e1, RestitutionComponent).restitution

                p2 = world.get_component(e2, PositionComponent).pos
                v2 = world.get_component(e2, VelocityComponent).vel
                r2 = world.get_component(e2, RadiusComponent).radius
                m2 = world.get_component(e2, MassComponent).mass
                res2 = world.get_component(e2, RestitutionComponent).restitution

                restitution = min(res1, res2)
                direction = p2 - p1
                d_sq = np.dot(direction, direction)
                r_sum = r1 + r2

                if d_sq == 0.0 or d_sq > r_sum * r_sum:
                    continue

                d = math.sqrt(d_sq)
                direction /= d # Normalize

                # Resolve penetration
                corr = (r_sum - d) / 2.0
                p1 += direction * -corr
                p2 += direction * corr

                # Resolve velocity
                vel1_dot = np.dot(v1, direction)
                vel2_dot = np.dot(v2, direction)

                new_v1_dot = (m1 * vel1_dot + m2 * vel2_dot - m2 * (vel1_dot - vel2_dot) * restitution) / (m1 + m2)
                new_v2_dot = (m1 * vel1_dot + m2 * vel2_dot - m1 * (vel2_dot - vel1_dot) * restitution) / (m1 + m2)

                v1 += direction * (new_v1_dot - vel1_dot)
                v2 += direction * (new_v2_dot - vel2_dot)

class PBDBallObstacleCollisions:
    def update(self, world, dt):
        ball_entities = world.query([BallTagComponent, PositionComponent, VelocityComponent, RadiusComponent, MassComponent])
        obstacle_entities = world.query([ObstacleTagComponent, PositionComponent, RadiusComponent, ObstaclePushComponent])

        for ball_id in ball_entities:
            p1 = world.get_component(ball_id, PositionComponent).pos
            v1 = world.get_component(ball_id, VelocityComponent).vel
            r1 = world.get_component(ball_id, RadiusComponent).radius
            ball_mass_comp = world.get_component(ball_id, MassComponent)
            ball_ang_vel_comp = world.get_component(ball_id, AngularVelocityComponent)
            ball_moi_comp = world.get_component(ball_id, MomentOfInertiaComponent)

            for obs_id in obstacle_entities:
                p2 = world.get_component(obs_id, PositionComponent).pos
                r2 = world.get_component(obs_id, RadiusComponent).radius
                push_vel = world.get_component(obs_id, ObstaclePushComponent).pushVel

                direction = p1 - p2
                d_sq = np.dot(direction, direction)
                r_sum = r1 + r2

                if d_sq == 0.0 or d_sq > r_sum * r_sum:
                    continue

                d = math.sqrt(d_sq)
                direction /= d # Normalize

                # Resolve penetration
                corr = r_sum - d
                p1 += direction * corr

                # Resolve velocity & rotation with friction and obstacle rotation
                obs_ang_vel_comp = world.get_component(obs_id, AngularVelocityComponent)
                obs_moi_comp = world.get_component(obs_id, MomentOfInertiaComponent)
                obs_friction_comp = world.get_component(obs_id, CoefficientOfFrictionComponent)

                omega_obs = obs_ang_vel_comp.angularVelocity if obs_ang_vel_comp else 0.0
                mu = obs_friction_comp.mu if obs_friction_comp else 0.0

                tangent = np.array([-direction[1], direction[0], direction[2]])

                v_surf_obs_tangential_comp = 0
                if omega_obs != 0:
                    v_surf_obs_tangential_comp = (-omega_obs * direction[1] * r2) * tangent[0] + (omega_obs * direction[0] * r2) * tangent[1]

                s_sign = 0
                if v_surf_obs_tangential_comp != 0 and mu != 0:
                    s_sign = math.copysign(1, v_surf_obs_tangential_comp)
                
                effective_push_dir = direction.copy()
                if s_sign != 0:
                    effective_push_dir += tangent * (mu * s_sign)
                v2_helpers.normalize_inplace(effective_push_dir)

                v1 += effective_push_dir * push_vel

                if mu > 0 and ball_mass_comp:
                    delta_v_ball_tangential_actual_scalar_comp = np.dot(effective_push_dir, tangent) * push_vel

                    if abs(delta_v_ball_tangential_actual_scalar_comp) > 1e-9:
                        j_t_on_ball_vec = tangent * (delta_v_ball_tangential_actual_scalar_comp * ball_mass_comp.mass)

                        if ball_ang_vel_comp and ball_moi_comp and ball_moi_comp.invInertia > 0:
                            r_contact_ball = direction * -r1
                            delta_l_ball = r_contact_ball[0] * j_t_on_ball_vec[1] - r_contact_ball[1] * j_t_on_ball_vec[0]
                            ball_ang_vel_comp.angularVelocity += delta_l_ball * ball_moi_comp.invInertia

                        if obs_ang_vel_comp and obs_moi_comp and obs_moi_comp.invInertia > 0:
                            j_t_on_obs_vec = j_t_on_ball_vec * -1
                            r_contact_obs = direction * r2
                            delta_l_obs = r_contact_obs[0] * j_t_on_obs_vec[1] - r_contact_obs[1] * j_t_on_obs_vec[0]
                            obs_ang_vel_comp.angularVelocity += delta_l_obs * obs_moi_comp.invInertia

                grabbed = world.get_resource('grabbedBall')
                if ball_id != grabbed:
                    world.add_component(ball_id, ScoredTagComponent())

class InputReplaySystem:
    run_in_pause = False
    def __init__(self, input_log, input_system):
        self.input_log = input_log
        self.current_index = 0
        self.frame = 0
        self.input_system = input_system

    def update(self, world, dt):
        if len(self.input_log) > 0:
            # Assuming input_log is a list of dicts with 'frame', 'clicks', 'releases'
            if self.input_system.frame == self.input_log[0]['frame']:
                frame_data = self.input_log.pop(0)
                self.input_system.clicks = frame_data['clicks'][:]
                self.input_system.releases = frame_data['releases'][:]
