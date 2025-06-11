import asyncio
import json
import numpy as np
import websockets

# Assuming the python ECS code is in a 'python' directory
from python.ecs import (
    World, PauseStateComponent, PositionComponent, VelocityComponent, RadiusComponent, MassComponent,
    RestitutionComponent, GravityAffectedComponent, OrientationComponent, AngularVelocityComponent,
    MomentOfInertiaComponent, ObstacleTagComponent, ScoredTagComponent, FlipperTagComponent,
    FlipperStateComponent, FlipperTipComponent, BorderComponent, RenderableComponent,
    PrevFinalPosComponent, PrevFinalOrientationComponent, BallTagComponent, ObstaclePushComponent,
    CableLinkComponent, CoefficientOfFrictionComponent
)
from python.common_systems import (
    PrevFinalPosSystem, GravitySystem, MovementSystem, AngularMovementSystem,
    PBDBallObstacleCollisions, PBDBallBallCollisions, PBDVelocityUpdateSystem,
    PrevFinalOrientationSystem, PBDAngularVelocityUpdateSystem, FlipperTipLinkSystem
)
from python.cable_attachment_update_system import CableAttachmentUpdateSystem
from python.pbd_cable_constraint_solver import PBDCableConstraintSolver
from python.geometry import right_of_line, tangent_from_point_to_circle, tangent_from_circle_to_circle
from python.cable_joints_components import CableJointComponent, CablePathComponent, create_cable_path_component
from python.ball_obstacle_bump_system import BallObstacleBumpSystem
from python.ball_border_or_flipper_velocity_contact_system import BallBorderOrFlipperVelocityContactSystem
from python.cable_attachment_cache_system import CableAttachmentCacheSystem
from python.cable_slip_slack_friction_system import CableSlipSlackFrictionSystem

# --- Server-Side Systems ---

class ScoreComponent:
    def __init__(self, score=0):
        self.value = score

class FlipperMotionSystem:
    run_in_pause = True
    def update(self, world, dt):
        for entity_id in world.query([FlipperStateComponent]):
            state = world.get_component(entity_id, FlipperStateComponent)
            prev_rotation = state.rotation
            if state.pressed:
                state.rotation = min(state.rotation + dt * state.angular_velocity, state.max_rotation)
            else:
                state.rotation = max(state.rotation - dt * state.angular_velocity, 0.0)
            state.current_angular_velocity = state.sign * (state.rotation - prev_rotation) / dt if dt > 1e-6 else 0.0

class PBDBallFlipperCollisions:
    def _get_flipper_tip(self, flipper_pos, flipper_state):
        angle = flipper_state.rest_angle + flipper_state.sign * flipper_state.rotation
        direction = np.array([np.cos(angle), np.sin(angle), 0.0])
        return flipper_pos + direction * flipper_state.length

    def _closest_point_on_segment(self, p, a, b):
        ab = b - a
        ap = p - a
        if np.dot(ab, ab) == 0: return a
        t = np.dot(ap, ab) / np.dot(ab, ab)
        t = np.clip(t, 0, 1)
        return a + ab * t

    def update(self, world, dt):
        ball_entities = world.query([BallTagComponent, PositionComponent, RadiusComponent, MassComponent])
        flipper_entities = world.query([FlipperTagComponent, PositionComponent, RadiusComponent, FlipperStateComponent])

        contacts = world.get_resource('ball_flipper_contacts')
        if contacts is None:
            contacts = []
            world.set_resource('ball_flipper_contacts', contacts)
        contacts.clear()

        for ball_id in ball_entities:
            p1_comp = world.get_component(ball_id, PositionComponent)
            p1 = p1_comp.pos
            r1 = world.get_component(ball_id, RadiusComponent).radius
            mass_comp = world.get_component(ball_id, MassComponent)
            inv_mass = 1.0 / mass_comp.mass if mass_comp and mass_comp.mass > 0 else 0.0

            for flip_id in flipper_entities:
                fp = world.get_component(flip_id, PositionComponent).pos
                fr = world.get_component(flip_id, RadiusComponent).radius
                fs = world.get_component(flip_id, FlipperStateComponent)

                tip = self._get_flipper_tip(fp, fs)
                closest = self._closest_point_on_segment(p1, fp, tip)

                direction = p1 - closest
                d_sq = np.dot(direction, direction)
                r_sum = r1 + fr

                if d_sq == 0.0 or d_sq > r_sum * r_sum:
                    continue

                d = np.sqrt(d_sq)
                direction /= d

                # Resolve penetration
                corr = r_sum - d
                if inv_mass > 0:
                    p1 += direction * corr

                # Calculate positional impulse for the velocity solver
                delta_lambda = 0
                if inv_mass > 0:
                    # Since the flipper is treated as having infinite mass in this positional solve,
                    # the generalized inverse mass is just the ball's inverse mass.
                    w_inv = inv_mass
                    delta_lambda = corr / w_inv

                # Store contact info for velocity system
                contacts.append({
                    'ball_id': ball_id,
                    'flip_id': flip_id,
                    'normal': direction.copy(),
                    'contact_point_on_flipper': closest.copy(),
                    'delta_lambda': delta_lambda
                })

class PBDBallBorderCollisions:
    def _closest_point_on_segment(self, p, a, b):
        ab = b - a
        ap = p - a
        if np.dot(ab, ab) == 0: return a
        t = np.dot(ap, ab) / np.dot(ab, ab)
        t = np.clip(t, 0, 1)
        return a + ab * t

    def update(self, world, dt):
        ball_entities = world.query([BallTagComponent, PositionComponent, RadiusComponent, MassComponent])
        border_entities = world.query([BorderComponent])
        if not border_entities: return

        border_comp = world.get_component(border_entities[0], BorderComponent)
        border_points = border_comp.points

        contacts = world.get_resource('ball_border_contacts')
        if contacts is None:
            contacts = []
            world.set_resource('ball_border_contacts', contacts)
        contacts.clear()

        for ball_id in ball_entities:
            p1_comp = world.get_component(ball_id, PositionComponent)
            p1 = p1_comp.pos
            r1 = world.get_component(ball_id, RadiusComponent).radius
            mass_comp = world.get_component(ball_id, MassComponent)
            inv_mass = 1.0 / mass_comp.mass if mass_comp and mass_comp.mass > 0 else 0.0

            min_dist_sq = float('inf')
            closest_seg_point = np.zeros(3)
            edge_start, edge_end = None, None

            for i in range(len(border_points)):
                a = border_points[i]
                b = border_points[(i + 1) % len(border_points)]
                closest_pt = self._closest_point_on_segment(p1, a, b)
                dist_sq = np.sum((p1 - closest_pt)**2)
                if dist_sq < min_dist_sq:
                    min_dist_sq = dist_sq
                    closest_seg_point = closest_pt
                    edge_start, edge_end = a, b

            if min_dist_sq > r1 * r1: continue

            ball_to_closest = p1 - closest_seg_point

            edge_vec = edge_end - edge_start
            normal = np.array([-edge_vec[1], edge_vec[0], 0.0])
            normal /= np.linalg.norm(normal)

            if np.sum(ball_to_closest**2) < 1e-9:
                collision_normal = normal
            else:
                collision_normal = ball_to_closest / np.linalg.norm(ball_to_closest)

            if np.dot(ball_to_closest, normal) < 0:
                collision_normal = normal

            dist = np.sqrt(min_dist_sq)
            penetration = r1 - dist
            delta_lambda = 0
            if penetration > 0:
                if inv_mass > 0:
                    p1 += collision_normal * penetration
                    # Since the border is static, the generalized inverse mass is just the ball's.
                    w_inv = inv_mass
                    delta_lambda = penetration / w_inv

            # Store contact info for the velocity-based system
            contacts.append({
                'ball_id': ball_id,
                'normal': collision_normal.copy(),
                'delta_lambda': delta_lambda
            })

class ScoreSystem:
    run_in_pause = False
    def update(self, world, dt):
        score_entity_query = world.query([ScoreComponent])
        if not score_entity_query: return
        score_entity = score_entity_query[0]
        score_comp = world.get_component(score_entity, ScoreComponent)
        scored_entities = world.query([ScoredTagComponent])
        for scored_id in scored_entities:
            world.remove_component(scored_id, ScoredTagComponent)
            score_comp.value += 1

class RemoteInputSystem:
    run_in_pause = True
    def __init__(self):
        self.clicks = []
        self.releases = []
        self.grab_spring = None
        self.events_queue = []

    def add_event(self, event):
        self.events_queue.append(event)

    def _handle_pointer_down(self, world, pos):
        if self.grab_spring:
            return

        closest_ball = None
        closest_dist_sq = float('inf')

        ball_entities = world.query([BallTagComponent, PositionComponent, RadiusComponent])
        for ball_id in ball_entities:
            ball_pos = world.get_component(ball_id, PositionComponent).pos
            radius = world.get_component(ball_id, RadiusComponent).radius
            dist_sq = np.sum((pos - ball_pos)**2)
            if dist_sq <= radius**2 and dist_sq < closest_dist_sq:
                closest_ball = ball_id
                closest_dist_sq = dist_sq

        if closest_ball is not None:
            ptr_e = world.create_entity()
            world.add_component(ptr_e, PositionComponent(pos.copy()))
            world.add_component(ptr_e, CableLinkComponent(prev_cable_attachment_time_pos=pos.copy()))

            ball_pos = world.get_component(closest_ball, PositionComponent).pos
            joint_e = world.create_entity()
            world.add_component(joint_e, CableJointComponent(
                entity_a=closest_ball, entity_b=ptr_e, rest_length=0.1,
                attachment_point_a_world=ball_pos.copy(), attachment_point_b_world=pos.copy()
            ))

            path_e = world.create_entity()
            path_comp = create_cable_path_component(
                world, joint_entities=[joint_e], link_types=['attachment', 'attachment'],
                cw=[True, True], spring_constant=10.0
            )
            world.add_component(path_e, path_comp)

            self.grab_spring = {'ptr_e': ptr_e, 'joint_e': joint_e, 'path_e': path_e, 'ball_e': closest_ball}
            world.set_resource('grabbedBall', closest_ball)

            pause_state = world.get_resource('pauseState')
            if pause_state:
                pause_state.paused = False
        else:
            self.clicks.append(pos)

    def _handle_pointer_move(self, world, pos):
        if self.grab_spring:
            ptr_e = self.grab_spring['ptr_e']
            ptr_pos_comp = world.get_component(ptr_e, PositionComponent)
            if ptr_pos_comp:
                ptr_pos_comp.pos[:] = pos

    def _handle_pointer_up(self, world, pos):
        if self.grab_spring:
            ball_e = self.grab_spring['ball_e']
            vel_comp = world.get_component(ball_e, VelocityComponent)
            pos_comp = world.get_component(ball_e, PositionComponent)
            prev_final_pos_comp = world.get_component(ball_e, PrevFinalPosComponent)
            dt = world.get_resource('dt')

            if vel_comp and pos_comp and prev_final_pos_comp and dt > 1e-9:
                np.subtract(pos_comp.pos, prev_final_pos_comp.pos, out=vel_comp.vel)
                np.divide(vel_comp.vel, dt, out=vel_comp.vel)

            world.destroy_entity(self.grab_spring['path_e'])
            world.destroy_entity(self.grab_spring['joint_e'])
            world.destroy_entity(self.grab_spring['ptr_e'])
            self.grab_spring = None
            world.set_resource('grabbedBall', None)
        else:
            self.releases.append(pos)

    def update(self, world, dt):
        for event in self.events_queue:
            event_type = event.get('type')
            pos = event.get('pos')
            if event_type == 'pointerdown':
                self._handle_pointer_down(world, pos)
            elif event_type == 'pointermove':
                self._handle_pointer_move(world, pos)
            elif event_type == 'pointerup':
                self._handle_pointer_up(world, pos)
        self.events_queue.clear()

        if self.clicks:
            click_pos = self.clicks.pop(0)
            border_ents = world.query([BorderComponent])
            if not border_ents: return
            border_points = world.get_component(border_ents[0], BorderComponent).points

            right_click = right_of_line(click_pos, border_points[0], border_points[1]) and \
                          right_of_line(click_pos, border_points[1], border_points[2])
            left_click = right_of_line(click_pos, border_points[5], border_points[6]) and \
                         right_of_line(click_pos, border_points[6], border_points[7])

            flipper_entities = world.query([FlipperTagComponent, PositionComponent, FlipperStateComponent])
            if not flipper_entities: return

            flipper_data = [{'id': fid, 'x': world.get_component(fid, PositionComponent).pos[0]} for fid in flipper_entities]
            flipper_data.sort(key=lambda f: f['x'])
            left_flipper_id = flipper_data[0]['id']
            right_flipper_id = flipper_data[-1]['id']

            if right_click:
                world.get_component(right_flipper_id, FlipperStateComponent).pressed = True
            elif left_click:
                world.get_component(left_flipper_id, FlipperStateComponent).pressed = True
            else:
                closest_flipper_id, min_dist_sq = None, float('inf')
                for fid in flipper_entities:
                    pos = world.get_component(fid, PositionComponent).pos
                    dist_sq = np.sum((click_pos - pos)**2)
                    if dist_sq < min_dist_sq:
                        min_dist_sq, closest_flipper_id = dist_sq, fid
                if closest_flipper_id:
                    state = world.get_component(closest_flipper_id, FlipperStateComponent)
                    if min_dist_sq < state.length**2:
                         world.get_component(closest_flipper_id, FlipperStateComponent).pressed = True

        if self.releases:
            release_pos = self.releases.pop(0)
            flipper_entities = world.query([FlipperTagComponent, PositionComponent, FlipperStateComponent])
            closest_id, min_dist_sq = None, float('inf')
            for id in flipper_entities:
                state = world.get_component(id, FlipperStateComponent)
                if not state.pressed: continue
                pos = world.get_component(id, PositionComponent).pos
                d2 = np.sum((release_pos - pos)**2)
                if d2 < min_dist_sq:
                    min_dist_sq, closest_id = d2, id
            if closest_id is not None:
                world.get_component(closest_id, FlipperStateComponent).pressed = False

# --- Scene and World Setup ---

def setup_scene(world):
    world.clear()

    sim_height = 1.7
    sim_width = 1.0 # Aspect ratio 1/1.7, will be updated by client

    world.set_resource('gravity', np.array([0.0, -2.0, 0.0]))
    world.set_resource('dt', 1.0 / 300.0)
    world.set_resource('simWidth', sim_width)
    world.set_resource('simHeight', sim_height)
    world.set_resource('pauseState', PauseStateComponent(True))
    world.set_resource('debugRenderPoints', {})
    world.set_resource('grabbedBall', None)
    world.set_resource('ball_obstacle_contacts', [])
    world.set_resource('ball_border_contacts', [])
    world.set_resource('ball_flipper_contacts', [])

    offset = 0.02
    border_points = [
        np.array([0.74, 0.0, 0.0]), np.array([0.74, 0.25, 0.0]),
        np.array([1.0 - offset, 0.4, 0.0]), np.array([1.0 - offset, sim_height - offset, 0.0]),
        np.array([offset, sim_height - offset, 0.0]), np.array([offset, 0.4, 0.0]),
        np.array([0.26, 0.25, 0.0]), np.array([0.26, 0.0, 0.0])
    ]
    border_entity = world.create_entity()
    world.add_component(border_entity, BorderComponent(border_points))
    world.add_component(border_entity, RenderableComponent('border', '#000000'))

    ball_radius = 0.03
    ball_mass = np.pi * ball_radius**2
    ball_restitution = 0.6

    ball_ids = []
    ball_positions = [(0.90, 0.95), (0.08, 0.5)]
    for pos_tuple in ball_positions:
        pos = np.array([pos_tuple[0], pos_tuple[1], 0.0])
        ball = world.create_entity()
        world.add_component(ball, BallTagComponent())
        world.add_component(ball, PositionComponent(pos.copy()))
        world.add_component(ball, VelocityComponent(np.zeros(3)))
        world.add_component(ball, RadiusComponent(ball_radius))
        world.add_component(ball, MassComponent(ball_mass))
        world.add_component(ball, RestitutionComponent(ball_restitution))
        world.add_component(ball, GravityAffectedComponent())
        world.add_component(ball, RenderableComponent('circle', '#a0a0a0'))
        world.add_component(ball, OrientationComponent(0.0))
        world.add_component(ball, AngularVelocityComponent(0.0))
        world.add_component(ball, MomentOfInertiaComponent(0.5 * ball_mass * ball_radius**2))
        world.add_component(ball, PrevFinalOrientationComponent(0.0))
        world.add_component(ball, PrevFinalPosComponent(pos.copy()))
        world.add_component(ball, CoefficientOfFrictionComponent(0.2)) # Add friction to balls
        ball_ids.append(ball)
    ball1, ball2 = ball_ids[0], ball_ids[1]

    obs_push = 2.7
    obstacles_data = [
        (0.25, 0.6, 0.1, "#0F7090", 0), (0.75, 0.5, 0.1, "#0F7090", 0),
        (0.7, 1.0, 0.12, "#FF8000", 100.0), (0.2, 1.2, 0.1, "#FF8000", -100.0)
    ]
    obs_ids = []
    for x, y, r, color, ang_vel in obstacles_data:
        obs = world.create_entity()
        world.add_component(obs, ObstacleTagComponent())
        world.add_component(obs, PositionComponent(np.array([x, y, 0.0])))
        world.add_component(obs, MassComponent(-1.0))
        world.add_component(obs, RadiusComponent(r))
        world.add_component(obs, ObstaclePushComponent(obs_push))
        world.add_component(obs, RenderableComponent('circle', color))
        if ang_vel != 0:
            world.add_component(obs, OrientationComponent(0.0))
            world.add_component(obs, AngularVelocityComponent(ang_vel))
            world.add_component(obs, MomentOfInertiaComponent(0.020 * r**2))
            world.add_component(obs, PrevFinalOrientationComponent(0.0))
        obs_ids.append(obs)
    obs3, obs4 = obs_ids[2], obs_ids[3]

    # Flipper Entities
    flip_radius = 0.03
    flip_length = 0.2
    flip_max_rot = 1.0
    flip_rest_angle = 0.5
    flip_ang_vel = 20.0
    flip_restitution = 0.6

    flipper1 = world.create_entity()
    flipper1_pos = np.array([0.26, 0.22, 0.0])
    world.add_component(flipper1, FlipperTagComponent())
    world.add_component(flipper1, PositionComponent(flipper1_pos))
    world.add_component(flipper1, RadiusComponent(flip_radius))
    world.add_component(flipper1, FlipperStateComponent(flip_length, -flip_rest_angle, flip_max_rot, flip_ang_vel))
    world.add_component(flipper1, RestitutionComponent(flip_restitution))
    world.add_component(flipper1, RenderableComponent('flipper', '#FF0000'))
    world.add_component(flipper1, CableLinkComponent(prev_cable_attachment_time_pos=flipper1_pos))

    flipper1_tip = world.create_entity()
    world.add_component(flipper1_tip, PositionComponent())
    world.add_component(flipper1_tip, RadiusComponent(flip_radius))
    world.add_component(flipper1_tip, FlipperTipComponent(flipper1))
    world.add_component(flipper1_tip, CableLinkComponent())
    world.add_component(flipper1_tip, CoefficientOfFrictionComponent(0.01))

    flipper2 = world.create_entity()
    flipper2_pos = np.array([0.74, 0.22, 0.0])
    world.add_component(flipper2, FlipperTagComponent())
    world.add_component(flipper2, PositionComponent(flipper2_pos))
    world.add_component(flipper2, RadiusComponent(flip_radius))
    world.add_component(flipper2, FlipperStateComponent(flip_length, np.pi + flip_rest_angle, -flip_max_rot, flip_ang_vel))
    world.add_component(flipper2, RestitutionComponent(flip_restitution))
    world.add_component(flipper2, RenderableComponent('flipper', '#FF0000'))
    world.add_component(flipper2, CableLinkComponent(prev_cable_attachment_time_pos=flipper2_pos))

    flipper2_tip = world.create_entity()
    world.add_component(flipper2_tip, PositionComponent())
    world.add_component(flipper2_tip, RadiusComponent(flip_radius))
    world.add_component(flipper2_tip, FlipperTipComponent(flipper2))
    world.add_component(flipper2_tip, CableLinkComponent())
    world.add_component(flipper2_tip, CoefficientOfFrictionComponent(0.01))

    score_entity = world.create_entity()
    world.add_component(score_entity, ScoreComponent(0))

    # --- Cable Setup ---
    # Connect: ball2 -> obs4 -> obs3 -> ball1
    friction_coefficient = 0.2

    pos_ball1 = world.get_component(ball1, PositionComponent).pos
    pos_ball2 = world.get_component(ball2, PositionComponent).pos
    pos_obs3 = world.get_component(obs3, PositionComponent).pos
    radius_obs3 = world.get_component(obs3, RadiusComponent).radius
    pos_obs4 = world.get_component(obs4, PositionComponent).pos
    radius_obs4 = world.get_component(obs4, RadiusComponent).radius

    world.add_component(obs4, CableLinkComponent(prev_cable_attachment_time_pos=pos_obs4.copy()))
    world.add_component(obs3, CableLinkComponent(prev_cable_attachment_time_pos=pos_obs3.copy()))
    world.add_component(obs4, CoefficientOfFrictionComponent(friction_coefficient))
    world.add_component(obs3, CoefficientOfFrictionComponent(friction_coefficient))
    world.add_component(ball1, CableLinkComponent(prev_cable_attachment_time_pos=pos_ball1.copy()))
    world.add_component(ball2, CableLinkComponent(prev_cable_attachment_time_pos=pos_ball2.copy()))
    world.add_component(ball1, CoefficientOfFrictionComponent(friction_coefficient))
    world.add_component(ball2, CoefficientOfFrictionComponent(friction_coefficient))

    # Joint 1: ball2 -> obs4
    joint1 = world.create_entity()
    tangent_obs4 = tangent_from_point_to_circle(pos_ball2, pos_obs4, radius_obs4, True)
    attach_obs4 = tangent_obs4['a_circle']
    dir1 = attach_obs4 - pos_ball2
    dir1 /= np.linalg.norm(dir1)
    attach_ball2 = pos_ball2 + dir1 * ball_radius
    initial_dist1 = np.linalg.norm(attach_ball2 - attach_obs4)
    world.add_component(joint1, CableJointComponent(ball2, obs4, initial_dist1, attach_ball2, attach_obs4))

    # Joint 2: obs4 -> obs3
    joint2 = world.create_entity()
    initial_points2 = tangent_from_circle_to_circle(pos_obs4, radius_obs4, True, pos_obs3, radius_obs3, True)
    initial_dist2 = np.linalg.norm(initial_points2['a_circle'] - initial_points2['b_circle'])
    world.add_component(joint2, CableJointComponent(obs4, obs3, initial_dist2, initial_points2['a_circle'], initial_points2['b_circle']))

    # Joint 3: obs3 -> ball1
    joint3 = world.create_entity()
    tangent_obs3 = tangent_from_point_to_circle(pos_ball1, pos_obs3, radius_obs3, False)
    attach_obs3 = tangent_obs3['a_circle']
    dir3 = attach_obs3 - pos_ball1
    dir3 /= np.linalg.norm(dir3)
    attach_ball1 = pos_ball1 + dir3 * ball_radius
    initial_dist3 = np.linalg.norm(attach_ball1 - attach_obs3)
    world.add_component(joint3, CableJointComponent(obs3, ball1, initial_dist3, attach_obs3, attach_ball1))

    # Create Cable Path
    cable_path = world.create_entity()
    path_comp = create_cable_path_component(
        world,
        [joint1, joint2, joint3],
        ['hybrid-attachment', 'rolling', 'rolling', 'hybrid-attachment'],
        [True, True, True, True],
        200.0
    )
    world.add_component(cable_path, path_comp)

    if not world.systems:
        # 1. Cache state from previous step
        world.register_system(PrevFinalPosSystem())
        world.register_system(PrevFinalOrientationSystem())

        # 2. Handle user input and non-physics state changes
        world.register_system(RemoteInputSystem())
        world.register_system(FlipperMotionSystem())

        # 3. PREDICTION: Apply forces and integrate velocity to get predicted positions
        world.register_system(GravitySystem())
        world.register_system(MovementSystem())
        world.register_system(AngularMovementSystem())

        # 4. Update derived geometry based on predicted positions
        world.register_system(FlipperTipLinkSystem())
        world.register_system(CableAttachmentUpdateSystem())
        world.register_system(CableAttachmentCacheSystem())
        world.register_system(CableSlipSlackFrictionSystem())

        # 5. POSITIONAL SOLVERS: Correct predicted positions to satisfy constraints.
        world.register_system(PBDCableConstraintSolver())
        world.register_system(PBDBallBorderCollisions())
        world.register_system(PBDBallBallCollisions())
        world.register_system(PBDBallObstacleCollisions())
        world.register_system(PBDBallFlipperCollisions())

        # 6. UPDATE VELOCITY: Derive final velocities from the position changes
        world.register_system(PBDVelocityUpdateSystem())
        world.register_system(PBDAngularVelocityUpdateSystem())

        # 7. VELOCITY SOLVERS: Apply restitution and dynamic friction
        world.register_system(BallObstacleBumpSystem())
        world.register_system(BallBorderOrFlipperVelocityContactSystem())

        # 9. Game Logic
        world.register_system(ScoreSystem())

def world_to_json(world):
    state = {
        'balls': [], 'obstacles': [], 'flippers': [], 'border': [], 'score': 0, 'cables': []
    }
    for ball_id in world.query([BallTagComponent, PositionComponent, RadiusComponent]):
        pos = world.get_component(ball_id, PositionComponent).pos
        radius = world.get_component(ball_id, RadiusComponent).radius
        ball_data = {'x': pos[0], 'y': pos[1], 'radius': radius}
        orientation_comp = world.get_component(ball_id, OrientationComponent)
        if orientation_comp:
            ball_data['angle'] = orientation_comp.angle
        state['balls'].append(ball_data)

    for obs_id in world.query([ObstacleTagComponent, PositionComponent, RadiusComponent]):
        pos = world.get_component(obs_id, PositionComponent).pos
        radius = world.get_component(obs_id, RadiusComponent).radius
        renderable = world.get_component(obs_id, RenderableComponent)
        color = renderable.color if renderable else '#ffffff'
        obs_data = {'x': pos[0], 'y': pos[1], 'radius': radius, 'color': color}
        orientation_comp = world.get_component(obs_id, OrientationComponent)
        if orientation_comp:
            obs_data['angle'] = orientation_comp.angle
        state['obstacles'].append(obs_data)

    for flip_id in world.query([FlipperTagComponent, PositionComponent, RadiusComponent, FlipperStateComponent]):
        pos = world.get_component(flip_id, PositionComponent).pos
        radius = world.get_component(flip_id, RadiusComponent).radius
        f_state = world.get_component(flip_id, FlipperStateComponent)
        state['flippers'].append({
            'x': pos[0], 'y': pos[1], 'radius': radius, 'length': f_state.length,
            'angle': f_state.rest_angle + f_state.sign * f_state.rotation
        })

    border_query = world.query([BorderComponent])
    if border_query:
        border_id = border_query[0]
        border_points = world.get_component(border_id, BorderComponent).points
        state['border'] = [p.tolist()[:2] for p in border_points]

    score_query = world.query([ScoreComponent])
    if score_query:
        score_id = score_query[0]
        state['score'] = world.get_component(score_id, ScoreComponent).value

    pause_comp = world.get_resource('pauseState')
    if pause_comp:
        state['isPaused'] = pause_comp.paused
    else:
        state['isPaused'] = True

    # Serialize Cables
    path_entities = world.query([CablePathComponent])
    for path_id in path_entities:
        path = world.get_component(path_id, CablePathComponent)
        if not path.joint_entities:
            continue

        cable_render_data = {
            'joints': [],
            'arcs': [],
            'links': []
        }

        # 1. Joints
        for joint_id in path.joint_entities:
            joint = world.get_component(joint_id, CableJointComponent)
            pA = joint.attachment_point_a_world.tolist()[:2]
            pB = joint.attachment_point_b_world.tolist()[:2]
            rest_length = joint.rest_length
            cable_render_data['joints'].append({'pA': pA, 'pB': pB, 'restLength': rest_length})

        # 2. Rolling arcs
        for i in range(1, len(path.link_types) - 1):
            if path.link_types[i] == 'rolling':
                j_prev_id = path.joint_entities[i - 1]
                j_next_id = path.joint_entities[i]
                j_prev = world.get_component(j_prev_id, CableJointComponent)
                j_next = world.get_component(j_next_id, CableJointComponent)

                roller_id = j_prev.entity_b
                center_comp = world.get_component(roller_id, PositionComponent)
                radius_comp = world.get_component(roller_id, RadiusComponent)
                if not center_comp or not radius_comp:
                    continue

                center = center_comp.pos.tolist()[:2]
                radius = radius_comp.radius
                p1 = j_prev.attachment_point_b_world.tolist()[:2]
                p2 = j_next.attachment_point_a_world.tolist()[:2]
                anticlockwise = not path.cw[i]

                epsilon = 1e-6
                dist_prev = np.linalg.norm(j_prev.attachment_point_a_world - j_prev.attachment_point_b_world)
                tension_prev = dist_prev > (j_prev.rest_length + epsilon)
                dist_next = np.linalg.norm(j_next.attachment_point_a_world - j_next.attachment_point_b_world)
                tension_next = dist_next > (j_next.rest_length + epsilon)
                is_taut = tension_prev and tension_next

                cable_render_data['arcs'].append({
                    'center': center,
                    'radius': radius,
                    'p1': p1,
                    'p2': p2,
                    'anticlockwise': anticlockwise,
                    'is_taut': bool(is_taut)
                })

        # 3. Hybrid link markers and arcs
        for i, link_type in enumerate(path.link_types):
            link_data = {"type": link_type}
            if link_type == 'hybrid-attachment':
                attachment_point = None
                if i == 0:
                    joint = world.get_component(path.joint_entities[0], CableJointComponent)
                    attachment_point = joint.attachment_point_a_world
                else:  # Last link
                    joint = world.get_component(path.joint_entities[-1], CableJointComponent)
                    attachment_point = joint.attachment_point_b_world

                if attachment_point is not None:
                    link_data['attachmentPoint'] = attachment_point.tolist()[:2]

            elif link_type == 'hybrid':
                roller_id = None
                tangent_point = None
                is_taut = False
                epsilon = 1e-6

                if i == 0:
                    joint = world.get_component(path.joint_entities[0], CableJointComponent)
                    roller_id = joint.entity_a
                    tangent_point = joint.attachment_point_a_world
                    dist = np.linalg.norm(joint.attachment_point_a_world - joint.attachment_point_b_world)
                    is_taut = dist > (joint.rest_length + epsilon)
                else:  # Last link
                    joint = world.get_component(path.joint_entities[-1], CableJointComponent)
                    roller_id = joint.entity_b
                    tangent_point = joint.attachment_point_b_world
                    dist = np.linalg.norm(joint.attachment_point_a_world - joint.attachment_point_b_world)
                    is_taut = dist > (joint.rest_length + epsilon)

                center_comp = world.get_component(roller_id, PositionComponent)
                radius_comp = world.get_component(roller_id, RadiusComponent)

                if center_comp and radius_comp and tangent_point is not None:
                    link_data['center'] = center_comp.pos.tolist()[:2]
                    link_data['radius'] = radius_comp.radius
                    link_data['tangentPoint'] = tangent_point.tolist()[:2]
                    link_data['storedLength'] = path.stored[i]
                    link_data['cw'] = path.cw[i]
                    link_data['is_taut'] = bool(is_taut)

            cable_render_data['links'].append(link_data)

        state['cables'].append(cable_render_data)

    return json.dumps(state)

# --- WebSocket Handler ---

async def handler(websocket):
    world = World()
    setup_scene(world)
    input_system = world.get_system(RemoteInputSystem)

    # Send initial state
    await websocket.send(world_to_json(world))

    async for message in websocket:
        data = json.loads(message)
        action = data.get('action')

        pause_state = world.get_resource('pauseState')

        if action == 'step':
            if not pause_state.paused:
                steps = data.get('steps', 0)
                if steps > 0:
                    dt = world.get_resource('dt')
                    for _ in range(steps):
                        world.update(dt)
        elif action == 'single_step':
            pause_state.paused = False
            world.update(world.get_resource('dt'))
            pause_state.paused = True
        elif action == 'reset':
            setup_scene(world)
            input_system = world.get_system(RemoteInputSystem)
        elif action == 'pause':
            pause_state.paused = data['paused']
        elif action == 'input':
            if input_system:
                pos = np.array([data['x'], data['y'], 0.0])
                event_type = data['type']

                if event_type in ['pointerdown', 'pointermove', 'pointerup']:
                    input_system.add_event({'type': event_type, 'pos': pos})
                elif event_type == 'click':
                    input_system.clicks.append(pos)
                elif event_type == 'release':
                    input_system.releases.append(pos)

        await websocket.send(world_to_json(world))

async def main():
    print("Starting WebSocket server on ws://localhost:8765")
    async with websockets.serve(handler, "localhost", 8765):
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    asyncio.run(main())
