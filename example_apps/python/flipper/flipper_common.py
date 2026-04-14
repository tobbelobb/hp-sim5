import numpy as np
from dataclasses import dataclass, field
import math
import sys
from pathlib import Path

root_dir = Path(__file__).resolve().parents[3]
src_python_path = root_dir / "src" / "python"
if str(src_python_path) not in sys.path:
    sys.path.insert(0, str(src_python_path))


from cable_joints.geometry import right_of_line

from cable_joints.ecs import (
    PositionComponent,
    VelocityComponent,
    MassComponent,
    RestitutionComponent,
    RadiusComponent,
    CoefficientOfFrictionComponent,
    PrevFinalPosComponent
)

from cable_joints.cable_joints_components import (
    CableLinkComponent,
    CableJointComponent,
    create_cable_path_component
)


@dataclass
class BallTagComponent:
    """A tag component for entities considered to be balls."""
    pass

@dataclass
class ObstacleTagComponent:
    """A tag component for entities considered to be obstacles."""
    pass

@dataclass
class ObstaclePushComponent:
    """Stores the push velocity for an obstacle entity."""
    pushVel: float = 2.0
    pass

@dataclass
class BorderComponent:
    """
    Stores a list of 2D or 3D points that define a border polygon or polyline.
    Each point is cloned on initialization to prevent shared references.
    """
    points: list = field(default_factory=list)

    def __init__(self, points=None):
        if points is None:
            self.points = []
        else:
            # Clone each point to avoid shared references
            self.points = [np.copy(p) for p in points]

@dataclass
class ScoredTagComponent:
    """A tag component for entities considered to have scored."""
    pass

@dataclass
class PauseStateComponent:
    """Stores the state that says whether the simulation is paused or not."""
    paused: bool = True


@dataclass
class FlipperTagComponent:
    """A tag component for entities considered to be flippers."""
    pass

@dataclass
class FlipperStateComponent:
    """
    Represents the state of a flipper mechanism with rotational behavior.
    """
    length: float
    rest_angle: float
    max_rotation: float
    angular_velocity: float

    sign: float = field(init=False)  # Direction of rotation (+1 or -1)
    rotation: float = 0.0            # Current rotation from rest_angle
    current_angular_velocity: float = 0.0  # Angular velocity in last frame
    pressed: bool = False            # Was the flipper activated?

    def __post_init__(self):
        self.sign = math.copysign(1.0, self.max_rotation)
        self.max_rotation = abs(self.max_rotation)



class ScoreComponent:
    def __init__(self, score=0):
        self.value = score


@dataclass
class FlipperTipComponent:
    """Associates a tip entity with its parent flipper entity."""
    flipper_entity_id: int

class FlipperTipLinkSystem:
    run_in_pause = False

    def update(self, world, dt):
        for tip_id in world.query([FlipperTipComponent, PositionComponent]):
            tip_comp = world.get_component(tip_id, FlipperTipComponent)
            flip_id = tip_comp.flipper_entity_id

            pivot_pos = world.get_component(flip_id, PositionComponent).pos
            state = world.get_component(flip_id, FlipperStateComponent)

            angle = state.rest_angle + state.sign * state.rotation
            dir_vec = np.array([math.cos(angle), math.sin(angle), 0.0])  # 3D assumed

            tip_pos = pivot_pos + dir_vec * state.length

            world.get_component(tip_id, PositionComponent).pos[:] = tip_pos


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

        border_id = border_entities[0]
        border_comp = world.get_component(border_id, BorderComponent)
        border_points = border_comp.points

        # Get physics properties from the border entity
        border_restitution_comp = world.get_component(border_id, RestitutionComponent)
        border_friction_comp = world.get_component(border_id, CoefficientOfFrictionComponent)
        restitution = border_restitution_comp.restitution if border_restitution_comp else None
        friction = border_friction_comp.mu if border_friction_comp else None

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
                'delta_lambda': delta_lambda,
                'restitution': restitution,
                'friction': friction
            })


class PBDBallBallCollisions:
    run_in_pause = False
    def update(self, world, dt):
        ball_entities = world.query([BallTagComponent, PositionComponent, VelocityComponent, RadiusComponent, MassComponent, RestitutionComponent])
        for i in range(len(ball_entities)):
            for j in range(i + 1, len(ball_entities)):
                e1 = ball_entities[i]
                e2 = ball_entities[j]

                p1_comp = world.get_component(e1, PositionComponent)
                p1 = p1_comp.pos
                r1 = world.get_component(e1, RadiusComponent).radius
                m1_comp = world.get_component(e1, MassComponent)
                m1 = m1_comp.mass if m1_comp else 0.0

                p2_comp = world.get_component(e2, PositionComponent)
                p2 = p2_comp.pos
                r2 = world.get_component(e2, RadiusComponent).radius
                m2_comp = world.get_component(e2, MassComponent)
                m2 = m2_comp.mass if m2_comp else 0.0

                direction = p2 - p1
                d_sq = np.dot(direction, direction)
                r_sum = r1 + r2

                if d_sq == 0.0 or d_sq > r_sum * r_sum:
                    continue

                d = math.sqrt(d_sq)
                direction /= d # Normalize

                # Resolve penetration
                penetration = r_sum - d
                inv_mass1 = 1.0 / m1 if m1 > 0 else 0.0
                inv_mass2 = 1.0 / m2 if m2 > 0 else 0.0
                total_inv_mass = inv_mass1 + inv_mass2

                if total_inv_mass <= 1e-9: continue

                corr = direction * (penetration / total_inv_mass)
                p1_comp.pos -= corr * inv_mass1
                p2_comp.pos += corr * inv_mass2


class PBDBallObstacleCollisions:
    run_in_pause = False
    def update(self, world, dt):
        ball_entities = world.query([BallTagComponent, PositionComponent, RadiusComponent])
        obstacle_entities = world.query([ObstacleTagComponent, PositionComponent, RadiusComponent, ObstaclePushComponent])

        contacts = world.get_resource('ball_obstacle_contacts')
        if contacts is None:
            contacts = []
            world.set_resource('ball_obstacle_contacts', contacts)
        contacts.clear()

        for ball_id in ball_entities:
            p1 = world.get_component(ball_id, PositionComponent).pos
            r1 = world.get_component(ball_id, RadiusComponent).radius

            for obs_id in obstacle_entities:
                p2 = world.get_component(obs_id, PositionComponent).pos
                r2 = world.get_component(obs_id, RadiusComponent).radius

                direction = p1 - p2
                d_sq = np.dot(direction, direction)
                r_sum = r1 + r2

                if d_sq == 0.0 or d_sq > r_sum * r_sum:
                    continue

                d = math.sqrt(d_sq)
                direction /= d # Normalize

                # Store contact info for the velocity-based bump system
                contacts.append({'ball_id': ball_id, 'obs_id': obs_id, 'direction': direction.copy()})

                # Resolve penetration
                corr = r_sum - d
                p1 += direction * corr

                # Velocity resolution is now handled by BallObstacleBumpSystem

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
        if self.grab_spring or pos is None:
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
        if self.grab_spring and pos is not None:
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
            if pos is not None:
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
