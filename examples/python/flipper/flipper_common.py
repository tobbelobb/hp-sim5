import numpy as np
from dataclasses import dataclass, field
import math

from cable_joints.ecs import (
    PositionComponent,
    VelocityComponent,
    MassComponent,
    RestitutionComponent,
    RadiusComponent,
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
