"""Optional ordered sphere contacts; preserve the reference's simple core model."""
import math

import numpy as np

from .ecs import BallTagComponent, MassComponent, ObstaclePushComponent, ObstacleTagComponent, PositionComponent, RadiusComponent


class PBDBallBallCollisions:
    def update(self, world, dt):
        balls = world.query([BallTagComponent, PositionComponent, RadiusComponent, MassComponent])
        for index, first in enumerate(balls):
            for second in balls[index + 1:]:
                pos_a = world.get_component(first, PositionComponent).pos
                pos_b = world.get_component(second, PositionComponent).pos
                radius = world.get_component(first, RadiusComponent).radius + world.get_component(second, RadiusComponent).radius
                direction = pos_b - pos_a
                distance_squared = np.dot(direction, direction)
                if distance_squared == 0 or distance_squared > radius * radius:
                    continue
                mass_a = world.get_component(first, MassComponent).mass
                mass_b = world.get_component(second, MassComponent).mass
                inverse_a, inverse_b = 1 / mass_a if mass_a > 0 else 0., 1 / mass_b if mass_b > 0 else 0.
                if inverse_a + inverse_b <= 1e-9:
                    continue
                distance = math.sqrt(distance_squared)
                direction *= 1 / distance
                correction = direction * ((radius - distance) / (inverse_a + inverse_b))
                pos_a -= correction * inverse_a
                pos_b += correction * inverse_b


class PBDBallObstacleCollisions:
    def update(self, world, dt):
        balls = world.query([BallTagComponent, PositionComponent, RadiusComponent])
        obstacles = world.query([ObstacleTagComponent, PositionComponent, RadiusComponent, ObstaclePushComponent])
        contacts = world.get_resource('ball_obstacle_contacts')
        if contacts is None:
            contacts = []
            world.set_resource('ball_obstacle_contacts', contacts)
        contacts.clear()
        for ball in balls:
            position = world.get_component(ball, PositionComponent).pos
            radius = world.get_component(ball, RadiusComponent).radius
            for obstacle in obstacles:
                center = world.get_component(obstacle, PositionComponent).pos
                radius_sum = radius + world.get_component(obstacle, RadiusComponent).radius
                direction = position - center
                distance_squared = np.dot(direction, direction)
                if distance_squared == 0 or distance_squared > radius_sum * radius_sum:
                    continue
                distance = math.sqrt(distance_squared)
                direction *= 1 / distance
                contacts.append({'ball_id': ball, 'obs_id': obstacle, 'direction': direction.copy()})
                position += direction * (radius_sum - distance)
