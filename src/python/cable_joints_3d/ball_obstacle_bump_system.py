"""Optional post-PBD obstacle push with full world-frame angular friction."""
import math

import numpy as np

from .ecs import AngularVelocityComponent, CoefficientOfFrictionComponent, MassComponent, MomentOfInertiaComponent, ObstaclePushComponent, OrientationComponent, RadiusComponent, VelocityComponent, layering_enabled
from .inertia_tensor import apply_world_inverse_inertia, has_any_inverse_inertia
from .vector3 import normalize


def _finite_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


class BallObstacleBumpSystem:
    def update(self, world, dt):
        contacts = world.get_resource('ball_obstacle_contacts')
        require_raw_hit = layering_enabled(world) and world.get_resource('layeringObstacleRawHitFilter') is not False
        for contact in contacts or []:
            if require_raw_hit and contact.get('raw_hit') is False:
                continue
            ball, obstacle = contact['ball_id'], contact['obs_id']
            velocity = world.get_component(ball, VelocityComponent)
            radius_a, radius_b = world.get_component(ball, RadiusComponent), world.get_component(obstacle, RadiusComponent)
            mass = world.get_component(ball, MassComponent)
            push = world.get_component(obstacle, ObstaclePushComponent)
            if any(component is None for component in (velocity, radius_a, radius_b, mass, push)):
                continue
            angular_a, angular_b = world.get_component(ball, AngularVelocityComponent), world.get_component(obstacle, AngularVelocityComponent)
            friction = world.get_component(obstacle, CoefficientOfFrictionComponent)
            mu_b = max(0., contact['obstacle_friction']) if _finite_number(contact.get('obstacle_friction')) else friction.mu if friction is not None else 0.
            mu_a = max(0., contact['ball_friction']) if _finite_number(contact.get('ball_friction')) else 0.
            mu = max(0., .5 * (mu_a + mu_b))
            normal = normalize(contact['direction']).copy()
            surface = np.zeros(3)
            if angular_b is not None and np.dot(angular_b.omega, angular_b.omega) > 1e-9:
                surface = np.cross(angular_b.omega, normal * radius_b.radius)
            relative = velocity.vel.copy()
            if angular_a is not None and np.dot(angular_a.omega, angular_a.omega) > 1e-9:
                relative += np.cross(angular_a.omega, normal * -radius_a.radius)
            relative -= surface
            tangent = relative - normal * np.dot(relative, normal)
            direction, tangent_direction = normal.copy(), None
            if mu != 0 and np.dot(tangent, tangent) > 1e-9:
                tangent_direction = normalize(tangent)
                direction = normalize(direction - tangent_direction * mu)
            velocity.vel += direction * push.push_vel
            if mu <= 0 or tangent_direction is None or mass.mass <= 0:
                continue
            delta_velocity = np.dot(direction, tangent_direction) * push.push_vel
            if abs(delta_velocity) <= 1e-9:
                continue
            impulse = tangent_direction * (delta_velocity * mass.mass)
            for entity, angular, lever, signed_impulse in [
                (ball, angular_a, normal * -radius_a.radius, impulse),
                (obstacle, angular_b, normal * radius_b.radius, -impulse),
            ]:
                moment = world.get_component(entity, MomentOfInertiaComponent)
                if angular is not None and has_any_inverse_inertia(moment):
                    orientation = world.get_component(entity, OrientationComponent)
                    angular.omega += apply_world_inverse_inertia(moment, orientation.quaternion if orientation is not None else None,
                                                               np.cross(lever, signed_impulse))
