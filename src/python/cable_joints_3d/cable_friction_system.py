"""Rest-length redistribution by extension and capstan friction bounds."""
import math

import numpy as np

from .cable_joints_components import CableJointComponent, CablePathComponent
from .ecs import CoefficientOfFrictionComponent, RadiusComponent, layering_enabled
from .spools import SpoolStateComponent
from .vector3 import length as norm3, normalize

EPSILON = 1e-9
MAX_ITERATIONS = 64


def _redistribute_pair(high, low, distance_high, distance_low, ratio):
    total_rest = high.rest_length + low.rest_length
    total_distance = distance_high + distance_low
    previous_high_rest = high.rest_length
    if not total_rest > 0 or not total_distance > EPSILON:
        return 0.
    if total_rest >= total_distance:
        high.rest_length = total_rest * distance_high / total_distance
        low.rest_length = total_rest - high.rest_length
        return abs(high.rest_length - previous_high_rest)
    ratio = max(1., ratio)
    high_rest = (distance_high - ratio * distance_low + ratio * total_rest) / (1. + ratio)
    high.rest_length = min(total_rest, max(0., high_rest))
    low.rest_length = total_rest - high.rest_length
    return abs(high.rest_length - previous_high_rest)


def _redistribute_path(world, path):
    max_change = 0.
    for index in range(len(path.joint_entities) - 1):
        first = world.get_component(path.joint_entities[index], CableJointComponent)
        second = world.get_component(path.joint_entities[index + 1], CableJointComponent)
        if first is None or second is None:
            continue
        if not np.isfinite([first.rest_length, second.rest_length]).all() or min(first.rest_length, second.rest_length) < 0:
            continue
        distance_first = norm3(first.attachment_point_a_world - first.attachment_point_b_world)
        distance_second = norm3(second.attachment_point_a_world - second.attachment_point_b_world)
        link_type = path.link_types[index + 1]
        friction_active, threshold = False, 1.
        if link_type in ('rolling', 'pinhole'):
            entity = first.entity_b
            friction = world.get_component(entity, CoefficientOfFrictionComponent)
            mu = friction.mu if friction is not None else 0.
            radius_component = world.get_component(entity, RadiusComponent)
            radius = radius_component.radius if radius_component is not None else 0.
            radius += path.cable_half_width if layering_enabled(world) else 0.
            free_rolling = link_type == 'rolling' and world.get_component(entity, SpoolStateComponent) is not None
            if not free_rolling and mu > EPSILON:
                wrap_angle = 0.
                stored = path.stored[index + 1]
                if link_type == 'rolling' and radius > EPSILON and abs(stored) > EPSILON:
                    wrap_angle = abs(stored / radius)
                elif link_type == 'pinhole':
                    incoming = normalize(first.attachment_point_a_world - first.attachment_point_b_world)
                    outgoing = normalize(second.attachment_point_b_world - second.attachment_point_a_world)
                    wrap_angle = math.acos(float(np.clip(np.dot(incoming, outgoing), -1., 1.)))
                if wrap_angle > EPSILON:
                    friction_active = True
                    threshold = float(np.exp(mu * wrap_angle))
        if not friction_active:
            if link_type != 'attachment':
                max_change = max(max_change, _redistribute_pair(first, second, distance_first, distance_second, 1.))
            continue
        tension_first = max(0., distance_first - first.rest_length)
        tension_second = max(0., distance_second - second.rest_length)
        if abs(tension_first - tension_second) < EPSILON:
            continue
        if tension_first > tension_second:
            high, low, d_high, d_low, t_high, t_low = first, second, distance_first, distance_second, tension_first, tension_second
        else:
            high, low, d_high, d_low, t_high, t_low = second, first, distance_second, distance_first, tension_second, tension_first
        if t_high > t_low * threshold + EPSILON:
            max_change = max(max_change, _redistribute_pair(high, low, d_high, d_low, threshold))
    return max_change


class CableFrictionSystem:
    def update(self, world, _dt):
        # Capstan equilibration is a constraint solve, not a rate of motion.
        paths = [world.get_component(entity, CablePathComponent) for entity in world.query([CablePathComponent])]
        for _ in range(MAX_ITERATIONS):
            if max((_redistribute_path(world, path) for path in paths), default=0.) <= EPSILON:
                break
