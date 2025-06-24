import numpy as np
import math

from .ecs import (
    RadiusComponent, CoefficientOfFrictionComponent
)
from .cable_joints_components import CablePathComponent, CableJointComponent
from .vector2 import normalize_inplace

# Similar to CableFrictionSystem we want a fixed amount of work per second.
# These constants reference `ai_docs/CableJoints/CableJoints.md` for the
# friction model and `ai_docs/Smallsteps/Smallsteps.md` for maintaining a
# constant compute budget as the timestep changes.

BASE_ITERATIONS = 4
TARGET_DT = 1.0 / 60.0

def _slip_slack(world):
    path_entities = world.query([CablePathComponent])
    for path_id in path_entities:
        path = world.get_component(path_id, CablePathComponent)
        if len(path.joint_entities) < 2:
            continue
        for i in range(len(path.joint_entities) - 1):
            if path.link_types[i + 1] == 'attachment':
                continue

            j0 = world.get_component(path.joint_entities[i], CableJointComponent)
            j1 = world.get_component(path.joint_entities[i + 1], CableJointComponent)

            d0 = np.linalg.norm(j0.attachment_point_a_world - j0.attachment_point_b_world)
            d1 = np.linalg.norm(j1.attachment_point_a_world - j1.attachment_point_b_world)

            l0 = j0.rest_length
            l1 = j1.rest_length

            slack0 = l0 - d0
            slack1 = l1 - d1

            if slack0 > 0 and slack1 < 0:
                slip = min(slack0, -slack1)
                j0.rest_length -= slip
                j1.rest_length += slip
            elif slack1 > 0 and slack0 < 0:
                slip = min(slack1, -slack0)
                j1.rest_length -= slip
                j0.rest_length += slip

def _even_out_tension_friction(world):
    path_entities = world.query([CablePathComponent])
    epsilon = 1e-9

    for path_id in path_entities:
        path = world.get_component(path_id, CablePathComponent)
        if not path or len(path.joint_entities) < 2:
            continue

        for i in range(len(path.joint_entities) - 1):
            j0_comp = world.get_component(path.joint_entities[i], CableJointComponent)
            j1_comp = world.get_component(path.joint_entities[i + 1], CableJointComponent)

            if not j0_comp or not j1_comp:
                continue

            d0 = np.linalg.norm(j0_comp.attachment_point_a_world - j0_comp.attachment_point_b_world)
            d1 = np.linalg.norm(j1_comp.attachment_point_a_world - j1_comp.attachment_point_b_world)
            l0_current = j0_comp.rest_length
            l1_current = j1_comp.rest_length

            if l0_current <= epsilon or l1_current <= epsilon:
                continue

            link_type = path.link_types[i + 1]
            friction_active = False
            friction_threshold = 1.0

            if link_type in ('rolling', 'pinhole'):
                link_entity_id = j0_comp.entity_b
                friction_comp = world.get_component(link_entity_id, CoefficientOfFrictionComponent)
                mu = friction_comp.mu if friction_comp else 0.0
                radius_comp = world.get_component(link_entity_id, RadiusComponent)
                radius = radius_comp.radius if radius_comp else 0.0

                if mu > epsilon:
                    wrap_angle = 0.0
                    if link_type == 'rolling' and radius > epsilon and abs(path.stored[i + 1]) > epsilon:
                        wrap_angle = abs(path.stored[i + 1] / radius)
                    elif link_type == 'pinhole':
                        v0 = j0_comp.attachment_point_a_world - j0_comp.attachment_point_b_world
                        v1 = j1_comp.attachment_point_a_world - j1_comp.attachment_point_b_world
                        normalize_inplace(v0)
                        normalize_inplace(v1)
                        dot = np.dot(v0[:2], v1[:2])
                        wrap_angle = np.arccos(np.clip(dot, -1.0, 1.0))

                    if wrap_angle > epsilon:
                        friction_active = True
                        friction_threshold = math.exp(mu * wrap_angle)

            if not friction_active:
                if link_type != 'attachment':
                    available_rest_length = l0_current + l1_current
                    total_dist = d0 + d1
                    if total_dist > epsilon:
                        j0_comp.rest_length = available_rest_length * d0 / total_dist
                        j1_comp.rest_length = available_rest_length * d1 / total_dist
                continue

            tension0 = d0 / l0_current if l0_current > epsilon else float('inf')
            tension1 = d1 / l1_current if l1_current > epsilon else float('inf')

            if abs(tension0 - tension1) < epsilon:
                continue

            if tension0 > tension1:
                T_high, L_high_current, D_high, is_j0_high = tension0, l0_current, d0, True
                T_low, L_low_current, D_low = tension1, l1_current, d1
            else:
                T_high, L_high_current, D_high, is_j0_high = tension1, l1_current, d1, False
                T_low, L_low_current, D_low = tension0, l0_current, d0

            if T_high > T_low * friction_threshold + epsilon:
                L_total = L_high_current + L_low_current
                denominator = D_high + D_low * friction_threshold
                if denominator > epsilon:
                    L_high_new = (D_high * L_total) / denominator
                    L_low_new = L_total - L_high_new
                    if L_high_new < 0: L_high_new = 0
                    if L_low_new < 0: L_low_new = 0
                    current_new_total = L_high_new + L_low_new
                    if current_new_total > epsilon and abs(current_new_total - L_total) > epsilon:
                        L_high_new = (L_high_new / current_new_total) * L_total
                        L_low_new = (L_low_new / current_new_total) * L_total

                    if is_j0_high:
                        j0_comp.rest_length, j1_comp.rest_length = L_high_new, L_low_new
                    else:
                        j1_comp.rest_length, j0_comp.rest_length = L_high_new, L_low_new

def _sanity_check(world):
    path_entities = world.query([CablePathComponent])
    for path_id in path_entities:
        path = world.get_component(path_id, CablePathComponent)
        if not path.joint_entities:
            continue

        total_current_rest_length = sum(path.stored)
        for joint_id in path.joint_entities:
            joint = world.get_component(joint_id, CableJointComponent)
            total_current_rest_length += joint.rest_length

        error = path.total_rest_length - total_current_rest_length
        if abs(error) > 1e-9:
            print(f"Warning: Non-zero error for path {path_id}: {error}")
        if any(s < -1e-9 for s in path.stored):
            print(f"Warning: Negative stored lengths for path {path_id}: {path.stored}")

class CableSlipSlackFrictionSystem:
    """
    Handles the transfer of cable rest length between joints to simulate
    the cable slipping over rollers or through pinholes. This includes
    slipping slack from loose segments to taut ones, and evening out
    tension based on friction. This system should run after the main
    positional and velocity solves for a substep.
    """
    def __init__(self):
        self.run_in_pause = False

    def update(self, world, dt):
        # Scale iterations with dt so the overall cost per second stays
        # approximately constant (see Smallsteps.md).
        iterations = max(1, int(BASE_ITERATIONS * dt / TARGET_DT))
        for _ in range(iterations):
            _slip_slack(world)
            _even_out_tension_friction(world)
        
        _sanity_check(world)
