import numpy as np
import math

from .ecs import (
    PositionComponent, RadiusComponent, OrientationComponent, CableLinkComponent,
    CoefficientOfFrictionComponent
)
from .cable_joints_components import CablePathComponent, CableJointComponent
from .geometry import (
    tangent_from_point_to_circle, tangent_from_circle_to_point,
    tangent_from_circle_to_circle, signed_arc_length_on_wheel
)
from .vector2 import rotate_inplace, normalize_inplace
from .update_hybrid_link_states import update_hybrid_link_states
from .split_joints import split_joints
from python.update_attachment_points import update_attachment_points
from python.merge_joints import merge_joints

def _clear_debug_points(world):
    debug_points = world.get_resource('debugRenderPoints')
    if debug_points is not None:
        debug_points.clear()

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
                    if link_type == 'rolling' and abs(path.stored[i + 1]) > epsilon:
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

            tension0 = d0 / l0_current
            tension1 = d1 / l1_current

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

def _store_cable_link_poses(world):
    link_entities = world.query([CableLinkComponent, PositionComponent])
    for link_id in link_entities:
        pos_comp = world.get_component(link_id, PositionComponent)
        orientation_comp = world.get_component(link_id, OrientationComponent)
        link_comp = world.get_component(link_id, CableLinkComponent)
        link_comp.prev_cable_attachment_time_pos = pos_comp.pos.copy()
        if orientation_comp:
            link_comp.prev_cable_attachment_time_angle = orientation_comp.angle

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

class CableAttachmentUpdateSystem:
    def __init__(self):
        self.run_in_pause = False

    def update(self, world, dt):
        _clear_debug_points(world)

        update_attachment_points(world)
        _slip_slack(world)
        _even_out_tension_friction(world)
        _slip_slack(world)

        merge_joints(world)
        _slip_slack(world)
        _even_out_tension_friction(world)
        _slip_slack(world)

        split_joints(world)
        _slip_slack(world)
        _even_out_tension_friction(world)
        _slip_slack(world)

        update_hybrid_link_states(world)
        _slip_slack(world)
        _even_out_tension_friction(world)
        _slip_slack(world)

        _store_cable_link_poses(world)
        _sanity_check(world)
