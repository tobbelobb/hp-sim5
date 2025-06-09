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
from .updateHybridLinkStates import update_hybrid_link_states
from .splitJoints import split_joints

def _effective_cw(path, link_index, travelling_from_circle):
    if link_index == 0 and travelling_from_circle:
        return not path.cw[link_index]
    return path.cw[link_index]

def _clear_debug_points(world):
    debug_points = world.get_resource('debugRenderPoints')
    if debug_points is not None:
        debug_points.clear()

def _is_attachment(value):
    return value in ('attachment', 'hybrid-attachment', 'pinhole')

def _is_rolling(value):
    return value in ('rolling', 'hybrid')

def _is_hybrid(value):
    return value in ('hybrid', 'hybrid-attachment')

def _update_attachment_points(world):
    path_entities = world.query([CablePathComponent])

    for path_id in path_entities:
        path = world.get_component(path_id, CablePathComponent)

        for i in range(len(path.joint_entities)):
            joint_id = path.joint_entities[i]
            joint = world.get_component(joint_id, CableJointComponent)

            A = i
            B = i + 1

            entity_a = joint.entity_a
            entity_b = joint.entity_b

            # Get components for Entity A
            pos_a_comp = world.get_component(entity_a, PositionComponent)
            radius_a_comp = world.get_component(entity_a, RadiusComponent)
            link_a_comp = world.get_component(entity_a, CableLinkComponent)
            orientation_a_comp = world.get_component(entity_a, OrientationComponent)
            pos_a = pos_a_comp.pos if pos_a_comp else None
            attachment_a_previous = joint.attachment_point_a_world
            prev_pos_a = link_a_comp.prev_cable_attachment_time_pos if link_a_comp else None
            radius_a = radius_a_comp.radius if radius_a_comp else None
            angle_a = orientation_a_comp.angle if orientation_a_comp else 0.0
            prev_angle_a = link_a_comp.prev_cable_attachment_time_angle if link_a_comp else 0.0
            delta_angle_a = angle_a - prev_angle_a
            cw_a = _effective_cw(path, A, True)
            attachment_link_a = _is_attachment(path.link_types[A])
            rolling_link_a = _is_rolling(path.link_types[A])
            is_hybrid_a = _is_hybrid(path.link_types[A])
            has_friction_a = world.get_component(entity_a, CoefficientOfFrictionComponent) is not None

            # Get components for Entity B
            pos_b_comp = world.get_component(entity_b, PositionComponent)
            radius_b_comp = world.get_component(entity_b, RadiusComponent)
            link_b_comp = world.get_component(entity_b, CableLinkComponent)
            orientation_b_comp = world.get_component(entity_b, OrientationComponent)
            pos_b = pos_b_comp.pos if pos_b_comp else None
            attachment_b_previous = joint.attachment_point_b_world
            prev_pos_b = link_b_comp.prev_cable_attachment_time_pos if link_b_comp else None
            radius_b = radius_b_comp.radius if radius_b_comp else None
            angle_b = orientation_b_comp.angle if orientation_b_comp else 0.0
            prev_angle_b = link_b_comp.prev_cable_attachment_time_angle if link_b_comp else 0.0
            delta_angle_b = angle_b - prev_angle_b
            cw_b = _effective_cw(path, B, False)
            attachment_link_b = _is_attachment(path.link_types[B])
            rolling_link_b = _is_rolling(path.link_types[B])
            is_hybrid_b = _is_hybrid(path.link_types[B])
            has_friction_b = world.get_component(entity_b, CoefficientOfFrictionComponent) is not None

            # Calculate Attachment Points
            attachment_a_current = pos_a.copy()
            attachment_b_current = pos_b.copy()

            if is_hybrid_a:
                p_a_diff_from_translation = pos_a - prev_pos_a
                temp_rot = attachment_a_previous.copy()
                rotate_inplace(temp_rot, delta_angle_a, prev_pos_a, True)
                p_a_diff_from_rotation = temp_rot - attachment_a_previous
                attachment_a_current = attachment_a_previous + p_a_diff_from_translation + p_a_diff_from_rotation

            if is_hybrid_b:
                p_b_diff_from_translation = pos_b - prev_pos_b
                temp_rot = attachment_b_previous.copy()
                rotate_inplace(temp_rot, delta_angle_b, prev_pos_b, True)
                p_b_diff_from_rotation = temp_rot - attachment_b_previous
                attachment_b_current = attachment_b_previous + p_b_diff_from_translation + p_b_diff_from_rotation

            if attachment_link_a and rolling_link_b:
                tangents = tangent_from_point_to_circle(attachment_a_current, pos_b, radius_b, cw_b)
                attachment_b_current = tangents['a_circle']
            elif rolling_link_a and attachment_link_b:
                tangents = tangent_from_circle_to_point(attachment_b_current, pos_a, radius_a, cw_a)
                attachment_a_current = tangents['a_circle']
            elif rolling_link_a and rolling_link_b:
                tangents = tangent_from_circle_to_circle(pos_a, radius_a, cw_a, pos_b, radius_b, cw_b)
                attachment_a_current = tangents['a_circle']
                attachment_b_current = tangents['b_circle']

            # Calculate Wrapping/Unwrapping Arc Lengths (sA, sB)
            s_a = 0.0
            if rolling_link_a:
                s_a = signed_arc_length_on_wheel(attachment_a_previous - prev_pos_a, attachment_a_current - pos_a, np.zeros(3), radius_a, cw_a)
                if is_hybrid_a or has_friction_a:
                    s_a += (delta_angle_a * radius_a if cw_a else -delta_angle_a * radius_a)

            s_b = 0.0
            if rolling_link_b:
                s_b = signed_arc_length_on_wheel(attachment_b_previous - prev_pos_b, attachment_b_current - pos_b, np.zeros(3), radius_b, cw_b)
                if is_hybrid_b or has_friction_b:
                    s_b += (delta_angle_b * radius_b if cw_b else -delta_angle_b * radius_b)

            path.stored[A] += s_a
            joint.rest_length -= s_a
            path.stored[B] -= s_b
            joint.rest_length += s_b

            joint.attachment_point_a_world = attachment_a_current
            joint.attachment_point_b_world = attachment_b_current

def _merge_joints(world):
    path_entities = world.query([CablePathComponent])
    for path_id in path_entities:
        path = world.get_component(path_id, CablePathComponent)
        if len(path.joint_entities) < 2:
            continue

        re_run_merge = True
        while re_run_merge:
            re_run_merge = False
            i = 0
            while i < len(path.joint_entities) - 1:
                if path.link_types[i + 1] != 'rolling':
                    i += 1
                    continue

                if path.stored[i + 1] < 0.0:
                    joint_i_id = path.joint_entities[i]
                    joint_i_plus_1_id = path.joint_entities[i + 1]
                    joint_i = world.get_component(joint_i_id, CableJointComponent)
                    joint_i_plus_1 = world.get_component(joint_i_plus_1_id, CableJointComponent)

                    if joint_i.entity_a == joint_i_plus_1.entity_b:
                        i += 1
                        continue

                    # Merge
                    p_a1 = joint_i.attachment_point_a_world
                    p_b2 = joint_i_plus_1.attachment_point_b_world
                    pos_a = world.get_component(joint_i.entity_a, PositionComponent).pos
                    radius_a_comp = world.get_component(joint_i.entity_a, RadiusComponent)
                    radius_a = radius_a_comp.radius if radius_a_comp else 0.0
                    cw_a = _effective_cw(path, i, True)
                    pos_b = world.get_component(joint_i_plus_1.entity_b, PositionComponent).pos
                    radius_b_comp = world.get_component(joint_i_plus_1.entity_b, RadiusComponent)
                    radius_b = radius_b_comp.radius if radius_b_comp else 0.0
                    cw_b = path.cw[i + 2]

                    joint_i.rest_length += joint_i_plus_1.rest_length + path.stored[i + 1]
                    joint_i.entity_b = joint_i_plus_1.entity_b

                    is_attachment_a = _is_attachment(path.link_types[i])
                    is_rolling_a = _is_rolling(path.link_types[i])
                    is_attachment_b = _is_attachment(path.link_types[i + 2])
                    is_rolling_b = _is_rolling(path.link_types[i + 2])

                    attachment_a_current = p_a1.copy()
                    attachment_b_current = p_b2.copy()

                    if is_rolling_a and is_rolling_b:
                        tangents = tangent_from_circle_to_circle(pos_a, radius_a, cw_a, pos_b, radius_b, cw_b)
                        attachment_a_current = tangents['a_circle']
                        attachment_b_current = tangents['b_circle']
                    elif is_rolling_a and is_attachment_b:
                        attachment_a_current = tangent_from_circle_to_point(p_b2, pos_a, radius_a, cw_a)['a_circle']
                    elif is_attachment_a and is_rolling_b:
                        attachment_b_current = tangent_from_point_to_circle(p_a1, pos_b, radius_b, cw_b)['a_circle']

                    s_a = 0.0
                    if is_rolling_a:
                        s_a = signed_arc_length_on_wheel(p_a1, attachment_a_current, pos_a, radius_a, cw_a)
                    
                    s_b = 0.0
                    if is_rolling_b:
                        s_b = signed_arc_length_on_wheel(p_b2, attachment_b_current, pos_b, radius_b, cw_b)

                    path.stored[i] += s_a
                    joint_i.rest_length -= s_a
                    path.stored[i + 2] -= s_b
                    joint_i.rest_length += s_b
                    
                    re_run_merge = path.stored[i] < 0.0 or path.stored[i + 2] < 0.0

                    joint_i.attachment_point_a_world = attachment_a_current
                    joint_i.attachment_point_b_world = attachment_b_current

                    del path.joint_entities[i + 1]
                    del path.stored[i + 1]
                    del path.cw[i + 1]
                    del path.link_types[i + 1]
                    world.destroy_entity(joint_i_plus_1_id)
                else:
                    i += 1

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
        _update_attachment_points(world)
        _slip_slack(world)
        _even_out_tension_friction(world)
        _slip_slack(world)

        _merge_joints(world)
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
