import math
import numpy as np

from .ecs import (
    PositionComponent,
    RadiusComponent,
    OrientationComponent,
    CoefficientOfFrictionComponent,
    MassComponent,
    VelocityComponent,
    MomentOfInertiaComponent,
    AngularVelocityComponent
)
from .cable_joints_components import (
    CableLinkComponent,
    CableJointComponent,
    CablePathComponent
)
from .geometry import (
    tangent_from_point_to_circle,
    tangent_from_circle_to_point,
    tangent_from_circle_to_circle,
    signed_arc_length_on_wheel,
)
from .vector2 import rotate_inplace
from .util import (
    is_attachment,
    is_rolling,
    is_hybrid,
    effective_cw
)

MIN_JOINT_REST_LENGTH = 1e-6

def calculate_attachment_points(world, joint, path, i):
    """
    Calculates the world-space attachment points for a cable joint based on the
    current state of the connected entities. This is a pure function that does
    not modify the world state.
    """
    A = i       # Index for link/cw/stored related to entity A side
    B = i + 1   # Index for link/cw/stored related to entity B side

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

    cw_a = effective_cw(path, A, True)
    attachment_link_a = is_attachment(path.link_types[A])
    rolling_link_a = is_rolling(path.link_types[A])
    is_hybrid_a = is_hybrid(path.link_types[A])

    # Pre-calculate translation and rotation differences (like JS version)
    p_a_diff_from_translation = pos_a - prev_pos_a if (pos_a is not None and prev_pos_a is not None) else None
    temp_rotated_point_a = attachment_a_previous.copy()
    if prev_pos_a is not None:
        rotate_inplace(temp_rotated_point_a, delta_angle_a, prev_pos_a, True)
    p_a_diff_from_rotation = temp_rotated_point_a - attachment_a_previous

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

    cw_b = effective_cw(path, B, False)
    attachment_link_b = is_attachment(path.link_types[B])
    rolling_link_b = is_rolling(path.link_types[B])
    is_hybrid_b = is_hybrid(path.link_types[B])

    # Pre-calculate translation and rotation differences (like JS version)
    p_b_diff_from_translation = pos_b - prev_pos_b if (pos_b is not None and prev_pos_b is not None) else None
    temp_rotated_point_b = attachment_b_previous.copy()
    if prev_pos_b is not None:
        rotate_inplace(temp_rotated_point_b, delta_angle_b, prev_pos_b, True)
    p_b_diff_from_rotation = temp_rotated_point_b - attachment_b_previous

    # --- Calculate Attachment Points based on this frame's positions and rotations ---
    attachment_a_current = pos_a.copy() if pos_a is not None else None
    attachment_b_current = pos_b.copy() if pos_b is not None else None

    if attachment_link_a and rolling_link_b:
        if is_hybrid_a and p_a_diff_from_translation is not None:
            attachment_a_current = attachment_a_previous + p_a_diff_from_translation + p_a_diff_from_rotation
        if attachment_a_current is not None and pos_b is not None and radius_b is not None:
            tangents = tangent_from_point_to_circle(attachment_a_current, pos_b, radius_b, cw_b)
            attachment_b_current = tangents['a_circle']
    elif rolling_link_a and attachment_link_b:
        if is_hybrid_b and p_b_diff_from_translation is not None:
            attachment_b_current = attachment_b_previous + p_b_diff_from_translation + p_b_diff_from_rotation
        if attachment_b_current is not None and pos_a is not None and radius_a is not None:
            tangents = tangent_from_circle_to_point(attachment_b_current, pos_a, radius_a, cw_a)
            attachment_a_current = tangents['a_circle']
    elif rolling_link_a and rolling_link_b:
        if pos_a is not None and pos_b is not None and radius_a is not None and radius_b is not None:
            tangents = tangent_from_circle_to_circle(pos_a, radius_a, cw_a, pos_b, radius_b, cw_b)
            attachment_a_current = tangents['a_circle']
            attachment_b_current = tangents['b_circle']
    else:  # attachment_link_a and attachment_link_b
        if is_hybrid_a and p_a_diff_from_translation is not None:
            attachment_a_current = attachment_a_previous + p_a_diff_from_translation + p_a_diff_from_rotation
        if is_hybrid_b and p_b_diff_from_translation is not None:
            attachment_b_current = attachment_b_previous + p_b_diff_from_translation + p_b_diff_from_rotation

    return attachment_a_current, attachment_b_current


def update_attachment_points(world):
    """
    Updates cable attachment points based on entity movements and rotations.

    This function iterates through all cable paths and recalculates the world-space
    attachment points for each joint. It handles different link types (attachment,
    rolling, hybrid) and updates the stored length of cable on rolling links
    and the rest length of flexible segments to conserve total cable length.
    """
    path_entities = world.query([CablePathComponent])
    transfer_tuning = world.get_resource('cableAttachmentTransferTuning') or {}
    enable_kinematic_transfer_clamp = transfer_tuning.get('enableKinematicClamp', False) is True
    transfer_budget_scale_raw = transfer_tuning.get('kinematicBudgetScale', 1.0)
    transfer_budget_scale = max(0.0, float(transfer_budget_scale_raw)) if math.isfinite(transfer_budget_scale_raw) else 1.0
    enable_potential_rise_clamp = transfer_tuning.get('limitPotentialRiseByKineticBudget', True) is not False
    potential_rise_budget_scale_raw = transfer_tuning.get('potentialRiseBudgetScale', 1.0)
    potential_rise_budget_scale = (
        max(0.0, float(potential_rise_budget_scale_raw))
        if math.isfinite(potential_rise_budget_scale_raw)
        else 1.0
    )
    potential_rise_slack_raw = transfer_tuning.get('potentialRiseSlack', 0.0)
    potential_rise_slack = (
        max(0.0, float(potential_rise_slack_raw))
        if math.isfinite(potential_rise_slack_raw)
        else 0.0
    )
    transfer_clamp_slack_raw = transfer_tuning.get('clampSlack', None)
    transfer_clamp_slack_configured = (
        max(0.0, float(transfer_clamp_slack_raw))
        if transfer_clamp_slack_raw is not None and math.isfinite(transfer_clamp_slack_raw)
        else None
    )
    attachment_diag = {
        'maxAbsSA': 0.0,
        'maxAbsSB': 0.0,
        'maxAbsJointRestDelta': 0.0,
        'maxAbsStoredDelta': 0.0,
        'largeTransferCount': 0,
        'clampedTransferCount': 0,
        'maxClampReduction': 0.0,
        'restLengthClampCount': 0,
        'maxRestLengthClamp': 0.0,
        'potentialRiseClampCount': 0,
        'maxPotentialRiseClamp': 0.0,
        'maxPotentialRise': 0.0,
        'maxPotentialRiseAllowed': 0.0,
    }

    for path_id in path_entities:
        path = world.get_component(path_id, CablePathComponent)

        for i in range(len(path.joint_entities)):
            joint_id = path.joint_entities[i]
            joint = world.get_component(joint_id, CableJointComponent)

            attachment_a_previous = joint.attachment_point_a_world.copy()
            attachment_b_previous = joint.attachment_point_b_world.copy()

            attachment_a_current, attachment_b_current = calculate_attachment_points(world, joint, path, i)

            # --- Calculate Wrapping/Unwrapping Arc Lengths (sA, sB) ---
            # This part requires re-fetching some data, but avoids duplicating the core logic.
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
            prev_pos_a = link_a_comp.prev_cable_attachment_time_pos if link_a_comp else None
            radius_a = radius_a_comp.radius if radius_a_comp else None
            angle_a = orientation_a_comp.angle if orientation_a_comp else 0.0
            prev_angle_a = link_a_comp.prev_cable_attachment_time_angle if link_a_comp else 0.0
            delta_angle_a = angle_a - prev_angle_a
            cw_a = effective_cw(path, A, True)
            rolling_link_a = is_rolling(path.link_types[A])
            is_hybrid_a = is_hybrid(path.link_types[A])
            has_friction_a = world.get_component(entity_a, CoefficientOfFrictionComponent) is not None

            # Get components for Entity B
            pos_b_comp = world.get_component(entity_b, PositionComponent)
            radius_b_comp = world.get_component(entity_b, RadiusComponent)
            link_b_comp = world.get_component(entity_b, CableLinkComponent)
            orientation_b_comp = world.get_component(entity_b, OrientationComponent)
            pos_b = pos_b_comp.pos if pos_b_comp else None
            prev_pos_b = link_b_comp.prev_cable_attachment_time_pos if link_b_comp else None
            radius_b = radius_b_comp.radius if radius_b_comp else None
            angle_b = orientation_b_comp.angle if orientation_b_comp else 0.0
            prev_angle_b = link_b_comp.prev_cable_attachment_time_angle if link_b_comp else 0.0
            delta_angle_b = angle_b - prev_angle_b
            cw_b = effective_cw(path, B, False)
            rolling_link_b = is_rolling(path.link_types[B])
            is_hybrid_b = is_hybrid(path.link_types[B])
            has_friction_b = world.get_component(entity_b, CoefficientOfFrictionComponent) is not None

            s_a = 0.0
            s_b = 0.0

            if rolling_link_a and attachment_a_previous is not None and attachment_a_current is not None and prev_pos_a is not None and pos_a is not None and radius_a is not None:
                s_a = signed_arc_length_on_wheel(
                    attachment_a_previous - prev_pos_a,
                    attachment_a_current - pos_a,
                    np.zeros(3),
                    radius_a,
                    cw_a
                )
                if is_hybrid_a or has_friction_a:
                    s_a += (delta_angle_a * radius_a if cw_a else -delta_angle_a * radius_a)

            if rolling_link_b and attachment_b_previous is not None and attachment_b_current is not None and prev_pos_b is not None and pos_b is not None and radius_b is not None:
                s_b = signed_arc_length_on_wheel(
                    attachment_b_previous - prev_pos_b,
                    attachment_b_current - pos_b,
                    np.zeros(3),
                    radius_b,
                    cw_b
                )
                if is_hybrid_b or has_friction_b:
                    s_b += (delta_angle_b * radius_b if cw_b else -delta_angle_b * radius_b)

            if enable_kinematic_transfer_clamp:
                transfer_clamp_slack = (
                    transfer_clamp_slack_configured
                    if transfer_clamp_slack_configured is not None
                    else max(0.0, 0.25 * (path.cable_half_width if path.cable_half_width is not None else 0.0))
                )
                if rolling_link_a:
                    translation_budget_a = np.linalg.norm(pos_a - prev_pos_a) if (pos_a is not None and prev_pos_a is not None) else 0.0
                    angular_budget_a = abs(delta_angle_a) * max(0.0, radius_a) if (radius_a is not None and math.isfinite(delta_angle_a)) else 0.0
                    max_transfer_a = transfer_budget_scale * (translation_budget_a + angular_budget_a) + transfer_clamp_slack
                    if math.isfinite(max_transfer_a) and max_transfer_a >= 0.0:
                        clamped_s_a = max(-max_transfer_a, min(max_transfer_a, s_a))
                        reduction = abs(s_a - clamped_s_a)
                        if reduction > 1e-9:
                            attachment_diag['clampedTransferCount'] += 1
                            attachment_diag['maxClampReduction'] = max(attachment_diag['maxClampReduction'], reduction)
                            s_a = clamped_s_a
                if rolling_link_b:
                    translation_budget_b = np.linalg.norm(pos_b - prev_pos_b) if (pos_b is not None and prev_pos_b is not None) else 0.0
                    angular_budget_b = abs(delta_angle_b) * max(0.0, radius_b) if (radius_b is not None and math.isfinite(delta_angle_b)) else 0.0
                    max_transfer_b = transfer_budget_scale * (translation_budget_b + angular_budget_b) + transfer_clamp_slack
                    if math.isfinite(max_transfer_b) and max_transfer_b >= 0.0:
                        clamped_s_b = max(-max_transfer_b, min(max_transfer_b, s_b))
                        reduction = abs(s_b - clamped_s_b)
                        if reduction > 1e-9:
                            attachment_diag['clampedTransferCount'] += 1
                            attachment_diag['maxClampReduction'] = max(attachment_diag['maxClampReduction'], reduction)
                            s_b = clamped_s_b

            def increase_rest_by(delta_rest_increase):
                nonlocal s_a, s_b
                if not (delta_rest_increase > 0.0):
                    return
                if rolling_link_a and rolling_link_b:
                    half_deficit = 0.5 * delta_rest_increase
                    s_a -= half_deficit
                    s_b += half_deficit
                elif rolling_link_a:
                    s_a -= delta_rest_increase
                elif rolling_link_b:
                    s_b += delta_rest_increase
                else:
                    half_deficit = 0.5 * delta_rest_increase
                    s_a -= half_deficit
                    s_b += half_deficit

            rest_before = joint.rest_length
            rest_after_candidate = rest_before - s_a + s_b
            if rest_after_candidate < MIN_JOINT_REST_LENGTH:
                deficit = MIN_JOINT_REST_LENGTH - rest_after_candidate
                increase_rest_by(deficit)
                rest_after_candidate = MIN_JOINT_REST_LENGTH
                attachment_diag['restLengthClampCount'] += 1
                attachment_diag['maxRestLengthClamp'] = max(attachment_diag['maxRestLengthClamp'], deficit)

            if (
                enable_potential_rise_clamp and
                attachment_a_current is not None and
                attachment_b_current is not None and
                path.spring_constant is not None and
                math.isfinite(path.spring_constant) and
                path.spring_constant > 1e-9
            ):
                segment_length = float(np.linalg.norm(attachment_a_current - attachment_b_current))
                stretch_before = max(0.0, segment_length - rest_before)
                stretch_after = max(0.0, segment_length - rest_after_candidate)
                potential_before = 0.5 * path.spring_constant * stretch_before * stretch_before
                potential_after = 0.5 * path.spring_constant * stretch_after * stretch_after
                potential_rise = potential_after - potential_before
                if potential_rise > 1e-9:
                    available_kinetic = 0.0

                    mass_a_comp = world.get_component(entity_a, MassComponent)
                    vel_a_comp = world.get_component(entity_a, VelocityComponent)
                    if mass_a_comp is not None and vel_a_comp is not None and mass_a_comp.mass > 1e-9:
                        available_kinetic += 0.5 * mass_a_comp.mass * float(np.dot(vel_a_comp.vel, vel_a_comp.vel))
                    moi_a_comp = world.get_component(entity_a, MomentOfInertiaComponent)
                    ang_a_comp = world.get_component(entity_a, AngularVelocityComponent)
                    if moi_a_comp is not None and ang_a_comp is not None and moi_a_comp.inertia > 1e-9:
                        available_kinetic += 0.5 * moi_a_comp.inertia * (ang_a_comp.angular_velocity ** 2)

                    mass_b_comp = world.get_component(entity_b, MassComponent)
                    vel_b_comp = world.get_component(entity_b, VelocityComponent)
                    if mass_b_comp is not None and vel_b_comp is not None and mass_b_comp.mass > 1e-9:
                        available_kinetic += 0.5 * mass_b_comp.mass * float(np.dot(vel_b_comp.vel, vel_b_comp.vel))
                    moi_b_comp = world.get_component(entity_b, MomentOfInertiaComponent)
                    ang_b_comp = world.get_component(entity_b, AngularVelocityComponent)
                    if moi_b_comp is not None and ang_b_comp is not None and moi_b_comp.inertia > 1e-9:
                        available_kinetic += 0.5 * moi_b_comp.inertia * (ang_b_comp.angular_velocity ** 2)

                    allowed_rise = potential_rise_budget_scale * available_kinetic + potential_rise_slack
                    attachment_diag['maxPotentialRise'] = max(attachment_diag['maxPotentialRise'], potential_rise)
                    attachment_diag['maxPotentialRiseAllowed'] = max(attachment_diag['maxPotentialRiseAllowed'], allowed_rise)
                    if potential_rise > allowed_rise + 1e-9:
                        target_potential_after = potential_before + allowed_rise
                        target_stretch = math.sqrt(max(0.0, (2.0 * target_potential_after) / path.spring_constant))
                        target_rest_after = max(MIN_JOINT_REST_LENGTH, segment_length - target_stretch)
                        rest_increase_needed = max(0.0, target_rest_after - rest_after_candidate)
                        if rest_increase_needed > 1e-9:
                            increase_rest_by(rest_increase_needed)
                            rest_after_candidate = target_rest_after
                            attachment_diag['potentialRiseClampCount'] += 1
                            attachment_diag['maxPotentialRiseClamp'] = max(attachment_diag['maxPotentialRiseClamp'], rest_increase_needed)

            path.stored[A] += s_a
            joint.rest_length -= s_a
            path.stored[B] -= s_b
            joint.rest_length += s_b
            abs_s_a = abs(s_a)
            abs_s_b = abs(s_b)
            abs_rest_delta = abs(-s_a + s_b)
            abs_stored_delta = max(abs_s_a, abs_s_b)
            attachment_diag['maxAbsSA'] = max(attachment_diag['maxAbsSA'], abs_s_a)
            attachment_diag['maxAbsSB'] = max(attachment_diag['maxAbsSB'], abs_s_b)
            attachment_diag['maxAbsJointRestDelta'] = max(attachment_diag['maxAbsJointRestDelta'], abs_rest_delta)
            attachment_diag['maxAbsStoredDelta'] = max(attachment_diag['maxAbsStoredDelta'], abs_stored_delta)
            if abs_stored_delta > 0.001:
                attachment_diag['largeTransferCount'] += 1

            if attachment_a_current is not None:
                joint.attachment_point_a_world = attachment_a_current
            if attachment_b_current is not None:
                joint.attachment_point_b_world = attachment_b_current
    world.set_resource('cableAttachmentUpdateDiag', attachment_diag)
