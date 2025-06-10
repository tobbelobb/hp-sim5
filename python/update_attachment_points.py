import numpy as np

from python.ecs import (
    PositionComponent,
    RadiusComponent,
    CableLinkComponent,
    OrientationComponent,
)
from python.cable_joints_components import CableJointComponent, CablePathComponent
from python.geometry import (
    tangent_from_point_to_circle,
    tangent_from_circle_to_point,
    tangent_from_circle_to_circle,
    signed_arc_length_on_wheel,
)
from python.vector2 import rotate_inplace

def _is_attachment(value):
    """Checks if a link type is an attachment point."""
    return value in ['attachment', 'hybrid-attachment', 'pinhole']

def _is_rolling(value):
    """Checks if a link type is a rolling contact."""
    return value in ['rolling', 'hybrid']

def _is_hybrid(value):
    """Checks if a link type is a hybrid or hybrid-attachment."""
    return value in ['hybrid', 'hybrid-attachment']

def _effective_cw(path, link_index, travelling_from_circle):
    """
    Determines the effective clockwise direction for tangent calculations.
    This is a special case for the first link in a path when calculating
    the tangent from that link (which is a circle).
    """
    if link_index == 0 and travelling_from_circle:
        return not path.cw[link_index]
    return path.cw[link_index]

def update_attachment_points(world):
    """
    Updates cable attachment points based on entity movements and rotations.

    This function iterates through all cable paths and recalculates the world-space
    attachment points for each joint. It handles different link types (attachment,
    rolling, hybrid) and updates the stored length of cable on rolling links
    and the rest length of flexible segments to conserve total cable length.

    This is a Python port of the _updateAttachmentPoints function from
    cable_joints_core.js.
    """
    path_entities = world.query([CablePathComponent])

    for path_id in path_entities:
        path = world.get_component(path_id, CablePathComponent)

        for i in range(len(path.joint_entities)):
            joint_id = path.joint_entities[i]
            joint = world.get_component(joint_id, CableJointComponent)

            A = i       # Index for link/cw/stored related to entity A side
            B = i + 1   # Index for link/cw/stored related to entity B side

            entity_a = joint.entity_a
            entity_b = joint.entity_b

            # Get components for Entity A
            pos_a_comp = world.get_component(entity_a, PositionComponent)
            radius_a_comp = world.get_component(entity_a, RadiusComponent)
            link_a_comp = world.get_component(entity_a, CableLinkComponent)
            orientation_a_comp = world.get_component(entity_a, OrientationComponent)

            pos_a = pos_a_comp.pos
            attachment_a_previous = joint.attachment_point_a_world
            prev_pos_a = link_a_comp.prev_cable_attachment_time_pos
            radius_a = radius_a_comp.radius if radius_a_comp else 0.0
            angle_a = orientation_a_comp.angle if orientation_a_comp else 0.0
            prev_angle_a = link_a_comp.prev_cable_attachment_time_angle
            delta_angle_a = angle_a - prev_angle_a

            cw_a = _effective_cw(path, A, True)
            attachment_link_a = _is_attachment(path.link_types[A])
            rolling_link_a = _is_rolling(path.link_types[A])
            is_hybrid_a = _is_hybrid(path.link_types[A])

            p_a_diff_from_translation = pos_a - prev_pos_a
            temp_rotated_point_a = attachment_a_previous.copy()
            rotate_inplace(temp_rotated_point_a, delta_angle_a, prev_pos_a, True)
            p_a_diff_from_rotation = temp_rotated_point_a - attachment_a_previous

            # Get components for Entity B
            pos_b_comp = world.get_component(entity_b, PositionComponent)
            radius_b_comp = world.get_component(entity_b, RadiusComponent)
            link_b_comp = world.get_component(entity_b, CableLinkComponent)
            orientation_b_comp = world.get_component(entity_b, OrientationComponent)

            pos_b = pos_b_comp.pos
            attachment_b_previous = joint.attachment_point_b_world
            prev_pos_b = link_b_comp.prev_cable_attachment_time_pos
            radius_b = radius_b_comp.radius if radius_b_comp else 0.0
            angle_b = orientation_b_comp.angle if orientation_b_comp else 0.0
            prev_angle_b = link_b_comp.prev_cable_attachment_time_angle
            delta_angle_b = angle_b - prev_angle_b

            cw_b = _effective_cw(path, B, False)
            attachment_link_b = _is_attachment(path.link_types[B])
            rolling_link_b = _is_rolling(path.link_types[B])
            is_hybrid_b = _is_hybrid(path.link_types[B])

            p_b_diff_from_translation = pos_b - prev_pos_b
            temp_rotated_point_b = attachment_b_previous.copy()
            rotate_inplace(temp_rotated_point_b, delta_angle_b, prev_pos_b, True)
            p_b_diff_from_rotation = temp_rotated_point_b - attachment_b_previous

            # --- Calculate Attachment Points based on this frame's positions and rotations ---
            attachment_a_current = pos_a.copy()
            attachment_b_current = pos_b.copy()

            if attachment_link_a and rolling_link_b:
                if is_hybrid_a:
                    attachment_a_current = attachment_a_previous + p_a_diff_from_translation + p_a_diff_from_rotation
                tangents = tangent_from_point_to_circle(attachment_a_current, pos_b, radius_b, cw_b)
                attachment_b_current = tangents['a_circle']
            elif rolling_link_a and attachment_link_b:
                if is_hybrid_b:
                    attachment_b_current = attachment_b_previous + p_b_diff_from_translation + p_b_diff_from_rotation
                tangents = tangent_from_circle_to_point(attachment_b_current, pos_a, radius_a, cw_a)
                attachment_a_current = tangents['a_circle']
            elif rolling_link_a and rolling_link_b:
                tangents = tangent_from_circle_to_circle(pos_a, radius_a, cw_a, pos_b, radius_b, cw_b)
                attachment_a_current = tangents['a_circle']
                attachment_b_current = tangents['b_circle']
            else:  # attachment_link_a and attachment_link_b
                if is_hybrid_a:
                    attachment_a_current = attachment_a_previous + p_a_diff_from_translation + p_a_diff_from_rotation
                if is_hybrid_b:
                    attachment_b_current = attachment_b_previous + p_b_diff_from_translation + p_b_diff_from_rotation

            # --- Calculate Wrapping/Unwrapping Arc Lengths (sA, sB) ---
            sA = 0.0
            sB = 0.0
            if rolling_link_a:
                sA = signed_arc_length_on_wheel(attachment_a_previous - prev_pos_a, attachment_a_current - pos_a, np.zeros(3), radius_a, cw_a)
                if is_hybrid_a:
                    sA += delta_angle_a * radius_a if cw_a else -delta_angle_a * radius_a
            if rolling_link_b:
                sB = signed_arc_length_on_wheel(attachment_b_previous - prev_pos_b, attachment_b_current - pos_b, np.zeros(3), radius_b, cw_b)
                if is_hybrid_b:
                    sB += delta_angle_b * radius_b if cw_b else -delta_angle_b * radius_b

            path.stored[A] += sA
            joint.rest_length -= sA
            path.stored[B] -= sB
            joint.rest_length += sB

            joint.attachment_point_a_world = attachment_a_current
            joint.attachment_point_b_world = attachment_b_current
