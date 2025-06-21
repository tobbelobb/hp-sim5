import numpy as np

from .ecs import (
    PositionComponent,
    RadiusComponent,
    OrientationComponent,
    CoefficientOfFrictionComponent
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

def update_attachment_points(world):
    """
    Updates cable attachment points based on entity movements and rotations.

    This function iterates through all cable paths and recalculates the world-space
    attachment points for each joint. It handles different link types (attachment,
    rolling, hybrid) and updates the stored length of cable on rolling links
    and the rest length of flexible segments to conserve total cable length.
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
            rotate_inplace(temp_rotated_point_a, delta_angle_a, prev_pos_a, True)
            p_a_diff_from_rotation = temp_rotated_point_a - attachment_a_previous

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

            cw_b = effective_cw(path, B, False)
            attachment_link_b = is_attachment(path.link_types[B])
            rolling_link_b = is_rolling(path.link_types[B])
            is_hybrid_b = is_hybrid(path.link_types[B])

            # Pre-calculate translation and rotation differences (like JS version)
            p_b_diff_from_translation = pos_b - prev_pos_b if (pos_b is not None and prev_pos_b is not None) else None
            temp_rotated_point_b = attachment_b_previous.copy()
            rotate_inplace(temp_rotated_point_b, delta_angle_b, prev_pos_b, True)
            p_b_diff_from_rotation = temp_rotated_point_b - attachment_b_previous

            has_friction_b = world.get_component(entity_b, CoefficientOfFrictionComponent) is not None

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

            # --- Calculate Wrapping/Unwrapping Arc Lengths (sA, sB) ---
            s_a = 0.0  # Change in stored length on side A due to wrapping/unwrapping this frame
            s_b = 0.0  # Change in stored length on side B due to wrapping/unwrapping this frame

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

            path.stored[A] += s_a
            joint.rest_length -= s_a
            path.stored[B] -= s_b
            joint.rest_length += s_b

            if attachment_a_current is not None:
                joint.attachment_point_a_world = attachment_a_current
            if attachment_b_current is not None:
                joint.attachment_point_b_world = attachment_b_current
