import numpy as np

from cable_joints.ecs import PositionComponent, RadiusComponent, CableLinkComponent
from cable_joints.cable_joints_components import CableJointComponent, CablePathComponent
from cable_joints.geometry import (
    line_segment_circle_intersection,
    right_of_line,
    tangent_from_circle_to_circle,
    tangent_from_point_to_circle,
    tangent_from_circle_to_point,
    signed_arc_length_on_wheel
)
from cable_joints.util import (
    is_attachment,
    is_rolling,
    effective_cw
)

def split_joints(world):
    """
    Splits cable joints that intersect with 'splitter' entities (e.g., wheels).

    This function iterates through all cable paths and their constituent joints.
    If a joint's straight line segment intersects a potential splitter entity,
    the joint is split into two new joints that wrap around the splitter. The
    rest lengths and stored lengths are recalculated to conserve the total
    cable length and maintain consistent tension.

    This is a Python port of the _splitJoints function from cable_joints_core.js.
    """
    potential_splitters = world.query([PositionComponent, RadiusComponent, CableLinkComponent])
    path_entities = world.query([CablePathComponent])

    for path_id in path_entities:
        path = world.get_component(path_id, CablePathComponent)
        if not path.joint_entities:
            continue

        i = 0
        # This loop mimics the JS `for (let i=0; ...)` over a mutating array.
        # If a split occurs, the list of joints grows, but we continue to the
        # next index, which is the correct behavior to check subsequent segments.
        while i < len(path.joint_entities):
            joint_id = path.joint_entities[i]
            joint = world.get_component(joint_id, CableJointComponent)

            pA = joint.attachment_point_a_world
            pB = joint.attachment_point_b_world

            for splitter_id in potential_splitters:
                if splitter_id == joint.entity_a or splitter_id == joint.entity_b:
                    continue

                pos_splitter_comp = world.get_component(splitter_id, PositionComponent)
                radius_splitter_comp = world.get_component(splitter_id, RadiusComponent)

                if not pos_splitter_comp or not radius_splitter_comp:
                    continue

                pos_splitter = pos_splitter_comp.pos
                radius_splitter = radius_splitter_comp.radius

                if line_segment_circle_intersection(pA, pB, pos_splitter, radius_splitter):
                    entity_a = joint.entity_a
                    entity_b = joint.entity_b

                    # Get components for Entity A
                    pos_a = world.get_component(entity_a, PositionComponent).pos
                    link_type_a = path.link_types[i]
                    is_attachment_a = is_attachment(link_type_a)
                    is_rolling_a = is_rolling(link_type_a)
                    radius_a_comp = world.get_component(entity_a, RadiusComponent)
                    radius_a = radius_a_comp.radius if radius_a_comp else 0.0
                    cw_a = effective_cw(path, i, True)

                    # Get components for Entity B
                    pos_b = world.get_component(entity_b, PositionComponent).pos
                    link_type_b = path.link_types[i + 1]
                    is_attachment_b = is_attachment(link_type_b)
                    is_rolling_b = is_rolling(link_type_b)
                    radius_b_comp = world.get_component(entity_b, RadiusComponent)
                    radius_b = radius_b_comp.radius if radius_b_comp else 0.0
                    cw_b = path.cw[i + 1]

                    # Determine wrapping direction around the new splitter
                    prev_pos_splitter = world.get_component(splitter_id, CableLinkComponent).prev_cable_attachment_time_pos
                    cw = right_of_line(prev_pos_splitter, pA, pB)

                    # Calculate new attachment points for the first new segment (A -> Splitter)
                    if is_rolling_a:
                        tangents = tangent_from_circle_to_circle(pos_a, radius_a, cw_a, pos_splitter, radius_splitter, cw)
                        new_attachment_point_a_for_joint = tangents['a_circle']
                        new_attachment_point_b_for_joint = tangents['b_circle']
                    elif is_attachment_a:
                        tangents = tangent_from_point_to_circle(pA, pos_splitter, radius_splitter, cw)
                        new_attachment_point_a_for_joint = tangents['a_attach']
                        new_attachment_point_b_for_joint = tangents['a_circle']
                    else:
                        continue

                    # Calculate new attachment points for the second new segment (Splitter -> B)
                    if is_rolling_b:
                        tangents = tangent_from_circle_to_circle(pos_splitter, radius_splitter, cw, pos_b, radius_b, cw_b)
                        attachment_point_a_for_new_joint = tangents['a_circle']
                        attachment_point_b_for_new_joint = tangents['b_circle']
                    elif is_attachment_b:
                        tangents = tangent_from_circle_to_point(pB, pos_splitter, radius_splitter, cw)
                        attachment_point_a_for_new_joint = tangents['a_circle']
                        attachment_point_b_for_new_joint = tangents['a_attach']
                    else:
                        continue

                    # Calculate change in stored length on adjacent links
                    sA = signed_arc_length_on_wheel(pA, new_attachment_point_a_for_joint, pos_a, radius_a, cw_a) if is_rolling_a else 0.0
                    sB = signed_arc_length_on_wheel(pB, attachment_point_b_for_new_joint, pos_b, radius_b, cw_b) if is_rolling_b else 0.0

                    # Calculate stored length on the new splitter link
                    s = signed_arc_length_on_wheel(
                        new_attachment_point_b_for_joint,
                        attachment_point_a_for_new_joint,
                        pos_splitter, radius_splitter, cw
                    )

                    if s <= 1e-9 or (s + 1e-9) >= 2.0 * np.pi * radius_splitter:
                        continue # Nothing wraps, or full wrap, abort split.

                    # Distribute original rest length to maintain tension
                    dAS = np.linalg.norm(new_attachment_point_a_for_joint - new_attachment_point_b_for_joint)
                    dSB = np.linalg.norm(attachment_point_a_for_new_joint - attachment_point_b_for_new_joint)

                    original_available_length = joint.rest_length + sB - sA
                    total_dist = dAS + dSB

                    new_rest_length_as = 0.0
                    new_rest_length_sb = 0.0
                    if total_dist > 1e-9:
                        available_rest_length_for_segments = original_available_length - s
                        if available_rest_length_for_segments < 1e-9:
                            continue # Not enough length to split
                        new_rest_length_as = available_rest_length_for_segments * dAS / total_dist
                        new_rest_length_sb = available_rest_length_for_segments * dSB / total_dist

                    # --- Mutate world state ---
                    new_joint_id = world.create_entity()

                    # Update stored lengths on adjacent links
                    path.stored[i] += sA
                    path.stored[i + 1] -= sB

                    # Insert new path elements for the splitter
                    path.joint_entities.insert(i + 1, new_joint_id)
                    path.cw.insert(i + 1, cw)
                    path.link_types.insert(i + 1, 'rolling')
                    path.stored.insert(i + 1, s)

                    # Update original joint (now entityA -> splitter)
                    joint.entity_b = splitter_id
                    joint.rest_length = new_rest_length_as
                    joint.attachment_point_a_world = new_attachment_point_a_for_joint
                    joint.attachment_point_b_world = new_attachment_point_b_for_joint

                    # Create the new joint (splitter -> entityB)
                    world.add_component(new_joint_id, CableJointComponent(
                        splitter_id, entity_b, new_rest_length_sb,
                        attachment_point_a_for_new_joint, attachment_point_b_for_new_joint
                    ))

                    # Break from the splitter loop to continue to the next joint
                    break

            i += 1
