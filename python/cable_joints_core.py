import numpy as np
from .ecs import (
    PositionComponent, RadiusComponent,
    CableJointComponent, CablePathComponent
)
from .geometry import (
    tangent_from_circle_to_circle, tangent_from_circle_to_point,
    tangent_from_point_to_circle, signed_arc_length_on_wheel
)

def _is_attachment(value):
    """Checks if a link type is an attachment type."""
    return value in ['attachment', 'hybrid-attachment', 'pinhole']

def _is_rolling(value):
    """Checks if a link type is a rolling type."""
    return value in ['rolling', 'hybrid']

def _effective_cw(path, link_index, travelling_from_circle):
    """Determines the effective clockwise direction for a link."""
    if link_index == 0 and travelling_from_circle:
        return not path.cw[link_index]
    return path.cw[link_index]

def _merge_joints(world):
    """
    Merges adjacent cable joints when a cable lifts off a rolling link.
    This is a Python port of the _mergeJoints function from cable_joints_core.js.
    """
    path_entities = world.query(CablePathComponent)
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

                joint_id_i = path.joint_entities[i]
                joint_id_i_plus_1 = path.joint_entities[i + 1]
                joint_i = world.get_component(joint_id_i, CableJointComponent)
                joint_i_plus_1 = world.get_component(joint_id_i_plus_1, CableJointComponent)

                if joint_i is None or joint_i_plus_1 is None:
                    i += 1
                    continue

                if joint_i.entity_b != joint_i_plus_1.entity_a:
                    print("Warning: Merge loop saw disconnected cable path")
                    i += 1
                    continue

                if joint_i.entity_a == joint_i_plus_1.entity_b:
                    # A cable has wrapped around a link, and back. This will not be a merge candidate.
                    i += 1
                    continue

                if path.stored[i + 1] < 0.0:
                    # A merge is needed.
                    pA1 = joint_i.attachment_point_a_world
                    pB2 = joint_i_plus_1.attachment_point_b_world

                    posA_comp = world.get_component(joint_i.entity_a, PositionComponent)
                    radiusA_comp = world.get_component(joint_i.entity_a, RadiusComponent)
                    posA = posA_comp.pos
                    radiusA = radiusA_comp.radius if radiusA_comp else None
                    cwA = _effective_cw(path, i, True)

                    posB_comp = world.get_component(joint_i_plus_1.entity_b, PositionComponent)
                    radiusB_comp = world.get_component(joint_i_plus_1.entity_b, RadiusComponent)
                    posB = posB_comp.pos
                    radiusB = radiusB_comp.radius if radiusB_comp else None
                    cwB = path.cw[i + 2]

                    joint_i.rest_length += joint_i_plus_1.rest_length + path.stored[i + 1]
                    joint_i.entity_b = joint_i_plus_1.entity_b

                    is_attachment_A = _is_attachment(path.link_types[i])
                    is_rolling_A = _is_rolling(path.link_types[i])
                    is_attachment_B = _is_attachment(path.link_types[i + 2])
                    is_rolling_B = _is_rolling(path.link_types[i + 2])

                    attachment_a_current = pA1.copy()
                    attachment_b_current = pB2.copy()

                    if is_rolling_A and is_rolling_B:
                        tangents = tangent_from_circle_to_circle(posA, radiusA, cwA, posB, radiusB, cwB)
                        attachment_a_current = tangents['a_circle']
                        attachment_b_current = tangents['b_circle']
                    elif is_rolling_A and is_attachment_B:
                        tangents = tangent_from_circle_to_point(pB2, posA, radiusA, cwA)
                        attachment_a_current = tangents['a_circle']
                    elif is_attachment_A and is_rolling_B:
                        tangents = tangent_from_point_to_circle(pA1, posB, radiusB, cwB)
                        attachment_b_current = tangents['a_circle']

                    sA = 0.0
                    sB = 0.0
                    if is_rolling_A and radiusA is not None:
                        sA = signed_arc_length_on_wheel(pA1, attachment_a_current, posA, radiusA, cwA)
                    if is_rolling_B and radiusB is not None:
                        sB = signed_arc_length_on_wheel(pB2, attachment_b_current, posB, radiusB, cwB)

                    path.stored[i] += sA
                    joint_i.rest_length -= sA
                    path.stored[i + 2] -= sB
                    joint_i.rest_length += sB
                    
                    if path.stored[i] < 0.0 or path.stored[i + 2] < 0.0:
                        re_run_merge = True

                    joint_i.attachment_point_a_world = attachment_a_current
                    joint_i.attachment_point_b_world = attachment_b_current

                    path.joint_entities.pop(i + 1)
                    path.stored.pop(i + 1)
                    path.cw.pop(i + 1)
                    path.link_types.pop(i + 1)
                    world.destroy_entity(joint_id_i_plus_1)
                    
                    # To replicate JS behavior, we do not increment `i` here.
                    # The while loop will continue with the same `i` but on a shorter list,
                    # effectively checking the newly formed adjacent pair.
                    # The outer `re_run_merge` loop handles cases where a merge
                    # at the end of the list requires another pass from the beginning.
                else:
                    i += 1
