import numpy as np
from .ecs import (
    PositionComponent, RadiusComponent,
)
from .cable_joints_components import (
    CableJointComponent, CablePathComponent
)
from .geometry import (
    tangent_from_circle_to_circle, tangent_from_circle_to_point,
    tangent_from_point_to_circle, signed_arc_length_on_wheel
)
from python.util import (
    is_attachment,
    is_rolling,
    effective_cw
)

def merge_joints(world):
    """
    Merges adjacent cable joints when a cable lifts off a rolling link.
    This is a Python port of the _mergeJoints function from cable_joints_core.js.
    """
    path_entities = world.query([CablePathComponent])
    for path_id in path_entities:
        path = world.get_component(path_id, CablePathComponent)
        if len(path.joint_entities) < 2:
            continue

        joints_in_path = path.joint_entities
        # A merge operation in itself might move attachment points in such a way
        # that an additional merge operation is required, or a single joint might
        # require a merge of both of its ends at the same time step. We re run merge
        # until all such cases are resolved for a given time step.
        re_run_merge = True
        while re_run_merge:
            re_run_merge = False
            for i in range(len(path.joint_entities) - 1):  # Iterate over adjacent pairs
                if path.link_types[i + 1] != 'rolling':
                    continue

                joint_id_i = path.joint_entities[i]
                joint_id_i_plus_1 = path.joint_entities[i + 1]
                joint_i = world.get_component(joint_id_i, CableJointComponent)
                joint_i_plus_1 = world.get_component(joint_id_i_plus_1, CableJointComponent)
                link_id = joint_i.entity_b
                link_id2 = joint_i_plus_1.entity_a

                if link_id != link_id2:
                    print("Warning: Merge loop saw disconnected cable path")
                    continue

                if joint_i.entity_a == joint_i_plus_1.entity_b:
                    # A cable has wrapped around a link, and back. This will not be a merge candidate.
                    continue

                if path.stored[i + 1] < 0.0:
                    # print(f"Merging joints {joint_id_i} and {joint_id_i_plus_1} (stored: {path.stored[i + 1]:.4f})")

                    # Calculate angle between the two segments, just for debug
                    p_a1 = joint_i.attachment_point_a_world
                    p_b2 = joint_i_plus_1.attachment_point_b_world
                    pos_a = world.get_component(joint_i.entity_a, PositionComponent).pos
                    radius_a_comp = world.get_component(joint_i.entity_a, RadiusComponent)
                    radius_a = radius_a_comp.radius if radius_a_comp else None
                    cw_a = effective_cw(path, i, True)
                    pos_b = world.get_component(joint_i_plus_1.entity_b, PositionComponent).pos
                    radius_b_comp = world.get_component(joint_i_plus_1.entity_b, RadiusComponent)
                    radius_b = radius_b_comp.radius if radius_b_comp else None
                    cw_b = path.cw[i + 2]

                    joint_i.rest_length += joint_i_plus_1.rest_length + path.stored[i + 1]
                    joint_i.entity_b = joint_i_plus_1.entity_b

                    is_attachment_a = is_attachment(path.link_types[i])
                    is_rolling_a = is_rolling(path.link_types[i])
                    is_attachment_b = is_attachment(path.link_types[i + 2])
                    is_rolling_b = is_rolling(path.link_types[i + 2])

                    attachment_a_current = p_a1
                    attachment_b_current = p_b2

                    if is_rolling_a and is_rolling_b:
                        tangents = tangent_from_circle_to_circle(pos_a, radius_a, cw_a, pos_b, radius_b, cw_b)
                        attachment_a_current = tangents['a_circle']
                        attachment_b_current = tangents['b_circle']
                    elif is_rolling_a and is_attachment_b:
                        attachment_a_current = tangent_from_circle_to_point(p_b2, pos_a, radius_a, cw_a)['a_circle']
                    elif is_attachment_a and is_rolling_b:
                        attachment_b_current = tangent_from_point_to_circle(p_a1, pos_b, radius_b, cw_b)['a_circle']

                    s_a = 0.0
                    s_b = 0.0

                    if is_rolling_a and is_rolling_b:
                        s_a = signed_arc_length_on_wheel(p_a1, attachment_a_current, pos_a, radius_a, cw_a)
                        s_b = signed_arc_length_on_wheel(p_b2, attachment_b_current, pos_b, radius_b, cw_b)
                    elif is_rolling_a and is_attachment_b:
                        s_a = signed_arc_length_on_wheel(p_a1, attachment_a_current, pos_a, radius_a, cw_a)
                    elif is_attachment_a and is_rolling_b:
                        s_b = signed_arc_length_on_wheel(p_b2, attachment_b_current, pos_b, radius_b, cw_b)

                    path.stored[i] += s_a
                    joint_i.rest_length -= s_a
                    path.stored[i + 2] -= s_b
                    joint_i.rest_length += s_b
                    re_run_merge = path.stored[i] < 0.0 or path.stored[i + 2] < 0.0

                    # Python equivalent of JavaScript's .set() method
                    joint_i.attachment_point_a_world[0] = attachment_a_current[0]
                    joint_i.attachment_point_a_world[1] = attachment_a_current[1]
                    joint_i.attachment_point_b_world[0] = attachment_b_current[0]
                    joint_i.attachment_point_b_world[1] = attachment_b_current[1]

                    # Remove elements using del (equivalent to splice)
                    del path.joint_entities[i + 1]
                    del path.stored[i + 1]
                    del path.cw[i + 1]
                    del path.link_types[i + 1]
                    world.destroy_entity(joint_id_i_plus_1)
