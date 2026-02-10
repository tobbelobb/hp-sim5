import numpy as np

from .ecs import PositionComponent, RadiusComponent
from .cable_joints_components import CablePathComponent, CableJointComponent
from .geometry import tangent_from_circle_to_point, signed_arc_length_on_wheel
from .vector2 import rotate_inplace

def update_hybrid_link_states(world):
    """
    Port of _updateHybridLinkStates from cable_joints_core.js.

    This system handles the state transitions for 'hybrid' and 'hybrid-attachment'
    links at the ends of a cable path.

    1.  'hybrid' -> 'hybrid-attachment':
        If a hybrid link (spool) has "fed out" a negative amount of cable
        (i.e., `path.stored < 0`), it means the cable has fully unwrapped and is
        now pulling taut from a fixed point on the spool's circumference.
        The link's state is changed to 'hybrid-attachment', its `stored` length
        is reset to zero, and the `rest_length` of the adjacent joint is adjusted
        to account for the "negative" length that was just consumed. The
        attachment point is rotated to its new position on the circumference.

    2.  'hybrid-attachment' -> 'hybrid':
        If a 'hybrid-attachment' link is positioned such that the cable should
        start wrapping around it again, this logic detects that condition. It
        calculates the point where the cable would become tangent to the spool
        and determines if the current attachment point has "crossed over" this
        tangent line. If so, it switches the state back to 'hybrid', calculates
        the new `stored` arc length on the spool, and updates the joint's
        `rest_length` and attachment point accordingly.
    """
    path_entities = world.query([CablePathComponent])
    epsilon = 1e-9

    for path_id in path_entities:
        path = world.get_component(path_id, CablePathComponent)
        
        if len(path.link_types) < 1:
            continue

        # This logic only applies to the first and last links of a path.
        # Use a set to handle paths with only one link correctly.
        indices_to_check = {0, len(path.link_types) - 1}

        for i in indices_to_check:
            if path.link_types[i] == 'hybrid':
                if path.stored[i] < 0.0:
                    # Switch from hybrid to hybrid-attachment
                    path.link_types[i] = 'hybrid-attachment'

                    if i == 0:
                        joint_id = path.joint_entities[0]
                        joint = world.get_component(joint_id, CableJointComponent)
                        link_entity = joint.entity_a
                    else: # i == len(path.link_types) - 1
                        joint_id = path.joint_entities[-1]
                        joint = world.get_component(joint_id, CableJointComponent)
                        link_entity = joint.entity_b

                    radius_comp = world.get_component(link_entity, RadiusComponent)
                    pos_comp = world.get_component(link_entity, PositionComponent)
                    
                    if not radius_comp or not pos_comp or radius_comp.radius < epsilon:
                        # Cannot rotate if there's no radius
                        path.stored[i] = 0.0
                        continue

                    # We have "fed out negative line", undo that
                    joint.rest_length += path.stored[i]
                    rot_ang = -path.stored[i] / radius_comp.radius
                    
                    if i == 0:
                        rotate_inplace(joint.attachment_point_a_world, rot_ang, pos_comp.pos, path.cw[i])
                    else: # i == len(path.link_types) - 1
                        rotate_inplace(joint.attachment_point_b_world, rot_ang, pos_comp.pos, path.cw[i])
                    
                    path.stored[i] = 0.0

            elif path.link_types[i] == 'hybrid-attachment':
                # Determine which joint and entities are involved
                if i == 0:
                    if not path.joint_entities: continue
                    joint_id = path.joint_entities[0]
                    joint = world.get_component(joint_id, CableJointComponent)
                    entity_id = joint.entity_a
                    attachment_point = joint.attachment_point_a_world
                    neighbor_attachment_point = joint.attachment_point_b_world
                else: # i == len(path.link_types) - 1
                    if not path.joint_entities: continue
                    joint_id = path.joint_entities[-1]
                    joint = world.get_component(joint_id, CableJointComponent)
                    entity_id = joint.entity_b
                    attachment_point = joint.attachment_point_b_world
                    neighbor_attachment_point = joint.attachment_point_a_world

                pos_comp = world.get_component(entity_id, PositionComponent)
                radius_comp = world.get_component(entity_id, RadiusComponent)

                if not pos_comp or not radius_comp or radius_comp.radius < epsilon:
                    continue

                C = pos_comp.pos
                R = radius_comp.radius

                # Calculate potential tangent points. Note: In the JS source, cw=true means CCW.
                tan_cw_res = tangent_from_circle_to_point(neighbor_attachment_point, C, R, True)
                tan_ccw_res = tangent_from_circle_to_point(neighbor_attachment_point, C, R, False)
                tan_cw = tan_cw_res['a_circle']
                tan_ccw = tan_ccw_res['a_circle']

                # Calculate arc lengths to see if we've crossed over
                # Note: In the JS source, clockwise_preference=true means positive for CW arc.
                crossed_cw = signed_arc_length_on_wheel(attachment_point, tan_cw, C, R, True)
                crossed_ccw = signed_arc_length_on_wheel(attachment_point, tan_ccw, C, R, False)
                
                dist_sq_cw = np.sum((attachment_point - tan_cw)**2)
                dist_sq_ccw = np.sum((attachment_point - tan_ccw)**2)

                new_cw = None
                crossing_tangent = None
                
                # Pick the shorter path to re-wrapping.
                # The JS logic for newCW is ported as-is (e.g., crossedCCW > 0 -> newCW = true).
                if crossed_ccw > 0.0 and dist_sq_ccw < dist_sq_cw:
                    new_cw = True
                    crossing_tangent = tan_ccw
                    path.stored[i] = crossed_ccw
                    joint.rest_length -= crossed_ccw
                elif crossed_cw > 0.0 and dist_sq_cw < dist_sq_ccw:
                    new_cw = False
                    crossing_tangent = tan_cw
                    path.stored[i] = crossed_cw
                    joint.rest_length -= crossed_cw

                if new_cw is not None:
                    # Switch from hybrid-attachment to hybrid
                    path.link_types[i] = 'hybrid'
                    path.cw[i] = new_cw
                    attachment_point[:] = crossing_tangent
