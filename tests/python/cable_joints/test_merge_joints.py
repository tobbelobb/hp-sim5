import pytest
import numpy as np
import math

from cable_joints.ecs import (
    World, PositionComponent, RadiusComponent
)
from cable_joints.cable_joints_components import (
    CableLinkComponent, CableJointComponent, CablePathComponent
)
from cable_joints.geometry import (
    tangent_from_point_to_circle, tangent_from_circle_to_point,
    tangent_from_circle_to_circle, signed_arc_length_on_wheel
)
from cable_joints.merge_joints import merge_joints

def create_cable_path_component(world, joint_entities, link_types, cw, stored=None):
    """Helper to create and initialize a CablePathComponent like in JS."""
    path = CablePathComponent(
        joint_entities=list(joint_entities),
        link_types=list(link_types),
        cw=list(cw),
        stored=[0.0] * len(cw),
        total_rest_length=0.0
    )

    total_rest_length = 0.0
    for joint_id in joint_entities:
        joint = world.get_component(joint_id, CableJointComponent)
        total_rest_length += joint.rest_length

    for i in range(len(joint_entities) - 1):
        joint_i = world.get_component(joint_entities[i], CableJointComponent)
        joint_i_plus_1 = world.get_component(joint_entities[i+1], CableJointComponent)

        link_id = joint_i.entity_b
        if link_id != joint_i_plus_1.entity_a:
            print("Warning: Path mismatch during creation")
            return None

        is_rolling = link_types[i+1] == 'rolling'
        if is_rolling:
            link_pos = world.get_component(link_id, PositionComponent).pos
            link_radius = world.get_component(link_id, RadiusComponent).radius
            is_cw = cw[i+1]

            initial_stored_length = signed_arc_length_on_wheel(
                joint_i.attachment_point_b_world,
                joint_i_plus_1.attachment_point_a_world,
                link_pos,
                link_radius,
                is_cw,
                force_positive=True
            )
            path.stored[i+1] = initial_stored_length
            total_rest_length += initial_stored_length

    path.total_rest_length = total_rest_length

    if stored is not None:
        for i in range(len(stored)):
            if stored[i] is not None:
                path.total_rest_length -= path.stored[i]
                path.total_rest_length += stored[i]
                path.stored[i] = stored[i]

    return path

def test_merge_joints_does_nothing_for_a_single_joint_path():
    world = World()
    anchor_a = world.create_entity()
    anchor_b = world.create_entity()
    world.add_component(anchor_a, PositionComponent(pos=np.array([0.0, 0.0, 0.0])))
    world.add_component(anchor_b, PositionComponent(pos=np.array([5.0, 0.0, 0.0])))

    joint_id = world.create_entity()
    rest_len = 5.0
    attach_comp = CableJointComponent(
        entity_a=anchor_a, entity_b=anchor_b, rest_length=rest_len,
        attachment_point_a_world=np.array([0.0, 0.0, 0.0]),
        attachment_point_b_world=np.array([5.0, 0.0, 0.0])
    )
    world.add_component(joint_id, attach_comp)

    path_id = world.create_entity()
    path_comp = create_cable_path_component(world, [joint_id], ['attachment', 'attachment'], [False, False])
    world.add_component(path_id, path_comp)

    merge_joints(world)

    assert len(path_comp.joint_entities) == 1
    assert path_comp.joint_entities[0] == joint_id
    assert world.get_component(joint_id, CableJointComponent) is not None

def test_merge_joints_does_not_merge_if_wheel_contact_is_needed():
    world = World()
    anchor_l = world.create_entity()
    anchor_r = world.create_entity()
    wheel = world.create_entity()
    world.add_component(anchor_l, PositionComponent(pos=np.array([-4.0, 2.0, 0.0])))
    world.add_component(anchor_r, PositionComponent(pos=np.array([4.0, 2.0, 0.0])))
    world.add_component(wheel, PositionComponent(pos=np.array([0.0, 0.0, 0.0])))
    world.add_component(wheel, RadiusComponent(radius=1.5))

    tang_l = tangent_from_point_to_circle(np.array([-4.0, 2.0, 0.0]), np.array([0.0, 0.0, 0.0]), 1.5, cw=False)
    tang_r = tangent_from_point_to_circle(np.array([4.0, 2.0, 0.0]), np.array([0.0, 0.0, 0.0]), 1.5, cw=True)

    contact_l = tang_l['a_circle']
    contact_r = tang_r['a_circle']
    dist_l = np.linalg.norm(contact_l - tang_l['a_attach'])
    dist_r = np.linalg.norm(contact_r - tang_r['a_attach'])

    joint1 = world.create_entity()
    world.add_component(joint1, CableJointComponent(anchor_l, wheel, dist_l, tang_l['a_attach'], contact_l))
    joint2 = world.create_entity()
    world.add_component(joint2, CableJointComponent(wheel, anchor_r, dist_r, contact_r, tang_r['a_attach']))

    path_id = world.create_entity()
    path_comp = create_cable_path_component(world, [joint1, joint2], ['attachment', 'rolling', 'attachment'], [False, True, True])
    world.add_component(path_id, path_comp)

    merge_joints(world)

    assert len(path_comp.joint_entities) == 2
    assert joint1 in path_comp.joint_entities
    assert joint2 in path_comp.joint_entities
    j1_comp = world.get_component(joint1, CableJointComponent)
    j2_comp = world.get_component(joint2, CableJointComponent)
    assert j1_comp is not None
    assert j2_comp is not None
    assert wheel in [j1_comp.entity_a, j1_comp.entity_b]
    assert wheel in [j2_comp.entity_a, j2_comp.entity_b]

def test_merge_joints_merges_two_joints_when_cable_detaches():
    world = World()
    anchor_l, anchor_r, wheel = world.create_entity(), world.create_entity(), world.create_entity()
    l_pos, r_pos, w_pos = np.array([-4., 2., 0.]), np.array([4., 2., 0.]), np.array([0., 0., 0.])
    r = 1.5
    world.add_component(anchor_l, PositionComponent(pos=l_pos)); world.add_component(anchor_l, CableLinkComponent(prev_cable_attachment_time_pos=l_pos))
    world.add_component(anchor_r, PositionComponent(pos=r_pos)); world.add_component(anchor_r, CableLinkComponent(prev_cable_attachment_time_pos=r_pos))
    world.add_component(wheel, PositionComponent(pos=w_pos)); world.add_component(wheel, CableLinkComponent(prev_cable_attachment_time_pos=w_pos))
    world.add_component(wheel, RadiusComponent(r))

    tang_l = tangent_from_point_to_circle(l_pos, w_pos, r, True)
    tang_r = tangent_from_circle_to_point(r_pos, w_pos, r, True)
    dist_l = np.linalg.norm(tang_l['a_circle'] - tang_l['a_attach'])
    dist_r = np.linalg.norm(tang_r['a_circle'] - tang_r['a_attach'])

    joint1 = world.create_entity()
    world.add_component(joint1, CableJointComponent(anchor_l, wheel, dist_l, tang_l['a_attach'], tang_l['a_circle']))
    joint2 = world.create_entity()
    world.add_component(joint2, CableJointComponent(wheel, anchor_r, dist_r, tang_r['a_circle'], tang_r['a_attach']))

    path_id = world.create_entity()
    path_comp = create_cable_path_component(world, [joint1, joint2], ['attachment', 'rolling', 'attachment'], [False, True, True])
    world.add_component(path_id, path_comp)

    assert path_comp.stored[1] > r * math.pi
    path_comp.stored[1] -= r * 2.0 * math.pi
    path_comp.total_rest_length -= r * 2.0 * math.pi
    assert path_comp.stored[1] < 0.0
    initial_total_rest_length = path_comp.total_rest_length

    merge_joints(world)

    assert path_comp.total_rest_length == initial_total_rest_length
    assert len(path_comp.joint_entities) == 1

    remaining_joint_id = path_comp.joint_entities[0]
    remaining_joint = world.get_component(remaining_joint_id, CableJointComponent)

    assert anchor_l in [remaining_joint.entity_a, remaining_joint.entity_b]
    assert anchor_r in [remaining_joint.entity_a, remaining_joint.entity_b]
    assert wheel not in [remaining_joint.entity_a, remaining_joint.entity_b]

    removed_joint_id = joint2 if remaining_joint_id == joint1 else joint1
    assert removed_joint_id not in path_comp.joint_entities
    assert world.get_component(removed_joint_id, CableJointComponent) is None

    assert math.isclose(path_comp.stored[0], 0.0, abs_tol=1e-8)
    assert math.isclose(path_comp.stored[1], 0.0, abs_tol=1e-8)
    assert math.isclose(remaining_joint.rest_length, initial_total_rest_length, abs_tol=1e-8)

    assert path_comp.link_types == ['attachment', 'attachment']

def test_merge_joints_merges_three_joints_into_one():
    world = World()
    anchor_l, anchor_r = world.create_entity(), world.create_entity()
    wheel1, wheel2 = world.create_entity(), world.create_entity()

    l_pos, r_pos = np.array([-4., 0., 0.]), np.array([4., 0., 0.])
    w1_pos, w2_pos = np.array([-2., -0.51, 0.]), np.array([2., 0.51, 0.])
    w1_cw, w2_cw = True, False
    r = 0.5

    world.add_component(anchor_l, PositionComponent(pos=l_pos)); world.add_component(anchor_l, CableLinkComponent(prev_cable_attachment_time_pos=l_pos))
    world.add_component(anchor_r, PositionComponent(pos=r_pos)); world.add_component(anchor_r, CableLinkComponent(prev_cable_attachment_time_pos=r_pos))
    world.add_component(wheel1, PositionComponent(pos=w1_pos)); world.add_component(wheel1, CableLinkComponent(prev_cable_attachment_time_pos=w1_pos)); world.add_component(wheel1, RadiusComponent(r))
    world.add_component(wheel2, PositionComponent(pos=w2_pos)); world.add_component(wheel2, CableLinkComponent(prev_cable_attachment_time_pos=w2_pos)); world.add_component(wheel2, RadiusComponent(r))

    tang_l = tangent_from_point_to_circle(l_pos, w1_pos, r, w1_cw)
    tang_cc = tangent_from_circle_to_circle(w1_pos, r, w1_cw, w2_pos, r, w2_cw)
    tang_r = tangent_from_circle_to_point(r_pos, w2_pos, r, w2_cw)

    dist_l = np.linalg.norm(tang_l['a_circle'] - tang_l['a_attach'])
    dist_m = np.linalg.norm(tang_cc['a_circle'] - tang_cc['b_circle'])
    dist_r = np.linalg.norm(tang_r['a_circle'] - tang_r['a_attach'])

    joint1 = world.create_entity()
    world.add_component(joint1, CableJointComponent(anchor_l, wheel1, dist_l, tang_l['a_attach'], tang_l['a_circle']))
    joint2 = world.create_entity()
    world.add_component(joint2, CableJointComponent(wheel1, wheel2, dist_m, tang_cc['a_circle'], tang_cc['b_circle']))
    joint3 = world.create_entity()
    world.add_component(joint3, CableJointComponent(wheel2, anchor_r, dist_r, tang_r['a_circle'], tang_r['a_attach']))

    path_id = world.create_entity()
    path_comp = create_cable_path_component(world, [joint1, joint2, joint3], ['attachment', 'rolling', 'rolling', 'attachment'], [False, w1_cw, w2_cw, True])
    world.add_component(path_id, path_comp)

    path_comp.stored[1] -= r * 2.0 * math.pi
    path_comp.total_rest_length -= r * 2.0 * math.pi
    path_comp.stored[2] -= r * 2.0 * math.pi
    path_comp.total_rest_length -= r * 2.0 * math.pi
    assert path_comp.stored[1] < 0.0
    assert path_comp.stored[2] < 0.0
    initial_total_rest_length = path_comp.total_rest_length

    merge_joints(world)

    assert math.isclose(path_comp.total_rest_length, initial_total_rest_length)
    assert len(path_comp.joint_entities) == 1

    remaining_joint_id = path_comp.joint_entities[0]
    remaining_joint = world.get_component(remaining_joint_id, CableJointComponent)

    assert anchor_l in [remaining_joint.entity_a, remaining_joint.entity_b]
    assert anchor_r in [remaining_joint.entity_a, remaining_joint.entity_b]
    assert wheel1 not in [remaining_joint.entity_a, remaining_joint.entity_b]
    assert wheel2 not in [remaining_joint.entity_a, remaining_joint.entity_b]

    removed_ids = {joint1, joint2, joint3} - {remaining_joint_id}
    for removed_id in removed_ids:
        assert removed_id not in path_comp.joint_entities
        assert world.get_component(removed_id, CableJointComponent) is None

    assert len(path_comp.stored) == 2
    assert math.isclose(path_comp.stored[0], 0.0, abs_tol=1e-8)
    assert math.isclose(path_comp.stored[1], 0.0, abs_tol=1e-8)
    assert math.isclose(remaining_joint.rest_length, initial_total_rest_length, abs_tol=1e-8)

    assert path_comp.link_types == ['attachment', 'attachment']
