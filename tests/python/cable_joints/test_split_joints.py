import pytest
import numpy as np

from python.ecs import World, PositionComponent, RadiusComponent, CableLinkComponent
from python.cable_joints_components import CableJointComponent, CablePathComponent
from python.split_joints import split_joints
from python.geometry import tangent_from_point_to_circle, tangent_from_circle_to_point

def test_split_joints_does_nothing_for_a_single_joint_path_that_misses_every_wheel():
    world = World()

    A = world.create_entity()
    B = world.create_entity()
    wheel = world.create_entity()

    world.add_component(A, PositionComponent(pos=np.array([0.0, 0.0, 0.0])))
    world.add_component(B, PositionComponent(pos=np.array([5.0, 0.0, 0.0])))
    world.add_component(wheel, PositionComponent(pos=np.array([0.0, 5.0, 0.0])))
    world.add_component(wheel, RadiusComponent(radius=1.5))
    world.add_component(wheel, CableLinkComponent())

    joint_id = world.create_entity()
    world.add_component(
        joint_id,
        CableJointComponent(
            entity_a=A, entity_b=B, rest_length=5.0,
            attachment_point_a_world=np.array([0.0, 0.0, 0.0]),
            attachment_point_b_world=np.array([5.0, 0.0, 0.0])
        )
    )

    path_id = world.create_entity()
    path_comp = CablePathComponent(
        joint_entities=[joint_id],
        link_types=['attachment', 'attachment'],
        cw=[False, False],
        stored=[0.0, 0.0]
    )
    world.add_component(path_id, path_comp)

    split_joints(world)

    assert len(path_comp.joint_entities) == 1
    assert path_comp.joint_entities[0] == joint_id
    assert world.get_component(joint_id, CableJointComponent) is not None

def test_split_joints_does_not_duplicate_joints_when_rope_already_bends_over_a_wheel():
    world = World()

    L = world.create_entity()
    R = world.create_entity()
    wheel = world.create_entity()

    pos_L = np.array([-4.0, 2.0, 0.0])
    pos_R = np.array([4.0, 2.0, 0.0])
    pos_wheel = np.array([0.0, 0.0, 0.0])
    radius_wheel = 1.5

    world.add_component(L, PositionComponent(pos=pos_L))
    world.add_component(R, PositionComponent(pos=pos_R))
    world.add_component(wheel, PositionComponent(pos=pos_wheel))
    world.add_component(wheel, RadiusComponent(radius=radius_wheel))
    world.add_component(wheel, CableLinkComponent(prev_cable_attachment_time_pos=pos_wheel))

    tang_L = tangent_from_point_to_circle(pos_L, pos_wheel, radius_wheel, False)
    tang_R = tangent_from_circle_to_point(pos_R, pos_wheel, radius_wheel, True)

    joint1_id = world.create_entity()
    world.add_component(
        joint1_id,
        CableJointComponent(
            entity_a=L, entity_b=wheel,
            rest_length=np.linalg.norm(tang_L['a_circle'] - tang_L['a_attach']),
            attachment_point_a_world=tang_L['a_attach'],
            attachment_point_b_world=tang_L['a_circle']
        )
    )

    joint2_id = world.create_entity()
    world.add_component(
        joint2_id,
        CableJointComponent(
            entity_a=wheel, entity_b=R,
            rest_length=np.linalg.norm(tang_R['a_circle'] - tang_R['a_attach']),
            attachment_point_a_world=tang_R['a_circle'],
            attachment_point_b_world=tang_R['a_attach']
        )
    )

    path_id = world.create_entity()
    path_comp = CablePathComponent(
        joint_entities=[joint1_id, joint2_id],
        link_types=['attachment', 'rolling', 'attachment'],
        cw=[False, True, True],
        stored=[0.0, 0.0, 0.0]
    )
    world.add_component(path_id, path_comp)

    split_joints(world)

    assert len(path_comp.joint_entities) == 2
    assert joint1_id in path_comp.joint_entities
    assert joint2_id in path_comp.joint_entities

def test_split_joints_inserts_a_wheel_joint_when_a_straight_segment_intersects_the_wheel():
    world = World()

    A = world.create_entity()
    B = world.create_entity()
    wheel = world.create_entity()

    pos_A = np.array([-4.0, 0.0, 0.0])
    pos_B = np.array([4.0, 0.0, 0.0])
    pos_wheel = np.array([0.0, 0.0, 0.0])

    world.add_component(A, PositionComponent(pos=pos_A))
    world.add_component(B, PositionComponent(pos=pos_B))
    world.add_component(wheel, PositionComponent(pos=pos_wheel))
    world.add_component(wheel, CableLinkComponent(prev_cable_attachment_time_pos=pos_wheel))
    world.add_component(wheel, RadiusComponent(radius=1.5))

    joint_id = world.create_entity()
    world.add_component(
        joint_id,
        CableJointComponent(
            entity_a=A, entity_b=B, rest_length=8.0,
            attachment_point_a_world=pos_A.copy(),
            attachment_point_b_world=pos_B.copy()
        )
    )

    path_id = world.create_entity()
    path_comp = CablePathComponent(
        joint_entities=[joint_id],
        link_types=['attachment', 'attachment'],
        cw=[False, False],
        stored=[0.0, 0.0]
    )
    world.add_component(path_id, path_comp)

    split_joints(world)

    assert len(path_comp.joint_entities) == 2

    new_joint_id = [j for j in path_comp.joint_entities if j != joint_id][0]
    new_joint = world.get_component(new_joint_id, CableJointComponent)
    old_joint = world.get_component(joint_id, CableJointComponent)

    all_entities = [old_joint.entity_a, old_joint.entity_b, new_joint.entity_a, new_joint.entity_b]
    assert wheel in all_entities

    assert path_comp.link_types == ['attachment', 'rolling', 'attachment']
    assert path_comp.stored[1] >= 0

def test_split_joints_creates_three_joints_when_a_straight_segment_intersects_two_wheels():
    world = World()

    A = world.create_entity()
    B = world.create_entity()
    wheel1 = world.create_entity()
    wheel2 = world.create_entity()

    a_pos = np.array([-4.0, 0.0, 0.0])
    b_pos = np.array([4.0, 0.0, 0.0])
    w1_pos = np.array([-1.5, -0.75, 0.0])
    w2_pos = np.array([1.5, 0.75, 0.0])
    r = 0.9

    world.add_component(A, PositionComponent(pos=a_pos))
    world.add_component(B, PositionComponent(pos=b_pos))

    world.add_component(wheel1, PositionComponent(pos=w1_pos))
    world.add_component(wheel1, CableLinkComponent(prev_cable_attachment_time_pos=w1_pos))
    world.add_component(wheel1, RadiusComponent(radius=r))

    world.add_component(wheel2, PositionComponent(pos=w2_pos))
    world.add_component(wheel2, CableLinkComponent(prev_cable_attachment_time_pos=w2_pos))
    world.add_component(wheel2, RadiusComponent(radius=r))

    joint_id = world.create_entity()
    world.add_component(
        joint_id,
        CableJointComponent(
            entity_a=A, entity_b=B,
            rest_length=np.linalg.norm(b_pos - a_pos),
            attachment_point_a_world=a_pos.copy(),
            attachment_point_b_world=b_pos.copy()
        )
    )

    path_id = world.create_entity()
    path_comp = CablePathComponent(
        joint_entities=[joint_id],
        link_types=['attachment', 'attachment'],
        cw=[False, False],
        stored=[0.0, 0.0]
    )
    world.add_component(path_id, path_comp)

    split_joints(world)

    assert len(path_comp.joint_entities) == 3

    comps = [world.get_component(jid, CableJointComponent) for jid in path_comp.joint_entities]

    # The path should be A -> W1 -> W2 -> B
    assert comps[0].entity_a == A
    assert comps[0].entity_b == wheel1
    assert comps[1].entity_a == wheel1
    assert comps[1].entity_b == wheel2
    assert comps[2].entity_a == wheel2
    assert comps[2].entity_b == B

    assert path_comp.link_types == ['attachment', 'rolling', 'rolling', 'attachment']
    assert len(path_comp.stored) == 4
    assert path_comp.stored[1] >= 0
    assert path_comp.stored[2] >= 0
