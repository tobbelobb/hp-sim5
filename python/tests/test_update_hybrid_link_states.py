import pytest
import numpy as np

from python.ecs import World, PositionComponent, RadiusComponent, CableLinkComponent
from python.cable_joints_components import CableJointComponent, CablePathComponent
from python.updateHybridLinkStates import update_hybrid_link_states

# Test setup helpers
def add_wheel(world, pos, r=1.0):
    entity_id = world.create_entity()
    world.add_component(entity_id, PositionComponent(pos=pos.copy()))
    world.add_component(entity_id, CableLinkComponent(prev_cable_attachment_time_pos=pos.copy()))
    world.add_component(entity_id, RadiusComponent(radius=r))
    return entity_id

def add_anchor(world, pos):
    entity_id = world.create_entity()
    world.add_component(entity_id, PositionComponent(pos=pos.copy()))
    world.add_component(entity_id, CableLinkComponent(prev_cable_attachment_time_pos=pos.copy()))
    return entity_id

# Tests ported from JS
def test_first_link_hybrid_to_hybrid_attachment():
    world = World()
    wheel = add_wheel(world, np.array([0.0, 0.0, 0.0]), 1.0)
    anchor = add_anchor(world, np.array([0.0, 3.0, 0.0]))

    joint_id = world.create_entity()
    initial_rest = 3.0
    world.add_component(joint_id, CableJointComponent(
        entity_a=wheel, entity_b=anchor, rest_length=initial_rest,
        attachment_point_a_world=np.array([1.0, 0.0, 0.0]),
        attachment_point_b_world=np.array([0.0, 3.0, 0.0])
    ))

    path_id = world.create_entity()
    path_comp = CablePathComponent(
        joint_entities=[joint_id],
        link_types=['hybrid', 'attachment'],
        cw=[False, False],
        stored=[0.0, 0.0]
    )
    path_comp.total_rest_length = initial_rest
    world.add_component(path_id, path_comp)

    # Feed out a bit of "negative" rope
    path_comp.stored[0] = -0.2

    update_hybrid_link_states(world)

    assert path_comp.link_types[0] == 'hybrid-attachment'
    assert path_comp.stored[0] == pytest.approx(0.0)
    
    joint = world.get_component(joint_id, CableJointComponent)
    assert joint.rest_length == pytest.approx(initial_rest - 0.2)

def test_last_link_hybrid_to_hybrid_attachment():
    world = World()
    anchor = add_anchor(world, np.array([-1.0, 0.0, 0.0]))
    wheel = add_wheel(world, np.array([0.0, 0.0, 0.0]), 1.0)

    joint_id = world.create_entity()
    initial_rest = 2.0
    world.add_component(joint_id, CableJointComponent(
        entity_a=anchor, entity_b=wheel, rest_length=initial_rest,
        attachment_point_a_world=np.array([-1.0, 0.0, 0.0]),
        attachment_point_b_world=np.array([1.0, 0.0, 0.0])
    ))

    path_id = world.create_entity()
    path_comp = CablePathComponent(
        joint_entities=[joint_id],
        link_types=['attachment', 'hybrid'],
        cw=[False, False],
        stored=[0.0, 0.0]
    )
    path_comp.total_rest_length = initial_rest
    world.add_component(path_id, path_comp)

    path_comp.stored[1] = -0.15

    update_hybrid_link_states(world)

    assert path_comp.link_types[1] == 'hybrid-attachment'
    assert path_comp.stored[1] == pytest.approx(0.0)
    
    joint = world.get_component(joint_id, CableJointComponent)
    assert joint.rest_length == pytest.approx(initial_rest - 0.15)

def test_first_link_hybrid_attachment_to_hybrid():
    world = World()
    wheel = add_wheel(world, np.array([0.0, 0.0, 0.0]), 1.0)
    anchor = add_anchor(world, np.array([0.0, 3.0, 0.0]))

    joint_id = world.create_entity()
    initial_rest = 3.2
    world.add_component(joint_id, CableJointComponent(
        entity_a=wheel, entity_b=anchor, rest_length=initial_rest,
        attachment_point_a_world=np.array([1.0, 0.0, 0.0]), # start at 0 degrees
        attachment_point_b_world=np.array([0.0, 3.0, 0.0])
    ))

    path_id = world.create_entity()
    path_comp = CablePathComponent(
        joint_entities=[joint_id],
        link_types=['hybrid-attachment', 'attachment'],
        cw=[False, False], # initial cw is arbitrary
        stored=[0.0, 0.0]
    )
    path_comp.total_rest_length = initial_rest
    world.add_component(path_id, path_comp)

    update_hybrid_link_states(world)

    assert path_comp.link_types[0] == 'hybrid'
    assert path_comp.stored[0] > 0
    
    arc = path_comp.stored[0]
    joint = world.get_component(joint_id, CableJointComponent)
    assert joint.rest_length == pytest.approx(initial_rest - arc)

def test_last_link_hybrid_attachment_to_hybrid():
    world = World()
    anchor = add_anchor(world, np.array([0.0, -3.0, 0.0]))
    wheel = add_wheel(world, np.array([0.0, 0.0, 0.0]), 1.0)

    joint_id = world.create_entity()
    initial_rest = 3.1
    world.add_component(joint_id, CableJointComponent(
        entity_a=anchor, entity_b=wheel, rest_length=initial_rest,
        attachment_point_a_world=np.array([0.0, -3.0, 0.0]),
        attachment_point_b_world=np.array([1.0, 0.0, 0.0]) # start at 0 degrees
    ))

    path_id = world.create_entity()
    path_comp = CablePathComponent(
        joint_entities=[joint_id],
        link_types=['attachment', 'hybrid-attachment'],
        cw=[False, False],
        stored=[0.0, 0.0]
    )
    path_comp.total_rest_length = initial_rest
    world.add_component(path_id, path_comp)

    update_hybrid_link_states(world)

    assert path_comp.link_types[1] == 'hybrid'
    assert path_comp.stored[1] > 0
    
    arc = path_comp.stored[1]
    joint = world.get_component(joint_id, CableJointComponent)
    assert joint.rest_length == pytest.approx(initial_rest - arc)
