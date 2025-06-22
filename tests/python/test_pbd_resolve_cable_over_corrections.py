import pytest
import numpy as np

from cable_joints.ecs import (
    World,
    PositionComponent,
    MassComponent,
    MomentOfInertiaComponent,
    OrientationComponent,
    RadiusComponent,
)
from cable_joints.cable_joints_components import (
    CableJointComponent,
    CablePathComponent,
    CableLinkComponent,
    create_cable_path_component,
)
from cable_joints.pbd_resolve_cable_over_corrections import PBDResolveCableOverCorrections
from cable_joints.cable_attachment_cache_system import CableAttachmentCacheSystem
from cable_joints.update_attachment_points import update_attachment_points


def test_resolve_single_over_corrected_joint_is_ignored():
    """
    Tests that if only a single joint in a path is over-corrected, no
    correction is applied, as it requires at least two joints pulling on
    the same object.
    """
    world = World()
    dt = 1.0 / 60.0
    world.set_resource("dt", dt)

    # --- Setup Entities ---
    entity_a = world.create_entity()
    world.add_component(entity_a, PositionComponent(np.array([0.0, 0.0])))
    world.add_component(entity_a, MassComponent(-1.0)) # Static
    world.add_component(entity_a, CableLinkComponent())

    entity_b = world.create_entity()
    initial_pos_b = np.array([8.0, 0.0])
    world.add_component(entity_b, PositionComponent(initial_pos_b.copy()))
    world.add_component(entity_b, MassComponent(1.0))
    world.add_component(entity_b, MomentOfInertiaComponent(1.0))
    world.add_component(entity_b, OrientationComponent(0.0))
    world.add_component(entity_b, RadiusComponent(1.0))
    world.add_component(entity_b, CableLinkComponent())

    # --- Setup Cable ---
    # A single joint that is made to look over-corrected.
    joint_entity = world.create_entity()
    joint = CableJointComponent(
        entity_a=entity_a,
        entity_b=entity_b,
        rest_length=10.0,
        attachment_point_a_world=np.array([0.0, 0.0]),
        attachment_point_b_world=np.array([7.0, 0.0]),
    )
    world.add_component(joint_entity, joint)

    path_entity = world.create_entity()
    path_comp = create_cable_path_component(world, [joint_entity], ['attachment', 'rolling'], [False, False])
    world.add_component(path_entity, path_comp)

    # --- Simulate Pre-Solver State ---
    # Manually set attachment points to a "stretched" state for the cache.
    joint.attachment_point_a_world = np.array([0.0, 0.0])
    joint.attachment_point_b_world = np.array([11.0, 0.0]) # Stretched state

    cache_system = CableAttachmentCacheSystem()
    cache_system.update(world, dt)

    # --- Run the System Under Test ---
    resolve_system = PBDResolveCableOverCorrections()
    resolve_system.update(world, dt)

    # --- Assertions ---
    # With only one over-corrected joint, no correction should be applied.
    final_pos_comp_b = world.get_component(entity_b, PositionComponent)
    assert np.allclose(final_pos_comp_b.pos, initial_pos_b), \
        "System should not apply correction for a single over-corrected joint."


def test_two_over_corrected_joints_on_one_body_push_it_apart():
    """
    Tests that two over-corrected joints on a single body cause a
    positional correction to push them apart.
    """
    world = World()
    dt = 1.0 / 60.0
    world.set_resource("dt", dt)

    # --- Setup Entities ---
    entity_a = world.create_entity()
    world.add_component(entity_a, PositionComponent(np.array([0.0, 0.0])))
    world.add_component(entity_a, MassComponent(-1.0))
    world.add_component(entity_a, CableLinkComponent())

    entity_b = world.create_entity()
    initial_pos_b = np.array([8.0, 0.0])
    world.add_component(entity_b, PositionComponent(initial_pos_b.copy()))
    world.add_component(entity_b, MassComponent(1.0))
    world.add_component(entity_b, MomentOfInertiaComponent(1.0))
    world.add_component(entity_b, OrientationComponent(0.0))
    world.add_component(entity_b, RadiusComponent(1.0))
    world.add_component(entity_b, CableLinkComponent())

    entity_c = world.create_entity()
    world.add_component(entity_c, PositionComponent(np.array([16.0, 0.0])))
    world.add_component(entity_c, MassComponent(-1.0))
    world.add_component(entity_c, CableLinkComponent())

    # --- Setup Cable ---
    joint1_entity = world.create_entity()
    j1 = CableJointComponent(
        entity_a=entity_a,
        entity_b=entity_b,
        rest_length=10.0,
        attachment_point_a_world=np.array([0.0, 0.0]),
        attachment_point_b_world=np.array([7.0, 0.0]),
    )
    world.add_component(joint1_entity, j1)

    joint2_entity = world.create_entity()
    j2 = CableJointComponent(
        entity_a=entity_b,
        entity_b=entity_c,
        rest_length=10.0,
        attachment_point_a_world=np.array([9.0, 0.0]),
        attachment_point_b_world=np.array([16.0, 0.0]),
    )
    world.add_component(joint2_entity, j2)

    path_entity = world.create_entity()
    path_comp = create_cable_path_component(
        world,
        [joint1_entity, joint2_entity],
        ['fixed', 'rolling', 'fixed'],
        [False, False, False]
    )
    world.add_component(path_entity, path_comp)

    # --- Simulate Pre-Solver State ---
    update_attachment_points(world)

    # Manually set attachment points to a "stretched" state for the cache.
    j1.attachment_point_a_world = np.array([0.0, 0.0])
    j1.attachment_point_b_world = np.array([11.0, 0.0])  # Stretched: length is 11 > 10

    # Stretched: C is at (16,0), set attachment on B to (5,0). Length is |16-5|=11 > 10
    j2.attachment_point_a_world = np.array([5.0, 0.0])
    j2.attachment_point_b_world = np.array([16.0, 0.0])

    cache_system = CableAttachmentCacheSystem()
    cache_system.update(world, dt)

    # --- Run the System Under Test ---
    resolve_system = PBDResolveCableOverCorrections()
    resolve_system.update(world, dt)

    # --- Assertions ---
    final_pos_comp_b = world.get_component(entity_b, PositionComponent)
    assert final_pos_comp_b.pos[0] > initial_pos_b[0], \
        "System should apply a correction to push the body."
