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

def test_resolve_over_correction_pushes_apart():
    """
    Tests that PBDResolveCableOverCorrections correctly applies a "push"
    correction to a joint that is slack, moving the connected bodies apart.
    """
    world = World()
    dt = 1.0 / 60.0
    world.set_resource("dt", dt)

    # --- Setup Entities ---
    # Entity A: A static anchor point at the origin
    entity_a = world.create_entity()
    world.add_component(entity_a, PositionComponent(np.array([0.0, 0.0, 0.0])))
    world.add_component(entity_a, MassComponent(-1.0))
    world.add_component(entity_a, CableLinkComponent())

    # Entity B: A dynamic body, positioned so the cable is slack
    entity_b = world.create_entity()
    initial_pos_b = np.array([8.0, 0.0, 0.0])
    world.add_component(entity_b, PositionComponent(initial_pos_b.copy()))
    world.add_component(entity_b, MassComponent(1.0))
    world.add_component(entity_b, MomentOfInertiaComponent(1.0))
    world.add_component(entity_b, OrientationComponent(0.0))
    world.add_component(entity_b, RadiusComponent(1.0))
    world.add_component(entity_b, CableLinkComponent())

    # --- Setup Cable ---
    # A single joint with rest length 10.
    # The attachment on A is its center (0,0).
    # The attachment on B is its left-most point (-1,0) locally.
    # With B at (8,0), the world attachment point is (7,0).
    # The initial distance is 7, which is < rest_length, so it's slack.
    joint_entity1 = world.create_entity()
    joint1 = CableJointComponent(
        entity_a=entity_a,
        entity_b=entity_b,
        rest_length=10.0,
        attachment_point_a_world=np.array([0.0, 0.0, 0.0]),
        attachment_point_b_world=np.array([7.0, 0.0, 0.0]),
    )
    world.add_component(joint_entity1, joint1)
    joint_entity2 = world.create_entity()
    joint2 = CableJointComponent(
        entity_a=entity_b,
        entity_b=entity_a,
        rest_length=10.0,
        attachment_point_a_world=np.array([7.0, 0.0, 0.0]),
        attachment_point_b_world=np.array([0.0, 0.0, 0.0]),
    )
    world.add_component(joint_entity2, joint2)

    path_entity = world.create_entity()
    path_comp = create_cable_path_component(world, [joint_entity1, joint_entity2], ['attachment', 'rolling', 'attachment'], [False, False, False])
    world.add_component(path_entity, path_comp)

    # --- Simulate Pre-Solver State ---
    # To trigger the over-correction logic, the system must believe the joint
    # *was* taut before the main solver ran. We simulate this by manually
    # setting the world attachment points to a stretched state and caching them.
    joint1.attachment_point_a_world = np.array([0.0, 0.0, 0.0])
    joint1.attachment_point_b_world = np.array([11.0, 0.0, 0.0]) # Stretched state
    joint2.attachment_point_a_world = np.array([11.0, 0.0, 0.0])
    joint2.attachment_point_b_world = np.array([0.0, 0.0, 0.0]) # Stretched state

    cache_system = CableAttachmentCacheSystem()
    cache_system.update(world, dt)

    # --- Run the System Under Test ---
    # The system should now see a joint that was taut (from cache) but is
    # now slack (based on current body positions), and apply a push correction.
    resolve_system = PBDResolveCableOverCorrections()
    resolve_system.update(world, dt)

    # --- Assertions ---
    final_pos_comp_b = world.get_component(entity_b, PositionComponent)
    final_orientation_comp_b = world.get_component(entity_b, OrientationComponent)

    # 1. The primary assertion: Entity B should have been pushed to the right.
    assert final_pos_comp_b.pos[0] > initial_pos_b[0], \
        "Entity B should be pushed away from Entity A to remove slack."

    # 2. The distance between attachment points should have increased.
    # The new attachment point on B is its center + rotated local attachment.
    # Since angle is 0, it's (final_pos_x - 1, 0, 0).
    initial_distance = 7.0
    final_attachment_b = final_pos_comp_b.pos + np.array([-1.0, 0, 0])
    final_distance = np.linalg.norm(final_attachment_b - joint1.attachment_point_a_world)
    assert final_distance > initial_distance

    # 3. In this symmetrical setup, there should be no vertical movement or rotation.
    assert final_pos_comp_b.pos[1] == pytest.approx(0.0)
    assert final_orientation_comp_b.angle == pytest.approx(0.0)

def test_resolve_over_correction_pushes_apart_multiple_paths():
    """
    Tests that PBDResolveCableOverCorrections correctly applies a "push"
    correction to a joint that is slack, moving the connected bodies apart,
    even if the pull came from different cable paths.
    """
    world = World()
    dt = 1.0 / 60.0
    world.set_resource("dt", dt)

    # --- Setup Entities ---
    # Entity A: A static anchor point at the origin
    entity_a = world.create_entity()
    world.add_component(entity_a, PositionComponent(np.array([0.0, 0.0, 0.0])))
    world.add_component(entity_a, MassComponent(-1.0))
    world.add_component(entity_a, CableLinkComponent())

    # Entity B: A dynamic body, positioned so the cable is slack
    entity_b = world.create_entity()
    initial_pos_b = np.array([8.0, 0.0, 0.0])
    world.add_component(entity_b, PositionComponent(initial_pos_b.copy()))
    world.add_component(entity_b, MassComponent(1.0))
    world.add_component(entity_b, MomentOfInertiaComponent(1.0))
    world.add_component(entity_b, OrientationComponent(0.0))
    world.add_component(entity_b, RadiusComponent(1.0))
    world.add_component(entity_b, CableLinkComponent())

    # --- Setup Cable ---
    # A single joint with rest length 10.
    # The attachment on A is its center (0,0).
    # The attachment on B is its left-most point (-1,0) locally.
    # With B at (8,0), the world attachment point is (7,0).
    # The initial distance is 7, which is < rest_length, so it's slack.
    joint_entity1 = world.create_entity()
    joint1 = CableJointComponent(
        entity_a=entity_a,
        entity_b=entity_b,
        rest_length=10.0,
        attachment_point_a_world=np.array([0.0, 0.0, 0.0]),
        attachment_point_b_world=np.array([7.0, 0.0, 0.0]),
    )
    world.add_component(joint_entity1, joint1)
    joint_entity2 = world.create_entity()
    joint2 = CableJointComponent(
        entity_a=entity_b,
        entity_b=entity_a,
        rest_length=10.0,
        attachment_point_a_world=np.array([7.0, 0.0, 0.0]),
        attachment_point_b_world=np.array([0.0, 0.0, 0.0]),
    )
    world.add_component(joint_entity2, joint2)

    path_entity1 = world.create_entity()
    path_comp1 = create_cable_path_component(world, [joint_entity1], ['attachment', 'attachment'], [False, False])
    world.add_component(path_entity1, path_comp1)
    path_entity2 = world.create_entity()
    path_comp2 = create_cable_path_component(world, [joint_entity2], ['attachment', 'attachment'], [False, False])
    world.add_component(path_entity2, path_comp2)

    # --- Simulate Pre-Solver State ---
    # To trigger the over-correction logic, the system must believe the joint
    # *was* taut before the main solver ran. We simulate this by manually
    # setting the world attachment points to a stretched state and caching them.
    joint1.attachment_point_a_world = np.array([0.0, 0.0, 0.0])
    joint1.attachment_point_b_world = np.array([11.0, 0.0, 0.0]) # Stretched state
    joint2.attachment_point_a_world = np.array([11.0, 0.0, 0.0])
    joint2.attachment_point_b_world = np.array([0.0, 0.0, 0.0]) # Stretched state

    cache_system = CableAttachmentCacheSystem()
    cache_system.update(world, dt)

    # --- Run the System Under Test ---
    # The system should now see a joint that was taut (from cache) but is
    # now slack (based on current body positions), and apply a push correction.
    resolve_system = PBDResolveCableOverCorrections()
    resolve_system.update(world, dt)

    # --- Assertions ---
    final_pos_comp_b = world.get_component(entity_b, PositionComponent)
    final_orientation_comp_b = world.get_component(entity_b, OrientationComponent)

    # 1. The primary assertion: Entity B should have been pushed to the right.
    assert final_pos_comp_b.pos[0] > initial_pos_b[0], \
        "Entity B should be pushed away from Entity A to remove slack."

    # 2. The distance between attachment points should have increased.
    # The new attachment point on B is its center + rotated local attachment.
    # Since angle is 0, it's (final_pos_x - 1, 0, 0).
    initial_distance = 7.0
    final_attachment_b = final_pos_comp_b.pos + np.array([-1.0, 0, 0])
    final_distance = np.linalg.norm(final_attachment_b - joint1.attachment_point_a_world)
    assert final_distance > initial_distance
    final_distance2 = np.linalg.norm(final_attachment_b - joint2.attachment_point_b_world)
    assert final_distance2 > initial_distance

    # 3. In this symmetrical setup, there should be no vertical movement or rotation.
    assert final_pos_comp_b.pos[1] == pytest.approx(0.0)
    assert final_orientation_comp_b.angle == pytest.approx(0.0)


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
