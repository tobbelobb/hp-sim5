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
    world.add_component(entity_a, PositionComponent(np.array([0.0, 0.0, 0.0])))
    world.add_component(entity_a, MassComponent(-1.0)) # Static
    world.add_component(entity_a, CableLinkComponent())

    entity_b = world.create_entity()
    initial_pos_b = np.array([8.0, 0.0, 0.0])
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
        attachment_a_local=np.array([0.0, 0.0, 0.0]),
        attachment_b_local=np.array([-1.0, 0.0, 0.0]),
    )
    world.add_component(joint_entity, joint)

    path_entity = world.create_entity()
    path_comp = create_cable_path_component(world, [joint_entity], ['fixed', 'rolling'], [False, False], 0.0)
    world.add_component(path_entity, path_comp)

    # --- Simulate Pre-Solver State ---
    # Manually set attachment points to a "stretched" state for the cache.
    joint.attachment_point_a_world = np.array([0.0, 0.0, 0.0])
    joint.attachment_point_b_world = np.array([11.0, 0.0, 0.0]) # Stretched state
    
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


def test_resolve_two_over_corrections_on_one_body_pushes_apart():
    """
    Tests that PBDResolveCableOverCorrections correctly applies a "push"
    correction when two joints in the same path pull on the same object,
    causing it to become slack.
    """
    world = World()
    dt = 1.0 / 60.0
    world.set_resource("dt", dt)

    # --- Setup Entities ---
    # Static anchor A at the origin
    entity_a = world.create_entity()
    world.add_component(entity_a, PositionComponent(np.array([0.0, 0.0, 0.0])))
    world.add_component(entity_a, MassComponent(-1.0))
    world.add_component(entity_a, CableLinkComponent())

    # Dynamic body B, positioned so the cable is slack
    entity_b = world.create_entity()
    initial_pos_b = np.array([8.0, 0.0, 0.0])
    world.add_component(entity_b, PositionComponent(initial_pos_b.copy()))
    world.add_component(entity_b, MassComponent(1.0))
    world.add_component(entity_b, MomentOfInertiaComponent(1.0))
    world.add_component(entity_b, OrientationComponent(0.0))
    world.add_component(entity_b, RadiusComponent(1.0))
    world.add_component(entity_b, CableLinkComponent())

    # Static anchor C
    entity_c = world.create_entity()
    world.add_component(entity_c, PositionComponent(np.array([16.0, 0.0, 0.0])))
    world.add_component(entity_c, MassComponent(-1.0))
    world.add_component(entity_c, CableLinkComponent())

    # --- Setup Cable Path: A -> B -> C ---
    joint1_id = world.create_entity()
    world.add_component(joint1_id, CableJointComponent(
        entity_a=entity_a, entity_b=entity_b, rest_length=10.0,
        attachment_a_local=np.array([0.0, 0.0, 0.0]),
        attachment_b_local=np.array([-1.0, 0.0, 0.0])
    ))

    joint2_id = world.create_entity()
    world.add_component(joint2_id, CableJointComponent(
        entity_a=entity_b, entity_b=entity_c, rest_length=10.0,
        attachment_a_local=np.array([1.0, 0.0, 0.0]),
        attachment_b_local=np.array([0.0, 0.0, 0.0])
    ))

    path_entity = world.create_entity()
    path_comp = create_cable_path_component(world, [joint1_id, joint2_id], ['fixed', 'rolling', 'fixed'], [False, False, False], 0.0)
    world.add_component(path_entity, path_comp)

    # --- Simulate Pre-Solver State ---
    # First, run update_attachment_points to populate the initial world points
    update_attachment_points(world)
    
    # Now, manually set the cached attachment points to a "stretched" state.
    # This simulates the state before the main solver ran.
    j1_comp = world.get_component(joint1_id, CableJointComponent)
    j1_comp.attachment_point_a_world = np.array([0.0, 0.0, 0.0])
    j1_comp.attachment_point_b_world = np.array([11.0, 0.0, 0.0])
    
    j2_comp = world.get_component(joint2_id, CableJointComponent)
    j2_comp.attachment_point_a_world = np.array([11.0, 0.0, 0.0])
    j2_comp.attachment_point_b_world = np.array([16.0, 0.0, 0.0])

    cache_system = CableAttachmentCacheSystem()
    cache_system.update(world, dt)

    # --- Run the System Under Test ---
    # The system will see that both joints were taut (from cache) but are now
    # slack (based on current body positions), and apply a push correction.
    resolve_system = PBDResolveCableOverCorrections()
    resolve_system.update(world, dt)

    # --- Assertions ---
    final_pos_comp_b = world.get_component(entity_b, PositionComponent)

    # The actual current attachment points for B are (7,0) and (9,0).
    # Distances are 7 and 7, both < 10. Both joints are over-corrected.
    # The system should push B to resolve the slack.
    assert final_pos_comp_b.pos[0] > initial_pos_b[0], \
        "Entity B should be pushed away from the anchors to remove slack."
