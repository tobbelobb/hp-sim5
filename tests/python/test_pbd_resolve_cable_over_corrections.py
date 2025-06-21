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
        attachment_point_a_world=np.array([0.0, 0.0, 0.0]),
        attachment_point_b_world=np.array([7.0, 0.0, 0.0]),
    )
    world.add_component(joint_entity, joint)

    path_entity = world.create_entity()
    path_comp = create_cable_path_component(world, [joint_entity], ['attachment', 'rolling'], [False, False])
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
