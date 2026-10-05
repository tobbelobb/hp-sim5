"""Mutable ownership invariants that numeric snapshots alone cannot observe."""
import numpy as np

from cable_joints_3d.cable_attachment_cache_system import CableAttachmentCacheSystem
from cable_joints_3d.cable_attachment_update_system import update_attachment_points
from cable_joints_3d.cable_joints_components import CableJointComponent, CableLinkComponent, create_cable_path_component
from cable_joints_3d.ecs import OrientationComponent, PositionComponent, World
from cable_joints_3d.quaternion import Quaternion


def test_cable_link_copies_constructor_scratch_frames():
    orientation = Quaternion(.2, .3, .4, .8)
    plane, local_plane = np.array([1., 2., 3.]), np.array([0., 2., 0.])
    link = CableLinkComponent(1, 2, 3, orientation, plane, local_plane)
    saved = link.prev_cable_attachment_time_orientation.as_xyzw().copy()
    orientation.x = 50
    plane[:] = 0
    local_plane[:] = 0
    assert np.array_equal(link.prev_cable_attachment_time_orientation.as_xyzw(), saved)
    assert np.array_equal(link.prev_cable_attachment_time_local_orientation.as_xyzw(), saved)
    assert np.array_equal(link.cable_plane_normal, [1, 2, 3])
    assert np.array_equal(link.cable_plane_normal_local, [0, 1, 0])
    link.prev_cable_attachment_time_orientation.x = 30
    assert np.array_equal(link.prev_cable_attachment_time_local_orientation.as_xyzw(), saved)


def test_joint_attachments_and_force_storage_are_independent():
    point = np.array([1., 2., 3.])
    first = CableJointComponent.from_world(0, 1, 1, point, point)
    second = CableJointComponent.from_world(0, 1, 1, point, point)
    point[:] = 0
    first.attachment_point_a_world[:] = 10
    first.constraint_force[:] = 4
    assert np.array_equal(first.attachment_point_b_world, [1, 2, 3])
    assert np.array_equal(second.attachment_point_a_world, [1, 2, 3])
    assert np.array_equal(second.constraint_force, [0, 0, 0])


def test_cache_updates_existing_frame_objects_without_aliasing_live_pose():
    world = World()
    entity = world.create_entity()
    position = PositionComponent(1, 2, 3)
    orientation = OrientationComponent(.2, .3, .4, .8)
    link = CableLinkComponent()
    for component in (position, orientation, link):
        world.add_component(entity, component)
    previous_position = link.prev_cable_attachment_time_pos
    previous_orientation = link.prev_cable_attachment_time_orientation
    previous_local_orientation = link.prev_cable_attachment_time_local_orientation
    CableAttachmentCacheSystem().update(world, .002)
    assert link.prev_cable_attachment_time_pos is previous_position
    assert link.prev_cable_attachment_time_orientation is previous_orientation
    assert link.prev_cable_attachment_time_local_orientation is previous_local_orientation
    snapshot = previous_orientation.as_xyzw().copy()
    position.pos[:] = 0
    orientation.quaternion.x = 40
    assert np.array_equal(previous_position, [1, 2, 3])
    assert np.array_equal(previous_orientation.as_xyzw(), snapshot)


def test_attachment_rebuild_preserves_storage_and_does_not_alias_live_positions():
    world = World()
    a, b, joint_entity, path_entity = [world.create_entity() for _ in range(4)]
    for entity in (a, b):
        world.add_component(entity, PositionComponent(entity, 0, 0))
        world.add_component(entity, CableLinkComponent(entity, 0, 0))
    joint = CableJointComponent.from_world(a, b, 2, np.zeros(3), np.ones(3))
    world.add_component(joint_entity, joint)
    world.add_component(path_entity, create_cable_path_component(
        world, [joint_entity], ['attachment', 'attachment'], [False, False]))
    first, second = joint.attachment_point_a_world, joint.attachment_point_b_world
    update_attachment_points(world)
    assert joint.attachment_point_a_world is first
    assert joint.attachment_point_b_world is second
    saved = first.copy()
    world.get_component(a, PositionComponent).pos[:] = 100
    assert np.array_equal(first, saved)
