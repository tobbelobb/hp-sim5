import pytest
import numpy as np

from python.ecs import World, PositionComponent, RadiusComponent, CableLinkComponent, OrientationComponent
from python.cable_joints_components import CableJointComponent, CablePathComponent
from python.update_attachment_points import update_attachment_points
from python.geometry import (
    tangent_from_point_to_circle,
    tangent_from_circle_to_point,
    tangent_from_circle_to_circle,
    signed_arc_length_on_wheel
)

def test_attachment_to_rolling_translation():
    world = World()

    # Entities
    attach_point_id = world.create_entity()
    rolling_link_id = world.create_entity()

    # Components
    attach_pos = np.array([0.0, 2.0, 0.0])
    rolling_pos = np.array([0.0, 0.0, 0.0])
    rolling_radius = 1.0
    cw = True

    world.add_component(attach_point_id, PositionComponent(pos=attach_pos.copy()))
    link_comp_attach = CableLinkComponent()
    link_comp_attach.prev_cable_attachment_time_pos = attach_pos.copy()
    world.add_component(attach_point_id, link_comp_attach)

    world.add_component(rolling_link_id, PositionComponent(pos=rolling_pos.copy()))
    world.add_component(rolling_link_id, RadiusComponent(radius=rolling_radius))
    link_comp_rolling = CableLinkComponent()
    link_comp_rolling.prev_cable_attachment_time_pos = rolling_pos.copy()
    world.add_component(rolling_link_id, link_comp_rolling)

    # Initial Tangent
    initial_tangent = tangent_from_point_to_circle(attach_pos, rolling_pos, rolling_radius, cw)
    initial_attach_a = initial_tangent['a_attach']
    initial_attach_b = initial_tangent['a_circle']
    initial_rest_length = np.linalg.norm(initial_attach_a - initial_attach_b)

    # Cable Joint
    joint_id = world.create_entity()
    joint_comp = CableJointComponent(attach_point_id, rolling_link_id, initial_rest_length, initial_attach_a.copy(), initial_attach_b.copy())
    world.add_component(joint_id, joint_comp)

    # Cable Path
    path_id = world.create_entity()
    path_comp = CablePathComponent(
        joint_entities=[joint_id],
        link_types=['attachment', 'hybrid'],
        cw=[cw, cw],
        stored=[0.0, 0.0],
        total_rest_length=initial_rest_length
    )
    world.add_component(path_id, path_comp)
    initial_stored_b = path_comp.stored[1]
    assert initial_stored_b == pytest.approx(0.0)

    # --- Simulate Movement ---
    move_vector = np.array([0.5, 0.0, 0.0])
    new_attach_pos = attach_pos + move_vector
    world.get_component(attach_point_id, PositionComponent).pos = new_attach_pos

    update_attachment_points(world)

    # --- Assertions ---
    expected_tangent = tangent_from_point_to_circle(new_attach_pos, rolling_pos, rolling_radius, cw)
    expected_attach_a = expected_tangent['a_attach']
    expected_attach_b = expected_tangent['a_circle']

    np.testing.assert_allclose(joint_comp.attachment_point_a_world, expected_attach_a)
    np.testing.assert_allclose(joint_comp.attachment_point_b_world, expected_attach_b)

    expected_sB = signed_arc_length_on_wheel(
        initial_attach_b - rolling_pos,
        expected_attach_b - rolling_pos,
        np.zeros(3),
        rolling_radius,
        cw
    )
    assert path_comp.stored[1] == pytest.approx(initial_stored_b - expected_sB)
    assert joint_comp.rest_length == pytest.approx(initial_rest_length + expected_sB)

    final_total_rest_length = joint_comp.rest_length + path_comp.stored[0] + path_comp.stored[1]
    assert final_total_rest_length == pytest.approx(path_comp.total_rest_length)

def test_rolling_to_rolling_hybrid_translation():
    world = World()

    # Entities
    attach0_id = world.create_entity()
    rolling_a_id = world.create_entity()
    rolling_b_id = world.create_entity()

    # Components
    pos0 = np.array([-2.5, -2.0, 0.0])
    pos_a = np.array([-2.0, 0.0, 0.0])
    radius_a = 0.5
    cw_a = True
    pos_b = np.array([2.0, 0.0, 0.0])
    radius_b = 0.5
    cw_b = True

    world.add_component(attach0_id, PositionComponent(pos=pos0.copy()))
    link0 = CableLinkComponent(); link0.prev_cable_attachment_time_pos = pos0.copy()
    world.add_component(attach0_id, link0)

    world.add_component(rolling_a_id, PositionComponent(pos=pos_a.copy()))
    world.add_component(rolling_a_id, RadiusComponent(radius=radius_a))
    linkA = CableLinkComponent(); linkA.prev_cable_attachment_time_pos = pos_a.copy()
    world.add_component(rolling_a_id, linkA)

    world.add_component(rolling_b_id, PositionComponent(pos=pos_b.copy()))
    world.add_component(rolling_b_id, RadiusComponent(radius=radius_b))
    linkB = CableLinkComponent(); linkB.prev_cable_attachment_time_pos = pos_b.copy()
    world.add_component(rolling_b_id, linkB)

    # Initial Tangents and joint attach->rolling
    initial_tangents_0 = tangent_from_point_to_circle(pos0, pos_a, radius_a, cw_a)
    initial_rest_length_0 = np.linalg.norm(initial_tangents_0['a_attach'] - initial_tangents_0['a_circle'])
    joint_id_0 = world.create_entity()
    joint_comp_0 = CableJointComponent(attach0_id, rolling_a_id, initial_rest_length_0, initial_tangents_0['a_attach'], initial_tangents_0['a_circle'])
    world.add_component(joint_id_0, joint_comp_0)

    # Initial Tangents Rolling links
    initial_tangents = tangent_from_circle_to_circle(pos_a, radius_a, cw_a, pos_b, radius_b, cw_b)
    initial_attach_a = initial_tangents['a_circle']
    initial_attach_b = initial_tangents['b_circle']
    initial_rest_length = np.linalg.norm(initial_attach_a - initial_attach_b)

    joint_id = world.create_entity()
    joint_comp = CableJointComponent(rolling_a_id, rolling_b_id, initial_rest_length, initial_attach_a.copy(), initial_attach_b.copy())
    world.add_component(joint_id, joint_comp)

    # Cable Path
    path_id = world.create_entity()

    # Manual path setup, replicating JS constructor logic
    stored_on_a = signed_arc_length_on_wheel(joint_comp_0.attachment_point_b_world, joint_comp.attachment_point_a_world, pos_a, radius_a, cw_a, True)
    total_rest = joint_comp_0.rest_length + joint_comp.rest_length + stored_on_a

    path_comp = CablePathComponent(
        joint_entities=[joint_id_0, joint_id],
        link_types=['attachment', 'rolling', 'hybrid'],
        cw=[True, cw_a, cw_b],
        stored=[0.0, stored_on_a, 0.0],
        total_rest_length=total_rest
    )
    world.add_component(path_id, path_comp)
    initial_stored_a = path_comp.stored[1]
    initial_stored_b = path_comp.stored[2]

    # --- Simulate Movement ---
    move_vector_a = np.array([0.0, 0.1, 0.0])
    new_pos_a = pos_a + move_vector_a
    world.get_component(rolling_a_id, PositionComponent).pos = new_pos_a

    update_attachment_points(world)

    # --- Assertions ---
    expected_tangents = tangent_from_circle_to_circle(new_pos_a, radius_a, cw_a, pos_b, radius_b, cw_b)
    expected_attach_a = expected_tangents['a_circle']
    expected_attach_b = expected_tangents['b_circle']

    np.testing.assert_allclose(joint_comp.attachment_point_a_world, expected_attach_a)
    np.testing.assert_allclose(joint_comp.attachment_point_b_world, expected_attach_b)

    expected_sA = signed_arc_length_on_wheel(initial_attach_a - pos_a, expected_attach_a - new_pos_a, np.zeros(3), radius_a, cw_a)
    expected_sB = signed_arc_length_on_wheel(initial_attach_b - pos_b, expected_attach_b - pos_b, np.zeros(3), radius_b, cw_b)

    assert path_comp.stored[1] == pytest.approx(initial_stored_a + expected_sA)
    assert path_comp.stored[2] == pytest.approx(initial_stored_b - expected_sB)
    assert joint_comp.rest_length == pytest.approx(initial_rest_length - expected_sA + expected_sB)

    final_total_rest_length = joint_comp_0.rest_length + joint_comp.rest_length + path_comp.stored[0] + path_comp.stored[1] + path_comp.stored[2]
    assert final_total_rest_length == pytest.approx(path_comp.total_rest_length)

def test_hybrid_attachment_to_hybrid_with_rotations():
    world = World()
    r = 0.5
    start_attach_id = world.create_entity()
    roll_a_id = world.create_entity()
    roll_b_id = world.create_entity()
    end_hybrid_id = world.create_entity()
    cw_start, cw_a, cw_b, cw_end = True, True, False, True
    small_rot = 0.1

    pos_start = np.array([-2.5, -2.0, 0.0])
    pos_a = np.array([-2.0, 0.0, 0.0])
    pos_b = np.array([2.0, 0.0, 0.0])
    pos_end = np.array([2.5, 2.0, 0.0])

    for e, pos, r_ in [(start_attach_id, pos_start, r), (roll_a_id, pos_a, r), (roll_b_id, pos_b, r), (end_hybrid_id, pos_end, r)]:
        world.add_component(e, PositionComponent(pos=pos.copy()))
        world.add_component(e, OrientationComponent(angle=0.0))
        world.add_component(e, RadiusComponent(radius=r_))
        link = CableLinkComponent(); link.prev_cable_attachment_time_pos = pos.copy()
        world.add_component(e, link)

    t0_tangent = tangent_from_point_to_circle(pos_start, pos_a, r, cw_a)
    t0_dir = (t0_tangent['a_circle'] - t0_tangent['a_attach'])
    t0_dir /= np.linalg.norm(t0_dir)
    t0_hybrid_attach = pos_start + t0_dir * r
    t1 = tangent_from_circle_to_circle(pos_a, r, cw_a, pos_b, r, cw_b)
    t2 = tangent_from_circle_to_circle(pos_b, r, cw_b, pos_end, r, cw_end)

    t_args = [
        (start_attach_id, roll_a_id, t0_hybrid_attach, t0_tangent['a_circle']),
        (roll_a_id, roll_b_id, t1['a_circle'], t1['b_circle']),
        (roll_b_id, end_hybrid_id, t2['a_circle'], t2['b_circle']),
    ]
    joint_ids = []
    for a, b, apt, bpt in t_args:
        jid = world.create_entity()
        world.add_component(jid, CableJointComponent(a, b, np.linalg.norm(apt - bpt), apt.copy(), bpt.copy()))
        joint_ids.append(jid)

    # Manual path setup
    j0, j1, j2 = [world.get_component(jid, CableJointComponent) for jid in joint_ids]
    s1 = signed_arc_length_on_wheel(j0.attachment_point_b_world, j1.attachment_point_a_world, pos_a, r, cw_a, True)
    s2 = signed_arc_length_on_wheel(j1.attachment_point_b_world, j2.attachment_point_a_world, pos_b, r, cw_b, True)
    total_rest = j0.rest_length + j1.rest_length + j2.rest_length + s1 + s2

    path_comp = CablePathComponent(
        joint_entities=joint_ids,
        link_types=['hybrid-attachment', 'rolling', 'rolling', 'hybrid'],
        cw=[cw_start, cw_a, cw_b, cw_end],
        stored=[0.0, s1, s2, 0.0],
        total_rest_length=total_rest
    )
    world.add_component(world.create_entity(), path_comp)
    initial_stored_a, initial_stored_b = s1, s2

    world.get_component(start_attach_id, OrientationComponent).angle += small_rot
    world.get_component(end_hybrid_id, OrientationComponent).angle += -small_rot

    update_attachment_points(world)

    assert path_comp.stored[0] == pytest.approx(0.0)
    assert path_comp.stored[1] < initial_stored_a
    assert path_comp.stored[2] == pytest.approx(initial_stored_b)
    assert path_comp.stored[3] == pytest.approx(r * small_rot)

    np.testing.assert_allclose(np.linalg.norm(j0.attachment_point_a_world - pos_start), r, atol=1e-6)
    np.testing.assert_allclose(np.linalg.norm(j0.attachment_point_b_world - pos_a), r, atol=1e-6)
    np.testing.assert_allclose(np.linalg.norm(j2.attachment_point_b_world - pos_end), r, atol=1e-6)

def test_hybrid_to_hybrid_attachment_with_rotations():
    world = World()
    r = 0.5
    start_hybrid_id = world.create_entity()
    roll_a_id = world.create_entity()
    roll_b_id = world.create_entity()
    end_hybrid_attach_id = world.create_entity()
    cw_start, cw_a, cw_b, cw_end = False, True, False, True
    small_rot = 0.1

    pos_start = np.array([-3.0, -2.0, 0.0])
    pos_a = np.array([-2.0, 0.0, 0.0])
    pos_b = np.array([2.0, 0.0, 0.0])
    pos_end = np.array([2.5, 2.0, 0.0])

    for e, pos, r_ in [(start_hybrid_id, pos_start, r), (roll_a_id, pos_a, r), (roll_b_id, pos_b, r), (end_hybrid_attach_id, pos_end, r)]:
        world.add_component(e, PositionComponent(pos=pos.copy()))
        world.add_component(e, OrientationComponent(angle=0.0))
        world.add_component(e, RadiusComponent(radius=r_))
        link = CableLinkComponent(); link.prev_cable_attachment_time_pos = pos.copy()
        world.add_component(e, link)

    # Note: !cw_start due to _effective_cw logic for first link
    t0 = tangent_from_circle_to_circle(pos_start, r, not cw_start, pos_a, r, cw_a)
    t1 = tangent_from_circle_to_circle(pos_a, r, cw_a, pos_b, r, cw_b)
    t2_tangent = tangent_from_circle_to_point(pos_end, pos_b, r, cw_b)
    t2_dir = t2_tangent['a_circle'] - t2_tangent['a_attach']
    t2_dir /= np.linalg.norm(t2_dir)
    t2_hybrid_attach = pos_end + t2_dir * r

    t_args = [
        (start_hybrid_id, roll_a_id, t0['a_circle'], t0['b_circle']),
        (roll_a_id, roll_b_id, t1['a_circle'], t1['b_circle']),
        (roll_b_id, end_hybrid_attach_id, t2_tangent['a_circle'], t2_hybrid_attach),
    ]
    joint_ids = []
    for a, b, apt, bpt in t_args:
        jid = world.create_entity()
        world.add_component(jid, CableJointComponent(a, b, np.linalg.norm(apt - bpt), apt.copy(), bpt.copy()))
        joint_ids.append(jid)

    j0, j1, j2 = [world.get_component(jid, CableJointComponent) for jid in joint_ids]
    s1 = signed_arc_length_on_wheel(j0.attachment_point_b_world, j1.attachment_point_a_world, pos_a, r, cw_a, True)
    s2 = signed_arc_length_on_wheel(j1.attachment_point_b_world, j2.attachment_point_a_world, pos_b, r, cw_b, True)
    total_rest = j0.rest_length + j1.rest_length + j2.rest_length + s1 + s2

    path_comp = CablePathComponent(
        joint_entities=joint_ids,
        link_types=['hybrid', 'rolling', 'rolling', 'hybrid-attachment'],
        cw=[cw_start, cw_a, cw_b, cw_end],
        stored=[0.0, s1, s2, 0.0],
        total_rest_length=total_rest
    )
    world.add_component(world.create_entity(), path_comp)
    initial_stored_a, initial_stored_b = s1, s2

    world.get_component(start_hybrid_id, OrientationComponent).angle += small_rot
    world.get_component(end_hybrid_attach_id, OrientationComponent).angle += -small_rot

    update_attachment_points(world)

    assert path_comp.stored[0] > 0
    assert path_comp.stored[0] == pytest.approx(r * small_rot)
    assert path_comp.stored[1] == pytest.approx(initial_stored_a)
    assert path_comp.stored[2] > initial_stored_b

    np.testing.assert_allclose(np.linalg.norm(j2.attachment_point_b_world - pos_end), r, atol=1e-6)
    np.testing.assert_allclose(np.linalg.norm(j2.attachment_point_a_world - pos_b), r, atol=1e-6)
    np.testing.assert_allclose(np.linalg.norm(j0.attachment_point_a_world - pos_start), r, atol=1e-6)
