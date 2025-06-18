import pytest
import numpy as np

# Use absolute imports from the 'python' package root.
from cable_joints.pbd_cable_constraint_solver import PBDCableConstraintSolver
from cable_joints.cable_attachment_update_system import CableAttachmentUpdateSystem
from cable_joints.ecs import (
    World, PositionComponent, VelocityComponent, MassComponent, MomentOfInertiaComponent,
    OrientationComponent, AngularVelocityComponent, CableLinkComponent,
    GravityAffectedComponent, RadiusComponent
)
from cable_joints.cable_joints_components import CableJointComponent, CablePathComponent, create_cable_path_component

# --- Mocks for Testing ---

class MockGravitySystem:
    """A mock system that applies gravity to entities."""
    def update(self, world, dt):
        gravity = world.get_resource('gravity')
        if gravity is None:
            return
        entities = world.query([GravityAffectedComponent])
        for entity_id in entities:
            vel_comp = world.get_component(entity_id, VelocityComponent)
            if vel_comp:
                vel_comp.vel += gravity * dt

# --- Tests for PBDCableConstraintSolver ---

def test_does_nothing_when_compliance_is_zero():
    """
    Tests that the solver makes no changes when compliance is zero
    (i.e., spring_constant is infinite).
    """
    world = World()
    e0, e1, e2 = world.create_entity(), world.create_entity(), world.create_entity()
    world.add_component(e0, PositionComponent(np.array([0.0, 0.0, 0.0])))
    world.add_component(e1, PositionComponent(np.array([1.0, 0.0, 0.0])))
    world.add_component(e2, PositionComponent(np.array([2.0, 0.0, 0.0])))

    j1 = world.create_entity()
    aA1, aB1 = np.array([0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0])
    world.add_component(j1, CableJointComponent(e0, e1, 1.0, aA1.copy(), aB1.copy()))

    j2 = world.create_entity()
    aA2, aB2 = np.array([1.0, 0.0, 0.0]), np.array([2.0, 0.0, 0.0])
    world.add_component(j2, CableJointComponent(e1, e2, 1.0, aA2.copy(), aB2.copy()))

    path_ent = world.create_entity()
    path_comp = create_cable_path_component(
        world,
        joint_entities=[j1, j2],
        link_types=['attachment', 'attachment', 'attachment'],
        cw=[True, True, True],
        spring_constant=np.inf
    )
    world.add_component(path_ent, path_comp)

    solver = PBDCableConstraintSolver()
    solver.update(world, 0.016)

    comp1 = world.get_component(j1, CableJointComponent)
    np.testing.assert_array_equal(comp1.attachment_point_a_world, aA1)
    np.testing.assert_array_equal(comp1.attachment_point_b_world, aB1)
    comp2 = world.get_component(j2, CableJointComponent)
    np.testing.assert_array_equal(comp2.attachment_point_a_world, aA2)
    np.testing.assert_array_equal(comp2.attachment_point_b_world, aB2)

def test_handles_empty_world_without_error():
    """Tests that the solver runs without error on an empty world."""
    world = World()
    solver = PBDCableConstraintSolver()
    try:
        solver.update(world, 0.1)
    except Exception as e:
        pytest.fail(f"Solver failed on empty world: {e}")

def test_clamps_each_segment_to_rest_length_when_stretched():
    """
    Tests that a stretched cable segment is correctly clamped to its rest length.
    """
    world = World()
    e0, e1 = world.create_entity(), world.create_entity()

    pos0 = np.array([0.0, 0.0, 0.0])
    pos1 = np.array([5.0, 0.0, 0.0])

    world.add_component(e0, PositionComponent(pos0.copy()))
    world.add_component(e1, PositionComponent(pos1.copy()))
    world.add_component(e0, VelocityComponent())
    world.add_component(e1, VelocityComponent())
    world.add_component(e0, MassComponent(1.0))
    world.add_component(e1, MassComponent(1.0))
    # Add MomentOfInertia to prevent rotation (infinite inertia)
    world.add_component(e0, MomentOfInertiaComponent(inertia=np.inf))
    world.add_component(e1, MomentOfInertiaComponent(inertia=np.inf))
    world.add_component(e0, OrientationComponent())
    world.add_component(e1, OrientationComponent())
    world.add_component(e0, AngularVelocityComponent())
    world.add_component(e1, AngularVelocityComponent())
    world.add_component(e0, RadiusComponent(0.0))
    world.add_component(e1, RadiusComponent(0.0))
    world.add_component(e0, CableLinkComponent(prev_cable_attachment_time_pos=pos0.copy()))
    world.add_component(e1, CableLinkComponent(prev_cable_attachment_time_pos=pos1.copy()))

    j1 = world.create_entity()
    world.add_component(j1, CableJointComponent(e0, e1, 3.0, pos0.copy(), pos1.copy()))

    path_ent = world.create_entity()
    path_comp = create_cable_path_component(
        world,
        joint_entities=[j1],
        link_types=['attachment', 'attachment'],
        cw=[True, True]
    )
    world.add_component(path_ent, path_comp)

    dt = 0.016
    world.set_resource('dt', dt)
    solver = PBDCableConstraintSolver()
    attachment_system = CableAttachmentUpdateSystem()

    for _ in range(2):
        attachment_system.update(world, dt)
        solver.update(world, dt)

    # Final update to get the latest attachment points after solver corrections
    attachment_system.update(world, dt)

    p_a = world.get_component(e0, PositionComponent).pos
    p_b = world.get_component(e1, PositionComponent).pos
    distance = np.linalg.norm(p_a - p_b)
    assert distance == pytest.approx(3.0, abs=1e-5)

def test_pendulum_constraint_keeps_mass_within_rest_length_under_gravity():
    """
    Tests that a pendulum mass, when dropped, is constrained by the cable's
    rest length.
    """
    start_length, rest_length = 2.0, 1.0
    world = World()

    # Fixed point at origin
    origin = world.create_entity()
    pos_origin = np.array([0.0, 0.0, 0.0])
    world.add_component(origin, PositionComponent(pos_origin.copy()))
    world.add_component(origin, MassComponent(mass=np.inf)) # Immovable
    world.add_component(origin, MomentOfInertiaComponent(inertia=np.inf)) # Non-rotatable
    world.add_component(origin, OrientationComponent())
    world.add_component(origin, AngularVelocityComponent())
    world.add_component(origin, RadiusComponent(0.0))
    world.add_component(origin, CableLinkComponent(prev_cable_attachment_time_pos=pos_origin.copy()))

    # Mass entity
    mass = world.create_entity()
    pos_mass = np.array([0.0, -start_length, 0.0])
    world.add_component(mass, PositionComponent(pos_mass.copy()))
    world.add_component(mass, VelocityComponent())
    world.add_component(mass, MassComponent(1.0))
    world.add_component(mass, MomentOfInertiaComponent(inertia=1.0))
    world.add_component(mass, GravityAffectedComponent())
    world.add_component(mass, OrientationComponent())
    world.add_component(mass, AngularVelocityComponent())
    world.add_component(mass, RadiusComponent(0.0))
    world.add_component(mass, CableLinkComponent(prev_cable_attachment_time_pos=pos_mass.copy()))

    # Cable joint
    j = world.create_entity()
    world.add_component(j, CableJointComponent(
        origin, mass, rest_length,
        pos_origin.copy(), pos_mass.copy()
    ))

    path_ent = world.create_entity()
    path_comp = create_cable_path_component(
        world,
        joint_entities=[j],
        link_types=['attachment', 'attachment'],
        cw=[True, True]
    )
    world.add_component(path_ent, path_comp)

    gravity_system = MockGravitySystem()
    attachment_system = CableAttachmentUpdateSystem()
    solver = PBDCableConstraintSolver()
    dt = 0.016
    world.set_resource('dt', dt)
    world.set_resource('gravity', np.array([0.0, -9.8, 0.0]))

    for _ in range(5):
        gravity_system.update(world, dt)
        attachment_system.update(world, dt)
        solver.update(world, dt)

    pos_origin_final = world.get_component(origin, PositionComponent).pos
    pos_mass_final = world.get_component(mass, PositionComponent).pos
    distance = np.linalg.norm(pos_origin_final - pos_mass_final)
    assert distance == pytest.approx(rest_length, abs=1e-5)
