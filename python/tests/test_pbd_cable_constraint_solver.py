import pytest
import numpy as np

# Adjust the relative imports based on your project structure.
# This assumes the tests are run from the root of the `python` directory.
from pbdCableConstraintSolver import PBDCableConstraintSolver
from ecs import (
    PositionComponent, VelocityComponent, MassComponent, MomentOfInertiaComponent,
    OrientationComponent, AngularVelocityComponent, CableLinkComponent,
    GravityAffectedComponent
)
from cable_joints_core import CableJointComponent, CablePathComponent

# --- Mocks for Testing ---

class MockWorld:
    """A minimal mock of the ECS World for testing purposes."""
    def __init__(self):
        self.components = {}
        self.next_entity_id = 0
        self.resources = {}

    def create_entity(self):
        entity_id = self.next_entity_id
        self.next_entity_id += 1
        return entity_id

    def add_component(self, entity_id, component):
        component_class = type(component)
        if component_class not in self.components:
            self.components[component_class] = {}
        self.components[component_class][entity_id] = component

    def get_component(self, entity_id, component_class):
        return self.components.get(component_class, {}).get(entity_id)

    def query(self, component_class):
        return list(self.components.get(component_class, {}).keys())

    def set_resource(self, name, value):
        self.resources[name] = value

    def get_resource(self, name):
        return self.resources.get(name)

class MockCableAttachmentUpdateSystem:
    """
    A mock system that updates joint attachment points from entity positions.
    This simplified version assumes attachment points are at the entity's center.
    """
    def update(self, world, dt):
        joint_entities = world.query(CableJointComponent)
        for joint_id in joint_entities:
            joint = world.get_component(joint_id, CableJointComponent)
            pos_a = world.get_component(joint.entity_a, PositionComponent)
            pos_b = world.get_component(joint.entity_b, PositionComponent)
            if pos_a:
                joint.attachment_point_a_world[:] = pos_a.pos
            if pos_b:
                joint.attachment_point_b_world[:] = pos_b.pos

class MockGravitySystem:
    """A mock system that applies gravity to entities."""
    def update(self, world, dt):
        gravity = world.get_resource('gravity')
        if gravity is None:
            return
        entities = world.query(GravityAffectedComponent)
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
    world = MockWorld()
    e0, e1, e2 = world.create_entity(), world.create_entity(), world.create_entity()
    world.add_component(e0, PositionComponent(np.array([0.0, 0.0])))
    world.add_component(e1, PositionComponent(np.array([1.0, 0.0])))
    world.add_component(e2, PositionComponent(np.array([2.0, 0.0])))

    j1 = world.create_entity()
    aA1, aB1 = np.array([0.0, 0.0]), np.array([1.0, 0.0])
    world.add_component(j1, CableJointComponent(e0, e1, 1.0, aA1.copy(), aB1.copy()))

    j2 = world.create_entity()
    aA2, aB2 = np.array([1.0, 0.0]), np.array([2.0, 0.0])
    world.add_component(j2, CableJointComponent(e1, e2, 1.0, aA2.copy(), aB2.copy()))

    path_ent = world.create_entity()
    path_comp = CablePathComponent(joint_entities=[j1, j2], spring_constant=np.inf)
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
    world = MockWorld()
    solver = PBDCableConstraintSolver()
    try:
        solver.update(world, 0.1)
    except Exception as e:
        pytest.fail(f"Solver failed on empty world: {e}")

def test_clamps_each_segment_to_rest_length_when_stretched():
    """
    Tests that a stretched cable segment is correctly clamped to its rest length.
    """
    world = MockWorld()
    e0, e1 = world.create_entity(), world.create_entity()
    world.add_component(e0, PositionComponent(np.array([0.0, 0.0])))
    world.add_component(e1, PositionComponent(np.array([5.0, 0.0])))
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

    j1 = world.create_entity()
    world.add_component(j1, CableJointComponent(e0, e1, 3.0, np.array([0.0, 0.0]), np.array([5.0, 0.0])))

    path_ent = world.create_entity()
    world.add_component(path_ent, CablePathComponent(joint_entities=[j1]))

    dt = 0.016
    world.set_resource('dt', dt)
    solver = PBDCableConstraintSolver()
    attachment_system = MockCableAttachmentUpdateSystem()

    for _ in range(2):
        attachment_system.update(world, dt)
        solver.update(world, dt)

    # Final update to get the latest attachment points after solver corrections
    attachment_system.update(world, dt)

    comp1 = world.get_component(j1, CableJointComponent)
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
    world = MockWorld()

    # Fixed point at origin
    origin = world.create_entity()
    world.add_component(origin, PositionComponent(np.array([0.0, 0.0])))
    world.add_component(origin, MassComponent(mass=np.inf)) # Immovable
    world.add_component(origin, MomentOfInertiaComponent(inertia=np.inf)) # Non-rotatable
    world.add_component(origin, OrientationComponent())
    world.add_component(origin, AngularVelocityComponent())

    # Mass entity
    mass = world.create_entity()
    world.add_component(mass, PositionComponent(np.array([0.0, -start_length])))
    world.add_component(mass, VelocityComponent())
    world.add_component(mass, MassComponent(1.0))
    world.add_component(mass, MomentOfInertiaComponent(inertia=1.0))
    world.add_component(mass, GravityAffectedComponent())
    world.add_component(mass, OrientationComponent())
    world.add_component(mass, AngularVelocityComponent())

    # Cable joint
    j = world.create_entity()
    world.add_component(j, CableJointComponent(
        origin, mass, rest_length,
        np.array([0.0, 0.0]), np.array([0.0, -start_length])
    ))

    path_ent = world.create_entity()
    world.add_component(path_ent, CablePathComponent(joint_entities=[j]))

    gravity_system = MockGravitySystem()
    attachment_system = MockCableAttachmentUpdateSystem()
    solver = PBDCableConstraintSolver()
    dt = 0.016
    world.set_resource('dt', dt)
    world.set_resource('gravity', np.array([0.0, -9.8]))

    for _ in range(5):
        gravity_system.update(world, dt)
        attachment_system.update(world, dt)
        solver.update(world, dt)

    pos_origin = world.get_component(origin, PositionComponent).pos
    pos_mass = world.get_component(mass, PositionComponent).pos
    distance = np.linalg.norm(pos_origin - pos_mass)
    assert distance == pytest.approx(rest_length, abs=1e-5)
