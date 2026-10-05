import math
import numpy as np

from cable_joints.ecs import GravityAffectedComponent, MassComponent, World
from cable_joints_3d.common_systems import (
    AngularMovementSystem, GravitySystem, MovementSystem,
    PBDAngularVelocityUpdateSystem,
)
from cable_joints_3d.ecs import (
    AngularVelocityComponent, OrientationComponent, PositionComponent,
    PrevFinalOrientationComponent, RigidBodyMemberComponent, VelocityComponent,
)
from cable_joints_3d.inertia_tensor import (
    MomentOfInertiaComponent, apply_world_inverse_inertia, parallel_axis_tensor,
)
from cable_joints_3d.quaternion import Quaternion


def test_quaternion_rotates_vector_like_js_engine():
    rotation = Quaternion().set_from_axis_angle([0, 0, 1], math.pi / 2)
    assert np.allclose(rotation.transform_vector([1, 0, 0]), [0, 1, 0], atol=1e-12)


def test_raw_quaternion_transform_owns_float_output_without_normalizing_inputs():
    rotation = Quaternion(0, 0, 0, 2)
    vector = np.array([1, 2, 3])
    result = rotation.transform_vector(vector)
    assert result.tolist() == [4, 8, 12]
    assert result.dtype == np.float64
    result[:] = 0
    assert vector.tolist() == [1, 2, 3]
    assert rotation.as_xyzw().tolist() == [0, 0, 0, 2]
    assert Quaternion(0, 0, 0, 0).transform_vector(vector).tolist() == [0, 0, 0]


def test_full_inertia_tensor_and_world_rotation():
    moment = MomentOfInertiaComponent([[2, 0, 0], [0, 4, 0], [0, 0, 8]])
    orientation = Quaternion().set_from_axis_angle([0, 0, 1], math.pi / 2)
    assert np.allclose(apply_world_inverse_inertia(moment, orientation, [1, 0, 0]), [0.25, 0, 0])
    assert np.allclose(parallel_axis_tensor(2, [1, 0, 0]), np.diag([0, 2, 2]))


def test_3d_world_integrates_linear_and_angular_motion():
    world = World(); world.set_resource("gravity", np.array([0, 0, -10.]))
    entity = world.create_entity()
    for component in (PositionComponent(), VelocityComponent(1, 0, 0), MassComponent(1),
                      GravityAffectedComponent(), OrientationComponent(), AngularVelocityComponent(0, 0, math.pi)):
        world.add_component(entity, component)
    GravitySystem().update(world, 0.1); MovementSystem().update(world, 0.1)
    AngularMovementSystem().update(world, 0.5)
    assert np.allclose(world.get_component(entity, PositionComponent).pos, [0.1, 0, -0.1])
    assert np.allclose(world.get_component(entity, OrientationComponent).quaternion.transform_vector([1, 0, 0]), [0, 1, 0], atol=1e-12)


def test_angular_movement_applies_world_rotation_before_current_rotation():
    world = World(); entity = world.create_entity()
    initial = Quaternion().set_from_axis_angle([1, 0, 0], math.pi / 2)
    orientation = OrientationComponent(*initial.as_xyzw())
    world.add_component(entity, orientation)
    world.add_component(entity, AngularVelocityComponent(0, 0, math.pi / 2))

    AngularMovementSystem().update(world, 1.)

    delta = Quaternion().set_from_axis_angle([0, 0, 1], math.pi / 2)
    expected = delta.copy().multiply(initial)
    assert np.allclose(orientation.quaternion.as_xyzw(), expected.as_xyzw())


def test_pbd_angular_velocity_is_reconstructed_from_corrected_pose():
    world = World(); entity = world.create_entity()
    corrected = Quaternion().set_from_axis_angle([0, 1, 0], math.pi / 3)
    world.add_component(entity, OrientationComponent(*corrected.as_xyzw()))
    world.add_component(entity, PrevFinalOrientationComponent())
    world.add_component(entity, AngularVelocityComponent())
    world.add_component(entity, MomentOfInertiaComponent(1.))

    PBDAngularVelocityUpdateSystem().update(world, 0.5)

    omega = world.get_component(entity, AngularVelocityComponent).omega
    assert np.allclose(omega, [0, 2 * math.pi / 3, 0])


def test_movement_integrates_zero_mass_kinematic_but_not_rigid_member():
    world = World()
    kinematic = world.create_entity()
    member = world.create_entity()
    for entity in (kinematic, member):
        world.add_component(entity, PositionComponent())
        world.add_component(entity, VelocityComponent(2, 0, 0))
        world.add_component(entity, MassComponent(0))
    world.add_component(member, RigidBodyMemberComponent())

    MovementSystem().update(world, 0.25)

    assert np.allclose(world.get_component(kinematic, PositionComponent).pos, [.5, 0, 0])
    assert np.allclose(world.get_component(member, PositionComponent).pos, [0, 0, 0])


def test_small_rotated_inertia_tensor_is_invertible():
    rotation = np.array([[.8, -.6, 0], [.6, .8, 0], [0, 0, 1.]])
    tensor = rotation @ np.diag([5e-7, 1e-6, 1.5e-6]) @ rotation.T
    moment = MomentOfInertiaComponent(tensor)
    assert np.allclose(moment.inv_inertia_tensor @ tensor, np.eye(3))


def test_rotated_rank_two_inertia_preserves_supported_dofs():
    rotation = np.array([[.8, -.6, 0], [.6, .8, 0], [0, 0, 1.]])
    tensor = rotation @ np.diag([0., 1e-6, 2e-6]) @ rotation.T
    moment = MomentOfInertiaComponent(tensor)
    expected = rotation @ np.diag([0., 1e6, .5e6]) @ rotation.T
    assert np.allclose(moment.inv_inertia_tensor, expected)
    assert np.linalg.matrix_rank(moment.inv_inertia_tensor) == 2


def test_rigid_body_member_copies_and_normalizes_constructor_values():
    position = np.array([1., 2., 3.])
    orientation = Quaternion(0, 0, 2, 2)
    member = RigidBodyMemberComponent(4, position, orientation, 2.)
    position[:] = 0.; orientation.x = 10.
    assert np.allclose(member.local_position, [1, 2, 3])
    assert math.isclose(np.linalg.norm(member.local_orientation.as_xyzw()), 1.)
    assert member.local_orientation.x == 0.
