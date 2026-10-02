import math
import numpy as np

from cable_joints.ecs import GravityAffectedComponent, MassComponent, World
from cable_joints_3d.common_systems import AngularMovementSystem, GravitySystem, MovementSystem
from cable_joints_3d.ecs import AngularVelocityComponent, OrientationComponent, PositionComponent, VelocityComponent
from cable_joints_3d.inertia_tensor import (
    MomentOfInertiaComponent, apply_world_inverse_inertia, parallel_axis_tensor,
)
from cable_joints_3d.quaternion import Quaternion


def test_quaternion_rotates_vector_like_js_engine():
    rotation = Quaternion().set_from_axis_angle([0, 0, 1], math.pi / 2)
    assert np.allclose(rotation.transform_vector([1, 0, 0]), [0, 1, 0], atol=1e-12)


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
