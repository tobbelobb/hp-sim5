import numpy as np
import pytest

from python.ecs import (
    World,
    PositionComponent,
    MassComponent,
    DistanceConstraintComponent,
)
from python.common_systems import XPBDDistanceConstraintSystem


def test_xpbd_distance_constraint_converges_to_rest_length():
    world = World()

    a = world.create_entity()
    world.add_component(a, PositionComponent(pos=np.array([0.0, 0.0, 0.0])))
    world.add_component(a, MassComponent(mass=1.0))

    b = world.create_entity()
    world.add_component(b, PositionComponent(pos=np.array([2.0, 0.0, 0.0])))
    world.add_component(b, MassComponent(mass=1.0))

    constraint = DistanceConstraintComponent(
        entityA=a,
        entityB=b,
        rest_length=1.0,
    )
    # Alias fields expected by the system's JS-port implementation
    constraint.restLength = constraint.rest_length
    constraint.lambda_val = constraint.lambda_

    constraint_entity = world.create_entity()
    world.add_component(constraint_entity, constraint)

    system = XPBDDistanceConstraintSystem()

    dt = 0.1
    for _ in range(5):
        system.update(world, dt)

    pos_a = world.get_component(a, PositionComponent).pos
    pos_b = world.get_component(b, PositionComponent).pos
    dist = np.linalg.norm(pos_b - pos_a)

    assert dist == pytest.approx(constraint.rest_length)
