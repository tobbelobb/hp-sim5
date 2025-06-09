import pytest
from ..vector2 import Vector2
from ..ecs import (
    World,
    PositionComponent,
    VelocityComponent,
    GravityAffectedComponent,
    BallTagComponent,
    RadiusComponent,
    MassComponent,
    RestitutionComponent
)
from ..common_systems import (
    GravitySystem,
    MovementSystem,
    PBDBallBallCollisions
)

# --- MovementSystem Tests ---

def test_movement_system_updates_position_based_on_velocity_and_dt():
    world = World()
    e = world.create_entity()
    world.add_component(e, PositionComponent(pos=Vector2(1, 2)))
    world.add_component(e, VelocityComponent(vel=Vector2(3, 4)))
    dt = 0.5
    system = MovementSystem()
    system.update(world, dt)

    pos = world.get_component(e, PositionComponent).pos
    assert pos.x == pytest.approx(1 + 3 * dt)
    assert pos.y == pytest.approx(2 + 4 * dt)

def test_movement_system_does_not_update_entities_missing_components():
    world = World()
    e1 = world.create_entity()
    world.add_component(e1, PositionComponent(pos=Vector2(5, 5)))
    # missing VelocityComponent
    e2 = world.create_entity()
    world.add_component(e2, VelocityComponent(vel=Vector2(1, 1)))
    dt = 1.0
    system = MovementSystem()
    system.update(world, dt)

    pos1 = world.get_component(e1, PositionComponent).pos
    assert pos1.x == pytest.approx(5)
    assert pos1.y == pytest.approx(5)

    # e2 has no PositionComponent
    assert world.get_component(e2, PositionComponent) is None

def test_movement_system_handles_empty_world_without_error():
    world = World()
    dt = 0.1
    system = MovementSystem()
    try:
        system.update(world, dt)
    except Exception as e:
        pytest.fail(f"MovementSystem crashed on empty world: {e}")

# --- GravitySystem Tests ---

def test_gravity_system_applies_gravity_to_velocity():
    world = World()
    e = world.create_entity()
    world.add_component(e, VelocityComponent(vel=Vector2(0, 0)))
    world.add_component(e, GravityAffectedComponent())
    gravity = Vector2(0, -9.8)
    world.set_resource('gravity', gravity)

    dt = 0.5
    system = GravitySystem()
    system.update(world, dt)

    vel_comp = world.get_component(e, VelocityComponent)
    assert vel_comp.vel.x == pytest.approx(gravity.x * dt)
    assert vel_comp.vel.y == pytest.approx(gravity.y * dt)

def test_gravity_system_does_not_change_velocity_when_no_gravity_affected_component():
    world = World()
    e = world.create_entity()
    world.add_component(e, VelocityComponent(vel=Vector2(1, 1)))
    gravity = Vector2(0, -9.8)
    world.set_resource('gravity', gravity)
    dt = 1.0
    system = GravitySystem()
    system.update(world, dt)

    vel_comp = world.get_component(e, VelocityComponent)
    assert vel_comp.vel.x == pytest.approx(1)
    assert vel_comp.vel.y == pytest.approx(1)

def test_gravity_system_does_not_crash_if_no_velocity_component():
    world = World()
    e = world.create_entity()
    world.add_component(e, GravityAffectedComponent())
    gravity = Vector2(0, -9.8)
    world.set_resource('gravity', gravity)
    dt = 1.0
    system = GravitySystem()
    try:
        system.update(world, dt)
    except Exception as e:
        pytest.fail(f"GravitySystem crashed with missing VelocityComponent: {e}")

def test_gravity_system_does_nothing_if_gravity_resource_undefined():
    world = World()
    e = world.create_entity()
    world.add_component(e, VelocityComponent(vel=Vector2(2, 3)))
    world.add_component(e, GravityAffectedComponent())
    dt = 1.0
    system = GravitySystem()
    system.update(world, dt)

    vel_comp = world.get_component(e, VelocityComponent)
    assert vel_comp.vel.x == pytest.approx(2)
    assert vel_comp.vel.y == pytest.approx(3)

# --- PBDBallBallCollisions Tests ---

def test_pbd_ball_ball_swaps_velocities_on_perfectly_elastic_head_on_collision():
    world = World()
    ball1 = world.create_entity()
    ball2 = world.create_entity()

    world.add_component(ball1, BallTagComponent())
    world.add_component(ball1, PositionComponent(pos=Vector2(0, 0)))
    world.add_component(ball1, VelocityComponent(vel=Vector2(1, 0)))
    world.add_component(ball1, RadiusComponent(radius=1))
    world.add_component(ball1, MassComponent(mass=1))
    world.add_component(ball1, RestitutionComponent(restitution=1))

    world.add_component(ball2, BallTagComponent())
    world.add_component(ball2, PositionComponent(pos=Vector2(2, 0))) # touching
    world.add_component(ball2, VelocityComponent(vel=Vector2(-1, 0)))
    world.add_component(ball2, RadiusComponent(radius=1))
    world.add_component(ball2, MassComponent(mass=1))
    world.add_component(ball2, RestitutionComponent(restitution=1))

    system = PBDBallBallCollisions()
    system.update(world, 0.016)

    v1 = world.get_component(ball1, VelocityComponent).vel
    v2 = world.get_component(ball2, VelocityComponent).vel
    assert v1.x == pytest.approx(-1)
    assert v2.x == pytest.approx(1)

def test_pbd_ball_ball_halves_velocities_with_restitution_0_5():
    world = World()
    ball1 = world.create_entity()
    ball2 = world.create_entity()

    world.add_component(ball1, BallTagComponent())
    world.add_component(ball1, PositionComponent(pos=Vector2(0, 0)))
    world.add_component(ball1, VelocityComponent(vel=Vector2(1, 0)))
    world.add_component(ball1, RadiusComponent(radius=1))
    world.add_component(ball1, MassComponent(mass=1))
    world.add_component(ball1, RestitutionComponent(restitution=0.5))

    world.add_component(ball2, BallTagComponent())
    world.add_component(ball2, PositionComponent(pos=Vector2(2, 0)))
    world.add_component(ball2, VelocityComponent(vel=Vector2(-1, 0)))
    world.add_component(ball2, RadiusComponent(radius=1))
    world.add_component(ball2, MassComponent(mass=1))
    world.add_component(ball2, RestitutionComponent(restitution=0.5))

    system = PBDBallBallCollisions()
    system.update(world, 0.016)

    v1 = world.get_component(ball1, VelocityComponent).vel
    v2 = world.get_component(ball2, VelocityComponent).vel
    assert v1.x == pytest.approx(-0.5)
    assert v2.x == pytest.approx(0.5)

def test_pbd_ball_ball_uses_smallest_restitution():
    world = World()
    ball1 = world.create_entity()
    ball2 = world.create_entity()

    world.add_component(ball1, BallTagComponent())
    world.add_component(ball1, PositionComponent(pos=Vector2(0, 0)))
    world.add_component(ball1, VelocityComponent(vel=Vector2(1, 0)))
    world.add_component(ball1, RadiusComponent(radius=1))
    world.add_component(ball1, MassComponent(mass=1))
    world.add_component(ball1, RestitutionComponent(restitution=1.0))

    world.add_component(ball2, BallTagComponent())
    world.add_component(ball2, PositionComponent(pos=Vector2(2, 0)))
    world.add_component(ball2, VelocityComponent(vel=Vector2(-1, 0)))
    world.add_component(ball2, RadiusComponent(radius=1))
    world.add_component(ball2, MassComponent(mass=1))
    world.add_component(ball2, RestitutionComponent(restitution=0.5))

    system = PBDBallBallCollisions()
    system.update(world, 0.016)

    v1 = world.get_component(ball1, VelocityComponent).vel
    v2 = world.get_component(ball2, VelocityComponent).vel
    assert v1.x == pytest.approx(-0.5)
    assert v2.x == pytest.approx(0.5)

def test_pbd_ball_ball_no_change_when_not_intersecting():
    world = World()
    ball1 = world.create_entity()
    ball2 = world.create_entity()

    world.add_component(ball1, BallTagComponent())
    world.add_component(ball1, PositionComponent(pos=Vector2(0, 0)))
    world.add_component(ball1, VelocityComponent(vel=Vector2(1, 0)))
    world.add_component(ball1, RadiusComponent(radius=1))
    world.add_component(ball1, MassComponent(mass=1))
    world.add_component(ball1, RestitutionComponent(restitution=1))

    world.add_component(ball2, BallTagComponent())
    world.add_component(ball2, PositionComponent(pos=Vector2(5, 0))) # far apart
    world.add_component(ball2, VelocityComponent(vel=Vector2(-1, 0)))
    world.add_component(ball2, RadiusComponent(radius=1))
    world.add_component(ball2, MassComponent(mass=1))
    world.add_component(ball2, RestitutionComponent(restitution=1))

    system = PBDBallBallCollisions()
    system.update(world, 0.016)

    v1 = world.get_component(ball1, VelocityComponent).vel
    v2 = world.get_component(ball2, VelocityComponent).vel
    assert v1.x == pytest.approx(1)
    assert v2.x == pytest.approx(-1)

def test_pbd_ball_ball_handles_overlaps_gracefully():
    world = World()
    ball1 = world.create_entity()
    ball2 = world.create_entity()

    world.add_component(ball1, BallTagComponent())
    world.add_component(ball1, PositionComponent(pos=Vector2(0, 0)))
    world.add_component(ball1, VelocityComponent(vel=Vector2(1, 0)))
    world.add_component(ball1, RadiusComponent(radius=1))
    world.add_component(ball1, MassComponent(mass=1))
    world.add_component(ball1, RestitutionComponent(restitution=1))

    world.add_component(ball2, BallTagComponent())
    world.add_component(ball2, PositionComponent(pos=Vector2(1.9, 0))) # overlapping
    world.add_component(ball2, VelocityComponent(vel=Vector2(-1, 0)))
    world.add_component(ball2, RadiusComponent(radius=1))
    world.add_component(ball2, MassComponent(mass=1))
    world.add_component(ball2, RestitutionComponent(restitution=1))

    system = PBDBallBallCollisions()
    system.update(world, 0.016)

    v1 = world.get_component(ball1, VelocityComponent).vel
    v2 = world.get_component(ball2, VelocityComponent).vel
    assert v1.x == pytest.approx(-1)
    assert v2.x == pytest.approx(1)

def test_pbd_ball_ball_distributes_impact_based_on_inverse_mass():
    world = World()
    ball1 = world.create_entity()
    ball2 = world.create_entity()

    world.add_component(ball1, BallTagComponent())
    world.add_component(ball1, PositionComponent(pos=Vector2(0, 0)))
    world.add_component(ball1, VelocityComponent(vel=Vector2(1, 0)))
    world.add_component(ball1, RadiusComponent(radius=1))
    world.add_component(ball1, MassComponent(mass=2)) # twice the mass
    world.add_component(ball1, RestitutionComponent(restitution=1))

    world.add_component(ball2, BallTagComponent())
    world.add_component(ball2, PositionComponent(pos=Vector2(2, 0)))
    world.add_component(ball2, VelocityComponent(vel=Vector2(-1, 0)))
    world.add_component(ball2, RadiusComponent(radius=1))
    world.add_component(ball2, MassComponent(mass=1))
    world.add_component(ball2, RestitutionComponent(restitution=1))

    system = PBDBallBallCollisions()
    system.update(world, 0.016)

    v1 = world.get_component(ball1, VelocityComponent).vel
    v2 = world.get_component(ball2, VelocityComponent).vel
    assert v1.x == pytest.approx(-1/3)
    assert v2.x == pytest.approx(5/3)

def test_pbd_ball_ball_swaps_velocities_based_on_inverse_mass_with_restitution_0_9():
    world = World()
    ball1 = world.create_entity()
    ball2 = world.create_entity()

    world.add_component(ball1, BallTagComponent())
    world.add_component(ball1, PositionComponent(pos=Vector2(0, 0)))
    world.add_component(ball1, VelocityComponent(vel=Vector2(1, 0)))
    world.add_component(ball1, RadiusComponent(radius=1))
    world.add_component(ball1, MassComponent(mass=2))
    world.add_component(ball1, RestitutionComponent(restitution=0.9))

    world.add_component(ball2, BallTagComponent())
    world.add_component(ball2, PositionComponent(pos=Vector2(2, 0)))
    world.add_component(ball2, VelocityComponent(vel=Vector2(-1, 0)))
    world.add_component(ball2, RadiusComponent(radius=1))
    world.add_component(ball2, MassComponent(mass=1))
    world.add_component(ball2, RestitutionComponent(restitution=0.9))

    system = PBDBallBallCollisions()
    system.update(world, 0.016)

    v1 = world.get_component(ball1, VelocityComponent).vel
    v2 = world.get_component(ball2, VelocityComponent).vel
    assert v1.x == pytest.approx(-0.26666667)
    assert v2.x == pytest.approx(1.53333333)
    assert v1.y == pytest.approx(0.0)
    assert v2.y == pytest.approx(0.0)
