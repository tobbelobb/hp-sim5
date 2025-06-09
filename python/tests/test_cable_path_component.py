import pytest
import numpy as np

from python.ecs import World, PositionComponent, RadiusComponent
from python.cable_joints_components import CableJointComponent, create_cable_path_component
from python.geometry import signed_arc_length_on_wheel

def test_initial_stored_lengths_and_total_rest_length_are_computed_correctly():
    world = World()
    # Entities: start, rolling center, end
    start_id = world.create_entity()
    center_id = world.create_entity()
    end_id = world.create_entity()

    center_pos = np.array([0.0, 0.0, 0.0])
    world.add_component(center_id, PositionComponent(pos=center_pos.copy()))
    radius = 1.0
    world.add_component(center_id, RadiusComponent(radius=radius))

    # Create two joints: start->center and center->end
    rest_len1 = 1.5
    rest_len2 = 2.0
    point_b1 = np.array([1.0, 0.0, 0.0]) # on circle at angle 0
    point_a2 = np.array([0.0, 1.0, 0.0]) # on circle at pi/2

    # Joint from start to center
    joint1_id = world.create_entity()
    world.add_component(
        joint1_id,
        CableJointComponent(
            entity_a=start_id,
            entity_b=center_id,
            rest_length=rest_len1,
            attachment_point_a_world=point_b1.copy(),
            attachment_point_b_world=point_b1.copy()
        )
    )

    # Joint from center to end
    joint2_id = world.create_entity()
    world.add_component(
        joint2_id,
        CableJointComponent(
            entity_a=center_id,
            entity_b=end_id,
            rest_length=rest_len2,
            attachment_point_a_world=point_a2.copy(),
            attachment_point_b_world=point_a2.copy()
        )
    )

    link_types = ['attachment', 'rolling', 'attachment']
    cw = [False, False, False]
    
    path = create_cable_path_component(world, [joint1_id, joint2_id], link_types, cw)

    # Expected stored: [0, arc length from (1,0) to (0,1) on circle radius 1, 0]
    expected_arc = signed_arc_length_on_wheel(
        point_b1,
        point_a2,
        center_pos,
        radius,
        False, # clockwise_preference
        True # force_positive
    )

    assert len(path.stored) == 3
    assert path.stored[0] == pytest.approx(0.0)
    assert path.stored[1] == pytest.approx(expected_arc)
    assert path.stored[2] == pytest.approx(0.0)

    # totalRestLength = restLen1 + restLen2 + expectedArc
    expected_total_rest_length = rest_len1 + rest_len2 + expected_arc
    assert path.total_rest_length == pytest.approx(expected_total_rest_length)
