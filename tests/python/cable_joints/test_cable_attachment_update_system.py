import pytest
import numpy as np

from cable_joints.ecs import World, PositionComponent, RadiusComponent, CableLinkComponent
from cable_joints.cable_joints_components import CableJointComponent, create_cable_path_component
from cable_joints.geometry import tangent_from_point_to_circle, tangent_from_circle_to_point, signed_arc_length_on_wheel
from cable_joints.cable_attachment_update_system import CableAttachmentUpdateSystem

def test_merge_joints_when_positions_opposite_vertically():
    world = World()
    center = np.array([0.0, 0.0, 0.0])
    radius = 1.0
    cw = True

    # Create entities
    point1_id = world.create_entity()
    wheel_id = world.create_entity()
    point2_id = world.create_entity()

    # Initial positions
    pos1_initial = np.array([0.9999, 2.0, 0.0])
    pos2 = np.array([1.0, -2.0, 0.0])

    # Components
    world.add_component(point1_id, CableLinkComponent(prev_cable_attachment_time_pos=pos1_initial.copy()))
    world.add_component(wheel_id, CableLinkComponent(prev_cable_attachment_time_pos=center.copy()))
    world.add_component(point2_id, CableLinkComponent(prev_cable_attachment_time_pos=pos2.copy()))
    
    world.add_component(wheel_id, PositionComponent(pos=center.copy()))
    world.add_component(point1_id, PositionComponent(pos=pos1_initial.copy()))
    world.add_component(point2_id, PositionComponent(pos=pos2.copy()))
    world.add_component(wheel_id, RadiusComponent(radius=radius))

    # Initial tangents
    tp1 = tangent_from_point_to_circle(pos1_initial, center, radius, cw)
    tp2 = tangent_from_circle_to_point(pos2, center, radius, cw)

    # Create joints
    joint1_id = world.create_entity()
    world.add_component(joint1_id, CableJointComponent(
        entity_a=point1_id, entity_b=wheel_id,
        rest_length=np.linalg.norm(tp1['a_attach'] - tp1['a_circle']),
        attachment_point_a_world=tp1['a_attach'],
        attachment_point_b_world=tp1['a_circle']
    ))
    joint2_id = world.create_entity()
    world.add_component(joint2_id, CableJointComponent(
        entity_a=wheel_id, entity_b=point2_id,
        rest_length=np.linalg.norm(tp2['a_attach'] - tp2['a_circle']),
        attachment_point_a_world=tp2['a_circle'],
        attachment_point_b_world=tp2['a_attach']
    ))

    # Build cable path
    cable_path_id = world.create_entity()
    path_comp = create_cable_path_component(
        world,
        [joint1_id, joint2_id],
        ['attachment', 'rolling', 'attachment'],
        [cw, cw, cw]
    )
    world.add_component(cable_path_id, path_comp)

    initial_total_rest = path_comp.total_rest_length
    assert len(path_comp.joint_entities) == 2
    assert len(path_comp.link_types) == 3
    assert path_comp.stored[1] < radius
    assert path_comp.stored[1] > 0.0

    # Simulate movement: let point1 travel to the right, causing the tangent points to cross
    pos1_new = np.array([1.001, 2.0, 0.0])
    world.get_component(point1_id, PositionComponent).pos = pos1_new

    # Run the system
    system = CableAttachmentUpdateSystem()
    world.set_resource('debugRenderPoints', {})
    system.update(world, dt=0.016)

    # Assertions
    path_comp = world.get_component(cable_path_id, path_comp.__class__) # Re-fetch component
    assert len(path_comp.joint_entities) == 1
    assert len(path_comp.link_types) == 2
    assert path_comp.total_rest_length == pytest.approx(initial_total_rest)
