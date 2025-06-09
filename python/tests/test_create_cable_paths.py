import pytest
import numpy as np
import warnings

from python.ecs import World, PositionComponent, RadiusComponent
from python.cable_joints_components import CableJointComponent, CablePathComponent
from python.geometry import signed_arc_length_on_wheel
from python.create_cable_paths import create_cable_paths

# Helper functions similar to the JS test setup
def _create_mock_joint_entity(world, entity_a, entity_b, rest_length=1.0, attach_a=np.zeros(3), attach_b=np.zeros(3)):
    joint_id = world.create_entity()
    world.add_component(joint_id, CableJointComponent(
        entity_a=entity_a,
        entity_b=entity_b,
        rest_length=rest_length,
        attachment_point_a_world=attach_a.copy(),
        attachment_point_b_world=attach_b.copy()
    ))
    return joint_id

def _create_mock_link_entity(world, entity_id, is_rolling=False, pos=np.zeros(3), radius=1.0):
    world.add_component(entity_id, PositionComponent(pos.copy()))
    if is_rolling:
        world.add_component(entity_id, RadiusComponent(radius))
    return entity_id

@pytest.fixture
def world():
    return World()

class TestCreateCablePaths:
    spring_constant = 1000

    def test_should_create_single_path_if_no_intermediate_attachment(self, world):
        e0 = world.create_entity()
        _create_mock_link_entity(world, e0, is_rolling=True, pos=np.array([0,0,0]), radius=1)
        e1 = world.create_entity()
        _create_mock_link_entity(world, e1, is_rolling=True, pos=np.array([2,0,0]), radius=1)
        e2 = world.create_entity()
        _create_mock_link_entity(world, e2, is_rolling=True, pos=np.array([4,0,0]), radius=1)

        j1 = _create_mock_joint_entity(world, e0, e1, 1, np.array([0,1,0]), np.array([2,1,0]))
        j2 = _create_mock_joint_entity(world, e1, e2, 1, np.array([2,-1,0]), np.array([4,-1,0]))

        joint_entities = [j1, j2]
        link_types = ['hybrid-attachment', 'rolling', 'hybrid-attachment']
        cw = [True, True, True]

        path_entity_ids = create_cable_paths(world, joint_entities, link_types, cw, self.spring_constant)

        assert len(path_entity_ids) == 1
        path_comp = world.get_component(path_entity_ids[0], CablePathComponent)
        assert path_comp.joint_entities == joint_entities
        assert path_comp.link_types == ['hybrid-attachment', 'rolling', 'hybrid-attachment']
        assert path_comp.cw == cw
        assert path_comp.spring_constant == self.spring_constant

        expected_stored_1 = signed_arc_length_on_wheel(
            world.get_component(j1, CableJointComponent).attachment_point_b_world,
            world.get_component(j2, CableJointComponent).attachment_point_a_world,
            world.get_component(e1, PositionComponent).pos,
            world.get_component(e1, RadiusComponent).radius,
            cw[1], True
        )
        assert path_comp.stored[0] == pytest.approx(0.0)
        assert path_comp.stored[1] == pytest.approx(expected_stored_1)
        assert path_comp.stored[2] == pytest.approx(0.0)

    def test_should_split_into_two_paths_for_one_intermediate_attachment(self, world):
        e0 = world.create_entity()
        _create_mock_link_entity(world, e0)
        e1 = world.create_entity()
        _create_mock_link_entity(world, e1, is_rolling=True, pos=np.array([0,0,0]), radius=1)
        e2 = world.create_entity()
        _create_mock_link_entity(world, e2) # Attachment
        e3 = world.create_entity()
        _create_mock_link_entity(world, e3, is_rolling=True, pos=np.array([2,0,0]), radius=1)
        e4 = world.create_entity()
        _create_mock_link_entity(world, e4)

        j1 = _create_mock_joint_entity(world, e0, e1, 1.0, np.array([0,0,0]), np.array([0,1,0]))
        j2 = _create_mock_joint_entity(world, e1, e2, 1.0, np.array([0,-1,0]), np.array([1,0,0]))
        j3 = _create_mock_joint_entity(world, e2, e3, 1.0, np.array([1,0,0]), np.array([2,1,0]))
        j4 = _create_mock_joint_entity(world, e3, e4, 1.0, np.array([2,-1,0]), np.array([3,0,0]))

        joint_entities = [j1, j2, j3, j4]
        link_types = ['hybrid-attachment', 'rolling', 'attachment', 'rolling', 'hybrid-attachment']
        cw = [True, True, False, True, False]
        user_stored = [0.1, None, 0.3, None, 0.5]

        path_entity_ids = create_cable_paths(world, joint_entities, link_types, cw, self.spring_constant, user_stored)

        assert len(path_entity_ids) == 2

        # Path 1
        path_comp1 = world.get_component(path_entity_ids[0], CablePathComponent)
        assert path_comp1.joint_entities == [j1, j2]
        assert path_comp1.link_types == ['hybrid-attachment', 'rolling', 'attachment']
        assert path_comp1.cw == [True, True, False]
        assert path_comp1.stored[0] == pytest.approx(user_stored[0])
        expected_stored_1_1 = signed_arc_length_on_wheel(
            world.get_component(j1, CableJointComponent).attachment_point_b_world,
            world.get_component(j2, CableJointComponent).attachment_point_a_world,
            world.get_component(e1, PositionComponent).pos,
            world.get_component(e1, RadiusComponent).radius,
            cw[1], True
        )
        assert path_comp1.stored[1] == pytest.approx(expected_stored_1_1)
        assert path_comp1.stored[2] == pytest.approx(user_stored[2])

        # Path 2
        path_comp2 = world.get_component(path_entity_ids[1], CablePathComponent)
        assert path_comp2.joint_entities == [j3, j4]
        assert path_comp2.link_types == ['attachment', 'rolling', 'hybrid-attachment']
        assert path_comp2.cw == [False, True, False]
        assert path_comp2.stored[0] == pytest.approx(user_stored[2])
        expected_stored_2_1 = signed_arc_length_on_wheel(
            world.get_component(j3, CableJointComponent).attachment_point_b_world,
            world.get_component(j4, CableJointComponent).attachment_point_a_world,
            world.get_component(e3, PositionComponent).pos,
            world.get_component(e3, RadiusComponent).radius,
            cw[3], True
        )
        assert path_comp2.stored[1] == pytest.approx(expected_stored_2_1)
        assert path_comp2.stored[2] == pytest.approx(user_stored[4])

    def test_should_split_into_four_paths_for_three_intermediate_attachments(self, world):
        e = [world.create_entity() for _ in range(7)]
        for i in range(7):
            is_rolling = i in [0, 2, 4, 6]
            _create_mock_link_entity(world, e[i], is_rolling, pos=np.array([float(i), 0, 0]))
        
        j = [_create_mock_joint_entity(world, e[i], e[i+1]) for i in range(6)]

        joint_entities = j
        link_types = ['hybrid-attachment', 'attachment', 'rolling', 'attachment', 'rolling', 'attachment', 'hybrid-attachment']
        cw = [True, False, True, False, True, False, True]

        path_entity_ids = create_cable_paths(world, joint_entities, link_types, cw, self.spring_constant)
        assert len(path_entity_ids) == 4

        path_comp1 = world.get_component(path_entity_ids[0], CablePathComponent)
        assert path_comp1.joint_entities == [j[0]]
        assert path_comp1.link_types == ['hybrid-attachment', 'attachment']
        assert path_comp1.cw == [True, False]

        path_comp2 = world.get_component(path_entity_ids[1], CablePathComponent)
        assert path_comp2.joint_entities == [j[1], j[2]]
        assert path_comp2.link_types == ['attachment', 'rolling', 'attachment']
        assert path_comp2.cw == [False, True, False]

        path_comp3 = world.get_component(path_entity_ids[2], CablePathComponent)
        assert path_comp3.joint_entities == [j[3], j[4]]
        assert path_comp3.link_types == ['attachment', 'rolling', 'attachment']
        assert path_comp3.cw == [False, True, False]

        path_comp4 = world.get_component(path_entity_ids[3], CablePathComponent)
        assert path_comp4.joint_entities == [j[5]]
        assert path_comp4.link_types == ['attachment', 'hybrid-attachment']
        assert path_comp4.cw == [False, True]

    def test_should_not_split_if_attachment_link_is_at_start(self, world):
        e0 = world.create_entity()
        _create_mock_link_entity(world, e0)
        e1 = world.create_entity()
        _create_mock_link_entity(world, e1)
        j1 = _create_mock_joint_entity(world, e0, e1)
        path_entity_ids = create_cable_paths(world, [j1], ['attachment', 'rolling'], [True, True], self.spring_constant)
        assert len(path_entity_ids) == 1
        path_comp = world.get_component(path_entity_ids[0], CablePathComponent)
        assert path_comp.link_types == ['attachment', 'rolling']

    def test_should_not_split_if_attachment_link_is_at_end(self, world):
        e0 = world.create_entity()
        _create_mock_link_entity(world, e0, is_rolling=True)
        e1 = world.create_entity()
        _create_mock_link_entity(world, e1)
        j1 = _create_mock_joint_entity(world, e0, e1)
        path_entity_ids = create_cable_paths(world, [j1], ['hybrid-attachment', 'attachment'], [True, True], self.spring_constant)
        assert len(path_entity_ids) == 1
        path_comp = world.get_component(path_entity_ids[0], CablePathComponent)
        assert path_comp.link_types == ['hybrid-attachment', 'attachment']

    def test_should_not_split_if_attachment_links_are_only_at_start_and_end(self, world):
        e0 = world.create_entity()
        _create_mock_link_entity(world, e0)
        e1 = world.create_entity()
        _create_mock_link_entity(world, e1, is_rolling=True)
        e2 = world.create_entity()
        _create_mock_link_entity(world, e2)
        j1 = _create_mock_joint_entity(world, e0, e1)
        j2 = _create_mock_joint_entity(world, e1, e2)
        path_entity_ids = create_cable_paths(world, [j1, j2], ['attachment', 'rolling', 'attachment'], [True, True, True], self.spring_constant)
        assert len(path_entity_ids) == 1
        path_comp = world.get_component(path_entity_ids[0], CablePathComponent)
        assert path_comp.link_types == ['attachment', 'rolling', 'attachment']

    def test_should_split_correctly_with_attachments_at_start_middle_and_end(self, world):
        e = [world.create_entity() for _ in range(4)]
        _create_mock_link_entity(world, e[0])
        _create_mock_link_entity(world, e[1], is_rolling=True, pos=np.array([1.0,0,0]))
        _create_mock_link_entity(world, e[2])
        _create_mock_link_entity(world, e[3], is_rolling=True, pos=np.array([3.0,0,0]))
        
        j = [_create_mock_joint_entity(world, e[i], e[i+1]) for i in range(3)]

        joint_entities = j
        link_types = ['attachment', 'rolling', 'attachment', 'hybrid-attachment']
        cw = [True, True, False, True]
        path_entity_ids = create_cable_paths(world, joint_entities, link_types, cw, self.spring_constant)

        assert len(path_entity_ids) == 2

        path_comp1 = world.get_component(path_entity_ids[0], CablePathComponent)
        assert path_comp1.joint_entities == [j[0], j[1]]
        assert path_comp1.link_types == ['attachment', 'rolling', 'attachment']
        assert path_comp1.cw == [True, True, False]

        path_comp2 = world.get_component(path_entity_ids[1], CablePathComponent)
        assert path_comp2.joint_entities == [j[2]]
        assert path_comp2.link_types == ['attachment', 'hybrid-attachment']
        assert path_comp2.cw == [False, True]

    def test_should_create_one_path_for_single_link_no_joints(self, world):
        link_types = ['hybrid-attachment']
        cw = [True]
        user_stored = [0.5]
        path_entity_ids = create_cable_paths(world, [], link_types, cw, self.spring_constant, user_stored)

        assert len(path_entity_ids) == 1
        path_comp = world.get_component(path_entity_ids[0], CablePathComponent)
        assert path_comp.joint_entities == []
        assert path_comp.link_types == ['hybrid-attachment']
        assert path_comp.cw == cw
        assert path_comp.stored == user_stored
        assert path_comp.total_rest_length == pytest.approx(0.5)

    def test_should_create_one_path_for_single_attachment_link_no_joints(self, world):
        link_types = ['attachment']
        cw = [True]
        path_entity_ids = create_cable_paths(world, [], link_types, cw, self.spring_constant)
        assert len(path_entity_ids) == 1
        path_comp = world.get_component(path_entity_ids[0], CablePathComponent)
        assert path_comp.joint_entities == []
        assert path_comp.link_types == link_types
        assert path_comp.cw == cw
        assert path_comp.stored == [0.0]
        assert path_comp.total_rest_length == pytest.approx(0.0)

    def test_should_return_empty_array_and_warn_for_mismatched_linktypes_and_joints(self, world):
        with pytest.warns(UserWarning, match="len\\(link_types\\) must be len\\(joint_entities\\) \\+ 1"):
            path_entity_ids = create_cable_paths(world, [_create_mock_joint_entity(world, 0, 1)], ['type1'], [True], self.spring_constant)
        assert path_entity_ids == []

    def test_should_return_empty_array_and_warn_for_mismatched_cw_and_linktypes(self, world):
        with pytest.warns(UserWarning, match="len\\(cw\\) must be len\\(link_types\\)"):
            path_entity_ids = create_cable_paths(world, [], ['type1'], [], self.spring_constant)
        assert path_entity_ids == []

    def test_should_return_empty_array_and_warn_for_mismatched_userstored_and_linktypes(self, world):
        with pytest.warns(UserWarning, match="len\\(user_stored\\) must be len\\(link_types\\)"):
            path_entity_ids = create_cable_paths(world, [], ['type1'], [True], self.spring_constant, [])
        assert path_entity_ids == []

    def test_should_return_empty_array_for_completely_empty_inputs(self, world):
        with pytest.warns(UserWarning, match="len\\(link_types\\) must be len\\(joint_entities\\) \\+ 1"):
            path_entity_ids = create_cable_paths(world, [], [], [], self.spring_constant)
        assert path_entity_ids == []
