"""Topology activation, lifecycle and ownership beyond numeric equivalence."""
import copy
import json

import numpy as np
import pytest

from cable_joints_3d.cable_joints_components import CableJointComponent, CableLinkComponent, CablePathComponent, create_cable_path_component
from cable_joints_3d.cable_topology import merge_joints, split_joints
from cable_joints_3d.ecs import MachineTagComponent, PositionComponent, RadiusComponent, World
from parity_harness import FIXTURES, assert_equivalent, run_js, run_python


def fixture(name):
    return json.loads((FIXTURES / f'topology_{name}.json').read_text())


def cable(snapshot):
    return snapshot['entities']['path']['CablePathComponent']


@pytest.mark.parametrize('name', ['split_multiple', 'split_reverse'])
def test_oracle_exercises_splitter_order_and_live_kept_attachments(name):
    snapshots = run_js(fixture(name))['snapshots']
    after = snapshots[1]
    joints = cable(after)['jointEntities']
    assert joints == (['joint', '@6', '@7'] if name == 'split_multiple' else ['joint', '@7', '@6'])
    assert [after['entities'][entity]['CableJointComponent']['entityB'] for entity in joints] == ['left', 'right', 'b']
    assert after['queries'][0] == ['joint', '@6', '@7']
    assert after['allocator']['nextEntityId'] == 8
    for entity in joints[1:]:
        assert after['entities'][entity]['MachineTagComponent']['id'] == '1'
        assert after['entities'][entity]['CableJointComponent']['constraintForceMagnitude'] == 0


def test_oracle_preserves_allocator_and_state_when_split_aborts():
    data = fixture('split_abort')
    before, aborted, succeeded, repeated = run_js(data)['snapshots']
    assert aborted | {'step': 0} == before
    assert cable(succeeded)['jointEntities'] == ['joint', '@6']
    assert succeeded['allocator']['nextEntityId'] == 7
    assert repeated | {'step': 2} == succeeded


def test_oracle_covers_layered_endpoint_and_live_tilted_member_frame():
    layered = run_js(fixture('split_layered'))['snapshots'][1]
    assert len(cable(layered)['jointEntities']) == 2
    point = layered['entities']['joint']['CableJointComponent']['attachmentPointA_world']
    assert np.linalg.norm(point) == pytest.approx(1.3, abs=1e-12)
    point = layered['entities']['joint']['CableJointComponent']['attachmentPointB_world']
    center = layered['entities']['guide']['PositionComponent']['pos']
    # A newly encountered guide retains its raw radius until it joins the path.
    assert np.linalg.norm(np.array(point) - center) == pytest.approx(.4, abs=1e-12)
    member = run_js(fixture('split_rigid_member'))['snapshots'][1]
    assert len(cable(member)['jointEntities']) == 2
    assert member['entities']['guide']['PositionComponent']['pos'] == [99, 99, 99]
    assert member['attachments'][0]['worldPoint'] == pytest.approx([.4, -.2, 1.3])
    assert abs(member['entities']['joint']['CableJointComponent']['attachmentPointB_world'][2] - 1.3) > .1


@pytest.mark.parametrize('name', ['attachment_cycle', 'solver_cycle', 'merge_cascade'])
def test_oracle_exercises_removal_and_conserves_total_cable(name):
    snapshots = run_js(fixture(name))['snapshots']
    counts = [len(cable(s)['jointEntities']) for s in snapshots]
    assert counts == ([3, 1, 1] if name == 'merge_cascade' else [1, 2, 1, 2, 1, 2] + ([2, 2, 2] if name == 'solver_cycle' else []))
    total = cable(snapshots[0])['totalRestLength']
    for snapshot in snapshots:
        path = cable(snapshot)
        assert path['totalRestLength'] == total
        joints = [snapshot['entities'][entity]['CableJointComponent'] for entity in path['jointEntities']]
        assert sum(joint['restLength'] for joint in joints) + sum(path['stored']) == pytest.approx(total, abs=1e-11)
    if name == 'merge_cascade':
        assert 'middle' not in snapshots[1]['entities']
        assert 'last' not in snapshots[1]['entities']
    else:
        assert '@5' not in snapshots[2]['entities']
        assert snapshots[2]['queries'][0] == ['joint']
        assert cable(snapshots[3])['jointEntities'] == ['joint', '@6']
    if name == 'solver_cycle':
        assert snapshots[1]['entities']['@5']['CableJointComponent']['constraintForceMagnitude'] > 1
        assert snapshots[3]['entities']['b']['PositionComponent']['pos'] != snapshots[0]['entities']['b']['PositionComponent']['pos']
        assert snapshots[3]['entities']['guide']['EncoderComponent']['angle'] != 0
        assert snapshots[7] | {'step': 6} == snapshots[6]


def test_oracle_respects_feature_flags_pause_and_error_before_topology():
    snapshots = run_js(fixture('feature_flags'))['snapshots']
    assert [len(cable(s)['jointEntities']) for s in snapshots] == [1, 1, 1, 2, 2, 1, 1, 1, 2]
    assert [s['resources']['cableHybridTransitionStep'] for s in snapshots] == [None, 1, 2, 3, 4, 5, 5, 5, 6]


def test_untagged_paths_create_untagged_namespace_joint():
    data = fixture('attachment_cycle')
    data['steps'] = data['steps'][:1]
    for entity in data['entities']:
        entity['components'].pop('MachineTagComponent', None)
    expected = run_js(data)
    assert expected['snapshots'][1]['entities']['@5']['MachineTagComponent']['id'] == ''
    assert_equivalent(run_python(data), expected, **data['tolerance'])


def test_comparator_rejects_deleted_entity_allocator_and_relationship_drift():
    data = fixture('attachment_cycle')
    expected = run_js(data)
    for mutate in [
        lambda s: s[2]['entities'].update({'@5': {}}),
        lambda s: s[2]['allocator'].update({'nextEntityId': 5}),
        lambda s: cable(s[1])['jointEntities'].reverse(),
    ]:
        changed = copy.deepcopy(expected)
        mutate(changed['snapshots'])
        with pytest.raises(AssertionError):
            assert_equivalent(changed, expected, **data['tolerance'])


@pytest.mark.parametrize('name', ['attachment_cycle', 'solver_cycle'])
def test_topology_cycles_match_js_and_repeat_exactly_for_200_steps(name):
    data = fixture(name)
    data['steps'] = [copy.deepcopy(data['steps'][index % 5]) for index in range(200)]
    expected, actual = run_js(data), run_python(data)
    assert_equivalent(actual, expected, **data['tolerance'])
    assert expected == run_js(data)
    assert actual == run_python(data)
    assert expected['snapshots'][-1]['allocator']['nextEntityId'] > 10


def make_topology_world():
    world = World()
    a, b, guide, joint_id, path_id = [world.create_entity() for _ in range(5)]
    for entity, x in [(a, -4), (b, 4), (guide, 0)]:
        world.add_component(entity, PositionComponent(x, 0, 0))
        world.add_component(entity, CableLinkComponent(x, 0, 0))
    world.add_component(guide, RadiusComponent(1.5))
    joint = CableJointComponent(a, b, 8, np.array([-4., 0., 0.]), np.array([4., 0., 0.]))
    world.add_component(joint_id, joint)
    path = create_cable_path_component(world, [joint_id], ['attachment', 'attachment'], [False, False])
    world.add_component(path_id, path)
    return world, joint_id, path_id


def test_native_split_merge_preserves_mutable_storage_and_cleans_components():
    world, joint_id, path_id = make_topology_world()
    joint = world.get_component(joint_id, CableJointComponent)
    path = world.get_component(path_id, CablePathComponent)
    lists = [path.joint_entities, path.stored, path.cw, path.link_types]
    points = [joint.attachment_point_a_world, joint.attachment_point_b_world]
    joint.constraint_force[:] = [2, 3, 4]
    total = path.total_rest_length
    split_joints(world)
    new_id = path.joint_entities[1]
    new_joint = world.get_component(new_id, CableJointComponent)
    assert world.get_component(joint_id, CableJointComponent) is joint
    assert all(after is before for after, before in zip([path.joint_entities, path.stored, path.cw, path.link_types], lists))
    assert joint.attachment_point_a_world is points[0]
    assert joint.attachment_point_b_world is points[1]
    assert np.array_equal(joint.constraint_force, [2, 3, 4])
    assert not np.shares_memory(points[1], new_joint.attachment_point_a_world)
    assert not np.shares_memory(new_joint.constraint_force, joint.constraint_force)
    assert world.get_component(new_id, MachineTagComponent).id == ''
    path.stored[1] = -.01
    merge_joints(world)
    assert path.joint_entities == [joint_id]
    assert all(after is before for after, before in zip([path.joint_entities, path.stored, path.cw, path.link_types], lists))
    assert joint.attachment_point_a_world is points[0]
    assert joint.attachment_point_b_world is points[1]
    assert path.total_rest_length == total
    assert new_id not in world.entities
    assert all(new_id not in store for store in world.components.values())
    split_joints(world)
    assert path.joint_entities[1] == new_id + 1
