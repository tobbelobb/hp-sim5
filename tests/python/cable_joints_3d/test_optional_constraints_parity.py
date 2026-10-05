"""Executable-reference guards for optional contacts and ordered cable passes."""
import copy
import json

import numpy as np
import pytest

from cable_joints_3d.ball_obstacle_bump_system import BallObstacleBumpSystem
from cable_joints_3d.ecs import BallTagComponent, MassComponent, ObstaclePushComponent, ObstacleTagComponent, PositionComponent, RadiusComponent, VelocityComponent, World
from cable_joints_3d.pbd_ball_collisions import PBDBallObstacleCollisions
from parity_harness import FIXTURES, assert_equivalent, run_js, run_python


def fixture(name):
    return json.loads((FIXTURES / f'{name}.json').read_text())


def test_ball_oracle_reaches_mass_and_distance_gates_and_sequential_order():
    data = fixture('collision_ball_pairs')
    expected = run_js(data)
    before, after = [s['entities'] for s in expected['snapshots'][:2]]
    for prefix in ['dynamic', 'negative', 'chain']:
        assert after[prefix + '_b']['PositionComponent'] != before[prefix + '_b']['PositionComponent']
    assert after['static_a']['PositionComponent'] == before['static_a']['PositionComponent']
    assert after['static_b']['PositionComponent'] != before['static_b']['PositionComponent']
    for prefix in ['immovable', 'threshold', 'coincident', 'missing_mass', 'separate', 'touching']:
        for suffix in ['_a', '_b']:
            assert after[prefix + suffix]['PositionComponent'] == before[prefix + suffix]['PositionComponent']
    # Mass is the smallest query store here. Reverse its insertion order without
    # changing any numeric initial data; sequential three-ball resolution changes.
    additions = []
    for entity in data['entities']:
        args = entity['components'].pop('MassComponent', None)
        if args is not None:
            additions.append([entity['name'], 'MassComponent', args])
    data['addComponents'] = additions[::-1]
    reversed_order = run_js(data)
    assert reversed_order['snapshots'][0]['queries'][0] == expected['snapshots'][0]['queries'][0][::-1]
    assert reversed_order['snapshots'][1]['entities']['chain_b']['PositionComponent'] != after['chain_b']['PositionComponent']
    assert_equivalent(run_python(data), reversed_order, **data['tolerance'])


def test_obstacle_oracle_records_ordered_contacts_and_geometric_zero_mass_push():
    snapshots = run_js(fixture('collision_obstacles'))['snapshots']
    contacts = snapshots[1]['resources']['ball_obstacle_contacts']
    assert [(c['ball_id'], c['obs_id']) for c in contacts] == [('ball', 'first'), ('ball', 'second'), ('second_ball', 'static')]
    assert snapshots[2]['resources']['ball_obstacle_contacts'] == []
    for name in ['ball', 'second_ball']:
        assert snapshots[1]['entities'][name]['PositionComponent'] != snapshots[0]['entities'][name]['PositionComponent']
    for name in ['coincident_ball', 'missing_push_ball']:
        assert snapshots[1]['entities'][name]['PositionComponent'] == snapshots[0]['entities'][name]['PositionComponent']


def test_bump_oracle_exercises_world_tensors_friction_and_raw_hit_filter():
    snapshots = run_js(fixture('collision_bump'))['snapshots']
    state = lambda step, prefix, component: snapshots[step]['entities'][prefix + '_ball'][component]
    omega = lambda step, prefix: state(step, prefix, 'AngularVelocityComponent')
    velocity = lambda step, prefix: state(step, prefix, 'VelocityComponent')
    for prefix in ['tensor', 'scalar', 'override', 'tiny_spin']:
        assert omega(1, prefix) != omega(0, prefix)
        assert snapshots[1]['entities'][prefix + '_obstacle']['AngularVelocityComponent'] != snapshots[0]['entities'][prefix + '_obstacle']['AngularVelocityComponent']
    assert omega(1, 'tensor') != omega(1, 'scalar')
    for prefix in ['zero_inertia', 'zero_mass', 'negative_mass', 'missing_required', 'zero_tangent', 'zero_normal', 'negative_mu', 'raw_filtered']:
        assert omega(1, prefix) == omega(0, prefix)
    assert velocity(1, 'zero_mass') != velocity(0, 'zero_mass')
    assert velocity(1, 'missing_required') == velocity(0, 'missing_required')
    assert velocity(1, 'raw_filtered') == velocity(0, 'raw_filtered')
    assert velocity(2, 'raw_filtered') != velocity(1, 'raw_filtered')
    assert velocity(3, 'raw_filtered') != velocity(2, 'raw_filtered')
    assert velocity(5, 'raw_filtered') == velocity(4, 'raw_filtered')
    assert snapshots[4] | {'step': 3} == snapshots[3]
    assert snapshots[1]['resources']['ball_obstacle_contacts'] == snapshots[0]['resources']['ball_obstacle_contacts']


def test_contact_pipeline_oracle_exercises_post_pbd_push_and_encoder():
    data = fixture('collision_pipeline')
    expected = run_js(data)
    assert expected['snapshots'][1]['resources']['ball_obstacle_contacts']
    data['systems'].remove('BallObstacleBumpSystem')
    unpushed = run_js(data)
    ball = expected['snapshots'][1]['entities']['ball']
    assert ball['VelocityComponent'] != unpushed['snapshots'][1]['entities']['ball']['VelocityComponent']
    assert ball['AngularVelocityComponent'] != unpushed['snapshots'][1]['entities']['ball']['AngularVelocityComponent']
    assert ball['PositionComponent'] == unpushed['snapshots'][1]['entities']['ball']['PositionComponent']
    assert ball['EncoderComponent']['angle'] != 0
    assert_equivalent(run_python(data), unpushed, **data['tolerance'])


@pytest.mark.parametrize('name', ['slack_pinhole', 'slack_slide'])
def test_slack_oracle_covers_single_ordered_pass_and_literal_attachment_gate(name):
    snapshots = run_js(fixture(name))['snapshots']
    before, after = [s['entities'] for s in snapshots[:2]]
    for entity, state in after.items():
        if 'CablePathComponent' not in state:
            continue
        joints = state['CablePathComponent']['jointEntities']
        assert sum(after[j]['CableJointComponent']['restLength'] for j in joints) == pytest.approx(
            sum(before[j]['CableJointComponent']['restLength'] for j in joints), abs=1e-12)
    for prefix in ['pinhole_loose', 'pinhole_reverse']:
        assert after[prefix + '_joint0']['CableJointComponent']['restLength'] == 3
        assert after[prefix + '_joint1']['CableJointComponent']['restLength'] == 4
    assert after['fixed_loose_joint0']['CableJointComponent'] == before['fixed_loose_joint0']['CableJointComponent']
    if name == 'slack_pinhole':
        assert after['pinhole_joint0']['CableJointComponent']['restLength'] < 10
        assert after['hybrid_loose_joint0']['CableJointComponent'] == before['hybrid_loose_joint0']['CableJointComponent']
        assert after['chain_joint0']['CableJointComponent']['restLength'] == pytest.approx(2.4)
    else:
        assert after['pinhole_joint0']['CableJointComponent']['restLength'] == 10
        assert after['hybrid_loose_joint0']['CableJointComponent']['restLength'] == 3
        assert after['chain_joint0']['CableJointComponent']['restLength'] == 3
    assert snapshots[4] | {'step': 3} == snapshots[3]


@pytest.mark.parametrize('name', ['slack_pinhole_pipeline', 'slack_slide_pipeline'])
def test_optional_slack_changes_real_solver_state(name):
    data = fixture(name)
    expected = run_js(data)
    data['systems'] = [system for system in data['systems'] if system not in ['CableSlackSystem', 'SlideLooseCableSystem']]
    baseline = run_js(data)
    assert any('CableJointComponent' in state and state['CableJointComponent'] != baseline['snapshots'][1]['entities'][entity]['CableJointComponent']
               for entity, state in expected['snapshots'][1]['entities'].items())
    assert expected['snapshots'][1]['entities']['body']['PositionComponent'] != baseline['snapshots'][1]['entities']['body']['PositionComponent']
    assert expected['snapshots'][1]['entities']['right']['CableJointComponent']['constraintForceMagnitude'] > 0


@pytest.mark.parametrize('name', ['collision_ball_pairs', 'collision_obstacles', 'collision_bump', 'collision_pipeline',
                                 'slack_pinhole', 'slack_slide', 'slack_pinhole_pipeline', 'slack_slide_pipeline'])
def test_optional_constraints_match_js_and_repeat_exactly_for_200_steps(name):
    data = fixture(name)
    data['steps'] = [{'dt': .002, 'resources': {'dt': .002}} for _ in range(200)]
    expected, actual = run_js(data), run_python(data)
    assert_equivalent(actual, expected, **data['tolerance'])
    assert expected == run_js(data)
    assert actual == run_python(data)


def test_native_contact_list_and_direction_storage_remain_owned():
    world = World()
    ball, obstacle = world.create_entity(), world.create_entity()
    for entity, components in [(ball, [BallTagComponent(), PositionComponent(.7, .2, .1), RadiusComponent(.5), MassComponent(1), VelocityComponent(1, 0, 0)]),
                               (obstacle, [ObstacleTagComponent(), PositionComponent(), RadiusComponent(1), ObstaclePushComponent()])]:
        for component in components:
            world.add_component(entity, component)
    contacts = [{'old': True}]
    world.set_resource('ball_obstacle_contacts', contacts)
    collision = PBDBallObstacleCollisions()
    collision.update(world, .002)
    assert world.get_resource('ball_obstacle_contacts') is contacts
    saved = contacts[0]['direction'].copy()
    BallObstacleBumpSystem().update(world, .002)
    assert np.array_equal(contacts[0]['direction'], saved)
    world.get_component(ball, PositionComponent).pos[:] = 10
    assert np.array_equal(contacts[0]['direction'], saved)
    collision.update(world, .002)
    assert world.get_resource('ball_obstacle_contacts') is contacts
    assert contacts == []
