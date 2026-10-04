import copy
import json
from contextlib import nullcontext

import numpy as np
import pytest

from parity_harness import FIXTURES, assert_equivalent, run_js, run_python


@pytest.mark.parametrize('fixture_path', sorted(FIXTURES.glob('*.json')), ids=lambda p: p.stem)
def test_live_js_differential(fixture_path):
    fixture = json.loads(fixture_path.read_text())
    expected = run_js(fixture)
    warning = pytest.warns(UserWarning, match='Insufficient available rest length') if fixture_path.stem == 'topology_split_abort' else nullcontext()
    with warning:
        actual = run_python(fixture)
    assert_equivalent(actual, expected, **fixture['tolerance'], path=fixture_path.stem)


def test_comparator_rejects_changed_physics_and_structure():
    fixture = json.loads((FIXTURES / 'motion.json').read_text())
    expected = run_js(fixture)
    changed = copy.deepcopy(expected)
    changed['snapshots'][1]['entities']['dynamic']['PositionComponent']['pos'][0] += .01
    with pytest.raises(AssertionError, match='snapshots.*PositionComponent.pos'):
        assert_equivalent(changed, expected, **fixture['tolerance'])
    changed = copy.deepcopy(expected)
    del changed['snapshots'][0]['entities']['dynamic']['VelocityComponent']
    with pytest.raises(AssertionError, match='keys differ'):
        assert_equivalent(changed, expected, **fixture['tolerance'])


def test_pbd_oracle_recovers_small_rotations_without_changing_its_cutoff():
    fixture = json.loads((FIXTURES / 'pbd_small_rotations.json').read_text())
    snapshots = run_js(fixture)['snapshots']
    omega = lambda name: snapshots[1]['entities'][name]['AngularVelocityComponent']['omega']
    assert omega('x') == pytest.approx([5e-6, 0, 0], abs=1e-14)
    assert omega('negative_y') == pytest.approx([0, -5e-6, 0], abs=1e-14)
    assert omega('opposite_sign') == pytest.approx([0, 0, 5e-6], abs=1e-14)
    assert omega('below_cutoff') == [0, 0, 0]
    assert snapshots[2]['entities'] == snapshots[1]['entities']


@pytest.mark.parametrize('name', ['cable_attachment_clamp', 'cable_attachment_members', 'cable_hybrid_transitions'])
def test_reference_fixtures_exercise_the_targeted_behavior(name):
    fixture = json.loads((FIXTURES / f'{name}.json').read_text())
    snapshots = run_js(fixture)['snapshots']
    entities = [s['entities'] for s in snapshots]
    if name == 'cable_attachment_clamp':
        for path in ['rolling_clamp', 'hybrid_clamp']:
            assert entities[2][path + '_joint']['CableJointComponent']['restLength'] == pytest.approx(1e-6, abs=1e-12)
        requested_orientation = fixture['steps'][1]['set'][0][3]
        assert entities[2]['ha']['RigidBodyMemberComponent']['localOrientation'] != requested_orientation
    elif name == 'cable_attachment_members':
        for state in entities[1:5]:
            assert state['internal']['CablePathComponent']['stored'][1] == pytest.approx(.08, abs=1e-12)
        assert entities[4]['external']['CablePathComponent']['stored'] != entities[0]['external']['CablePathComponent']['stored']
        assert entities[5]['internal']['CablePathComponent']['stored'][1] != pytest.approx(.08)
        assert entities[5]['spool']['EncoderComponent']['angle'] != pytest.approx(0)
    else:
        for path in ['unwind_first', 'unwind_last', 'zero_radius']:
            assert 'hybrid-attachment' in entities[1][path]['CablePathComponent']['linkTypes']
        for path in ['rewrap_first', 'rewrap_last']:
            assert 'hybrid' in entities[1][path]['CablePathComponent']['linkTypes']
        for path in ['hysteresis', 'tiny_arc', 'degenerate']:
            assert entities[1][path]['CablePathComponent']['linkTypes'] == entities[0][path]['CablePathComponent']['linkTypes']


@pytest.mark.parametrize('name', ['cable_solver_spools', 'cable_solver_pinhole',
                                 'cable_solver_pinhole_stale_inlet', 'cable_zero_stiffness'])
def test_solver_oracle_reaches_spin_load_and_limit_cases(name):
    fixture = json.loads((FIXTURES / f'{name}.json').read_text())
    snapshot = run_js(fixture)['snapshots'][1]
    entities, maps = snapshot['entities'], snapshot['entityMaps']
    if name == 'cable_zero_stiffness':
        assert entities['undamped_joint']['CableJointComponent']['constraintForceMagnitude'] == 0
        assert entities['damped_joint']['CableJointComponent']['constraintForceMagnitude'] > 0
        assert entities['damped_b']['PositionComponent']['pos'] == pytest.approx(
            entities['near_zero_b']['PositionComponent']['pos'], abs=1e-10)
    elif name == 'cable_solver_spools':
        angle = lambda name: entities[name + '_spool']['EncoderComponent']['angle']
        assert angle('release') == pytest.approx(angle('free'), abs=1e-10)
        assert angle('free') < angle('held') < angle('closed')
        assert entities['fixed_torque_joint']['CableJointComponent']['constraintForceMagnitude'] == 0
        assert maps['torqueModeCableLoadTorques']['fixed_torque_spool'] < 0
        # JS accumulates torque loads on every iteration, even with no mechanical
        # DOF, while joint telemetry records only the first iteration.
        fixed_path = next(e for e in fixture['entities'] if e['name'] == 'fixed_torque')
        fixed_path['components']['CablePathComponent'][-1] = 1
        once = run_js(fixture)['snapshots'][1]['entityMaps']['torqueModeCableLoadTorques']['fixed_torque_spool']
        assert maps['torqueModeCableLoadTorques']['fixed_torque_spool'] == pytest.approx(3 * once)
    else:
        for prefix, internal, external in [('upstream', 0, 1), ('downstream', 1, 0),
                                           ('torque_up', 0, 1), ('torque_down', 1, 0), ('rolling', 0, 1)]:
            inside = entities[f'{prefix}_j{internal}']['CableJointComponent']
            outside = entities[f'{prefix}_j{external}']['CableJointComponent']
            assert outside['constraintForceMagnitude'] > 0
            assert inside['transferredConstraintForceMagnitude'] == outside['constraintForceMagnitude']
        assert maps['torqueModeCableLoadTorques']['torque_up_spool'] < 0
        assert maps['torqueModeCableLoadTorques']['torque_down_spool'] > 0


@pytest.mark.parametrize('name', ['cable_over_correction', 'cable_over_correction_members',
                                 'cable_over_correction_pinhole'])
def test_over_correction_oracle_reaches_shared_reactions(name):
    fixture = json.loads((FIXTURES / f'{name}.json').read_text())
    snapshots = run_js(fixture)['snapshots']
    before, after = [s['entities'] for s in snapshots[:2]]
    if name == 'cable_over_correction':
        for prefix in ['rolling', 'hybrid']:
            assert after[prefix + '_wheel']['PositionComponent'] != before[prefix + '_wheel']['PositionComponent']
        for prefix in ['single', 'zero', 'slack_before', 'shared']:
            assert after[prefix + '_wheel']['PositionComponent'] == before[prefix + '_wheel']['PositionComponent']
        assert snapshots[7]['entities'] == snapshots[6]['entities']
        # All paths share just one qualifying joint; the global gate also applies.
        fixture['entities'] = [e for e in fixture['entities'] if e['name'].startswith('single')]
        assert_equivalent(run_python(fixture), run_js(fixture), **fixture['tolerance'])
    elif name == 'cable_over_correction_members':
        for prefix in ['external_first', 'external_last']:
            assert after[prefix + '_body']['PositionComponent'] != before[prefix + '_body']['PositionComponent']
            assert after[prefix + '_spool']['OrientationComponent'] != before[prefix + '_spool']['OrientationComponent']
            assert after[prefix + '_spool']['PositionComponent'] == before[prefix + '_spool']['PositionComponent']
        assert after['internal_body']['OrientationComponent'] != before['internal_body']['OrientationComponent']
        assert after['internal_spool']['OrientationComponent'] == before['internal_spool']['OrientationComponent']
        assert after['internal_full_tensor_spool']['OrientationComponent'] != before['internal_full_tensor_spool']['OrientationComponent']
    else:
        for prefix in ['upstream', 'downstream', 'attached']:
            assert after[prefix + '_spool']['OrientationComponent'] != before[prefix + '_spool']['OrientationComponent']
        assert after['rolling_spool']['OrientationComponent'] == before['rolling_spool']['OrientationComponent']


@pytest.mark.parametrize('name', ['position_motor_standalone', 'position_motor_members'])
def test_position_motor_oracle_exercises_reactions_and_integration(name):
    fixture = json.loads((FIXTURES / f'{name}.json').read_text())
    snapshots = run_js(fixture)['snapshots']
    before, after = [s['entities'] for s in snapshots[:2]]
    if name == 'position_motor_standalone':
        assert after['open']['OrientationComponent'] == before['open']['OrientationComponent']
        assert after['open']['AngularVelocityComponent'] != before['open']['AngularVelocityComponent']
        for prefix in ['torque', 'zero']:
            assert after[prefix]['OrientationComponent'] == before[prefix]['OrientationComponent']
            assert after[prefix]['AngularVelocityComponent'] == before[prefix]['AngularVelocityComponent']
        for prefix in ['closed', 'zero_closed']:
            assert after[prefix]['OrientationComponent'] != before[prefix]['OrientationComponent']
            assert after[prefix]['AngularVelocityComponent']['omega'] == [0, 0, 0]
    else:
        assert after['open_body']['AngularVelocityComponent']['omega'] != [0, 0, 0]
        assert after['open_body']['AngularVelocityComponent']['omega'] == pytest.approx(
            after['live_mass_body']['AngularVelocityComponent']['omega'], abs=1e-12)
        for prefix in ['closed', 'fallback']:
            assert after[prefix + '_body']['OrientationComponent'] != before[prefix + '_body']['OrientationComponent']
        assert after['open_body']['OrientationComponent'] == before['open_body']['OrientationComponent']
        # AngularMovement may move the host but must not integrate member rotors again.
        fixture['steps'] = fixture['steps'][:1]
        fixture['systems'].append('AngularMovementSystem')
        moved = run_js(fixture)
        assert moved['snapshots'][1]['entities']['open_spool']['RigidBodyMemberComponent'] == after['open_spool']['RigidBodyMemberComponent']
        assert_equivalent(run_python(fixture), moved, **fixture['tolerance'])


def test_stiff_position_motor_integration_at_small_timestep():
    fixture = json.loads((FIXTURES / 'position_motor_cables.json').read_text())
    motor = next(e for e in fixture['entities'] if e['name'] == 'held_spool')
    motor['components']['StepperMotorComponent'][2] = 100
    fixture['steps'] = [{'dt': .00002, 'resources': {'dt': .00002}} for _ in range(200)]
    fixture['steps'][0]['set'] = [['held_spool', 'StepperMotorComponent', 'commandedAngle', .07]]
    assert_equivalent(run_python(fixture), run_js(fixture), **fixture['tolerance'])


@pytest.mark.parametrize('name', ['torque_motor_standalone', 'torque_motor_loads',
                                 'torque_motor_members', 'torque_motor_cables'])
def test_torque_motor_oracle_exercises_loads_and_drive_only_reactions(name):
    fixture = json.loads((FIXTURES / f'{name}.json').read_text())
    snapshots = run_js(fixture)['snapshots']
    before, after = [s['entities'] for s in snapshots[:2]]
    speed = lambda name: np.linalg.norm(after[name]['AngularVelocityComponent']['omega'])
    if name == 'torque_motor_standalone':
        for prefix in ['fast', 'reverse_fast']:
            assert speed(prefix) < np.linalg.norm(before[prefix]['AngularVelocityComponent']['omega'])
        for prefix in ['zero', 'position']:
            assert after[prefix]['AngularVelocityComponent'] == before[prefix]['AngularVelocityComponent']
        assert speed('no_losses') > speed('forward')
        assert after['tuned']['AngularVelocityComponent'] != after['reverse']['AngularVelocityComponent']
        assert speed('rest') > 0
        assert after['rest']['OrientationComponent'] == before['rest']['OrientationComponent']
    elif name == 'torque_motor_loads':
        assert after['invalid']['AngularVelocityComponent'] == after['free']['AngularVelocityComponent']
        assert after['negative_coefficients']['AngularVelocityComponent'] == after['signed']['AngularVelocityComponent']
        assert speed('implicit') < speed('signed') < speed('free')
        # Both supported JS resource encodings have identical physics.
        fixture['steps'] = fixture['steps'][:1]
        for definition in fixture['entityResources'].values():
            definition['kind'] = 'object'
        objects = run_js(fixture)
        assert objects['snapshots'][1]['entities'] == after
        assert_equivalent(run_python(fixture), objects, **fixture['tolerance'])
    elif name == 'torque_motor_members':
        assert speed('live_mass_spool') < speed('open_spool')
        assert after['live_mass_body']['AngularVelocityComponent'] == after['open_body']['AngularVelocityComponent']
        assert after['open_body']['OrientationComponent'] == before['open_body']['OrientationComponent']
        assert after['open_spool']['OrientationComponent'] != before['open_spool']['OrientationComponent']
        assert after['open_spool']['EncoderComponent'] == before['open_spool']['EncoderComponent']
    else:
        assert snapshots[2]['entityMaps']['torqueModeCableLoadTorques']['held_spool'] != 0
        assert 'held_spool' not in snapshots[3]['entityMaps']['torqueModeCableLoadTorques']
        assert snapshots[4]['entities']['held_spool']['StepperMotorComponent']['torqueMode'] is False
        assert snapshots[4]['entities']['held_spool']['StepperMotorComponent']['closedLoop'] is True
        assert snapshots[5]['entities']['held_spool']['StepperMotorComponent']['torqueMode'] is True


def test_diagnostics_oracle_preserves_full_turn_slips_and_rounding():
    fixture = json.loads((FIXTURES / 'motor_diagnostics_encoders.json').read_text())
    snapshots = run_js(fixture)['snapshots']
    motor = lambda step, name: snapshots[step]['entities'][name]['StepperMotorComponent']
    assert motor(0, 'positive_half')['currentMissedSteps'] == 1
    assert motor(0, 'negative_half')['currentMissedSteps'] == 0
    assert motor(0, 'rounded_pairs')['currentMissedSteps'] == 1
    assert motor(0, 'peak')['missedSteps'] == 3
    assert motor(1, 'slip')['currentMissedSteps'] == 50
    assert motor(2, 'slip')['currentMissedSteps'] == 0
    assert motor(2, 'slip')['missedSteps'] == 50
    assert motor(3, 'slip')['currentMissedSteps'] == 0
    assert motor(3, 'slip')['missedSteps'] == 3
    assert motor(4, 'slip')['missedStepEncoderOffset'] is None
    # A diagnostic read updates encoder-derived state even while World is paused.
    assert motor(6, 'slip')['currentMissedSteps'] == 100
    assert motor(7, 'slip')['currentMissedSteps'] == 0


@pytest.mark.parametrize('name', ['extruder_rigid_constraints', 'extruder_rigid_fallback'])
def test_extruder_oracle_reads_live_constrained_members(name):
    fixture = json.loads((FIXTURES / f'{name}.json').read_text())
    entities = run_js(fixture)['snapshots'][1]['entities']
    extruder = entities['extruder']['ExtruderComponent']
    assert extruder['effectorCenterPos'] == pytest.approx(entities['body']['PositionComponent']['pos'])
    assert entities['member0']['PositionComponent']['pos'] == [-1, -1 / 3, 0]
    assert np.linalg.norm(extruder['effectorCenterPos']) > 1
    offset = np.array(extruder['tipPos']) - extruder['centerPos']
    if name == 'extruder_rigid_fallback':
        assert offset == pytest.approx([.1, 0, -.1])
    else:
        assert abs(offset[1]) > .01


def test_extruder_oracle_selects_numeric_machine_keys_in_js_order():
    fixture = json.loads((FIXTURES / 'extruder_frames.json').read_text())
    snapshots = run_js(fixture)['snapshots']
    extruder = snapshots[1]['entities']['extruder']['ExtruderComponent']
    assert extruder['effectorCenterPos'] == extruder['machineEffectorCenters']['1']
    assert extruder['effectorCenterPos'] != extruder['machineEffectorCenters']['2']
    assert snapshots[4] | {'step': 3} == snapshots[3]
    assert snapshots[1]['entities']['ignored_extruder'] == snapshots[0]['entities']['ignored_extruder']


def test_command_oracle_exercises_order_modes_history_and_callbacks():
    fixture = json.loads((FIXTURES / 'commands_state.json').read_text())
    snapshots = run_js(fixture)['snapshots']
    first = snapshots[1]
    assert first['commandState']['queueLength'] == 12
    assert first['commandState']['history'][0]['__touchedMachines'] == ['alpha', 'beta', 'other']
    assert '__touchedMachines' not in first['commandEvents'][0]['value']
    extrusions = first['entities']['extruder']['ExtruderComponent']['extrusions']
    assert [entry['color'] for entry in extrusions] == ['#00aa00', '#222222', '#ff00ff']
    assert snapshots[2] | {'step': 1} == first
    assert snapshots[3]['entities']['alphaA']['StepperMotorComponent']['torqueMode'] is True
    assert snapshots[4]['entities']['alphaA']['StepperMotorComponent']['commandedAngle'] == .03
    assert snapshots[4]['entities']['alphaB']['StepperMotorComponent']['commandedAngle'] == .05
    assert snapshots[5]['entities']['alphaA']['StepperMotorComponent']['deltaAngle'] == .02
    assert len(snapshots[-1]['commandState']['history']) == 12  # null consumes a step without history
    assert snapshots[-1]['commandState']['queueLength'] == 0


def test_command_oracle_preserves_numeric_axis_order_and_playback_state():
    fixture = json.loads((FIXTURES / 'commands_numeric_order.json').read_text())
    state = run_js(fixture)['snapshots'][1]
    assert state['commandState']['history'][0]['__touchedMachines'] == ['1', '2']
    assert [entry['machineId'] for entry in state['entities']['extruder']['ExtruderComponent']['extrusions']] == ['1', '2']
    fixture = json.loads((FIXTURES / 'commands_playback.json').read_text())
    snapshots = run_js(fixture)['snapshots']
    assert 'D' not in snapshots[2]['commandState']['axisToEntity']
    assert 'D' in snapshots[3]['commandState']['axisToEntity']
    assert snapshots[4]['commandState']['queueLength'] == 1
    assert snapshots[4]['commandState']['history'][0]['type'] == 'restored'
    assert snapshots[7]['commandState']['history'] == []
    assert snapshots[7]['entities']['extruder']['ExtruderComponent']['extrusions']


def test_extrusion_oracle_records_the_tip_before_current_step_physics():
    fixture = json.loads((FIXTURES / 'commands_extrusion_order.json').read_text())
    snapshots = run_js(fixture)['snapshots']
    extruder = lambda step: snapshots[step]['entities']['extruder']['ExtruderComponent']
    for step in [1, 2, 4, 5]:
        assert extruder(step)['extrusions'][-1]['pos'] == extruder(step - 1)['tipPos']
        assert extruder(step)['tipPos'] != extruder(step - 1)['tipPos']
    assert snapshots[3] | {'step': 2} == snapshots[2]


@pytest.mark.parametrize('name', ['rigid_members', 'distance_members', 'spool_projection',
                                 'cable_cache_members', 'cable_friction_chain',
                                 'cable_attachment_motion', 'cable_attachment_members',
                                 'cable_solver_bodies', 'cable_solver_spools',
                                 'cable_solver_pinhole', 'cable_solver_pinhole_stale_inlet',
                                 'cable_over_correction', 'cable_over_correction_members',
                                 'cable_over_correction_pinhole', 'position_motor_standalone',
                                 'position_motor_members', 'position_motor_cables',
                                 'torque_motor_standalone', 'torque_motor_loads',
                                 'torque_motor_members', 'torque_motor_cables', 'torque_motor_pinhole',
                                 'motor_diagnostics_cables', 'motor_diagnostics_frames',
                                 'extruder_frames', 'extruder_rigid_constraints',
                                 'commands_extrusion_order', 'commands_cable_pipeline'])
def test_long_sequence_is_deterministic_and_matches_js(name):
    fixture = json.loads((FIXTURES / f'{name}.json').read_text())
    fixture['steps'] = [{'dt': .002, 'resources': {'dt': .002}} for _ in range(200)]
    expected = run_js(fixture)
    actual = run_python(fixture)
    assert_equivalent(actual, expected, **fixture['tolerance'], path=name)
    assert actual == run_python(fixture)
    assert expected == run_js(fixture)


@pytest.mark.parametrize('field', ['quaternion', 'prevCableAttachmentTimeOrientation',
                                 'prevCableAttachmentTimeLocalOrientation'])
def test_comparator_accepts_quaternion_sign_only(field):
    state = {field: [.2, -.3, .4, .8426149773176358]}
    opposite = {field: [-v for v in state[field]]}
    assert_equivalent(state, opposite, atol=1e-10, rtol=1e-9)
    opposite[field][0] += .01
    with pytest.raises(AssertionError, match=field):
        assert_equivalent(state, opposite, atol=1e-10, rtol=1e-9)


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
def test_comparator_rejects_nonfinite_values(value):
    with pytest.raises(AssertionError, match='nonfinite'):
        assert_equivalent(value, value, atol=1e-10, rtol=1e-9)
