import copy
import json
import re

import numpy as np
import pytest

from cable_joints_3d.ecs import PositionComponent, RigidBodyComponent
from cable_joints_3d.machine_simulation import load_machine_world, register_machine_systems
from parity_harness import FIXTURES, ROOT, assert_equivalent, run_js, run_python


MACHINES = ['minimal', 'hp3_rigid_body', 'hp4_rigid_body',
            'slideprinter_single_pinholes_rigid_body']
SYSTEM_ORDER = [
    'PrevFinalPosSystem', 'PrevFinalOrientationSystem', 'RemoteSpoolSystem',
    'StepperMotorSystem', 'GravitySystem', 'MovementSystem', 'AngularMovementSystem',
    'RigidBodySyncSystem', 'CableAttachmentUpdateSystem', 'CableAttachmentCacheSystem',
    'CableFrictionSystem', 'PBDCableConstraintSolver', 'PBDResolveCableOverCorrections',
    'PBDVelocityUpdateSystem', 'PBDAngularVelocityUpdateSystem', 'TorqueModeSystem',
    'ExtruderSystem', 'EncoderUpdateSystem', 'MissedStepTrackingSystem',
]


def fixture(name):
    return json.loads((FIXTURES / f'machine_pipeline_{name}.json').read_text())


@pytest.mark.parametrize('name', MACHINES)
def test_authored_machine_pipeline_for_200_repeatable_steps(name):
    definition = fixture(name)
    definition['steps'] = [{'dt': .002} for _ in range(200)]
    expected, actual = run_js(definition), run_python(definition)
    assert expected['snapshots'][0]['systemOrder'] == SYSTEM_ORDER
    assert_equivalent(actual, expected, **definition['tolerance'], path=name)
    # Repeat production execution, rather than comparing a saved engine's state
    # to itself. Within-engine determinism is exact, with no tolerance allowance.
    assert run_js(definition) == expected
    assert run_python(definition) == actual
    before, after = [state['entities'] for state in (expected['snapshots'][0], expected['snapshots'][-1])]
    assert any(value['PositionComponent'] != before[entity]['PositionComponent']
               for entity, value in after.items() if 'RigidBodyComponent' in value)


@pytest.mark.parametrize('name', MACHINES)
def test_identical_double_authored_inputs_keep_strict_short_term_engine_bounds(name):
    definition = fixture(name)
    scene = definition['scenes'][0]
    kinds = {'float': 'double', 'float3': 'double3', 'vector3f': 'vector3d',
             'color3f': 'color3d', 'quatf': 'quatd'}
    # This changes the shared test input's authored types, never the loader.
    scene['source'] = re.sub(
        r'(?m)^(\s*(?:custom\s+)?)(float3|float|vector3f|color3f|quatf)(\s+)',
        lambda match: match[1] + kinds[match[2]] + match[3],
        (ROOT / scene['path']).read_text(),
    )
    definition['steps'] = [{'dt': .002} for _ in range(20)]
    assert_equivalent(run_python(definition), run_js(definition), atol=1e-10, rtol=1e-9)


def test_native_composition_loads_the_authored_default_root_and_registers_once():
    world = load_machine_world(ROOT / 'public/usd_scenes/hp4_rigid_body.usda')
    assert len(world.systems) == len(SYSTEM_ORDER)
    systems = list(world.systems)
    register_machine_systems(world)
    assert world.systems == systems
    body = world.query([RigidBodyComponent])[0]
    initial = world.get_component(body, PositionComponent).pos.copy()
    world.update(world.get_resource('dt'))
    assert not np.array_equal(world.get_component(body, PositionComponent).pos, initial)


def test_full_machine_commands_exercise_modes_pause_diagnostics_and_pre_prediction_deposition():
    definition = fixture('hp4_commands')
    snapshots = run_js(definition)['snapshots']
    motor = lambda step: snapshots[step]['entities']['default::SpoolA']['StepperMotorComponent']
    extruder = lambda step: next(state['ExtruderComponent'] for state in snapshots[step]['entities'].values()
                                if 'ExtruderComponent' in state)
    assert motor(9)['torqueMode']
    assert motor(16)['commandedAngle'] == .0003  # position request while torque controlled
    assert not motor(19)['torqueMode']
    assert snapshots[5]['entities'] == snapshots[4]['entities']  # pause
    assert snapshots[5]['commandState'] == snapshots[4]['commandState']
    assert snapshots[10]['resources']['dt'] == .002  # zero update dt does not rewrite cable dt
    assert snapshots[10]['commandState']['queueLength'] < snapshots[9]['commandState']['queueLength']
    assert snapshots[-1]['commandState']['queueLength'] == 0
    deposits = extruder(48)['extrusions']
    assert [record['length'] for record in deposits] == [.002, .001, .001, .001]
    assert deposits[0]['pos'] == extruder(0)['tipPos']
    assert deposits[-1]['pos'] == extruder(19)['tipPos']
    assert extruder(20)['tipPos'] != deposits[-1]['pos']
    assert any(value['CableJointComponent']['constraintForceMagnitude'] > 0
               for value in snapshots[-1]['entities'].values() if 'CableJointComponent' in value)
    assert snapshots[-1]['motorDiagnostics']


def test_full_machine_settling_after_commands_for_1000_steps():
    definition = fixture('hp4_commands')
    definition['steps'].extend({'dt': .002} for _ in range(1000 - len(definition['steps'])))
    expected = run_js(definition)
    assert_equivalent(run_python(definition), expected, **definition['tolerance'])
    final = expected['snapshots'][-1]
    assert final['step'] == 1000
    assert final['commandState']['queueLength'] == 0
    assert len(final['motorDiagnostics'][0]['motors']) == 4
    assert all(not value['StepperMotorComponent']['torqueMode'] for value in final['entities'].values()
               if 'StepperMotorComponent' in value)


def test_sustained_hp4_motion_and_extrusion_for_1000_strict_repeatable_steps():
    definition = fixture('hp4_rigid_body')
    definition['steps'] = [{'dt': .002} for _ in range(1000)]
    rates = {'A': .0016, 'B': -.0012, 'C': .0008, 'D': .0004}
    # Commands are absolute radians, not cable lengths. A turns at .8 rad/s;
    # its authored .03 m spool pays out about .048 m over these two seconds.
    definition['commands'] = [dict(type='Move', **{axis: rate * i for axis, rate in rates.items()},
                                   **({'E': .001} if i % 100 == 0 else {})) for i in range(1000)]
    expected, actual = run_js(definition), run_python(definition)
    assert_equivalent(actual, expected, atol=1e-10, rtol=1e-9)
    assert run_js(definition) == expected
    assert run_python(definition) == actual
    before, after = [snapshot['entities'] for snapshot in (expected['snapshots'][0], expected['snapshots'][-1])]
    displacement = np.array(after['default::Effector']['PositionComponent']['pos']) - before['default::Effector']['PositionComponent']['pos']
    assert np.linalg.norm(displacement) > .02  # exercise commanded motion beyond initial settling
    for axis, rate in rates.items():
        motor = after[f'default::Spool{axis}']['StepperMotorComponent']
        assert motor['commandedAngle'] == rate * 999
        assert motor['missedSteps'] == 0
        assert after[f'default::Spool{axis}']['EncoderComponent']['angle'] == pytest.approx(rate * 999, abs=.003)
    extruder = lambda step: next(state['ExtruderComponent'] for state in expected['snapshots'][step]['entities'].values()
                                if 'ExtruderComponent' in state)
    deposits = extruder(1000)['extrusions']
    assert [record['length'] for record in deposits] == [.001] * 10
    assert [record['pos'] for record in deposits] == [extruder(i)['tipPos'] for i in range(0, 1000, 100)]
    assert np.linalg.norm(np.array(deposits[-1]['pos']) - deposits[0]['pos']) > .015
    assert any(value['CableJointComponent']['constraintForceMagnitude'] > 0
               for value in after.values() if 'CableJointComponent' in value)


def test_machine_tolerances_reject_material_drift_and_keep_deposited_length_strict():
    definition = fixture('hp4_commands')
    expected = run_js(definition)
    entity = expected['snapshots'][-1]['entities']['default::SpoolA']
    for component, field, change in [('PositionComponent', 'pos', 1e-4),
                                     ('OrientationComponent', 'quaternion', 1e-3),
                                     ('AngularVelocityComponent', 'omega', .01)]:
        changed = copy.deepcopy(expected)
        changed['snapshots'][-1]['entities']['default::SpoolA'][component][field][0] += change
        with pytest.raises(AssertionError, match=component):
            assert_equivalent(changed, expected, **definition['tolerance'])
    assert 'PositionComponent' in entity
    changed = copy.deepcopy(expected)
    extruder = next(value['ExtruderComponent'] for value in changed['snapshots'][-1]['entities'].values()
                    if 'ExtruderComponent' in value)
    extruder['extrusions'][0]['length'] += 1e-8
    with pytest.raises(AssertionError, match=r'extrusions\[0\].length'):
        assert_equivalent(changed, expected, **definition['tolerance'])
    changed = copy.deepcopy(expected)
    joint = next(value['CableJointComponent'] for value in changed['snapshots'][-1]['entities'].values()
                 if 'CableJointComponent' in value)
    joint['constraintForceMagnitude'] += .01
    with pytest.raises(AssertionError, match='constraintForceMagnitude'):
        assert_equivalent(changed, expected, **definition['tolerance'])
