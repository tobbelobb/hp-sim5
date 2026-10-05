import copy
import json

import numpy as np
import pytest

from parity_harness import FIXTURES, ROOT, assert_equivalent, run_js, run_python


def _fixture(name='usd_scene_minimal'):
    return json.loads((FIXTURES / f'{name}.json').read_text())


def test_construction_oracle_exercises_aggregate_members_and_authored_bindings():
    fixture = _fixture()
    snapshot = run_js(fixture)['snapshots'][0]
    assert [len(query) for query in snapshot['queries']] == [1, 3, 1, 1]
    entities = snapshot['entities']
    body = entities['default::Effector']
    assert body['MassComponent']['mass'] == pytest.approx(.8)
    assert body['AngularVelocityComponent']['omega'] == [0, 0, 0]
    assert body['RigidBodyComponent']['renderSegments'] == [[0, 1, 2, 0]]
    for name, mass in [('Wheel', .4), ('Pinhole', .3), ('Corner', .1)]:
        member = entities['default::' + name]
        assert member['MassComponent']['mass'] == 0
        assert member['RigidBodyMemberComponent']['physicalMass'] == mass
        assert 'GravityAffectedComponent' not in member
    assert entities['default::Pinhole']['RadiusComponent']['radius'] == .002
    assert entities['default::Wheel']['RenderableComponent']['height'] == .04
    assert entities['default::SpoolA']['RenderableComponent']['color'] == '#1a334d'
    assert entities['default::SpoolA']['StepperMotorComponent']['numPolePairs'] == 2
    assert entities['default::Path']['CablePathComponent']['spring_constant'] == 0
    assert entities['default::Path']['CablePathComponent']['compliance'] == 'Infinity'
    extruder = next(state['ExtruderComponent'] for state in entities.values() if 'ExtruderComponent' in state)
    assert extruder['centerSources']['default'] == ['default::Wheel', 'default::Pinhole', 'default::Corner']
    assert extruder['centerOffsets']['default'] == [.01, -.02, .03]
    assert extruder['tipOffsets']['default'] == [0, 0, -.1]
    assert extruder['coldEndOffsets']['default'] == [0, 0, .2]


def test_legacy_group_spelling_builds_the_same_native_and_js_assembly():
    fixture = _fixture()
    definition = fixture['scenes'][0]
    definition['source'] = (ROOT / definition['path']).read_text().replace('def RigidBody', 'def RigidGroup').replace('rigidBody:members', 'rigidGroup:members')
    expected = run_js(fixture)
    assert len(expected['snapshots'][0]['queries'][0]) == 1
    assert_equivalent(run_python(fixture), expected, **fixture['tolerance'])


def test_appended_scene_preserves_machine_identity_colors_and_first_timestep():
    fixture = _fixture('usd_scene_append')
    state = run_js(fixture)['snapshots'][0]
    assert state['resources']['sceneGeneration'] == 1
    assert state['resources']['dt'] == .002
    assert len(state['queries'][0]) == 2
    assert set(state['mapResources']['machineColors']) == {'1', '2'}
    assert state['mapResources']['machineColors']['1']['extrusionColor'] == '#00ff00'
    assert state['entities']['2::SpoolA']['RenderableComponent']['color'] == '#222222'
    assert state['entities']['1::JointA_0']['RenderableComponent']['color'] == '#aabbcc'


def test_float32_authored_values_give_both_engines_identical_initial_inputs():
    fixture = _fixture('usd_scene_hp4_rigid_body')
    native, reference = [run(fixture)['snapshots'][0] for run in (run_python, run_js)]
    py_friction = native['entities']['default::SpoolA']['CoefficientOfFrictionComponent']['mu']
    js_friction = reference['entities']['default::SpoolA']['CoefficientOfFrictionComponent']['mu']
    assert py_friction == js_friction == float(np.float32(.2))
    assert native['resources']['gravity'] == reference['resources']['gravity']
    assert_equivalent(native, reference, atol=1e-10, rtol=1e-9)
    changed = copy.deepcopy(native)
    changed['entities']['default::WheelAL_top']['RigidBodyMemberComponent']['localPosition'][2] += 1e-4
    with pytest.raises(AssertionError, match='localPosition'):
        assert_equivalent(changed, reference, **fixture['tolerance'])
