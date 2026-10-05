import copy
import json

import pytest

from parity_harness import FIXTURES, assert_equivalent, run_js, run_python
from test_machine_pipeline_parity import SYSTEM_ORDER


LOADS = ['torqueModeCableLoadTorques', 'torqueModeCableLoadStiffnesses', 'torqueModeCableLoadDampings']


def fixture(name):
    return json.loads((FIXTURES / (name + '.json')).read_text())


@pytest.mark.parametrize('name', ['machine_lifecycle_replace', 'machine_lifecycle_append'])
def test_scene_lifecycle_oracle_exercises_loaded_pause_cache_reset_and_deposition(name):
    definition = fixture(name)
    expected = run_js(definition)
    assert_equivalent(run_python(definition), expected, **definition['tolerance'])
    snapshots = expected['snapshots']
    old, new = ('old', 'new') if name.endswith('replace') else ('2', '1')
    for state in snapshots:
        assert state['systemOrder'] == SYSTEM_ORDER
    assert all(snapshots[3]['entityMaps'][key][old + '::SpoolD'] > 0 for key in LOADS)
    assert snapshots[4]['entities'] == snapshots[3]['entities']  # pause retains loaded physics
    assert snapshots[4]['commandState'] == snapshots[3]['commandState']
    loaded = snapshots[5]
    assert loaded['commandState']['axisToEntity'] == {}  # paused load cannot repopulate the cache
    assert loaded['commandState']['queueLength'] == snapshots[4]['commandState']['queueLength'] == 1
    assert loaded['commandState']['history'] == snapshots[4]['commandState']['history']
    if name.endswith('replace'):
        assert all(not key.startswith(old + '::') for key in loaded['entities'])
        assert loaded['resources']['sceneGeneration'] == 2
        assert loaded['allocator']['nextEntityId'] < snapshots[4]['allocator']['nextEntityId']
        assert list(loaded['mapResources']['machineColors']) == [new]
        assert all(loaded['entityMaps'][key] == {} for key in LOADS)
        assert 'D' not in snapshots[6]['commandState']['axisToEntity']
        touched = [new]
    else:
        assert old + '::SpoolD' in loaded['entities']
        assert loaded['resources']['sceneGeneration'] == 1
        assert loaded['allocator']['nextEntityId'] > snapshots[4]['allocator']['nextEntityId']
        assert list(loaded['mapResources']['machineColors']) == [new, old]  # JS numeric object-key order
        assert loaded['entityMaps'] == snapshots[4]['entityMaps']
        assert snapshots[6]['commandState']['axisToEntity']['A'] == [old + '::SpoolA', new + '::SpoolA']
        touched = [old, new]
    assert snapshots[6]['commandState']['history'][-1]['__touchedMachines'] == touched
    tool = lambda state: next(entity['ExtruderComponent'] for entity in state['entities'].values()
                              if 'ExtruderComponent' in entity)
    deposits = tool(snapshots[6])['extrusions'][-len(touched):]
    assert [record['machineId'] for record in deposits] == touched
    assert [record['pos'] for record in deposits] == [tool(loaded)['machineTips'][machine] for machine in touched]
    assert [record['color'] for record in deposits] == ['#ff0000' if machine == old else '#00ff00' for machine in touched]


@pytest.mark.slow
@pytest.mark.parametrize('name', ['machine_lifecycle_replace', 'machine_lifecycle_append', 'machine_pipeline_hp4_loaded_torque'])
def test_live_scene_and_loaded_torque_pipeline_for_200_repeatable_steps(name):
    definition = fixture(name)
    definition['steps'].extend({'dt': .002} for _ in range(200 - len(definition['steps'])))
    expected, actual = run_js(definition), run_python(definition)
    assert_equivalent(actual, expected, **definition['tolerance'])
    assert run_js(definition) == expected
    assert run_python(definition) == actual
    if name.endswith('loaded_torque'):
        for state in expected['snapshots'][1:]:
            assert all(state['entityMaps'][key]['default::SpoolD'] > 0 for key in LOADS)
            assert state['entities']['default::SpoolD']['StepperMotorComponent']['torqueMode']
        assert expected['snapshots'][-1]['entities']['default::SpoolD']['EncoderComponent']['angle'] != 0


def test_loaded_torque_parity_and_load_map_comparison():
    definition = fixture('machine_pipeline_hp4_loaded_torque')
    expected = run_js(definition)
    assert_equivalent(run_python(definition), expected, **definition['tolerance'])
    for state in expected['snapshots'][1:]:
        assert all(state['entityMaps'][key]['default::SpoolD'] > 0 for key in LOADS)
        assert state['entities']['default::SpoolD']['StepperMotorComponent']['torqueMode']
    assert expected['snapshots'][-1]['entities']['default::SpoolD']['EncoderComponent']['angle'] != 0
    for key, change in zip(LOADS, [1e-6, 1e-4, 1e-7]):
        changed = copy.deepcopy(expected)
        changed['snapshots'][-1]['entityMaps'][key]['default::SpoolD'] += change
        with pytest.raises(AssertionError, match=key):
            assert_equivalent(changed, expected, **definition['tolerance'])
