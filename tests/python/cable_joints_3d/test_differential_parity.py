import copy
import json

import pytest

from parity_harness import FIXTURES, assert_equivalent, run_js, run_python


@pytest.mark.parametrize('fixture_path', sorted(FIXTURES.glob('*.json')), ids=lambda p: p.stem)
def test_live_js_differential(fixture_path):
    fixture = json.loads(fixture_path.read_text())
    expected = run_js(fixture)
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


@pytest.mark.parametrize('name', ['rigid_members', 'distance_members', 'spool_projection',
                                 'cable_cache_members', 'cable_friction_chain',
                                 'cable_attachment_motion', 'cable_attachment_members'])
def test_long_sequence_is_deterministic_and_matches_js(name):
    fixture = json.loads((FIXTURES / f'{name}.json').read_text())
    fixture['steps'] = [{'dt': .002} for _ in range(200)]
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
