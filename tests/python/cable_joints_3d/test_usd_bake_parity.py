import copy
import json

import pytest

from parity_harness import FIXTURES, ROOT, run_js, run_python


def test_bake_oracle_exercises_authored_values_and_derivation_policies():
    fixture = json.loads((FIXTURES / 'usd_bake_frames.json').read_text())
    source_path = ROOT / fixture['usdBake']['path']
    original = source_path.read_bytes()
    paths = run_js(fixture)['usdBake']
    auto, manual = paths
    assert auto['resolvedStored'][:2] == [2, .03]
    assert 0 < auto['resolvedStored'][2] < 1
    assert auto['jointResults'][1]['restLength'] == .75
    assert auto['jointResults'][1]['local0'] == [.1, .3, .2]
    assert manual['resolvedStored'] == [.5, .1]
    assert manual['jointResults'][0]['restLength'] == 1.25
    assert manual['jointResults'][0]['world0'] != manual['jointResults'][0]['local0']
    changed = copy.deepcopy(fixture)
    changed['usdBake']['options'] = {'deriveAll': True}
    rederived = run_js(changed)['usdBake']
    assert rederived[0]['resolvedStored'][1] != .03
    assert rederived[0]['jointResults'][1]['local0'] != [.1, .3, .2]
    assert rederived[1]['jointResults'][0]['restLength'] != 1.25
    run_python(fixture)
    assert source_path.read_bytes() == original


@pytest.mark.parametrize('old,new', [
    ('point3d localPos0 = (0.01, 0.02, 0.03)', ''),
    ('double restLength = 1.25', ''),
    ('double[] cablePath:stored = [0.5, 0.1]', ''),
    ('double[] cablePath:stored = [2, 0.03, 9, 0]', 'double[] cablePath:stored = [2, 0.03]'),
    ('["manual", "manual", "auto", "manual"]', '["manual", "manual", "invalid", "manual"]'),
    ('double radius = 0.03', 'double radius = 0'),
    ('</World/Scene/J0>, </World/Scene/J1>', '</World/Scene/J0>, </World/Scene/J0>'),
])
def test_both_bakers_reject_invalid_authored_initialization(old, new):
    fixture = json.loads((FIXTURES / 'usd_bake_frames.json').read_text())
    source = (ROOT / fixture['usdBake']['path']).read_text()
    assert source.count(old) == 1
    fixture['usdBake']['source'] = source.replace(old, new)
    with pytest.raises(AssertionError, match='JS oracle failed'):
        run_js(fixture)
    with pytest.raises(ValueError):
        run_python(fixture)
