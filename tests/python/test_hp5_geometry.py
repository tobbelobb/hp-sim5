"""Authored pivot geometry and reeving must agree with both firmware configs."""
import configparser
from pathlib import Path
import re

import numpy as np
import pytest
from pxr import Usd

ROOT = Path(__file__).resolve().parents[2]
ANCHORS = np.array([[0.,-1900.,-280.], [1900.,0.,-280.], [0.,1900.,-280.],
                    [-1900.,0.,-280.], [0.,0.,1900.]])


@pytest.mark.parametrize('name,reduced', [('hp5', False), ('hp5_pulley_reduced', True)])
def test_all_pivots_parallel_lines_and_reeving_match_firmware(name, reduced):
    stage = Usd.Stage.Open(str(ROOT / f'public/usd_scenes/{name}.usda'))
    base = '/World/HangprinterScene/'
    def prim(name):
        return stage.GetPrimAtPath(base+name)
    def pos(name):
        return np.asarray(prim(name).GetAttribute('xformOp:translate').Get())
    rrf = (ROOT / f'RRF/run/vsd/sys/config_{name}.g').read_text()
    for axis, target in zip('ABCDI', ANCHORS):
        match = re.search(rf'\b{axis}([-\d.]+:[-\d.]+:[-\d.]+)', rrf)
        np.testing.assert_allclose(list(map(float, match[1].split(':'))), target)
    corners = ['CornerSW', 'CornerSE', 'CornerNE', 'CornerNW']
    expected = np.array([[-.25,-.25,.1],[.25,-.25,.1],[.25,.25,.1],[-.25,.25,.1]])
    np.testing.assert_allclose([pos(c) for c in corners], expected)
    members = prim('Effector').GetRelationship('rigidBody:members').GetTargets()
    assert len(members) == (44 if reduced else 4)
    assert list(prim('Effector').GetRelationship('rigidBody:frameCorners').GetTargets()) == [prim(c).GetPath() for c in corners]
    for i, axis in enumerate('ABCD'):
        for side in 'LR':
            if reduced:
                for level in ('top', 'bottom'):
                    np.testing.assert_allclose(1000*(pos(f'Anchor{axis}{side}_{level}')-pos(f'Pinhole{axis}{side}_{level}')), ANCHORS[i], atol=1e-8)
            else:
                np.testing.assert_allclose(1000*(pos(f'Anchor{axis}{side}')-pos(corners[(i + (side == 'R')) % 4])), ANCHORS[i], atol=1e-8)
    for index in range(4):
        if reduced:
            for side in ('left', 'right'):
                np.testing.assert_allclose(1000*(pos(f'WheelI{index}_{side}_top')-pos(f'WheelI{index}_{side}_bottom')), ANCHORS[4], atol=1e-8)
        else:
            np.testing.assert_allclose(1000*(pos(f'AnchorI{index}')-pos(corners[index])), ANCHORS[4], atol=1e-8)
    paths = [p for p in stage.Traverse() if p.GetAttribute('cablePath:linkTypes')]
    assert len(paths) == 12
    reference = Usd.Stage.Open(str(ROOT / f'public/usd_scenes/{"hp4" if reduced else "hp3"}.usda'))
    if reduced:
        # Identical bearing/pinhole spread to HP4, translated to the new corner.
        reference_corner = np.asarray(reference.GetPrimAtPath(base+'CornerAL').GetAttribute('xformOp:translate').Get())
        for part in ('WheelAL_top', 'WheelAL_bottom', 'PinholeAL_top', 'PinholeAL_bottom'):
            old_position = np.asarray(reference.GetPrimAtPath(base+part).GetAttribute('xformOp:translate').Get())
            np.testing.assert_allclose(pos(part)-pos('CornerSW'), old_position-reference_corner, atol=1e-8)
    attaches = [p for p in stage.Traverse() if p.GetAttribute('ecs:tags')
                and 'Attachment' in p.GetAttribute('ecs:tags').Get()]
    assert {p.GetName() for p in attaches} == set(corners)
    for path in paths:
        top = path.GetName().startswith('CablePathI')
        old = reference.GetPrimAtPath(base + ('CablePathD0' if top else 'CablePathAL'))
        for attr in ('cablePath:linkTypes', 'cablePath:clockwise', 'cablePath:storedMode'):
            assert path.GetAttribute(attr).Get() == old.GetAttribute(attr).Get()
    ma = [2,2,2,2,4] if reduced else [1]*5
    assert f'M666 U{":".join(map(str,ma))}' in rrf
    assert 'M666 N2:2:2:2:4' in rrf
    assert 'M666 L20:20:20:20:20' in rrf and 'M666 H20:20:20:20:20' in rrf
    klipper = configparser.ConfigParser()
    klipper.read(ROOT / f'public/klipper/{name}/printer-{name}-linux-mcu.cfg')
    assert klipper['printer']['winch_lines_per_axis'] == '2, 2, 2, 2, 4'
    for index, section in enumerate(['stepper_a','stepper_b','stepper_c','stepper_d','stepper_e']):
        np.testing.assert_allclose([float(klipper[section][f'anchor_{axis}']) for axis in 'xyz'], ANCHORS[index])
        assert float(klipper[section]['rotation_distance']) == pytest.approx(2*np.pi*30/ma[index], abs=1e-3)
        assert klipper[section]['m569_address'] == f'{40+index}.0'
    y = re.search(r'^M666 Y([^\s]+)', rrf, re.M)[1].split(':')
    assert list(map(float,y)) == [429.1]*4 + [352.3]
