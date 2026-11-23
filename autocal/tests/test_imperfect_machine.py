import re

import numpy as np

from autocal._autocal_common import REPO_ROOT, catalogued_machine
from autocal.tools.imperfect_machine import convention_frame, draw, errors, gauge_truth, nominal_anchors, perturbed_scene, scaled_cogging


def test_draws_are_seeded_and_overrides_keep_the_other_draws():
    full = draw('hangprinter_3', 4)
    assert full == draw('hangprinter_3', 4)
    only_dz = draw('hangprinter_3', 4, top_offset_mm=[0, 0], stiffness_scale=1)
    assert only_dz['shifts_mm']['A'] == full['shifts_mm']['A']
    assert only_dz['shifts_mm']['D'] == [0, 0, 0] and only_dz['stiffness_scale'] == 1
    assert all(abs(full['shifts_mm'][name][2]) <= 30 for name in 'ABC')
    assert np.hypot(*full['shifts_mm']['D'][:2]) <= 50 and 0.8 <= full['stiffness_scale'] <= 1.2


def test_scene_moves_spool_and_pivots_rigidly_and_scales_stiffness():
    source = (REPO_ROOT / catalogued_machine('hangprinter_3')['scene']).read_text()
    shifts = {'A': [0, 0, 10.0], 'B': [0, 0, 0], 'C': [0, 0, 0], 'D': [5.0, -3.0, 0]}
    scene, moved = perturbed_scene(source, shifts, 1.1)
    assert moved['D'] == ['SpoolD', 'AnchorD0', 'AnchorD1', 'AnchorD2']
    assert '(0.0, -2.5, -0.16999999999999998)' in scene and '(0.005, -0.003, 2.0)' in scene
    assert set(re.findall(r'stiffnessPerUnitLength = (\S+)', scene)) == {'22000'}


def test_gauge_truth_is_the_rotation_the_length_data_cannot_see():
    truth = np.array(list(nominal_anchors('hangprinter_3').values()))
    truth[:3, 2] += [12.0, -7.0, 3.0]
    truth[3, :2] += [20.0, -10.0]
    gauge, tilt = gauge_truth(truth)
    assert tilt > 0.1 and abs(gauge[0, 0]) < 1e-9
    assert np.ptp(gauge[:3, 2]) < 1e-9 and gauge[0, 1] < 0
    distances = lambda a: np.linalg.norm(a[:, None] - a[None], axis=2)
    np.testing.assert_allclose(distances(gauge), distances(truth), atol=1e-9)
    np.testing.assert_allclose(np.linalg.norm(gauge, axis=1), np.linalg.norm(truth, axis=1), atol=1e-9)


def test_convention_frame_puts_top_anchor_on_z_and_ignores_rigid_tilt():
    truth = np.array(list(nominal_anchors('hangprinter_3').values()))
    truth[3, :2] += [30.0, -20.0]
    frame, tilt = convention_frame(truth)
    np.testing.assert_allclose(frame[3, :2], 0, atol=1e-9)
    assert abs(frame[0, 0]) < 1e-9 and frame[0, 1] < 0 and tilt > 0.5
    angle = np.radians(0.7)
    tilted = truth @ np.array([[1, 0, 0], [0, np.cos(angle), -np.sin(angle)], [0, np.sin(angle), np.cos(angle)]]).T
    gauge, _ = gauge_truth(truth)
    _, _, columns = errors(tilted, truth, frame, gauge)
    assert np.abs(columns['frame']).max() < 1e-9 and np.abs(columns['rigid']).max() < 1e-9
    assert np.abs(columns['raw']).max() > 10


def test_reeved_machines_move_anchor_side_wheels_but_never_effector_members():
    for machine, top in (('hangprinter_4', 'D'), ('hangprinter_5_pulley_reduced', 'I')):
        source = (REPO_ROOT / catalogued_machine(machine)['scene']).read_text()
        members = set(re.findall(r'/(\w+)>', re.search(
            r'def RigidBody "Effector".*?rigidBody:members = \[([^\]]*)\]', source, re.S)[1]))
        shifts = {name: [0, 0, 0] for name in nominal_anchors(machine)}
        _, moved = perturbed_scene(source, shifts, 1.0)
        assert not members & {prim for prims in moved.values() for prim in prims}
        assert {'AttachD0' if top == 'D' else 'AttachI0', f'Wheel{top}0_left_top'} <= set(moved[top])
        assert not any('bottom' in prim for prim in moved[top])


def test_cogging_scale_adds_an_explicit_cogging_torque_per_stepper():
    text = (REPO_ROOT / catalogued_machine('hangprinter_5_pulley_reduced')['scene']).read_text()
    holding = [float(v) for v in re.findall(r'stepper:holdingTorque\s*=\s*([-+\d.eE]+)', text)]
    cogging = [float(v) for v in re.findall(r'stepper:coggingTorque\s*=\s*([-+\d.eE]+)', scaled_cogging(text, 2.0))]
    assert len(holding) == 5 and np.allclose(cogging, [0.02 * h for h in holding])
    with_freq = scaled_cogging(text, 1.0, 137)
    assert re.findall(r'stepper:coggingFreq\s*=\s*([-+\d.eE]+)', with_freq) == ['137'] * 5
