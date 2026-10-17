"""Physical checks, independent of JS/backend agreement."""
import json
from contextlib import nullcontext
from unittest.mock import patch

import pytest
from parity_harness import FIXTURES, run_python
from cable_joints_3d.pbd_cable_constraint_solver import PBDCableConstraintSolver


@pytest.mark.parametrize('passive', [False, True])
@pytest.mark.parametrize('compiled', [False, True], ids=['python', 'warp-cpu'])
def test_central_cable_force_conserves_assembly_momentum_and_rotors_coast(compiled, passive):
    fixture = json.loads((FIXTURES / 'mounted_rotor_momentum.json').read_text())
    if passive:
        del next(e for e in fixture['entities'] if e['name'] == 'passive')['components']['StepperMotorComponent']
    context = nullcontext()
    if compiled:
        pytest.importorskip('warp')
        from cable_joints_3d.warp_solver import WarpCableConstraintSolver
        solver = WarpCableConstraintSolver('cpu')
        context = patch.object(PBDCableConstraintSolver, 'update', lambda self, world, dt: solver.update(world, dt))
    with context:
        snapshots = run_python(fixture)['snapshots']
    multiplier = -.0005 / (1 + .1**2 / .1 + .1**2 / .7 + 250)
    body_angle = multiplier * .1 / .7
    active_angle = -multiplier * .1 * (1 / .1 + 1 / .7)
    passive_angle = -body_angle
    assert snapshots[1]['entities']['outer']['CableJointComponent']['constraintLambda'] == pytest.approx(multiplier, abs=1e-14)
    for index, snapshot in enumerate(snapshots[1:], 1):
        entities = snapshot['entities']
        omega = lambda name: entities[name]['AngularVelocityComponent']['omega'][2]
        assert omega('body') == pytest.approx(body_angle / .002, abs=1e-12)
        assert omega('spool') == pytest.approx(active_angle / .002, abs=1e-12)
        assert omega('passive') == pytest.approx(passive_angle / .002, abs=1e-12)
        assert omega('body') + .1 * omega('spool') + .2 * omega('passive') == pytest.approx(0, abs=1e-12)
        assert omega('body') + omega('passive') == pytest.approx(0, abs=1e-12)
        assert entities['spool']['EncoderComponent']['angle'] == pytest.approx(index * active_angle, abs=1e-12)
        assert entities['passive']['EncoderComponent']['angle'] == pytest.approx(index * passive_angle, abs=1e-12)
