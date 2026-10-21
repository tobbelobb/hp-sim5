"""Physical equilibrium, independently of JS/Python/Warp parity."""
import json
import math
from contextlib import nullcontext
from unittest.mock import patch

import pytest

from parity_harness import FIXTURES, run_python
from cable_joints_3d.pbd_cable_constraint_solver import PBDCableConstraintSolver


@pytest.mark.parametrize('substeps', [1, 4, 16])
@pytest.mark.parametrize('compiled', [False, True])
def test_open_loop_static_torque_balance(substeps, compiled):
    fixture = json.loads((FIXTURES / 'open_loop_static_load.json').read_text())
    fixture['resources']['dt'] = dt = .002 / substeps
    fixture['steps'] = [{'dt': dt} for _ in range(500 * substeps)]
    context = nullcontext()
    if compiled:
        pytest.importorskip('warp')
        from cable_joints_3d.warp_solver import WarpCableConstraintSolver
        solver = WarpCableConstraintSolver('cpu')
        context = patch.object(PBDCableConstraintSolver, 'update', lambda self, world, dt: solver.update(world, dt))
    with context:
        state = run_python(fixture)['snapshots'][-1]['entities']
    angle = state['spool']['EncoderComponent']['angle']
    motor_torque = .5 * math.sin(50 * angle)
    # Independent analytic cable extension and actual constraint force agree.
    assert motor_torque == pytest.approx(.2 - 9 * angle, abs=1e-9)
    cable_torque = .03 * state['joint']['CableJointComponent']['constraintForceMagnitude']
    assert motor_torque == pytest.approx(cable_torque, abs=1e-9)
