import json
import math
from contextlib import nullcontext
from unittest.mock import patch
import pytest
from parity_harness import FIXTURES, run_python
from cable_joints_3d.pbd_cable_constraint_solver import PBDCableConstraintSolver

@pytest.mark.parametrize('damper', [False, True])
@pytest.mark.parametrize('substeps', [1, 4])
@pytest.mark.parametrize('compiled', [False, True])
def test_open_loop_static_torque_balance(substeps, compiled, damper):
    fixture = json.loads((FIXTURES / 'open_loop_static_load.json').read_text())
    fixture['resources']['dt'] = dt = .002/substeps
    fixture['steps'] = [{'dt': dt} for _ in range(500*substeps)]
    if damper:
        fixture['steps'] = [{'dt': dt}]
        motor = fixture['entities'][1]['components']['StepperMotorComponent']
        motor[2], motor[4] = 0, 0
        fixture['entities'][1]['components']['AngularVelocityComponent'] = [0, 0, -1]
        path = fixture['entities'][-1]['components']['CablePathComponent']
        path[3], path[6] = 0, 5
    context = nullcontext()
    if compiled:
        pytest.importorskip('warp')
        from cable_joints_3d.warp_solver import WarpCableConstraintSolver
        solver = WarpCableConstraintSolver('cpu')
        context = patch.object(PBDCableConstraintSolver, 'update', lambda self, world, dt: solver.update(world, dt))
    with context:
        state = run_python(fixture)['snapshots'][-1]['entities']
    if damper:
        expected = -1/(1+5*.03**2*dt/5e-5)
        assert state['spool']['AngularVelocityComponent']['omega'][2] == pytest.approx(expected, abs=1e-12)
        force = state['joint']['CableJointComponent']['constraintForceMagnitude']
        assert .03*force == pytest.approx(5*.03**2*abs(expected), abs=1e-12)
        return
    angle = state['spool']['EncoderComponent']['angle']
    assert .5*math.sin(50*angle) == pytest.approx(.2-9*angle, abs=1e-9)
    force = state['joint']['CableJointComponent']['constraintForceMagnitude']
    assert .5*math.sin(50*angle) == pytest.approx(.03*force, abs=1e-9)
