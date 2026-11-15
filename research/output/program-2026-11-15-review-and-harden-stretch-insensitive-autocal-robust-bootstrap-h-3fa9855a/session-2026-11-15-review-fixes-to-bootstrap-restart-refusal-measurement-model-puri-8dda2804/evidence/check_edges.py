import copy
import json
import tempfile
from pathlib import Path
from unittest.mock import patch
import numpy as np
from scipy.optimize._numdiff import approx_derivative
from autocal.rough_bootstrap import bootstrap_state_error, write_bootstrap_state
from autocal.tests.test_pose_fit import _dataset, SEED
from autocal.tools import joint_pose_baseline as joint
from autocal.pose_fit import dataset_line_lengths

with tempfile.TemporaryDirectory() as tmp:
    p = Path(tmp) / 'sweeps.json'
    for stage in ['collecting_rough', 'checking_rough', 'remeasuring', 'failed']:
        write_bootstrap_state(p, stage)
        print(stage, bootstrap_state_error(p).split(';')[0])
    write_bootstrap_state(p, 'complete')
    print('complete missing:', bootstrap_state_error(p))
    p.write_text('{')
    print('complete malformed dataset:', bootstrap_state_error(p))
    p.with_suffix('.bootstrap_state.json').write_text('{')
    print('malformed marker:', bootstrap_state_error(p))
    p.with_suffix('.bootstrap_state.json').unlink()
    p.with_suffix('.rough_bootstrap.json').write_text('{}')
    p.write_text('[]')
    try:
        print('legacy list:', bootstrap_state_error(p))
    except Exception as exc:
        print('legacy list exception:', type(exc).__name__, str(exc))
    p.write_text(json.dumps({'sweeps': [{'metadata': {'sensor_force_profile': True}}]}))
    print('legacy complete profiled with stale sidecar:', bootstrap_state_error(p))

d = _dataset((0., 0., 0., 1.5))
real = joint.least_squares
checks = []
def wrapped(fun, x, **kwargs):
    jac = kwargs['jac'](x, *kwargs['args']).toarray()
    numeric = approx_derivative(lambda v: fun(v, *kwargs['args']), x, method='3-point')
    loss = kwargs['loss'](np.array([.1] * (len(fun(x, *kwargs['args'])) - 1) + [100.]))
    checks.append((float(np.max(np.abs(jac-numeric))), loss[:, -1].tolist()))
    return real(fun, x, **kwargs)
with patch.object(joint, 'least_squares', wrapped):
    result = joint.fit_joint_ranges(d, SEED)
print('jacobian max abs errors, prior rho at z=100:', checks)
print('joint convergence:', result['success'], result['message'], 'nfev', result['nfev'], 'optimality', result['optimality'])
a = dataset_line_lengths(d, use_noise_mean=False)['lengths_mm']
d2 = copy.deepcopy(d)
for sweep in d2['sweeps']:
    for point in sweep['data_points']:
        point['raw_angles_zero_tension_deg'] = [123., 456., 789., 1000.]
b = dataset_line_lengths(d2, use_noise_mean=False)['lengths_mm']
print('changing projected angles changes joint input:', not np.array_equal(a, b))

import autocal.autocal as ac
import autocal.headless_sim as hs
with tempfile.TemporaryDirectory() as tmp:
    p = Path(tmp) / 'sweeps.json'
    p.write_text('{')
    write_bootstrap_state(p, 'complete')
    events = []
    class FakeHeadless:
        def __init__(self, *args):
            events.append('constructed')
        def __enter__(self):
            events.append('entered')
            raise RuntimeError('stopped before real physics')
        def __exit__(self, *args):
            pass
    with patch.object(hs, 'HeadlessSimulation', FakeHeadless):
        try:
            ac.main(['--headless-sim', '--machine-type', 'hangprinter_3', '--dataset', str(p)])
        except RuntimeError as exc:
            print('complete malformed headless:', events, str(exc))
