"""Small command/observation probe for the HP5 direct world.

Calibrates spool-angle sign/scale against physical cable rest lengths, measures
force-per-torque and slack onset on one low axis.  Read-only with respect to
root files: the world lives in a private temporary directory.
"""
from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path

WORKTREE = Path(__file__).resolve().parents[5]
ROOT = Path('/home/torbjorn/repos/hp-sim5')
sys.path.insert(0, str(WORKTREE / 'src/python'))
from hp_sim5_research.js_physics import JSPhysics  # noqa: E402

SCENE = ROOT / 'public/usd_scenes/hp5.usda'
LOW_SCALE = 15.0     # mm per rad of spool angle (2 lines, r=30 mm)
TOP_SCALE = 7.5      # mm per rad (4 lines, r=30 mm)


def decode(state):
    comps = {name: rows for name, rows in state['components']}
    names = dict(state['identities'])
    out = dict(step=state['step'], dt=state['dt_s'])
    out['angle'] = {names[e]: c['angle'] for e, c in comps['EncoderComponent']}
    out['commanded'] = {names[e]: c['commanded_angle'] for e, c in comps['StepperMotorComponent']}
    out['torque_mode'] = {names[e]: c['torque_mode'] for e, c in comps['StepperMotorComponent']}
    out['pos'] = {names[e]: c['pos'] for e, c in comps['PositionComponent']}
    out['joints'] = {names[e]: dict(rest=c['rest_length'],
                                    force=c['constraint_force_magnitude'])
                     for e, c in comps['CableJointComponent']}
    return out


def axis_lengths(s):
    """Average physical segment length per axis, from cable rest lengths."""
    out = {}
    for axis, keys in dict(A=('AL', 'AR'), B=('BL', 'BR'), C=('CL', 'CR'),
                           D=('DL', 'DR'), I=('I0', 'I1', 'I2', 'I3')).items():
        out[axis] = sum(s['joints'][f'Joint{k}_1']['rest'] for k in keys) / len(keys)
    return out


def main():
    rows = []
    with tempfile.TemporaryDirectory() as directory:
        physics = JSPhysics(ROOT, Path(directory), SCENE, layering=False, dt=0.002)
        try:
            st = physics.request(steps=0)
            for _ in range(20):
                st = physics.request(steps=50)
            s0 = decode(st)
            l0 = axis_lengths(s0)
            rows.append(dict(tag='initial', **s0))
            # Calibrate Move sign/scale on axis A.
            physics.request(commands=[{'type': 'Move', 'A': s0['angle']['SpoolA'] + 0.05}])
            for _ in range(10):
                st = physics.request(steps=50)
            s1 = decode(st)
            l1 = axis_lengths(s1)
            rows.append(dict(tag='A_move_+0.05rad', **s1, dlen={k: l1[k] - l0[k] for k in l0}))
            physics.request(commands=[{'type': 'Move', 'A': s0['angle']['SpoolA']}])
            for _ in range(10):
                st = physics.request(steps=50)
            s2 = decode(st)
            l2 = axis_lengths(s2)
            rows.append(dict(tag='A_move_back', **s2, dlen={k: l2[k] - l0[k] for k in l0}))
            # Torque sweep on C while A,B,D,I hold position.
            for torque in (0.0, 0.01, 0.02, 0.03, 0.04, 0.06, 0.09, 0.12, -0.01, -0.02, -0.03):
                physics.request(commands=[{'type': 'SetTorqueMode', 'axis': 'C', 'torqueNm': torque}])
                for _ in range(15):
                    st = physics.request(steps=50)
                s = decode(st)
                l = axis_lengths(s)
                rows.append(dict(tag=f'C_torque_{torque}', **s, dlen={k: l[k] - l0[k] for k in l0}))
        finally:
            physics.close()
    print(json.dumps(rows, indent=1))


if __name__ == '__main__':
    main()
