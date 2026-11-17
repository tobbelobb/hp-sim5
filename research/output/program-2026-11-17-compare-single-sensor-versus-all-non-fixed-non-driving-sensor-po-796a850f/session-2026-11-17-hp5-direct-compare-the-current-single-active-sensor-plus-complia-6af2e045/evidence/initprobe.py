"""Find a repeatable taut initialization and measure force/torque/slack behaviour.

Protocol under test: strong positive torque on all five axes to take up stored
slack, then hold fixed/drive axes at their reached angles and run force levels.
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
AXES = {'A': ('AL', 'AR'), 'B': ('BL', 'BR'), 'C': ('CL', 'CR'),
        'D': ('DL', 'DR'), 'I': ('I0', 'I1', 'I2', 'I3')}
SPOOL = {'A': 'SpoolA', 'B': 'SpoolB', 'C': 'SpoolC', 'D': 'SpoolD', 'I': 'SpoolI'}
N_LINES = {'A': 2, 'B': 2, 'C': 2, 'D': 2, 'I': 4}
R_SPOOL = 0.03


def decode(state):
    if 'components' not in state:
        raise RuntimeError(f'physics reply without components: {state}')
    comps = {name: rows for name, rows in state['components']}
    names = dict(state['identities'])
    d = dict(step=state['step'], dt=state['dt_s'])
    d['angle'] = {names[e]: c['angle'] for e, c in comps['EncoderComponent']}
    d['mode'] = {names[e]: c['torque_mode'] for e, c in comps['StepperMotorComponent']}
    d['pos'] = {names[e]: c['pos'] for e, c in comps['PositionComponent']}
    d['joint'] = {names[e]: dict(rest=c['rest_length'], force=c['constraint_force_magnitude'])
                  for e, c in comps['CableJointComponent']}
    return d


def virtual(s, base=None):
    out = {}
    for axis, keys in AXES.items():
        out[axis] = sum(s['joint'][f'Joint{k}_0']['rest'] + s['joint'][f'Joint{k}_1']['rest']
                        for k in keys) / N_LINES[axis]
    if base:
        out = {k: (out[k] - base[k]) * 1000.0 for k in out}
    return out


def geom(s):
    """True geometric mean path length per axis from the current effector pose."""
    p = s['pos']['Effector']
    anchors = {'A': [(0, -2.15, -0.18)], 'B': [(2.15, 0, -0.18)],
               'C': [(0, 2.15, -0.18)], 'D': [(-2.15, 0, -0.18)], 'I': [(0, 0, 2.0)]}
    corners = {'AL': (-0.25, -0.25), 'AR': (0.25, -0.25),
               'BL': (0.25, -0.25), 'BR': (0.25, 0.25),
               'CL': (0.25, 0.25), 'CR': (-0.25, 0.25),
               'DL': (-0.25, 0.25), 'DR': (-0.25, -0.25),
               'I0': (-0.25, -0.25), 'I1': (0.25, -0.25),
               'I2': (0.25, 0.25), 'I3': (-0.25, 0.25)}
    out = {}
    for axis, keys in AXES.items():
        a = anchors[axis][0]
        tot = 0.0
        for k in keys:
            c = corners[k]
            tot += ((a[0] - (p[0] + c[0])) ** 2 + (a[1] - (p[1] + c[1])) ** 2
                    + (a[2] - p[2]) ** 2) ** 0.5
        out[axis] = tot / N_LINES[axis]
    return out


def force_summary(s, axis):
    fs = [s['joint'][f'Joint{k}_1']['force'] for k in AXES[axis]]
    fs += [s['joint'][f'Joint{k}_0']['force'] for k in AXES[axis]]
    return dict(min=min(fs), mean=sum(fs) / len(fs), max=max(fs))


def step(physics, n):
    st = None
    while n > 0:
        take = min(50, n)
        st = physics.request(steps=take)
        n -= take
    return decode(st)


def main():
    rows = []
    with tempfile.TemporaryDirectory() as directory:
        physics = JSPhysics(ROOT, Path(directory), SCENE, layering=False, dt=0.002)
        try:
            st = step(physics, 500)
            s = st
            v0 = virtual(s)
            rows.append(dict(tag='fresh', pos=s['pos']['Effector'], angle=s['angle'],
                             virt=v0, forces={a: force_summary(s, a) for a in AXES}))
            # Strong take-up, positive torque.
            physics.request(commands=[{'type': 'SetTorqueMode', 'axis': a, 'torqueNm': 0.12} for a in AXES])
            for t in (500, 1000, 2000, 4000, 6000):
                s = step(physics, t if t == 500 else 1000)
                rows.append(dict(tag=f'takeup_{t}', pos=s['pos']['Effector'], angle=s['angle'],
                                 virt=virtual(s, v0), geom=geom(s),
                                 forces={a: force_summary(s, a) for a in AXES}))
            # Hold A,B,I; C,D at explicit forces.
            hold = {a: s['angle'][SPOOL[a]] for a in ('A', 'B', 'I')}
            physics.request(commands=[{'type': 'SetPositionMode', 'axis': a} for a in ('A', 'B', 'I')]
                            + [{'type': 'Move', **{a: hold[a]}} for a in ('A', 'B', 'I')])
            for torque in (0.003, 0.01, 0.03, 0.06):
                for axis in ('C', 'D'):
                    physics.request(commands=[{'type': 'SetTorqueMode', 'axis': axis, 'torqueNm': torque}])
                s = step(physics, 1500)
                rows.append(dict(tag=f'force_{torque}', pos=s['pos']['Effector'], angle=s['angle'],
                                 virt=virtual(s, v0), geom=geom(s),
                                 forces={a: force_summary(s, a) for a in AXES}))
        finally:
            physics.close()
    print(json.dumps(rows, indent=1))


if __name__ == '__main__':
    main()
