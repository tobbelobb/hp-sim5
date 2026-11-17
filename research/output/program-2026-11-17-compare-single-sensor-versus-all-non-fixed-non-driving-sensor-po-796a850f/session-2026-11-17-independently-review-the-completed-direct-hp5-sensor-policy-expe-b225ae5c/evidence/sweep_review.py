"""Matched small HP5-direct drive-sweep experiments for sensor policies.

Policies differ only in the torque command and recording of the two
non-fixed, non-driving low axes (C = redundant, B = drive, D = sensor,
A = fixed low, I = fixed top).  Every world starts from the same scripted
pullout and every pose is reached by the same drive moves.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import tempfile
import time
from pathlib import Path

WORKTREE = Path(__file__).resolve().parents[5]
ROOT = Path('/home/torbjorn/repos/hp-sim5')
sys.path.insert(0, str(WORKTREE / 'src/python'))
from hp_sim5_research.js_physics import JSPhysics  # noqa: E402

SCENE = ROOT / 'public/usd_scenes/hp5.usda'
AXES = ('A', 'B', 'C', 'D', 'I')
SEGMENTS = {'A': ('AL', 'AR'), 'B': ('BL', 'BR'), 'C': ('CL', 'CR'),
            'D': ('DL', 'DR'), 'I': ('I0', 'I1', 'I2', 'I3')}
SPOOL = {a: f'Spool{a}' for a in AXES}
N_LINES = {'A': 2, 'B': 2, 'C': 2, 'D': 2, 'I': 4}
R_SPOOL = 0.03
TORQUE_PER_NEWTON = {a: N_LINES[a] * R_SPOOL for a in AXES}
ANCHOR = {'A': (0.0, -2.15, -0.18), 'B': (2.15, 0.0, -0.18),
          'C': (0.0, 2.15, -0.18), 'D': (-2.15, 0.0, -0.18), 'I': (0.0, 0.0, 2.0)}
CORNER_ENTITY = {'AL': 'CornerSW', 'AR': 'CornerSE', 'BL': 'CornerSE', 'BR': 'CornerNE',
                 'CL': 'CornerNE', 'CR': 'CornerNW', 'DL': 'CornerNW', 'DR': 'CornerSW',
                 'I0': 'CornerSW', 'I1': 'CornerSE', 'I2': 'CornerNE', 'I3': 'CornerNW'}


def decode(state):
    if 'components' not in state:
        raise RuntimeError(f'physics reply without components: {state}')
    comps = {name: rows for name, rows in state['components']}
    names = dict(state['identities'])
    d = dict(step=state['step'], dt=state['dt_s'])
    d['angle'] = {names[e]: c['angle'] for e, c in comps['EncoderComponent']}
    d['commanded'] = {names[e]: c['commanded_angle'] for e, c in comps['StepperMotorComponent']}
    d['mode'] = {names[e]: c['torque_mode'] for e, c in comps['StepperMotorComponent']}
    d['pos'] = {names[e]: c['pos'] for e, c in comps['PositionComponent']}
    d['joint'] = {names[e]: dict(rest=c['rest_length'], force=c['constraint_force_magnitude'])
                  for e, c in comps['CableJointComponent']}
    return d


def step(physics, n):
    st = None
    while n > 0:
        take = min(50, n)
        st = physics.request(steps=take)
        n -= take
    return decode(st)


def virtual_from_joints(s):
    """Mean physical path length per axis, in mm (encoder-equivalent length)."""
    return {a: 1000.0 * sum(s['joint'][f'Joint{k}_0']['rest'] + s['joint'][f'Joint{k}_1']['rest']
                            for k in SEGMENTS[a]) / N_LINES[a] for a in AXES}


def geometric_paths(s):
    """True geometric mean free-line length per axis (spool segment excluded)."""
    out = {}
    for a in AXES:
        ax, ay, az = ANCHOR[a]
        tot = 0.0
        for k in SEGMENTS[a]:
            c = s['pos'][CORNER_ENTITY[k]]
            tot += math.dist((ax, ay, az), c)
        out[a] = 1000.0 * tot / N_LINES[a]
    return out


def per_segment_slack(s):
    """anchor-to-corner rest length minus true segment length, in mm."""
    out = {}
    for a in AXES:
        ax, ay, az = ANCHOR[a]
        for k in SEGMENTS[a]:
            dist = math.dist((ax, ay, az), s['pos'][CORNER_ENTITY[k]])
            out[k] = 1000.0 * (s['joint'][f'Joint{k}_1']['rest'] - dist)
    return out


def force_per_segment(s):
    out = {}
    for a in AXES:
        for k in SEGMENTS[a]:
            out[k] = s['joint'][f'Joint{k}_1']['force']
    return out


def snapshot(s):
    return dict(step=s['step'], pos=list(s['pos']['Effector']),
                angle={a: s['angle'][SPOOL[a]] for a in AXES},
                mode={a: s['mode'][SPOOL[a]] for a in AXES},
                virt=virtual_from_joints(s), geom=geometric_paths(s),
                slack_mm=per_segment_slack(s), force=force_per_segment(s))


def init_world(physics, takeup_torque, takeup_steps):
    st = step(physics, 500)
    physics.request(commands=[{'type': 'SetTorqueMode', 'axis': a,
                               'torqueNm': takeup_torque} for a in AXES])
    s = step(physics, takeup_steps)
    return s, None  # review fix: published script referenced undefined home_geom


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--policy', required=True,
                    help='c|alt|mid|profile: force on C (redundant) and D (sensor) in N, e.g. 0.05,1.0')
    ap.add_argument('--steps', type=int, default=6, help='drive half-range in drive_step_rad units')
    ap.add_argument('--drive-step-rad', type=float, default=0.1,
                    help='drive spool angle per step (0.1 rad ~ 3 mm payout)')
    ap.add_argument('--drive-speed-rad-s', type=float, default=0.8,
                    help='constant drive spool speed during each move')
    ap.add_argument('--direction', choices=('both', 'up', 'down'), default='both')
    ap.add_argument('--settle', type=int, default=500)
    ap.add_argument('--samples', type=int, default=8)
    ap.add_argument('--recover-steps', type=int, default=0)
    ap.add_argument('--sample-steps', type=int, default=25)
    ap.add_argument('--takeup-torque', type=float, default=0.12)
    ap.add_argument('--takeup-steps', type=int, default=6000)
    ap.add_argument('--out', required=True)
    ap.add_argument('--scene', default=str(SCENE))
    ap.add_argument('--fixed-low', type=int, default=0)
    ap.add_argument('--drive', type=int, default=1)
    ap.add_argument('--sensor', type=int, default=3)
    ap.add_argument('--redundant', type=int, default=2)
    args = ap.parse_args()

    c_force, d_force = (float(x) for x in args.policy.split(','))
    role = 'ABCDI'
    red, sen = role[args.redundant], role[args.sensor]
    drive, fixed_low, top = role[args.drive], role[args.fixed_low], 'I'
    t_c, t_d = c_force * TORQUE_PER_NEWTON[red], d_force * TORQUE_PER_NEWTON[sen]
    # Fixed low A and top I hold position; B drives; C/D are force axes.
    if args.direction == 'both':
        offsets = list(range(0, args.steps + 1)) + [0] + list(range(-1, -args.steps - 1, -1)) + [0]
    elif args.direction == 'up':
        offsets = list(range(0, args.steps + 1))
    else:
        offsets = list(range(0, -args.steps - 1, -1))
    records = []
    manifest = dict(policy=args.policy, c_force_n=c_force, d_force_n=d_force,
                    c_torque_nm=t_c, d_torque_nm=t_d, steps=args.steps,
                    settle=args.settle, samples=args.samples, sample_steps=args.sample_steps,
                    recover_steps=args.recover_steps,
                    drive_step_rad=args.drive_step_rad, drive_speed_rad_s=args.drive_speed_rad_s,
                    direction=args.direction,
                    takeup_torque=args.takeup_torque, takeup_steps=args.takeup_steps,
                    scene=str(args.scene), machine='hp5_direct', offsets=offsets,
                    roles=dict(fixed_low=fixed_low, drive=drive, sensor=sen, redundant=red, top=top))
    wall0 = time.monotonic()
    with tempfile.TemporaryDirectory() as directory:
        physics = JSPhysics(ROOT, Path(directory), Path(args.scene), layering=False, dt=0.002)
        try:
            s, _ = init_world(physics, args.takeup_torque, args.takeup_steps)
            manifest['physics_artifact'] = physics.physics_artifact
            manifest['physics_engine'] = physics.physics_engine
            manifest['physics_host_sha256'] = physics.physics_host_sha256
            physics.request(commands=[{'type': 'SetPositionMode', 'axis': a} for a in (fixed_low, drive, top)]
                            + [{'type': 'Rebase reference'}]
                            + [{'type': 'Move', fixed_low: 0.0}, {'type': 'Move', drive: 0.0}, {'type': 'Move', top: 0.0}])
            s = step(physics, 500)
            set_forces = [{'type': 'SetTorqueMode', 'axis': red, 'torqueNm': t_c},
                          {'type': 'SetTorqueMode', 'axis': sen, 'torqueNm': t_d}]
            physics.request(commands=set_forces)
            s = step(physics, 1500)
            records.append(dict(offset=0, phase='pre', **snapshot(s)))
            current = 0.0
            for offset in offsets:
                target = offset * args.drive_step_rad
                delta = target - current
                ramp = max(2, int(abs(delta) / (args.drive_speed_rad_s * 0.002)))
                cmds = [{'type': 'Move', **{drive: current + delta * (i + 1) / ramp}}
                        for i in range(ramp)]
                physics.request(commands=cmds)
                s = step(physics, ramp + 50)
                current = target
                s = step(physics, args.settle)
                samples = []
                for _ in range(args.samples):
                    s = step(physics, args.sample_steps)
                    samples.append(snapshot(s))
                entry = dict(offset=offset, phase='settled', samples=samples)
                if args.recover_steps:
                    s2 = step(physics, args.recover_steps)
                    entry['recovered'] = snapshot(s2)
                records.append(entry)
        finally:
            physics.close()
    manifest['wall_s'] = time.monotonic() - wall0
    with open(args.out, 'w') as f:
        json.dump(dict(manifest=manifest, records=records), f)
    print(json.dumps({k: manifest[k] for k in ('policy', 'c_force_n', 'd_force_n', 'wall_s')}))


if __name__ == '__main__':
    main()
