"""Read-only structural probe of the HP5 direct physics world.

Prints axes, spool entities, virtual anchors inferred from the scene, cable
paths/joints and a settled initial observation.  No commands are issued.
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

scene = ROOT / 'public/usd_scenes/hp5.usda'


def summarized(state):
    comps = {name: rows for name, rows in state['components']}
    info = {eid: name for eid, name in state['identities']}
    stepper = {eid: c for eid, c in comps['StepperMotorComponent']}
    encoder = {eid: c for eid, c in comps['EncoderComponent']}
    pos = {eid: c['pos'] for eid, c in comps['PositionComponent']}
    rbm = {eid: c for eid, c in comps['RigidBodyMemberComponent']}
    joints = {eid: c for eid, c in comps['CableJointComponent']}
    paths = {eid: c for eid, c in comps['CablePathComponent']}
    return dict(info=info, stepper=stepper, encoder=encoder, pos=pos, rbm=rbm,
                joints=joints, paths=paths,
                cable_half_widths=state['cable_half_widths'], dt_s=state['dt_s'],
                physics_engine=state['physics_engine'],
                physics_artifact=state['physics_artifact'],
                axes=state['axes'])


def main():
    out = {}
    with tempfile.TemporaryDirectory() as directory:
        physics = JSPhysics(ROOT, Path(directory), scene, layering=False, dt=0.002)
        try:
            state = physics.request(steps=0)
            for _ in range(40):
                state = physics.request(steps=50)
            s = summarized(state)
            names = {eid: name for eid, name in s['info'].items()}
            axes = {}
            for axis, ids in s['axes'].items():
                ids = ids if isinstance(ids, list) else [ids]
                axes[axis] = [names.get(i, i) for i in ids]
            out['axes'] = axes
            out['spool'] = {}
            for axis, ids in axes.items():
                for eid in s['axes'][axis] if isinstance(s['axes'][axis], list) else [s['axes'][axis]]:
                    if eid in s['stepper']:
                        out['spool'][axis] = dict(
                            entity=names.get(eid), angle=s['encoder'].get(eid, {}).get('angle'),
                            pos=s['pos'].get(eid),
                            torque_mode=s['stepper'][eid]['torque_mode'],
                            target_torque=s['stepper'][eid]['target_torque'])
            out['attachments'] = {names.get(eid): c for eid, c in s['rbm'].items()
                                  if 'Anchor' in (names.get(eid) or '') or 'Corner' in (names.get(eid) or '')}
            out['joints'] = {names.get(eid): {k: c[k] for k in
                            ('rest_length', 'constraint_force_magnitude', 'transferred_constraint_force_magnitude')}
                            for eid, c in s['joints'].items()}
            out['paths'] = {names.get(eid): c for eid, c in s['paths'].items()}
            out['cable_half_widths'] = s['cable_half_widths']
            out['dt_s'] = s['dt_s']
            out['physics_engine'] = s['physics_engine']
            out['physics_artifact'] = s['physics_artifact']
            out['positions'] = {names.get(eid): p for eid, p in s['pos'].items()}
        finally:
            physics.close()
    print(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()
