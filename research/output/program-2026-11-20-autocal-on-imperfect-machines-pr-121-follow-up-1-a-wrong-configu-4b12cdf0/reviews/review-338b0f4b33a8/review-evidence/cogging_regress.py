"""Regress realised spool tension minus commanded tension on the encoder angle.

usage: cogging_regress.py events.jsonl replay.jsonl spool_radius_m [pole_pairs] [settle_steps]

Commanded spool tension = |torqueNm| / r from the SetTorqueMode commands in the
recorded events. Realised spool tension = sum of the spool-side segment
("<Joint><axis>*_0") constraint forces at every replayed full encoder read.
Only reads taken while the axis is in torque mode at >= 0.05 Nm and the rotor is
still (|d angle| < 0.002 deg since the previous read) are used. Per axis it fits
err = c + a sin(p th) + b cos(p th) (+ s * commanded) and prints the residual sd.
"""
import json
import sys

import numpy as np

events, replay, radius = sys.argv[1], sys.argv[2], float(sys.argv[3])
poles = int(sys.argv[4]) if len(sys.argv) > 4 else 50

torque = {}  # axis -> [(step, torqueNm or None)]
with open(events) as f:
    for line in f:
        if '"SetTorqueMode"' not in line and '"SetPositionMode"' not in line:
            continue
        e = json.loads(line)
        if e.get('type') != 'bridge_payload':
            continue
        c = e['payload'].get('command') or {}
        if c.get('type') == 'SetTorqueMode':
            torque.setdefault(c['axis'], []).append((e['epoch'], e['step'], abs(c['torqueNm'])))
        elif c.get('type') == 'SetPositionMode':
            torque.setdefault(c['axis'], []).append((e['epoch'], e['step'], None))


SETTLE = int(sys.argv[5]) if len(sys.argv) > 5 else 500


def command(axis, epoch, step):
    """Torque held for at least SETTLE steps (any re-send counts as a change)."""
    cur, since = None, None
    for ep, st, t in torque.get(axis, []):
        if (ep, st) <= (epoch, step):
            if t != cur or ep != epoch:
                since = st
            cur = t
        else:
            break
    return cur if since is not None and step - since >= SETTLE else None


rows = {}
prev = {}
with open(replay) as f:
    for line in f:
        r = json.loads(line)
        for i, axis in enumerate(r['axes']):
            ang = r['angles_deg'][i]
            p = prev.get(axis)
            prev[axis] = ang
            t = command(axis, r['epoch'], r['step'])
            if t is None or t < 0.05 or p is None or abs(ang - p) > 0.002:
                continue
            realised = sum(v[0] for k, v in r['joints'].items()
                           if k.startswith(f'Joint{axis}') and k.endswith('_0'))
            rows.setdefault(axis, []).append((np.deg2rad(ang), t / radius, realised))

out = {}
for axis, data in sorted(rows.items()):
    th, cmd, real = map(np.array, zip(*data))
    err = real - cmd
    X = np.column_stack([np.ones_like(th), np.sin(poles * th), np.cos(poles * th), cmd])
    coef, *_ = np.linalg.lstsq(X, err, rcond=None)
    res = err - X @ coef
    X0 = X[:, :1]
    c0, *_ = np.linalg.lstsq(np.column_stack([X0, cmd]), err, rcond=None)
    out[axis] = dict(n=len(err), err_mean=float(err.mean()), err_sd=float(err.std()),
                     offset=float(coef[0]), cog_amp=float(np.hypot(coef[1], coef[2])),
                     sin=float(coef[1]), cos=float(coef[2]),
                     scale=float(coef[3]), resid_sd=float(res.std()),
                     resid_sd_no_cog=float((err - np.column_stack([X0, cmd]) @ c0).std()),
                     cmd_range=[float(cmd.min()), float(cmd.max())])
    print(axis, json.dumps(out[axis]))
json.dump(out, open(replay.rsplit('/', 1)[-1] + '.cogging.json', 'w'), indent=1)
