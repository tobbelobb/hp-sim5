"""Scope check: does the synthetic information gain survive on the sweep the
experiment actually ran?

The source Monte Carlo uses a broad 3D pose box (x,y in +/-0.24 m, z 0.02-0.55 m)
and a reference pose at the origin.  The direct experiment instead swept the B
spool by ~90 mm at one height.  This script reuses the *measured effector
positions* of the preserved taut runs as the pose set, then runs the same
paired synthetic fit and the same linearized Fisher comparison.

Read-only with respect to the source evidence; writes only sweep_scope.json.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

import stats_review as sr


def load_poses(path, direction):
    d = json.load(open(path))
    return [np.array(r['samples'][0]['pos']) for r in d['records']
            if 'samples' in r and r.get('phase') == 'settled']


def main():
    here = Path(__file__).resolve().parent
    src = Path('/home/torbjorn/repos/hp-sim5/research/output/'
               'program-2026-11-17-compare-single-sensor-versus-all-non-fixed-non-driving-'
               'sensor-po-796a850f/session-2026-11-17-hp5-direct-compare-the-current-'
               'single-active-sensor-plus-complia-6af2e045/evidence')
    poses = []
    used = {}
    for direction in ('down', 'up'):
        p = load_poses(src / f'taut_{direction}_0.3,1.0.json', direction)
        used[direction] = dict(n=len(p), x_span_mm=1000 * (max(q[0] for q in p) - min(q[0] for q in p)),
                               y_span_mm=1000 * (max(q[1] for q in p) - min(q[1] for q in p)),
                               z_span_mm=1000 * (max(q[2] for q in p) - min(q[2] for q in p)))
        poses.extend(p)
    poses = np.array(poses)

    # Linearized Fisher on exactly these poses, same model as info.py.
    fish = {}
    for sigma in (0.01,):
        c = sr.spectrum(sr.fisher(sr.ANCHORS, poses, sr.CURRENT, sigma))
        a = sr.spectrum(sr.fisher(sr.ANCHORS, poses, sr.ALT, sigma))
        fish = dict(sigma_trace_mm=[c['sigma_trace_mm'], a['sigma_trace_mm']],
                    sigma_trace_ratio=c['sigma_trace_mm'] / a['sigma_trace_mm'],
                    weakest_info=[c['weakest'], a['weakest']],
                    weakest_sigma_ratio=float(np.sqrt(a['weakest'] / c['weakest'])),
                    median_sigma_ratio=float(np.sqrt(a['median'] / c['median'])),
                    strongest_sigma_ratio=float(np.sqrt(a['strongest'] / c['strongest'])))

    # Paired synthetic fit on the same 14 poses.
    seeds = range(8)
    cur, alt = [], []
    for s in seeds:
        rng = np.random.default_rng(5000 + s)
        truth = sr.lengths(sr.ANCHORS, poses, [0, 1, 2, 3, 4])
        meas = truth + rng.normal(0, 0.01, truth.shape)
        a0 = sr.ANCHORS + rng.normal(0, 0.01, sr.ANCHORS.shape)
        p0 = poses + rng.normal(0, 0.01, poses.shape)
        fixed = truth[:, sr.FIXED]
        cur.append(sr.aligned_rms(sr.fit(a0, p0, meas, [1, 3], fixed), sr.ANCHORS))
        alt.append(sr.aligned_rms(sr.fit(a0, p0, meas, [1, 2, 3], fixed), sr.ANCHORS))
    d = np.array(alt) - np.array(cur)
    out = dict(pose_source='taut_down_0.3 + taut_up_0.3 effector positions',
               poses=len(poses), spans_mm=used, fisher=fish,
               synthetic_fit=dict(cur_mean=float(np.mean(cur)), alt_mean=float(np.mean(alt)),
                                  cur_se=float(np.std(cur, ddof=1) / np.sqrt(len(cur))),
                                  alt_se=float(np.std(alt, ddof=1) / np.sqrt(len(alt))),
                                  paired_diff_mean=float(np.mean(d)),
                                  paired_diff_se=float(np.std(d, ddof=1) / np.sqrt(len(d))),
                                  cur=cur, alt=alt))
    with open(here / 'sweep_scope.json', 'w') as fh:
        json.dump(out, fh, indent=1)
    print(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()
