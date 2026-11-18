"""Refit HP4 anchors with the production pose fitter on substituted lengths to attribute the bias."""
import json, sys
import numpy as np
sys.path.insert(0, '/home/torbjorn/repos/hp-sim5'); sys.path.insert(0, '/tmp/claude-1000/-home-torbjorn-repos-hp-sim5/b907ee07-5eb6-4cf9-b0e4-c029dc1d17dd/scratchpad')
import autocal.pose_fit as pf
from geom import HP4, HP3
import os, json as _j
HOME = np.array(_j.loads(open(os.environ.get('POSES', 'hp4_poses.jsonl')).readline())['pos']) * 1000

dataset = json.load(open(sys.argv[1])); z = np.load(sys.argv[2])
m = dataset['config']['m669']; TRUE = np.array([m[k] for k in 'ABCD'], float)
orig = pf.dataset_line_lengths(dataset)
keep = z['k']
def sub(d):
    d = dict(d); d['sweeps'] = d['sweeps']; return d
def fit(L, label, offsets=True):
    def fake(ds, use_noise_mean=True):
        return {'lengths_mm': L, 'sweep_index': orig['sweep_index'][keep]}
    pf.dataset_line_lengths = fake
    r = pf.fit_pose_least_squares(dataset, TRUE * 1.01, fit_home_offsets=offsets, home_offset_prior_sigma_mm=pf.HOME_OFFSET_PRIOR_SIGMA_MM)
    err = r['anchors'] - TRUE
    print(f"{label:42s} worst={np.abs(err).max():6.2f}  Dz={err[3,2]:+6.2f} ABCz={err[0,2]:+6.2f} Ay={err[0,1]:+6.2f} Bx={err[1,0]:+6.2f} Cx={err[2,0]:+6.2f} off={np.round(r['home_offsets_mm'],3)} rms={r['rms_mm']:.3f}")
    return r
if __name__ == "__main__" and len(sys.argv) < 4:
    import os
    g = (HP3('/home/torbjorn/repos/hp-sim5/public/usd_scenes/hp3.usda', HOME) if os.environ.get('MACHINE') == 'hp3' else HP4('/home/torbjorn/repos/hp-sim5/public/usd_scenes/hp4.usda', HOME))
    h = g.lengths(np.zeros(3), None)
    rot = np.array([[(g.lengths(e, q)[k] - h[k]).mean() for k in 'ABCD'] for e, q in zip(z['E'], z['Q'])])
    pm = z['model']
    L = z['L']
    print('points', len(L))
    fit(pm, 'true point model (translation only)')
    fit(rot, 'geometry incl. effector rotation')
    fit(L, 'measured')
    fit(L - (rot - pm), 'measured minus rotation effect')
    stretch = L - rot
    for ax in range(4):
        s = np.zeros_like(L); s[:, ax] = stretch[:, ax]
        fit(pm + s, f'point model + stretch on axis {"ABCD"[ax]} only')
    s = stretch - stretch.mean(0)
    fit(pm + s, 'point model + stretch minus its per-axis mean')
    fit(pm + stretch.mean(0), 'point model + per-axis mean stretch only')

def extra(dataset_path, poses):
    """Correct measured lengths with stretch predicted from the simulator's recorded strand tensions."""
    t = np.load('tension.npz')
    L = z['L']
    fit(L + t['pred'], 'measured + stretch from recorded tensions')
    for s in (0.5, 2.0):
        fit(L + s * t['pred'], f'measured + {s}x stretch from recorded tensions')
if len(sys.argv) >= 4:
    extra(None, None)
