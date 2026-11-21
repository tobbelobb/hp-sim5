"""Refit with the per-point stretch model (pose_fit.dataset_stretch: equilibrium tensions from
commanded sensor forces, firmware flex with S/k) forced on for HP3, for several k.

usage: full_stretch.py sweeps.json truth_source k...
truth_source: an imperfection.json, or 'nominal'.
"""
import json, sys, copy
import numpy as np
sys.path.insert(0, __file__.rsplit('/', 1)[0])
import synth
import autocal.pose_fit as pf
from autocal.flex import FlexModel
from autocal.tools.imperfect_machine import gauge_truth

ds = synth.load(sys.argv[1])
truth = synth.TRUTH if sys.argv[2] == 'nominal' else np.array(list(json.loads(open(sys.argv[2]).read())['truth_mm'].values()))
gauge, _ = gauge_truth(truth)
orig = pf.dataset_stretch


def stretch_for(k):
    def build(dataset):
        saved = dataset['num_anchors']
        # dataset_stretch only corrects redundant machines; pretend so to reuse it.
        dataset = dict(dataset, num_anchors=5)
        out = None
        try:
            dataset['num_anchors'] = saved
            m666 = dataset['config']['m666_before_data_collection']
            model = FlexModel.from_m666(m666, num_axes=saved, spring_k_multiplier=1.0 / k)
            points = [(s, p) for s in dataset['sweeps'] for p in s['data_points']]
            out = dict(model=model,
                       sensors=np.array([int(p.get('source_sensor_anchor', s['sensor_anchor'])) for s, p in points]),
                       known=np.full((len(points), saved), np.nan),
                       forces=np.array([p['sensor_force_n'] for _, p in points]),
                       weight=float(m666['W']) * 9.81, passive=0.0)
        finally:
            pass
        return out
    return build


for k in map(float, sys.argv[3:]):
    pf.dataset_stretch = stretch_for(k)
    try:
        fit, off = synth.production_fit(ds)
    finally:
        pf.dataset_stretch = orig
    g = fit['anchors'] - gauge
    print(f'full stretch model k {k:.3f}: max gauge {np.abs(g).max():.3f} Dz {g[3,2]:+.3f} lowz {g[:3,2].mean():+.3f} '
          f'offsets {np.round(off,3).tolist()} rms {fit["rms_mm"]:.4f} stretch_corrected {fit["stretch_corrected"]}')
