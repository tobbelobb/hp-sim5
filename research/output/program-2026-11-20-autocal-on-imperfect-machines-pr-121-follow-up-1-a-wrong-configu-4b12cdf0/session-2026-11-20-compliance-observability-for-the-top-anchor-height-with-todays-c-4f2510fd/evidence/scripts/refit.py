"""Refit one imperfect-machine HP3 run four ways; print JSON with errors in the harness gauge.

usage: refit.py RUN_DIR

  today      the sweep autocal's two-stage point fit (no compliance term on HP3)
  config     + ABC home strain (T/S)(d0 + Y/U) with configured S (the HP4 method)
  probe      + ABC home strain with S/k, k from the fixed-pose force-step probes
  oracle     + ABC home strain with the scene's true EA
"""
import json, sys
from pathlib import Path
import numpy as np

sys.path.insert(0, __file__.rsplit('/', 1)[0])
import synth, probe
from probe_time import probe_seconds
import autocal.pose_fit as pf
from autocal.tools.imperfect_machine import gauge_truth, convention_frame

run = Path(sys.argv[1])
ds = synth.load(run / 'sweeps.json')
imperfection = json.loads((run / 'imperfection.json').read_text())
truth = np.array(list(imperfection['truth_mm'].values()))
gauge, _ = gauge_truth(truth)
truth_frame, _ = convention_frame(truth)
stiffness, strands, advantage, extra = probe.line_description(ds)
target = float(ds['sweeps'][0]['metadata']['strand_tension_n'])
original = pf._hp4_home_strain


def fit(k):
    pf._hp4_home_strain = (lambda d: None) if k is None else (lambda d: (k * target / stiffness, extra[:3] / advantage[:3]))
    try:
        return synth.production_fit(ds)
    finally:
        pf._hp4_home_strain = original


def summary(result):
    anchors = result[0]['anchors']
    e, g = anchors - truth, anchors - gauge
    f = convention_frame(anchors)[0] - truth_frame
    return dict(max_abs_frame_mm=float(np.abs(f).max()), Dz_frame_mm=float(f[3, 2]), low_z_frame_mm=float(f[:3, 2].mean()),
                frame_error_mm=f.round(3).tolist(),max_abs_mm=float(np.abs(e).max()), max_abs_gauge_mm=float(np.abs(g).max()),
                Dz_gauge_mm=float(g[3, 2]), low_z_gauge_mm=float(g[:3, 2].mean()), gauge_error_mm=g.round(3).tolist(),
                rms_mm=result[0]['rms_mm'], home_offsets_mm=np.round(result[1], 3).tolist())


today = fit(None)
k_hat, rows = probe.estimate(ds, today[0]['anchors'], today[1])
out = dict(run=run.name, true_stiffness_scale=imperfection['stiffness_scale'], k_true=1 / imperfection['stiffness_scale'],
           k_probe=k_hat, k_probe_steps=[round(r['k'], 4) for r in rows],
           probe_steps=[{key: round(float(v), 4) for key, v in r.items()} for r in rows],
           strand_target_n=target, today=summary(today), config=summary(fit(1.0)),
           probe=summary(fit(k_hat)) if k_hat is not None else None,
           oracle=summary(fit(1 / imperfection['stiffness_scale'])))
manifest = json.loads((run / 'sweeps.headless' / 'manifest.json').read_text())
out['simulated_s'] = manifest.get('simulated_s')
out['probe_s'], out['num_probes'] = probe_seconds(run)
print(json.dumps(out, indent=1))
