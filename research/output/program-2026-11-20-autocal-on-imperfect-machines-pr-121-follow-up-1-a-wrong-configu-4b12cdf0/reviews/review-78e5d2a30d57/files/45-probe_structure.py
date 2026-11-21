"""Per-step probe k / true k against pose height, step size and sensor; subset estimators."""
import json, sys
import numpy as np
sys.path.insert(0, __file__.rsplit('/', 1)[0])
import synth, probe
from autocal.pose_fit import fit_point_positions
for run in sys.argv[1:]:
    ds = synth.load(f'{run}/sweeps.json')
    r = json.load(open(f'{run}/../../../../{run}_refit.json')) if False else None
    imp = json.load(open(f'{run}/imperfection.json')); k_true = 1 / imp['stiffness_scale']
    fit, off = synth.production_fit(ds)
    k, rows = probe.estimate(ds, fit['anchors'], off)
    # pose of each probe
    heights = []
    for sensor, forces, lengths in probe.probe_rows(ds):
        p = fit_point_positions(fit['anchors'], lengths[:1], off)[0][0]
        heights += [p[2]] * (len(forces) - 1)
    rows = [dict(r, z=h) for r, h in zip(rows, heights) if r['d_force'] > 0]
    ratio = np.array([r['k'] for r in rows]) / k_true
    z = np.array([r['z'] for r in rows]); big = np.array([r['d_force'] for r in rows])
    first = np.array([i % 2 == 0 for i in range(len(rows))])
    print(f'{run.split("/")[-1]}: all {np.mean(ratio):.3f}; 1x step {np.mean(ratio[first]):.3f} 2x step {np.mean(ratio[~first]):.3f}; '
          f'z<300 {np.mean(ratio[z < 300]):.3f} (n={np.sum(z<300)//2}) z>300 {np.mean(ratio[z >= 300]):.3f}; corr(ratio,z) {np.corrcoef(ratio, z)[0,1]:+.2f}')
    print('   per probe (z, ratio 1x, 2x):', [(int(zz), round(a, 3), round(b, 3)) for zz, a, b in zip(z[first], ratio[first], ratio[~first])])
