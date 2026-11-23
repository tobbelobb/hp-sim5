"""Per-motor and common force offset/scale on top of the oracle ripple map; rms + errors, also joint k."""
import copy, json, sys, time
from pathlib import Path
import numpy as np
P = Path('/home/torbjorn/.t3/worktrees/hp-sim5/t3-8ab5d557-4e1ca8e08b845775/research/output/program-2026-11-20-autocal-on-imperfect-machines-pr-121-follow-up-1-a-wrong-configu-4b12cdf0')
sys.path.insert(0, str(next(P.glob('session-2026-11-21-force-realisation*')) / 'evidence/scripts'))
from cog_refit import load, corrected
from refit2 import fit_at
from joint_cog import force_anchors
from autocal.dataset_roles import normalize_dataset_point_roles
from autocal.tools.imperfect_machine import convention_frame

run = Path(sys.argv[1]); mode = sys.argv[2]
imp = json.loads((run / 'imperfection.json').read_text())
fitter = 'ellipsoid' if imp.get('ellipsoid_shape') else 'sweep'
ds = load(run); ds.setdefault('machine_type', imp['machine']); normalize_dataset_point_roles(ds)
truth = convention_frame(np.array(list(imp['truth_mm'].values())))[0]
live = np.asarray(json.loads((run / 'result.json').read_text())['fitted_mm'], float)
kt = 1 / imp['stiffness_scale']
base = corrected(ds, 0.01 / 0.03, 50) if 'nocog' not in mode else ds
anchors = force_anchors(ds)

def mapped(d, fn):
    out = copy.deepcopy(d)
    for sw in out['sweeps']:
        for p in sw['data_points']:
            s = int(p.get('source_sensor_anchor', sw['sensor_anchor']))
            p['sensor_force_n'] = fn(s, p['sensor_force_n'])
            for e in p.get('extra_sensor_ranges', []):
                if 'force_n' in e: e['force_n'] = fn(int(e['anchor']), e['force_n'])
    return out

def ev(d, k):
    r = fit_at(d, fitter, k, live)
    f = convention_frame(np.asarray(r['anchors']))[0] - truth
    return dict(k=round(k, 4), rms=round(float(r['rms_mm']), 6), top_z=round(float(f[-1, 2]), 3), max=round(float(np.abs(f).max()), 3))

cases = [('none', lambda a, f: f)]
for a in anchors:
    cases += [(f'off{a}+0.5N', lambda x, f, a=a: f + 0.5 if x == a else f),
              (f'scale{a}+10%', lambda x, f, a=a: f * 1.1 if x == a else f)]
cases += [('all+0.5N', lambda x, f: f + 0.5), ('all+10%', lambda x, f: f * 1.1)]
print(json.dumps(dict(run=run.name, k_true=kt, anchors=anchors, mode=mode)), flush=True)
for name, fn in cases:
    d = mapped(base, fn)
    row = dict(case=name, at_true_k=ev(d, kt))
    if 'jointk' in mode:
        grid = np.round(np.arange(0.85, 1.151, 0.05) * kt, 4)
        scan = [ev(d, float(k)) for k in grid]
        r = np.array([s['rms'] for s in scan]); i = int(min(max(r.argmin(), 1), len(grid) - 2))
        c = np.polyfit(grid[i-1:i+2], r[i-1:i+2], 2); kj = float(-c[1] / (2 * c[0])) if c[0] > 0 else float(grid[r.argmin()])
        row['joint'] = ev(d, kj)
    print(json.dumps(row), flush=True)
