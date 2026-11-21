"""Can the sweep data's own fit rms identify a force-realisation parameter?

usage: force_param_scan.py RUN_DIR {cog|scale|offset}

Profiles the fit rms (and true-anchor errors, print frame) at the true compliance over:
  cog     s * the cogging map (cog_refit.corrected), s in -1..1.5; s = 1 is the sim's truth
  scale   every force-mode line force multiplied by (1 + s), s in -10%..+10%; truth s = 0
  offset  every force-mode line force plus s N, s in -1..1 N; truth s = 0
A joint fit of the parameter with the anchors can only work if the rms minimum sits at the truth.
"""
import copy, json, sys
from pathlib import Path
import numpy as np

sys.path.insert(0, __file__.rsplit('/', 1)[0])
from cog_refit import PROBE, load, corrected  # noqa: E402,F401
from refit2 import fit_at  # noqa: E402
from autocal.dataset_roles import normalize_dataset_point_roles  # noqa: E402
from autocal.tools.imperfect_machine import convention_frame  # noqa: E402

GRIDS = dict(cog=[-1, -0.5, 0, 0.5, 1, 1.5], scale=[-0.1, -0.05, 0, 0.05, 0.1], offset=[-1, -0.5, 0, 0.5, 1])


def mapped(ds, fn):
    out = copy.deepcopy(ds)
    for sweep in out['sweeps']:
        for p in sweep['data_points']:
            p['sensor_force_n'] = fn(p['sensor_force_n'])
            for e in p.get('extra_sensor_ranges', []):
                if 'force_n' in e:
                    e['force_n'] = fn(e['force_n'])
    return out


def main():
    run, kind = Path(sys.argv[1]), sys.argv[2]
    imp = json.loads((run / 'imperfection.json').read_text())
    fitter = 'ellipsoid' if imp.get('ellipsoid_shape') else 'sweep'
    ds = load(run)
    ds.setdefault('machine_type', imp['machine'])
    normalize_dataset_point_roles(ds)
    truth = convention_frame(np.array(list(imp['truth_mm'].values())))[0]
    live = np.asarray(json.loads((run / 'result.json').read_text())['fitted_mm'], float)
    k = 1 / imp['stiffness_scale']
    rows = []
    for s in GRIDS[kind]:
        data = (corrected(ds, 0.01 / 0.03, 50, s) if kind == 'cog' else
                mapped(ds, (lambda f: f * (1 + s)) if kind == 'scale' else (lambda f: f + s)))
        r = fit_at(data, fitter, k, live)
        f = convention_frame(np.asarray(r['anchors']))[0] - truth
        rows.append(dict(s=s, rms_mm=round(float(r['rms_mm']), 6), top_z_mm=round(float(f[-1, 2]), 3),
                         max_abs_mm=round(float(np.abs(f).max()), 3)))
    print(json.dumps(dict(run=run.name, machine=imp['machine'], kind=kind, rows=rows)))


if __name__ == '__main__':
    main()
