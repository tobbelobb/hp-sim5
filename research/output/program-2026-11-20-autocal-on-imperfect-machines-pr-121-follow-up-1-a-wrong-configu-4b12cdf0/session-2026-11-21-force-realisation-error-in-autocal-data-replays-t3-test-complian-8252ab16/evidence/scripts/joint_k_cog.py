"""Joint compliance scale k + per-motor torque-ripple map + anchors, from the sweep data alone.

usage: joint_k_cog.py RUN_DIR [harmonics=50]

For every k on a fixed grid (0.75..1.35, the prior study's range; no use of the truth) the
ripple map is profiled out (one exact quadratic step, joint_cog.quadratic_step) and the
fit rms recorded; the plain rms (commanded forces) comes free at the same k. Each k
estimate is the parabola vertex through the grid minimum. Then the map is refined at the
chosen k (two steps) and the anchors scored in the print frame against truth.
YFIX=1 adds HP5r's effector-side reeving per path (85.512 mm, compliance probe report 7.2)
to the low lines' M666 Y first. JOBS=n spreads the k grid over n forked processes. PERTURB / HOMESTRAIN: see perturb.py.
"""
import json, multiprocessing, os, sys
from pathlib import Path
import numpy as np

sys.path.insert(0, __file__.rsplit('/', 1)[0])
from joint_cog import apply, force_anchors, quadratic_step, load, fit_at  # noqa: E402
from perturb import perturb, home_strain  # noqa: E402
from autocal.dataset_roles import normalize_dataset_point_roles  # noqa: E402
from autocal.tools.imperfect_machine import convention_frame  # noqa: E402

GRID = np.round(np.arange(0.75, 1.351, 0.05), 3)


_CTX = {}


def _grid_point(k):
    """(plain rms, rms with the ripple map profiled out) at compliance scale k."""
    rms2, d = _CTX['rms2'], _CTX['d']
    p, f0 = quadratic_step(lambda q: rms2(q, k), np.zeros(d), 0.2)
    return np.sqrt(f0), np.sqrt(max(rms2(p, k), 0.0))


def vertex(grid, rms):
    i = int(np.argmin(rms))
    j = min(max(i, 1), len(grid) - 2)
    c = np.polyfit(grid[j - 1:j + 2], rms[j - 1:j + 2], 2)
    return float(-c[1] / (2 * c[0])) if c[0] > 0 else float(grid[i])


def main():
    run = Path(sys.argv[1])
    harmonics = [float(h) for h in (sys.argv[2] if len(sys.argv) > 2 else '50').split(',')]
    imp = json.loads((run / 'imperfection.json').read_text())
    fitter = 'ellipsoid' if imp.get('ellipsoid_shape') else 'sweep'
    ds = load(run)
    ds.setdefault('machine_type', imp['machine'])
    normalize_dataset_point_roles(ds)
    if os.environ.get('YFIX'):
        m = ds['config']['m666_before_data_collection']
        m['Y'] = [y + (85.512 if i < len(m['Y']) - 1 else 0.0) for i, y in enumerate(m['Y'])]
    ds = perturb(ds)
    truth = convention_frame(np.array(list(imp['truth_mm'].values())))[0]
    live = np.asarray(json.loads((run / 'result.json').read_text())['fitted_mm'], float)
    k_true = 1 / imp['stiffness_scale']
    anchors = force_anchors(ds)
    d = len(anchors) * len(harmonics) * 2
    homed = os.environ.get('HOMESTRAIN') and home_strain(ds, 1.0, live) is not None
    fit_k = lambda data, k: fit_at(home_strain(data, k, live) if homed else data, fitter, k, live)
    rms2 = lambda p, k: fit_k(apply(ds, anchors, harmonics, p), k)['rms_mm'] ** 2
    _CTX.update(rms2=rms2, d=d)
    jobs = int(os.environ.get('JOBS', 1))
    if jobs > 1:
        with multiprocessing.get_context('fork').Pool(jobs) as pool:
            points = pool.map(_grid_point, GRID)
    else:
        points = [_grid_point(k) for k in GRID]
    plain, cog = map(np.array, zip(*points))

    def score(data, k):
        r = fit_k(data, k)
        f = convention_frame(np.asarray(r['anchors']))[0] - truth
        return dict(k=round(float(k), 4), k_over_true=round(float(k / k_true), 4), top_z_mm=round(float(f[-1, 2]), 3),
                    max_abs_mm=round(float(np.abs(f).max()), 3), rms_mm=round(float(r['rms_mm']), 5))
    k_plain, k_cog = vertex(GRID, plain), vertex(GRID, cog)
    p = np.zeros(d)
    for step in (0.2, 0.05):
        p, _ = quadratic_step(lambda q: rms2(q, k_cog), p, step)
    out = dict(run=run.name, machine=imp['machine'], seed=imp['seed'], k_true=k_true, harmonics=harmonics,
               perturb=os.environ.get('PERTURB'), home_strain=bool(homed), commanded_true_k=score(ds, k_true),
               scan=[dict(k=float(k), rms_plain=round(float(a), 5), rms_cog=round(float(b), 5))
                     for k, a, b in zip(GRID, plain, cog)],
               joint_k_plain=score(ds, k_plain), joint_k_cog=score(apply(ds, anchors, harmonics, p), k_cog),
               params=np.round(p.reshape(len(anchors), len(harmonics), 2), 4).tolist())
    print(json.dumps(out))


if __name__ == '__main__':
    main()
