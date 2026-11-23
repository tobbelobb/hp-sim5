"""Fit a per-motor torque-ripple map jointly with the anchors, from the sweep data alone.

usage: joint_cog.py RUN_DIR [harmonics=50] [iterations=2]

Every force-mode line's effector force becomes
  F = commanded + U_a * sum_h (A_ah sin(h th_a) + B_ah cos(h th_a)),
th_a = that motor's encoder angle, h = harmonics per revolution (comma list; 50 = the
motor's electrical cycles per turn, M666 J200 / 4). h = 0 instead fits A_a * direction + B_a:
a per-motor offset that flips with the line's direction of motion (static friction /
hysteresis) plus a common offset (unidentifiable; the min-norm step leaves it near 0). The A, B (spool-side N) minimise the
production fit's rms: rms^2 is quadratic in them, so each iteration fits that quadratic from
1 + 2d + d(d-1)/2 full refits and jumps to its minimum. Nothing here reads the sim's
cogging amplitude or phase. Reported at the true and the configured compliance, plus
the compliance probe's k re-estimated with the corrected probe-level forces.
"""
import copy, itertools, json, sys
from pathlib import Path
import numpy as np

sys.path.insert(0, __file__.rsplit('/', 1)[0])
from cog_refit import PROBE, load  # noqa: E402,F401
from refit2 import fit_at  # noqa: E402
from perturb import point_directions  # noqa: E402
import probe  # noqa: E402
from autocal.dataset_roles import normalize_dataset_point_roles  # noqa: E402
from autocal.tools.imperfect_machine import convention_frame  # noqa: E402


def force_anchors(ds):
    out = set()
    for sweep in ds['sweeps']:
        for p in sweep['data_points']:
            out.add(int(p.get('source_sensor_anchor', sweep['sensor_anchor'])))
            out.update(int(e['anchor']) for e in p.get('extra_sensor_ranges', []) if 'force_n' in e)
    return sorted(out)


def apply(ds, anchors, harmonics, params):
    """Dataset with the ripple map added to every force-mode line's force (also probe levels)."""
    out = copy.deepcopy(ds)
    U = out['config']['m666_before_data_collection']['U']
    P = np.asarray(params, float).reshape(len(anchors), len(harmonics), 2)
    index = {a: i for i, a in enumerate(anchors)}

    def ripple(a, deg, direction=0.0):
        th = np.deg2rad(deg)
        return U[a] * sum(P[index[a], j, 0] * direction + P[index[a], j, 1] if h == 0 else
                          P[index[a], j, 0] * np.sin(h * th) + P[index[a], j, 1] * np.cos(h * th)
                          for j, h in enumerate(harmonics))
    for sweep in out['sweeps']:
        dirs = point_directions(sweep)
        for p, d in zip(sweep['data_points'], dirs):
            ang = np.asarray(p.get('mu') or p['raw_angles_deg'], float)
            s = int(p.get('source_sensor_anchor', sweep['sensor_anchor']))
            p['sensor_force_n'] = float(p['sensor_force_n'] + ripple(s, ang[s], d[s]))
            for e in p.get('extra_sensor_ranges', []):
                if 'force_n' in e:
                    a = int(e['anchor'])
                    e['force_n'] = float(e['force_n'] + ripple(a, ang[a], d[a]))
        for pr in (sweep.get('metadata') or {}).get('compliance_probes', []):
            s = int(pr['sensor_anchor'])
            if s in index:
                for level in pr['levels']:
                    level['force_n'] = float(level['force_n'] + ripple(s, level['mu_deg'][s]))
    return out


def quadratic_step(f, centre, step):
    """Minimum of the quadratic through f at centre, centre +- step e_i and centre + step (e_i + e_j)."""
    d = len(centre)
    E = np.eye(d) * step
    f0 = f(centre)
    fp = np.array([f(centre + E[i]) for i in range(d)])
    fm = np.array([f(centre - E[i]) for i in range(d)])
    g = (fp - fm) / (2 * step)
    H = np.diag((fp + fm - 2 * f0) / step ** 2)
    for i, j in itertools.combinations(range(d), 2):
        H[i, j] = H[j, i] = (f(centre + E[i] + E[j]) - fp[i] - fp[j] + f0) / step ** 2
    # Min-norm step: a direction the rms cannot see (a common force offset) is left alone.
    return centre - np.linalg.lstsq(H, g, rcond=1e-6)[0], f0


def main():
    run = Path(sys.argv[1])
    harmonics = [float(h) for h in (sys.argv[2] if len(sys.argv) > 2 else '50').split(',')]
    iterations = int(sys.argv[3]) if len(sys.argv) > 3 else 2
    imp = json.loads((run / 'imperfection.json').read_text())
    fitter = 'ellipsoid' if imp.get('ellipsoid_shape') else 'sweep'
    ds = load(run)
    ds.setdefault('machine_type', imp['machine'])
    normalize_dataset_point_roles(ds)
    truth = convention_frame(np.array(list(imp['truth_mm'].values())))[0]
    live = np.asarray(json.loads((run / 'result.json').read_text())['fitted_mm'], float)
    k_true = 1 / imp['stiffness_scale']
    anchors = force_anchors(ds)
    d = len(anchors) * len(harmonics) * 2
    calls = [0]

    def rms2(params, k):
        calls[0] += 1
        return fit_at(apply(ds, anchors, harmonics, params), fitter, k, live)['rms_mm'] ** 2

    def errors(data, k):
        r = fit_at(data, fitter, k, live)
        f = convention_frame(np.asarray(r['anchors']))[0] - truth
        return r, dict(k=round(float(k), 4), top_z_mm=round(float(f[-1, 2]), 3),
                       max_abs_mm=round(float(np.abs(f).max()), 3), rms_mm=round(float(r['rms_mm']), 5))

    out = dict(run=run.name, machine=imp['machine'], seed=imp['seed'], k_true=k_true, harmonics=harmonics,
               force_anchors=anchors)
    for kname, k in (('true_k', k_true), ('config_k', 1.0)):
        params, history = np.zeros(d), []
        for it in range(iterations):
            params, f0 = quadratic_step(lambda p: rms2(p, k), params, 0.2 if it == 0 else 0.05)
            history.append(round(float(np.sqrt(f0)), 5))
        data = apply(ds, anchors, harmonics, params)
        base_fit, base = errors(ds, k)
        joint_fit, joint = errors(data, k)
        out[kname] = dict(commanded=base, joint=joint, rms_history=history,
                          params=np.round(params.reshape(len(anchors), len(harmonics), 2), 4).tolist())
        if kname == 'config_k':
            for name, src, fit in (('commanded', ds, base_fit), ('joint', data, joint_fit)):
                k_probe, rows = probe.estimate(src, fit['anchors'], fit['home_offsets_mm'])
                out[f'probe_{name}'] = errors(src, k_probe)[1] if k_probe else None
    out['refits'] = calls[0]
    print(json.dumps(out))


if __name__ == '__main__':
    main()
