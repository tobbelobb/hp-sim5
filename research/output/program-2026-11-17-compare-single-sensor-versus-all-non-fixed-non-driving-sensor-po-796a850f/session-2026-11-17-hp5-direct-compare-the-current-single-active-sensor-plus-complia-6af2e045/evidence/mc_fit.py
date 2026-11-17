"""Small Monte Carlo: recover five virtual anchors from synthetic HP5 ranges.

Compares the current measured pair (drive B, sensor D, with fixed A/I exact)
against measuring the passive transverse axis C as well.  Geometry-only model:
l_j = |a_j - p| - |a_j|.  Fixed lines are exact constraints.  Anchors are
aligned to truth before scoring, so the rigid gauge is not counted.

Home offsets are deliberately *not* fitted here: a constant per-line bias can
be absorbed by a production home offset, while a pose-dependent slack error
cannot.  This offset-free comparison is therefore a conservative view of a
constant bias and an optimistic view of a taut extra range.
"""
from __future__ import annotations

import json
import numpy as np
from scipy.optimize import least_squares

ANCHORS = np.array([[0., -2.15, -0.18], [2.15, 0., -0.18], [0., 2.15, -0.18],
                    [-2.15, 0., -0.18], [0., 0., 2.0]])
FIXED = [0, 4]


def make_poses(n, seed):
    rng = np.random.default_rng(seed)
    return np.column_stack([rng.uniform(-0.24, 0.24, n), rng.uniform(-0.24, 0.24, n),
                            rng.uniform(0.02, 0.55, n)])


def lengths(anchors, p, lines):
    d = np.linalg.norm(anchors[None, :, :] - p[:, None, :], axis=2)
    home = np.linalg.norm(anchors, axis=1)
    return d[:, lines] - home[lines]


def fit(anchors0, poses0, measured, lines, fixed_values):
    n, P = len(anchors0), len(poses0)
    x0 = np.r_[anchors0.ravel(), poses0.ravel()]

    def residual(x):
        a = x[:3 * n].reshape(n, 3)
        p = x[3 * n:].reshape(P, 3)
        pred = lengths(a, p, lines)
        full = lengths(a, p, [0, 1, 2, 3, 4])
        rows = [(pred[:, col] - measured[:, j]) for col, j in enumerate(lines)]
        for col, j in enumerate(FIXED):
            rows.append(10.0 * (full[:, j] - fixed_values[:, col]))
        return np.concatenate(rows)

    lb = np.r_[np.tile([-3.0, -3.0, -0.6], n), np.tile([-0.5, -0.5, -0.2], P)]
    ub = np.r_[np.tile([3.0, 3.0, 2.5], n), np.tile([0.5, 0.5, 0.8], P)]
    sol = least_squares(residual, x0, bounds=(lb, ub), method='trf', max_nfev=400)
    return sol.x[:3 * n].reshape(n, 3)


def aligned_rms(a, truth):
    ca, ct = a - a.mean(0), truth - truth.mean(0)
    u, _, vt = np.linalg.svd(ca.T @ ct)
    r = u @ vt
    return float(np.sqrt(np.mean(np.sum((ca @ r - ct) ** 2, axis=1))))


def trial(seed, n_points, extra_c, sigma=0.01, c_bias=0.0):
    rng = np.random.default_rng(seed)
    poses = make_poses(n_points, seed + 100)
    truth = lengths(ANCHORS, poses, [0, 1, 2, 3, 4])
    meas = truth + rng.normal(0, sigma, truth.shape)
    meas[:, 2] += c_bias
    lines = [1, 3] + ([2] if extra_c else [])
    a0 = ANCHORS + rng.normal(0, 0.01, ANCHORS.shape)
    p0 = poses + rng.normal(0, 0.01, poses.shape)
    est = fit(a0, p0, meas, lines, truth[:, FIXED])
    return aligned_rms(est, ANCHORS)


if __name__ == '__main__':
    out = {}
    for n_points in (10, 20, 30):
        for extra in (False, True):
            errs = [trial(seed, n_points, extra) for seed in range(6)]
            out[f'{n_points}_{"alt" if extra else "cur"}'] = dict(
                errors=errs, mean=float(np.mean(errs)), median=float(np.median(errs)))
    for extra in (False, True):
        errs = [trial(seed, 20, extra, c_bias=0.05) for seed in range(6)]
        out[f'20_bias_{"alt" if extra else "cur"}'] = dict(
            errors=errs, mean=float(np.mean(errs)), median=float(np.median(errs)))
    print(json.dumps(out, indent=1))
    with open('mc_fit.json', 'w') as f:
        json.dump(out, f, indent=1)
