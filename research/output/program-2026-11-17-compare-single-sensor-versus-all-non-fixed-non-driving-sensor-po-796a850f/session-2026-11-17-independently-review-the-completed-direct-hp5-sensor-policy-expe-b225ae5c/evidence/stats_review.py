"""Independent recomputation of the HP5-direct information and synthetic-fit claims.

Part 1 recomputes the linearized Fisher spectrum of evidence/info.py and reports
per-mode sigma ratios, so the headline 'sigma-trace' and 'median direction'
numbers can be checked.

Part 2 reruns the offset-free synthetic anchor recovery of evidence/mc_fit.py
(a) with its original non-nested design, to verify the published means, and
(b) with a matched, nested, paired design: the same poses, the same noise
realisation and the same initial guess for both policies at 10/20/30 points.
That removes pose-set and seed differences from the comparison.

Part 3 tests the report's main open question: can a production-style per-line
home offset absorb a constant C bias? It refits with one free constant offset per
measured line. Part 4 adds a pose-dependent 1.2 mm slack spike on one C pose and
shows that an offset cannot absorb it.

All inputs are synthetic geometry; no production data or processes are used.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

ANCHORS = np.array([[0., -2.15, -0.18], [2.15, 0., -0.18], [0., 2.15, -0.18],
                    [-2.15, 0., -0.18], [0., 0., 2.0]])
CURRENT = [0, 1, 3, 4]
ALT = [0, 1, 2, 3, 4]
FIXED = [0, 4]


# ---------------------------------------------------------------- Fisher part
def poses(n=30):
    rng = np.random.default_rng(7)
    u = rng.uniform(-1, 1, (n, 2))
    return np.column_stack([0.22 * u[:, 0], 0.22 * u[:, 1],
                            rng.uniform(0.02, 0.55, n)])


def rows(point, anchors, lines, sigma_fixed=1e-3, sigma_meas=0.01):
    n = len(anchors)
    J = np.zeros((len(lines), 3 * n + 3))
    w = np.zeros(len(lines))
    for row, j in enumerate(lines):
        d = anchors[j] - point
        unit = d / np.linalg.norm(d)
        J[row, 3 * j:3 * j + 3] = unit - anchors[j] / np.linalg.norm(anchors[j])
        J[row, 3 * n:3 * n + 3] = -unit
        w[row] = 1.0 / (sigma_fixed if j in FIXED else sigma_meas) ** 2
    return J, w


def fisher(anchors, ps, lines, sigma_meas):
    n = len(anchors)
    F = np.zeros((3 * n, 3 * n))
    for p in ps:
        J, w = rows(p, anchors, lines, sigma_meas=sigma_meas)
        Ja, Jp = J[:, :3 * n], J[:, 3 * n:]
        W = np.diag(w)
        A = Ja - Jp @ np.linalg.solve(Jp.T @ W @ Jp, Jp.T @ W @ Ja)
        F += A.T @ W @ A
    return F


def spectrum(F):
    vals = np.clip(np.linalg.eigvalsh(F), 0, None)
    inf = vals[6:]                      # six rigid-body gauge modes
    return dict(info=inf,
                sigma_trace_mm=float(np.sqrt(np.sum(1.0 / inf))),
                harmonic=float(len(inf) / np.sum(1.0 / inf)),
                weakest=float(inf[0]), median=float(np.median(inf)),
                strongest=float(inf[-1]))


# ------------------------------------------------------------- synthetic part
def lengths(anchors, p, lines):
    d = np.linalg.norm(anchors[None, :, :] - p[:, None, :], axis=2)
    home = np.linalg.norm(anchors, axis=1)
    return d[:, lines] - home[lines]


def fit(anchors0, poses0, measured, lines, fixed_values, offsets=False):
    n, P = len(anchors0), len(poses0)
    k = len(lines) if offsets else 0
    x0 = np.r_[anchors0.ravel(), poses0.ravel(), np.zeros(k)]

    def unpack(x):
        return (x[:3 * n].reshape(n, 3), x[3 * n:3 * n + 3 * P].reshape(P, 3),
                x[3 * n + 3 * P:])

    def residual(x):
        a, p, off = unpack(x)
        pred = lengths(a, p, lines)
        full = lengths(a, p, [0, 1, 2, 3, 4])
        rows_ = [(pred[:, col] - measured[:, j] + (off[col] if offsets else 0.0))
                 for col, j in enumerate(lines)]
        for col, j in enumerate(FIXED):
            rows_.append(10.0 * (full[:, j] - fixed_values[:, col]))
        return np.concatenate(rows_)

    lo = np.r_[np.tile([-3.0, -3.0, -0.6], n), np.tile([-0.5, -0.5, -0.2], P),
               np.full(k, -2.0)]
    hi = np.r_[np.tile([3.0, 3.0, 2.5], n), np.tile([0.5, 0.5, 0.8], P),
               np.full(k, 2.0)]
    sol = least_squares(residual, x0, bounds=(lo, hi), method='trf', max_nfev=400)
    return unpack(sol.x)[0]


def aligned_rms(a, truth):
    ca, ct = a - a.mean(0), truth - truth.mean(0)
    u, _, vt = np.linalg.svd(ca.T @ ct)
    return float(np.sqrt(np.mean(np.sum((ca @ (u @ vt) - ct) ** 2, axis=1))))


def make_poses(n, seed):
    rng = np.random.default_rng(seed)
    return np.column_stack([rng.uniform(-0.24, 0.24, n), rng.uniform(-0.24, 0.24, n),
                            rng.uniform(0.02, 0.55, n)])


def original_trial(seed, n_points, extra_c, sigma=0.01, c_bias=0.0):
    rng = np.random.default_rng(seed)
    ps = make_poses(n_points, seed + 100)
    truth = lengths(ANCHORS, ps, [0, 1, 2, 3, 4])
    meas = truth + rng.normal(0, sigma, truth.shape)
    meas[:, 2] += c_bias
    lines = [1, 3] + ([2] if extra_c else [])
    a0 = ANCHORS + rng.normal(0, 0.01, ANCHORS.shape)
    p0 = ps + rng.normal(0, 0.01, ps.shape)
    est = fit(a0, p0, meas, lines, truth[:, FIXED], offsets=False)
    return aligned_rms(est, ANCHORS)


def matched_trial(seed, n_points, extra_c, *, c_bias=0.0, slack_mm=0.0,
                  offset_fit=False, sigma=0.01, fixed_sigma=0.0):
    """Same poses/noise/initial guess for both policies; nested point subsets."""
    rng = np.random.default_rng(seed + 1000)
    ps30 = make_poses(30, seed + 100)
    ps = ps30[:n_points]
    truth30 = lengths(ANCHORS, ps30, [0, 1, 2, 3, 4])
    noise30 = rng.normal(0, sigma, truth30.shape)
    meas = truth30[:n_points] + noise30[:n_points]
    meas[:, 2] += c_bias
    if slack_mm and n_points > 3:
        meas[n_points // 2, 2] += slack_mm
    fixed = truth30[:n_points][:, FIXED]
    if fixed_sigma:
        fixed = fixed + rng.normal(0, fixed_sigma, fixed.shape)
    a0 = ANCHORS + rng.normal(0, 0.01, ANCHORS.shape)
    p0 = ps + rng.normal(0, 0.01, ps.shape)
    lines = [1, 3] + ([2] if extra_c else [])
    est = fit(a0, p0, meas, lines, fixed, offsets=offset_fit)
    return aligned_rms(est, ANCHORS)


def stat(x):
    x = np.asarray(x, float)
    return dict(mean=float(x.mean()), median=float(np.median(x)),
                sd=float(x.std(ddof=1)), se=float(x.std(ddof=1) / math.sqrt(len(x))))


def main():
    here = Path(__file__).resolve().parent
    out = {}

    # ---- Fisher
    ps = poses()
    f = {}
    for sigma in (0.005, 0.01, 0.02, 0.05, 0.1):
        f[str(sigma)] = {'current': spectrum(fisher(ANCHORS, ps, CURRENT, sigma)),
                         'alt': spectrum(fisher(ANCHORS, ps, ALT, sigma))}
    c, a = f['0.01']['current'], f['0.01']['alt']
    ratio = a['info'] / c['info']
    sigma_ratio = np.sqrt(ratio)
    out['fisher'] = {
        'sigma_trace_mm': [c['sigma_trace_mm'], a['sigma_trace_mm']],
        'sigma_trace_ratio': c['sigma_trace_mm'] / a['sigma_trace_mm'],
        'variance_ratio': (c['sigma_trace_mm'] / a['sigma_trace_mm']) ** 2,
        'weakest_info': [c['weakest'], a['weakest']],
        'weakest_sigma_ratio': math.sqrt(a['weakest'] / c['weakest']),
        'median_info': [c['median'], a['median']],
        'median_sigma_ratio': math.sqrt(a['median'] / c['median']),
        'harmonic_sigma_ratio': c['harmonic'] / a['harmonic'],
        'strongest_info': [c['strongest'], a['strongest']],
        'strongest_sigma_ratio': math.sqrt(a['strongest'] / c['strongest']),
        'per_mode_sigma_ratio_sorted': [float(x) for x in np.sort(sigma_ratio)],
        'n_modes_improved_gt_2x_sigma': int(np.sum(sigma_ratio > 2)),
        'n_modes_improved_gt_5x_sigma': int(np.sum(sigma_ratio > 5)),
        'sigma_scale_invariant_trace_ratios': {
            s: f[s]['current']['sigma_trace_mm'] / f[s]['alt']['sigma_trace_mm']
            for s in f},
    }

    # ---- Original (non-nested) design, exact public seeds, 6 trials
    orig = {}
    for n in (10, 20, 30):
        for extra in (False, True):
            vals = [original_trial(s, n, extra) for s in range(6)]
            orig[f'{n}_{"alt" if extra else "cur"}'] = stat(vals)
    for extra in (False, True):
        vals = [original_trial(s, 20, extra, c_bias=0.05) for s in range(6)]
        orig[f'20_bias_{"alt" if extra else "cur"}'] = stat(vals)
    out['original_design'] = orig

    # ---- Matched paired nested design, 12 trials
    seeds = range(6)
    matched = {}
    for n in (10, 20):
        cur = [matched_trial(s, n, False) for s in seeds]
        alt = [matched_trial(s, n, True) for s in seeds]
        d = np.array(alt) - np.array(cur)
        matched[f'{n}_cur'] = stat(cur)
        matched[f'{n}_alt'] = stat(alt)
        matched[f'{n}_paired_diff'] = stat(d)
    # Equal-time in the same 20-pose geometry: alt from 10 points vs cur from 20.
    eq = np.array([matched_trial(s, 10, True) for s in seeds]) - \
        np.array([matched_trial(s, 20, False) for s in seeds])
    matched['equal_time_alt10_minus_cur20'] = stat(eq)
    out['matched_paired'] = matched

    # ---- Constant 0.05 mm C bias: offset-free vs production-style home offsets
    bias = {}
    for offset_fit in (False, True):
        for policy, extra in (('cur', False), ('alt', True)):
            vals = [matched_trial(s, 20, extra, c_bias=0.05,
                                  offset_fit=offset_fit) for s in seeds]
            bias[f'{"offset" if offset_fit else "offsetfree"}_{policy}'] = stat(vals)
    bias['offsetfree_gain_mm'] = (bias['offsetfree_cur']['mean']
                                  - bias['offsetfree_alt']['mean'])
    bias['offset_gain_mm'] = bias['offset_cur']['mean'] - bias['offset_alt']['mean']
    out['constant_bias_0.05mm'] = bias

    # ---- Pose-dependent slack: +1.2 mm on one C pose, both fit styles
    slack = {}
    for offset_fit in (False, True):
        vals = [matched_trial(s, 20, True, slack_mm=1.2,
                              offset_fit=offset_fit) for s in seeds]
        slack['offset' if offset_fit else 'offsetfree'] = stat(vals)
    out['pose_slack_1.2mm'] = slack

    # ---- Noisy fixed lines instead of exact fixed constraints
    noisy = {}
    for policy, extra in (('cur', False), ('alt', True)):
        vals = [matched_trial(s, 20, extra, fixed_sigma=0.01) for s in seeds]
        noisy[policy] = stat(vals)
    noisy['gain_mm'] = noisy['cur']['mean'] - noisy['alt']['mean']
    out['fixed_line_noise_0.01mm'] = noisy

    with open(here / 'stats_review.json', 'w') as fh:
        json.dump(out, fh, indent=1)

    print(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()
