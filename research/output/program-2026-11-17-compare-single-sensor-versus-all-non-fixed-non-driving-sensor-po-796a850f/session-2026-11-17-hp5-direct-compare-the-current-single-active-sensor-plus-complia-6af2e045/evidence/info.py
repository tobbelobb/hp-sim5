"""Geometry-only information comparison for HP5-direct measured-line sets.

At each pose the point position is eliminated analytically, leaving the
sensitivity of the consistency residuals to the five anchor positions.  Fixed
lines are treated as near-exact length constraints; measured lines carry a
common range noise.  The anchor Fisher information is compared for the current
(fixed A,I + drive B + sensor D) and alternative (+ passive C measured) sets.
"""
from __future__ import annotations

import json
import numpy as np

ANCHORS = np.array([[0., -2.15, -0.18], [2.15, 0., -0.18], [0., 2.15, -0.18],
                    [-2.15, 0., -0.18], [0., 0., 2.0]])
CURRENT = [0, 1, 3, 4]      # fixed A, drive B, sensor D, fixed top I
ALT = [0, 1, 2, 3, 4]       # plus passive C
FIXED = [0, 4]


def poses(n=30):
    rng = np.random.default_rng(7)
    # Broad 3D patch: corners of a box plus jitter, restrained inside the frame.
    u = rng.uniform(-1, 1, (n, 2))
    p = np.column_stack([0.22 * u[:, 0], 0.22 * u[:, 1], rng.uniform(0.02, 0.55, n)])
    return p


def rows(point, anchors, lines, sigma_fixed=1e-3, sigma_meas=0.01):
    """Jacobian rows wrt [all anchor coords, point coords] and weights."""
    n = len(anchors)
    J = np.zeros((len(lines), 3 * n + 3))
    w = np.zeros(len(lines))
    for row, j in enumerate(lines):
        d = anchors[j] - point
        dist = np.linalg.norm(d)
        unit = d / dist
        J[row, 3 * j:3 * j + 3] = unit - anchors[j] / np.linalg.norm(anchors[j])
        J[row, 3 * n:3 * n + 3] = -unit
        w[row] = 1.0 / (sigma_fixed if j in FIXED else sigma_meas) ** 2
    return J, w


def fisher(anchors, positions, lines, sigma_meas):
    n = len(anchors)
    F = np.zeros((3 * n, 3 * n))
    for p in positions:
        J, w = rows(p, anchors, lines, sigma_meas=sigma_meas)
        Ja, Jp = J[:, :3 * n], J[:, 3 * n:]
        W = np.diag(w)
        # Eliminate the point: effective residual sensitivity to anchors.
        A = Ja - Jp @ np.linalg.solve(Jp.T @ W @ Jp, Jp.T @ W @ Ja)
        F += A.T @ W @ A
    return F


def spectrum(F):
    vals = np.linalg.eigvalsh(F)
    vals = np.clip(vals, 0, None)
    # Six rigid-body gauge modes are (near) zero.
    informative = vals[6:]
    return dict(smallest=float(informative[0]), median=float(np.median(informative)),
                largest=float(informative[-1]),
                harmonic=float(len(informative) / np.sum(1.0 / informative)),
                trace=float(np.sum(informative)),
                sigma_trace_mm=float(np.sqrt(np.sum(1.0 / informative))),
                vals=[float(v) for v in vals])


if __name__ == '__main__':
    ps = poses()
    out = {}
    for sigma in (0.005, 0.01, 0.02, 0.05, 0.1):
        out[sigma] = {'current': spectrum(fisher(ANCHORS, ps, CURRENT, sigma)),
                      'alt': spectrum(fisher(ANCHORS, ps, ALT, sigma))}
    print(json.dumps(dict(poses=ps.tolist(), anchors=ANCHORS.tolist(),
                          current=CURRENT, alt=ALT, results=out), indent=1))
    with open('info.json', 'w') as f:
        json.dump(dict(poses=ps.tolist(), anchors=ANCHORS.tolist(), current=CURRENT,
                       alt=ALT, results=out), f, indent=1)
