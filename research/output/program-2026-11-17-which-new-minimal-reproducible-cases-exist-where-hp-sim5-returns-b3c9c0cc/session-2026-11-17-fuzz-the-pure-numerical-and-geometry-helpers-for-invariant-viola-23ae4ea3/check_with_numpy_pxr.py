"""Independent reference check for hp-sim5's numerical helpers.

References:
  * pxr (OpenUSD 26.3 bindings) for xformOp:rotate*ABC and quaternion -> matrix.
  * numpy for quaternion algebra, inertia transforms, matrix inverses.
  * scipy for an independent catenary fit and arc-length integration.

Reads js_reference.json produced by emit_js_reference.mjs.

Usage:  .venv/bin/python check_with_numpy_pxr.py
Exits 1 if any comparison exceeds its stated tolerance.
"""
import json
import math
import os
import sys
import warnings

warnings.filterwarnings('ignore', category=RuntimeWarning)

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom
from scipy.integrate import quad
from scipy.optimize import least_squares

np.seterr(all='ignore')

HERE = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(HERE, 'js_reference.json')) as fh:
    DATA = json.load(fh)

failures = []


def report(name, worst, tol, detail=''):
    ok = worst <= tol
    print(f"{'pass' if ok else 'FAIL'}  {name:<50} worst={worst:.3e} tol={tol:.1e}{('  (' + detail + ')') if detail else ''}")
    if not ok:
        failures.append(name)


def quat_to_matrix_xyzw(q):
    """Rotation matrix acting on column vectors, from (x, y, z, w)."""
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def axis_angle(axis, degrees):
    t = np.deg2rad(degrees)
    c, s = math.cos(t), math.sin(t)
    if axis == 'X':
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == 'Y':
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


# ---------------------------------------------------------------- 1. USD rot
print('USD xformOp:rotate*ABC (JS readOrientationAttribute vs pxr)')
stage = Usd.Stage.CreateInMemory()
per_order = {}
for case in DATA['rotateABC']:
    order = case['order']
    angles = case['angles']
    prim = stage.DefinePrim(Sdf.Path('/World/A'), 'Xform')
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    getattr(xf, f'AddRotate{order}Op')().Set(Gf.Vec3f(*angles))
    # Gf matrices use the row-vector convention: transpose for column vectors.
    C = np.array(xf.GetLocalTransformation()).reshape(4, 4)[:3, :3].T
    R = quat_to_matrix_xyzw(case['quaternion'])
    per_order[order] = max(per_order.get(order, 0.0), float(np.max(np.abs(C - R))))
    # also confirm both composition conventions against USD itself
    r_post = np.eye(3)
    for ax in order:
        r_post = axis_angle(ax, angles['XYZ'.index(ax)]) @ r_post
    if np.max(np.abs(C - r_post)) > 1e-6:
        failures.append(f'{order}: USD does not compose as R_last*...*R_first')
for order in sorted(per_order):
    report(f'  rotate{order} vs pxr', per_order[order], 1e-5, 'float32 authored')

# ------------------------------------------------------- 2. quaternion algebra
worst_matrix = 0.0
worst_transform = 0.0
worst_pxr = 0.0
for case in DATA['quaternions']:
    q = case['q']
    Rnp = quat_to_matrix_xyzw(q)
    worst_matrix = max(worst_matrix, float(np.max(np.abs(np.array(case['R']) - Rnp))))
    worst_transform = max(worst_transform, float(np.max(np.abs(Rnp @ np.array(case['v']) - np.array(case['tv'])))))
    x, y, z, w = q
    M = np.array(Gf.Matrix4d(1.0).SetRotate(Gf.Rotation(Gf.Quatd(w, Gf.Vec3d(x, y, z))))).reshape(4, 4)[:3, :3].T
    worst_pxr = max(worst_pxr, float(np.max(np.abs(M - Rnp))))
report('rotationMatrixFromQuaternion vs numpy', worst_matrix, 1e-14)
report('Quaternion.transformVector vs numpy R@v', worst_transform, 1e-14)
report('pxr Gf.Rotation(quat) vs numpy formula', worst_pxr, 1e-14)

# ------------------------------------------------------------- 3. inertia
worst_world = 0.0
worst_eff = 0.0
for case in DATA['inertia']:
    R = quat_to_matrix_xyzw(case['q'])
    I = np.array(case['tensor'])
    ref = R @ I @ R.T
    worst_world = max(worst_world, float(np.max(np.abs(np.array(case['world']) - ref))))
    axis = np.array(case['axis'])
    worst_eff = max(worst_eff, abs(case['effective'] - float(axis @ ref @ axis)) / abs(float(axis @ ref @ axis)))
report('transformInertiaTensorToWorld vs R I R^T', worst_world, 1e-13)
report('effectiveInertiaAboutWorldAxis vs u^T R I R^T u', worst_eff, 1e-12)

# ------------------------------------------------------------- 4. inverse
worst_inv = 0.0
worst_pinv = 0.0
for case in DATA['inverse']:
    M = np.array(case['M'])
    P = np.array(case['P'])
    if case['singular']:
        ref = np.linalg.pinv(M, rcond=1e-15, hermitian=True)
        worst_pinv = max(worst_pinv, float(np.max(np.abs(P - ref))))
        worst_pinv = max(worst_pinv, float(np.max(np.abs(M @ P @ M - M))))
    else:
        prod = P @ M
        worst_inv = max(worst_inv, float(np.max(np.abs(prod - np.eye(3)))))
report('invertMatrix3(W) * W = I (SPD)', worst_inv, 1e-12)
report('invertMatrix3(singular) vs numpy pinv', worst_pinv, 1e-12)

# ------------------------------------------------------------- 5. catenary
print('\nSlack cable shape (JS polyline vs independent catenary fit and arc length)')
worst_residual = 0.0
worst_arclen = 0.0
for case in DATA['catenary']:
    start = np.array(case['start'])
    end = np.array(case['end'])
    up = np.array(case['up'])
    up = up / np.linalg.norm(up)
    chord = end - start
    vertical = float(chord @ up)
    horizontal = chord - up * vertical
    span = float(np.linalg.norm(horizontal))
    u = horizontal / span
    pts = np.array(case['points']).reshape(-1, 3)
    x = (pts - start) @ u
    y = (pts - start) @ up

    def model(params):
        a, c, b = params
        return a * np.cosh((x - c) / a) + b - y

    # Multi-start fit with positivity bounds: a catenary is nearly a parabola for
    # large a, so an unconstrained fit can wander to a negative a.
    best = None
    for start_a in np.geomspace(1e-4 * span, 30 * case['L'], 40):
        for start_c in (0.5 * span, 0.0, span):
            try:
                fit = least_squares(
                    model, x0=[start_a, start_c, -0.5 * case['L']],
                    bounds=([1e-9, -10 * case['L'], -10 * case['L']],
                            [1e6 * case['L'], 10 * case['L'], 10 * case['L']]),
                    xtol=1e-15, ftol=1e-15, gtol=1e-15, max_nfev=20000,
                )
            except Exception:  # noqa: BLE001 - a failed start is just skipped
                continue
            score = float(np.max(np.abs(model(fit.x))))
            if best is None or score < best[1]:
                best = (fit.x, score)
    a, c, b = best[0]
    residual = best[1]
    worst_residual = max(worst_residual, residual / case['L'])
    # arc length of the fitted catenary over the actual horizontal span
    dy = lambda xx: math.sinh((xx - c) / a)
    arc = quad(lambda xx: math.sqrt(1.0 + dy(xx) ** 2), 0.0, span, epsabs=1e-12, epsrel=1e-12)[0]
    worst_arclen = max(worst_arclen, abs(arc - case['L']) / case['L'])
    print(f"  span={span:.3f} vertical={vertical:+.3f} L={case['L']:.3f}: fit a={a:.6f} "
          f"max|residual|/L={residual / case['L']:.2e} fitted arc/L-1={arc / case['L'] - 1:+.2e}")
report('cable polyline lies on one catenary', worst_residual, 1e-9)
report('cable catenary arc length = rest length', worst_arclen, 1e-6)

print('')
if failures:
    print(f'FAILED: {len(failures)} comparison(s): {failures}')
    sys.exit(1)
print('all independent-reference comparisons passed')
