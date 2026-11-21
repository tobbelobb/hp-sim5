"""Spread, attribution and score-flagging statistics over imperfect-machine runs.

usage: analyze.py RUN_DIR... > analysis.txt
"""
import json
import sys
from pathlib import Path

import numpy as np

runs = []
for directory in map(Path, sys.argv[1:]):
    path = directory / 'result.json'
    if path.exists():
        r = json.loads(path.read_text())
        runs.append((directory.name, r))
failed = [(n, r['stop_reason']) for n, r in runs if 'fitted_mm' not in r]
done = [(n, r) for n, r in runs if 'fitted_mm' in r]
print(f'{len(done)} completed, {len(failed)} failed: {failed}')


def spread(label, values):
    values = np.asarray(values)
    print(f'  {label:28s} mean {values.mean():+.2f}  sd {values.std(ddof=1):.2f}  '
          f'min {values.min():+.2f}  max {values.max():+.2f}')


names = done[0][1]['anchor_names']
frame = np.array([r['frame_error_mm'] for _, r in done])
print('\nPer-coordinate frame error (mm) across runs:')
for i, name in enumerate(names):
    for j, axis in enumerate('xyz'):
        if name == names[0] and axis == 'x' or name == names[-1] and axis in 'xy':
            continue  # fixed by the convention
        spread(f'{name}.{axis}', frame[:, i, j])
print('\nSummary columns:')
for key in ('max_abs_frame_error_mm', 'max_abs_rigid_error_mm', 'max_abs_raw_error_mm', 'simulated_s'):
    spread(key, [r[key] for _, r in done])
common = frame[:, :, 2].mean(axis=1)
spread('mean z error (common shift)', common)
spread('max |frame - common z|', [np.abs(f - np.array([0, 0, c])).max() for f, c in zip(frame, common)])

# Attribution: least squares of each z error on the perturbation parameters.
rows, target = [], {}
for name, r in done:
    imp = r['imperfection']
    dz = np.array([imp['shifts_mm'][n][2] for n in names[:-1]])
    top = np.array(imp['shifts_mm'][names[-1]][:2])
    rows.append([1.0, imp['stiffness_scale'] - 1, np.hypot(*top), np.ptp(dz)])
X = np.array(rows)
print('\nLinear attribution, coefficient (mm per unit) and t-ratio; columns: '
      'intercept, (k-1), |top offset| mm, low dz range mm')
for label, y in (('mean low z', frame[:, :-1, 2].mean(axis=1)), ('top z', frame[:, -1, 2]),
                 ('max frame', [r['max_abs_frame_error_mm'] for _, r in done]),
                 ('max rigid', [r['max_abs_rigid_error_mm'] for _, r in done])):
    y = np.asarray(y)
    beta, *_ = np.linalg.lstsq(X, y, rcond=None)
    resid = y - X @ beta
    dof = max(1, len(y) - X.shape[1])
    cov = np.linalg.pinv(X.T @ X) * (resid @ resid) / dof
    t = beta / np.sqrt(np.diag(cov))
    print(f'  {label:10s} ' + '  '.join(f'{b:+.3f} (t {tt:+.1f})' for b, tt in zip(beta, t))
          + f'   residual sd {resid.std(ddof=X.shape[1]):.2f}')

# Do the fit's own scores flag the worst runs?
print('\nSpearman rank correlation of a score with max frame / rigid error:')
def rank(v):
    return np.argsort(np.argsort(v)).astype(float)
for key, get in (('pose_fit_rms_mm', lambda r: r['iterations'][-1]['pose_fit_rms_mm']),
                 ('chi2_red', lambda r: r['iterations'][-1]['chi2_red']),
                 ('jackknife anchor_uncertainty_mm', lambda r: r['selection'].get('anchor_uncertainty_mm')),
                 ('max_std_mm', lambda r: r['selection'].get('max_std_mm')),
                 ('split_fit_difference_mm', lambda r: r.get('split_fit_difference_mm'))):
    score = np.array([get(r) for _, r in done], dtype=float)
    for target_key in ('max_abs_frame_error_mm', 'max_abs_rigid_error_mm'):
        error = np.array([r[target_key] for _, r in done])
        rho = np.corrcoef(rank(score), rank(error))[0, 1]
        print(f'  {key:32s} vs {target_key:24s} rho {rho:+.2f}')
