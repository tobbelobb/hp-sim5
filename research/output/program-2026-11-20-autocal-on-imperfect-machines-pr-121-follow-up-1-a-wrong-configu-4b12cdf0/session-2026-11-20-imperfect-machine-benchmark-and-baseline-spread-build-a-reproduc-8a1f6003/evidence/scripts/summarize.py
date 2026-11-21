"""Table of imperfect-machine runs: usage summarize.py RUN_DIR... > table.txt

Columns: perturbation, raw and gauge-equivalent max error, per-coordinate gauge
errors, and the fit's own quality scores at the accepted fit.
"""
import json
import sys
from pathlib import Path

import numpy as np

rows = []
for directory in map(Path, sys.argv[1:]):
    path = directory / 'result.json'
    if not path.exists():
        continue
    r = json.loads(path.read_text())
    if 'fitted_mm' not in r:
        print(f'{directory.name}: no accepted fit ({r.get("stop_reason")})')
        continue
    imp = r['imperfection']
    names = r['anchor_names']
    low = [imp['shifts_mm'][n][2] for n in names[:-1]]
    top = imp['shifts_mm'][names[-1]][:2]
    g = np.asarray(r['frame_error_mm'])
    sel = r['selection']
    last = r['iterations'][-1]
    rows.append(dict(name=directory.name, dz=' '.join(f'{v:+5.1f}' for v in low),
                     top=f'{np.hypot(*top):4.1f}', k=f'{imp["stiffness_scale"]:.3f}',
                     tiltD=f'{r["truth_frame_tilt_deg"]:.2f}', frame=f'{r["max_abs_frame_error_mm"]:4.2f}',
                     worst=r['worst_frame_coordinate'], lowz=' '.join(f'{v:+5.2f}' for v in g[:-1, 2]),
                     Dz=f'{g[-1, 2]:+5.2f}', xy=f'{np.abs(g[:, :2]).max():4.2f}',
                     raw=f'{r["max_abs_raw_error_mm"]:5.2f}', gauge=f'{r["max_abs_gauge_error_mm"]:4.2f}',
                     rigid=f'{r["max_abs_rigid_error_mm"]:4.2f}',
                     rms=f'{last["pose_fit_rms_mm"]:.4f}', chi2=f'{last["chi2_red"]:.2f}',
                     jack=f'{sel.get("anchor_uncertainty_mm") or float("nan"):.2f}',
                     split=f'{r.get("split_fit_difference_mm") or float("nan"):.2f}',
                     sweeps=str(last['num_sweeps']), sim=f'{r["simulated_s"]:.0f}', stop=r['stop_reason']))
if rows:
    keys = list(rows[0])
    width = {k: max(len(k), *(len(row[k]) for row in rows)) for k in keys}
    print('  '.join(k.ljust(width[k]) for k in keys))
    for row in rows:
        print('  '.join(row[k].ljust(width[k]) for k in keys))
