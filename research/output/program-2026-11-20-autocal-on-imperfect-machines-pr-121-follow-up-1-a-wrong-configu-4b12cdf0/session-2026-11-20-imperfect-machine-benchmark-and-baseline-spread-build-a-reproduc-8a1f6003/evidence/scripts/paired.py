"""Paired old-fit vs new-fit comparison of the same imperfect machines.

usage: paired.py OLD_ROOT NEW_ROOT
"""
import json
import re
import sys
from pathlib import Path

import numpy as np

old_root, new_root = map(Path, sys.argv[1:3])
rows = []
for new in sorted(new_root.glob('*/result.json')):
    old = old_root / new.parent.name / 'result.json'
    if not old.exists():
        continue
    a, b = json.loads(old.read_text()), json.loads(new.read_text())
    if 'fitted_mm' not in a or 'fitted_mm' not in b:
        print(new.parent.name, 'missing fit', a.get('stop_reason'), b.get('stop_reason'))
        continue
    rows.append((new.parent.name, a, b))
keys = [('frame', 'max_abs_frame_error_mm'), ('rigid', 'max_abs_rigid_error_mm'),
        ('sim_s', 'simulated_s')]
print(f'{"case":12s} ' + ' '.join(f'{k:>16s}' for k, _ in keys) + '   jack old/new   sweeps old/new  stop new')
diffs = {k: [] for k, _ in keys}
for name, a, b in rows:
    cells = []
    for k, key in keys:
        diffs[k].append(b[key] - a[key])
        cells.append(f'{a[key]:7.2f}>{b[key]:<7.2f}' if k != 'sim_s' else f'{a[key]:6.0f}>{b[key]:<6.0f}')
    ja, jb = a['selection'].get('anchor_uncertainty_mm'), b['selection'].get('anchor_uncertainty_mm')
    print(f'{name:12s} ' + ' '.join(f'{c:>16s}' for c in cells)
          + f'   {ja:.2f}/{jb:.2f}      {a["iterations"][-1]["num_sweeps"]}/{b["iterations"][-1]["num_sweeps"]}'
          + f'            {b["stop_reason"]}')
for subset, pick in (('all pairs', lambda n: True), ('random seeds s0-s9', lambda n: re.fullmatch(r'hp3-s\d', n) is not None)):
    idx = [i for i, (n, _, _) in enumerate(rows) if pick(n)]
    print(f'\n{subset} (n={len(idx)}): paired new - old')
    for k, _ in keys:
        d = np.array(diffs[k])[idx]
        print(f'  {k:6s} mean {d.mean():+.3f}  sd {d.std(ddof=1):.3f}  se {d.std(ddof=1) / np.sqrt(len(d)):.3f}  '
              f'min {d.min():+.3f}  max {d.max():+.3f}  new better in {(d < 0).sum()}/{len(d)}')
    for label, key in (('old', 1), ('new', 2)):
        v = np.array([rows[i][key]['max_abs_frame_error_mm'] for i in idx])
        print(f'  {label} frame error mean {v.mean():.2f} sd {v.std(ddof=1):.2f} max {v.max():.2f}')
