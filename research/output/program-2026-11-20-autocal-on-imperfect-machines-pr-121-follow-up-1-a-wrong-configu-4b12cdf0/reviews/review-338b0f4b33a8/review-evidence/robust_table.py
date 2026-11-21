"""Summary of the robustness refits: usage robust_table.py DIR...   (reads RUN.CASE.json from robust_lane.sh,
plus RUN.jointk.json files, which count as case 'base' (or 'cog2live' under cog2/)).
Per case and machine: mean ± sd (worst) of print-frame max |error| for commanded forces at true k,
plain joint k, and joint k + fitted force terms; mean ± sd of k/k_true for the last."""
import glob, json, os, sys
from collections import defaultdict
import numpy as np

cases = defaultdict(list)
for d in sys.argv[1:]:
    for f in sorted(glob.glob(os.path.join(d, '*.json'))):
        name = os.path.basename(f)[:-5]
        if '.' not in name or name.endswith('.cog'):
            continue
        run, case = name.split('.', 1)
        if case == 'jointk':
            case = 'cog2live' if 'cog2' in run else 'base'
        r = json.load(open(f))
        cases[(r['machine'], case)].append(r)
stat = lambda a: f'{np.mean(a):.2f} ± {np.std(a):.2f} ({np.max(a):.2f})'
print('| machine | case | n | cmd@true | joint k | joint k + force terms | k/k_true |')
print('| --- | --- | --- | --- | --- | --- | --- |')
for (m, case), R in sorted(cases.items()):
    cmd = [r['commanded_true_k']['max_abs_mm'] for r in R if 'commanded_true_k' in r]
    jk = [r['joint_k_plain']['max_abs_mm'] for r in R]
    jc = [r['joint_k_cog']['max_abs_mm'] for r in R]
    k = np.array([r['joint_k_cog']['k_over_true'] for r in R])
    print(f'| {m} | {case} | {len(R)} | {stat(cmd) if cmd else "–"} | {stat(jk)} | {stat(jc)} | {(k.mean() - 1) * 100:+.1f} ± {k.std() * 100:.1f}% |')
