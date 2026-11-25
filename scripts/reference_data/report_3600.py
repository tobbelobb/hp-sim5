#!/usr/bin/env python3
"""Stage 1 report for DaRUS-3600: how rope travel (LSV) tracks drum travel across force and speed.

Reads the per-run CSV from summarize_3600.py. For each control mode, direction and speed it
fits the LSV/drum travel ratio against measured force. hp-sim5 stores rope on a spool as rest
length, so at constant tension its span rope moves (1 + F/EA) times faster than the drum
surface: a slope of +1e6/EA ppm/N. Rope wound under the same tension it runs at gives zero.
"""
import argparse
import collections
import csv
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', type=Path)
    args = parser.parse_args()
    rows = list(csv.DictReader(args.runs.open()))
    cells = collections.defaultdict(list)
    for r in rows:
        cells[(r['control'], r['direction'], float(r['speed']))].append(r)
    print(f'{len(rows)} runs. Ratios in ppm of drum travel; LSV and ASCO are the two optical sensors.\n')
    print('control   direction  speed  force: LSV mean ± s.e. (n)            '
          'slope ppm/N ± s.e.   ASCO sd ppm')
    for key in sorted(cells):
        rs = cells[key]
        force = np.array([float(r['force_test']) for r in rs])
        lsv = 1e6 * (np.array([float(r['lsv_ratio']) for r in rs]) - 1)
        asco = 1e6 * (np.array([float(r['asco_ratio']) for r in rs]) - 1)
        groups = []
        for target in sorted({int(r['force_target']) for r in rs}):
            sel = np.array([int(r['force_target']) == target for r in rs])
            groups.append(f'{target}N {lsv[sel].mean():5.0f}±{lsv[sel].std(ddof=1) / np.sqrt(sel.sum()):3.0f} ({sel.sum()})')
        slope = se = np.nan
        if len(set(np.round(force, -1))) > 1:
            (slope, icept), cov = np.polyfit(force, lsv, 1, cov=True)
            se = np.sqrt(cov[0, 0])
        asco_sd = np.mean([asco[[int(r['force_target']) == t for r in rs]].std(ddof=1)
                           for t in {int(r['force_target']) for r in rs}])
        print(f'{key[0]:9s} {key[1]:9s} {key[2]:4.1f}   ' + '  '.join(groups)
              + f'   {slope:6.2f} ± {se:4.2f}   {asco_sd:5.0f}')


if __name__ == '__main__':
    main()
