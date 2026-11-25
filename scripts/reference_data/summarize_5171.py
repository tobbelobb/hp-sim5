#!/usr/bin/env python3
"""Summarize DaRUS-5171 storage-vs-unwinding slip identification and validation runs.

Produces two compact CSV tables across all three ropes (LIROS D-Pro XTR, Trowis Vectran HT T190,
Trowis PolyTrop FOX):
  1. `FDP-FC-MAP` identification runs (25 per rope, 5 storage tensions F_S x 5 unwinding tensions F_L).
  2. `StatCompensationValid` held-out validation unwinding passes (9 cells x 2 speeds per rope).

Each row compares the optical LSV travel and the load-drum encoder against test-drum travel using
linear-fit slopes over the recorded 10 m window, so constant sensor offsets drop out.
"""
import argparse
import csv
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src' / 'python'))
from hp_sim5_research.reference_data import dataset_root, read_scope  # noqa: E402

ROPES = [
    ('LIROS_D-Pro', '2025-03-11a_SlipIdent', '2025-03-11f_StatCompensationValid'),
    ('Trowis_VectranHT_T190PU', '2025-02-15b_SlipIdent', '2025-02-20b_StatCompensationValid'),
    ('Trowis_PolyTropFOX', '2025-02-21_SlipIdent', '2025-03-03a_StatCompensationValid'),
]

IDENT_FIELDS = [
    'rope', 'wc_nom', 'export', 'f_wind', 'f_unwind', 'f_load',
    'speed_test', 'lsv_ratio', 'drum_l_ratio', 'lsv_rms_mm',
]
VALID_FIELDS = [
    'rope', 'f_s_nom', 'f_l_nom', 'export', 'f_unwind', 'f_load',
    'speed_test', 'lsv_ratio', 'drum_l_ratio', 'lsv_rms_mm', 'x_slip_mm',
]


def slope_and_rms(y, x):
    fit = np.polyfit(x, y, 1)
    rms_mm = 1e3 * float(np.sqrt(np.mean((y - np.polyval(fit, x)) ** 2)))
    return float(fit[0]), rms_mm


def summarize_ident(root, out_path):
    rows = []
    for rope, ident_dir, _ in ROPES:
        fdp = root / rope / ident_dir / 'Data' / 'FDP-FC-MAP'
        for wc_dir in sorted(p for p in fdp.iterdir() if p.is_dir()):
            wc_nom = int(re.search(r'(\d+)', wc_dir.name).group(1))
            for path in sorted(wc_dir.glob('*.csv'), key=lambda p: int(re.search(r'(\d+)', p.stem).group(1))):
                s = read_scope(path)
                rec = s.recorded('trig_rec')
                wind = s['v_T'] < -1.0
                f_wind = float(s['f_T'][wind].mean()) if np.any(wind) else float('nan')
                x_t = rec['x_T']
                lsv_ratio, lsv_rms = slope_and_rms(rec['x_lsv_enc'], x_t)
                drum_l_ratio, _ = slope_and_rms(-rec['x_L'], x_t)
                rows.append(dict(
                    rope=rope, wc_nom=wc_nom,
                    export=int(re.search(r'(\d+)', path.stem).group(1)),
                    f_wind=f_wind, f_unwind=float(rec['f_T'].mean()),
                    f_load=float(rec['f_L'].mean()),
                    speed_test=float(rec['v_T'].mean()),
                    lsv_ratio=lsv_ratio, drum_l_ratio=drum_l_ratio, lsv_rms_mm=lsv_rms,
                ))
                print(f'[ident] {rope}/{wc_dir.name}/{path.name} F_S={f_wind:.1f} F_L={rec["f_T"].mean():.1f}', flush=True)
    with out_path.open('w', newline='') as f:
        w = csv.DictWriter(f, IDENT_FIELDS)
        w.writeheader()
        w.writerows(rows)


def summarize_valid(root, out_path):
    rows = []
    for rope, _, valid_dir in ROPES:
        vdir = root / rope / valid_dir / 'Data'
        for cell in sorted(p for p in vdir.iterdir() if p.is_dir()):
            m = re.fullmatch(r'WC(\d+)_DP(\d+)', cell.name)
            if not m:
                continue
            f_l_nom, f_s_nom = map(int, m.groups())
            # Export 4 (+2 m/s, matching FDP-FC-MAP) and Export 6 (+3 m/s, thesis Table 5.1).
            for ex in (4, 6):
                path = cell / f'Export {ex}.csv'
                if not path.exists():
                    continue
                s = read_scope(path)
                rec = s.recorded('trig_rec')
                x_t = rec['x_T']
                lsv_ratio, lsv_rms = slope_and_rms(rec['x_lsv_enc'], x_t)
                drum_l_ratio, _ = slope_and_rms(-rec['x_L'], x_t)
                x_slip_mm = 1e3 * float(rec['x_slip'][-1] - rec['x_slip'][0]) if 'x_slip' in rec.channels else 0.0
                rows.append(dict(
                    rope=rope, f_s_nom=f_s_nom, f_l_nom=f_l_nom, export=ex,
                    f_unwind=float(rec['f_T'].mean()), f_load=float(rec['f_L'].mean()),
                    speed_test=float(rec['v_T'].mean()),
                    lsv_ratio=lsv_ratio, drum_l_ratio=drum_l_ratio,
                    lsv_rms_mm=lsv_rms, x_slip_mm=x_slip_mm,
                ))
                print(f'[valid] {rope}/{cell.name}/Export {ex}.csv v={rec["v_T"].mean():+.2f}', flush=True)
    with out_path.open('w', newline='') as f:
        w = csv.DictWriter(f, VALID_FIELDS)
        w.writeheader()
        w.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ident-out', type=Path, required=True)
    parser.add_argument('--valid-out', type=Path, required=True)
    args = parser.parse_args()
    root = dataset_root('5171')
    summarize_ident(root, args.ident_out)
    summarize_valid(root, args.valid_out)


if __name__ == '__main__':
    main()
