#!/usr/bin/env python3
"""Extract compact decimated measured motor trajectories for closed-loop solver replay tests.

Generates two small CSV fixtures (committed to `tests/fixtures/reference_data/`):
  1. `darus5171_replay_wc180_dp75.csv` — full wind (F_S ~ 77 N) -> turnaround -> unwind
     (F_L ~ 177 N) measured trajectory from DaRUS-5171 (`LIROS_D-Pro`, `FDP-FC-MAP/WC180/Export 1.csv`),
     decimated to 25 ms (~800 rows).
  2. `darus3600_replay_pair.csv` — measured unwind and rewind trajectories (125 N, 2 m/s)
     from the committed `darus3600_WF125_F120000_torque_pair.npz`, decimated to 20 ms (500 rows).
"""
import argparse
import csv
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src' / 'python'))
from hp_sim5_research.reference_data import dataset_root, read_scope  # noqa: E402


def extract_5171_replay(out_path: Path, step_ms: int = 25):
    root = dataset_root('5171')
    src = root / 'LIROS_D-Pro/2025-03-11a_SlipIdent/Data/FDP-FC-MAP/WC180/Export 1.csv'
    s = read_scope(src)
    # Keep t in [9.5, 29.5] s: full 18 m wind pass at ~77 N, turnaround/re-tension, and 10 m unwind at ~177 N.
    mask = (s.t >= 9.5) & (s.t <= 29.5)
    idx = np.where(mask)[0][::step_ms]
    t0 = float(s.t[idx[0]])
    with out_path.open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'x_t', 'x_l', 'f_t', 'f_l', 'x_lsv', 'trig_rec'])
        for i in idx:
            w.writerow([
                f'{s.t[i] - t0:.4f}',
                f'{s["x_T"][i]:.6f}',
                f'{s["x_L"][i]:.6f}',
                f'{s["f_T"][i]:.3f}',
                f'{s["f_L"][i]:.3f}',
                f'{s["x_lsv_enc"][i]:.6f}',
                f'{int(s["trig_rec"][i] > 0.5)}',
            ])
    print(f'Wrote {len(idx)} rows to {out_path}')


def extract_3600_replay(npz_path: Path, out_path: Path, stride: int = 2):
    z = np.load(npz_path)
    n_rows = 0
    with out_path.open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['phase', 't', 'x_t', 'x_l', 'f_t', 'f_l', 'x_lsv'])
        for phase in ('unwind', 'rewind'):
            t = z[f'{phase}/t'][::stride]
            xt = z[f'{phase}/dPosition_test'][::stride]
            xl = z[f'{phase}/dPosition_load'][::stride]
            ft = z[f'{phase}/dForce_test'][::stride]
            fl = z[f'{phase}/dForce_load'][::stride]
            lsv = z[f'{phase}/dPosition_lsv_enc'][::stride]
            for i in range(len(t)):
                w.writerow([
                    phase,
                    f'{float(t[i]):.4f}',
                    f'{float(xt[i]):.6f}',
                    f'{float(xl[i]):.6f}',
                    f'{float(ft[i]):.3f}',
                    f'{float(fl[i]):.3f}',
                    f'{float(lsv[i]):.6f}',
                ])
                n_rows += 1
    print(f'Wrote {n_rows} rows to {out_path}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--out-dir',
        type=Path,
        default=Path(__file__).resolve().parents[2] / 'tests' / 'fixtures' / 'reference_data',
    )
    args = parser.parse_args()
    extract_5171_replay(args.out_dir / 'darus5171_replay_wc180_dp75.csv')
    extract_3600_replay(
        args.out_dir / 'darus3600_WF125_F120000_torque_pair.npz',
        args.out_dir / 'darus3600_replay_pair.csv',
    )


if __name__ == '__main__':
    main()
