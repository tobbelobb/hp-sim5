#!/usr/bin/env python3
"""Summarize DaRUS-4614 (COPacabana 8-cable robot) PlaneA measurements.

Registers the FARO laser-tracker frame via Kabsch against `00_Lasertracker_Ref` and writes one
CSV row per (experiment, pose) covering home P0 and ISO 9283 poses P1..P5 across both payloads
(13.9 and 29.1 kg) and all four preload modes (STD, APC25, APC50, APC75): mean laser-tracker
position (both raw registered and P0-aligned), cycle-to-cycle standard deviation, mean motor
positions x[0..7], target x_tar[0..7], and mean cable forces f[0..7].
"""
import argparse
import csv
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src' / 'python'))
from hp_sim5_research.reference_data import dataset_root, read_scope, read_tracker  # noqa: E402

EXPERIMENTS = [
    ('20_Std_Load13900', 'STD', 13.9),
    ('40_Apc_Load13900_V25', 'APC25', 13.9),
    ('41_Apc_Load13900_V50', 'APC50', 13.9),
    ('42_Apc_Load13900_V75', 'APC75', 13.9),
    ('10_Std_Load29100', 'STD', 29.1),
    ('30_Apc_Load29100_V25', 'APC25', 29.1),
    ('31_Apc_Load29100_V50', 'APC50', 29.1),
    ('32_Apc_Load29100_V75', 'APC75', 29.1),
]

P0_TAR = np.array([0.0, 0.0, 0.6])
P_TARGETS = np.array([
    [97.40, -2.50, 449.40],
    [847.90, 448.30, 698.80],
    [847.90, -450.80, 698.80],
    [-750.50, -450.80, 200.00],
    [-750.50, 448.30, 200.00],
]) / 1000.0

FIELDS = (
    ['experiment', 'mode', 'mass_kg', 'pose', 'tar_x', 'tar_y', 'tar_z',
     'raw_x', 'raw_y', 'raw_z', 'meas_x', 'meas_y', 'meas_z',
     'std_x', 'std_y', 'std_z', 'cycles']
    + [f'x{i}' for i in range(8)]
    + [f'xtar{i}' for i in range(8)]
    + [f'f{i}' for i in range(8)]
)


def kabsch(p_src, p_dst):
    cp, cq = p_src.mean(axis=0), p_dst.mean(axis=0)
    u, _, vt = np.linalg.svd((p_src - cp).T @ (p_dst - cq))
    if np.linalg.det(vt.T @ u.T) < 0:
        vt[-1] *= -1
    rot = vt.T @ u.T
    return rot, cq - rot @ cp


def filter_static_plane_a(pos, dt=0.01, thres=0.01, vmax=5.0, n_pts=5):
    dp = np.diff(pos, axis=0)
    v_norm = np.linalg.norm(np.vstack([dp[1:2], dp]) / dt, axis=1)
    static = pos[v_norm < thres]
    dpt = np.diff(static, axis=0)
    vt = np.vstack([dpt[1:2], dpt]) / dt
    idx = np.unique(np.argwhere(np.abs(vt) > vmax)[:, 0])
    p0 = static[:idx[0]].mean(axis=0)
    means = np.array([static[idx[k - 1]:idx[k]].mean(axis=0) for k in range(1, len(idx))])
    n_cycles = len(means) // n_pts
    return p0, [means[i:n_cycles * n_pts:n_pts] for i in range(n_pts)]


def load_registration(tracker_root):
    ref_dir = tracker_root / '00_Lasertracker_Ref'
    lines = (ref_dir / 'COPacabana_Ref' / 'COP_Ref.csv').read_text().splitlines()[3:]
    c0 = np.array([[float(x) for x in l.split(';')] for l in lines if l.strip()]) / 1000.0
    c_t = np.array([read_tracker(p)[0].mean(axis=0) for p in sorted(ref_dir.glob('p*.csv'))])
    return kabsch(c_t, c0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--base', type=Path, default=None,
                        help='Directory containing robot/ and tracker/ (defaults to DaRUS-4614 dataset)')
    parser.add_argument('--robot-exports', type=int, default=10,
                        help='Number of PlaneA robot exports to average per experiment')
    args = parser.parse_args()

    if args.base is not None:
        robot_root = args.base / 'robot'
        tracker_root = args.base / 'tracker'
    else:
        exp_dir = dataset_root('4614') / 'exp-apc_darus-upload' / 'experiments'
        robot_root = exp_dir / 'robot'
        tracker_root = exp_dir / 'tracker'

    rot_ref, t_ref = load_registration(tracker_root)
    rows = []

    for exp, mode, mass_kg in EXPERIMENTS:
        pos, _ = read_tracker(tracker_root / exp / 'PlaneA.csv')
        p0_t, p_i_t = filter_static_plane_a(pos)
        p0_raw = rot_ref @ p0_t + t_ref
        p_ref = P0_TAR - p0_raw

        # Average robot encoder positions and cable forces over the first N PlaneA cycles.
        all_targets = np.vstack([P0_TAR, P_TARGETS])
        x_acc, xt_acc, f_acc = [], [], []
        for ex in range(1, args.robot_exports + 1):
            p = robot_root / exp / 'PlaneA' / f'Export {ex}.csv'
            if not p.exists():
                continue
            s = read_scope(p)
            pr = np.column_stack([s[f'p[{i}]'] for i in range(3)])
            xr = np.column_stack([s[f'x[{i}]'] for i in range(8)])
            xtr = np.column_stack([s[f'x_tar[{i}]'] for i in range(8)])
            fr = np.column_stack([s[f'f[{i}]'] for i in range(8)])
            idxs = [int(np.argmin(np.linalg.norm(pr - pt, axis=1))) for pt in all_targets]
            # Export 1 starts at P0=[0,0,0.6]; Exports 2..N start at P5, so only use Export 1 for P0.
            x_acc.append(xr[idxs])
            xt_acc.append(xtr[idxs])
            f_acc.append(fr[idxs])
        x_mean = np.mean(x_acc, axis=0)
        xt_mean = np.mean(xt_acc, axis=0)
        f_mean = np.mean(f_acc, axis=0)
        # Keep Export 1 for P0 since only Export 1 starts at Home Z=600.
        x_mean[0], xt_mean[0], f_mean[0] = x_acc[0][0], xt_acc[0][0], f_acc[0][0]

        for idx_p, name in enumerate(['P0', 'P1', 'P2', 'P3', 'P4', 'P5']):
            if idx_p == 0:
                raw = p0_raw
                meas = P0_TAR
                std = np.zeros(3)
                cycles = 1
            else:
                pts_raw = p_i_t[idx_p - 1] @ rot_ref.T + t_ref
                raw = pts_raw.mean(axis=0)
                meas = raw + p_ref
                std = pts_raw.std(axis=0)
                cycles = len(pts_raw)
            row = dict(
                experiment=exp, mode=mode, mass_kg=mass_kg, pose=name,
                tar_x=all_targets[idx_p, 0], tar_y=all_targets[idx_p, 1], tar_z=all_targets[idx_p, 2],
                raw_x=raw[0], raw_y=raw[1], raw_z=raw[2],
                meas_x=meas[0], meas_y=meas[1], meas_z=meas[2],
                std_x=std[0], std_y=std[1], std_z=std[2], cycles=cycles,
            )
            for i in range(8):
                row[f'x{i}'] = x_mean[idx_p, i]
                row[f'xtar{i}'] = xt_mean[idx_p, i]
                row[f'f{i}'] = f_mean[idx_p, i]
            rows.append(row)
        print(f'[4614] {exp} ({len(p_i_t[0])} tracker cycles, {len(x_acc)} robot exports)', flush=True)

    with args.output.open('w', newline='') as f:
        w = csv.DictWriter(f, FIELDS)
        w.writeheader()
        w.writerows(rows)


if __name__ == '__main__':
    main()
