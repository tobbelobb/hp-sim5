#!/usr/bin/env python3
"""Write a small, decimated fixture of one matched DaRUS-3600 unwind/rewind pair.

Only the recorded constant-speed window and the channels needed for a replay are kept,
so tests and notebooks can run without the Drive mount.
"""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src' / 'python'))
from hp_sim5_research.reference_data import dataset_root, read_scope  # noqa: E402

CHANNELS = ['dPosition_test', 'dPosition_load', 'dVelocity_test', 'dVelocity_load',
            'dForce_test', 'dForce_load', 'dForce_tar', 'dPosition_lsv_enc', 'dPosition_asco_enc',
            'dTorque_test', 'dTorque_load']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--cell', default='Torque_based/WF125_F120000')
    parser.add_argument('--unwind', default='Export 11.csv')
    parser.add_argument('--rewind', default='Export 10.csv')
    parser.add_argument('--every', type=int, default=10, help='Keep every n-th 1 ms sample')
    args = parser.parse_args()
    out = {}
    for direction, name in [('unwind', args.unwind), ('rewind', args.rewind)]:
        path = dataset_root('3600') / 'Data' / args.cell / direction.capitalize() / name
        s = read_scope(path).recorded('bTrigger_Record')
        out[f'{direction}/t'] = s.t[::args.every]
        for c in CHANNELS:
            out[f'{direction}/{c}'] = s[c][::args.every].astype(np.float32)
        out[f'{direction}/source'] = np.array(str(path.relative_to(dataset_root('3600'))))
    np.savez_compressed(args.output, **out)


if __name__ == '__main__':
    main()
