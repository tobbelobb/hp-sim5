#!/usr/bin/env python3
"""Per-run summary of DaRUS-3600 (cable actuator test rig) constant-speed runs.

Each export holds one 10 m constant-speed pass; only samples with bTrigger_Record set are
used. Optical travel is compared with drum travel (dPosition_test) as a slope over the pass,
so a constant offset between sensors drops out. Writes one CSV row per export.
"""
import argparse
import csv
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src' / 'python'))
from hp_sim5_research.reference_data import dataset_root, read_scope  # noqa: E402

FIELDS = ['control', 'force_target', 'speed', 'direction', 'export', 'duration',
          'force_test', 'force_load', 'force_diff', 'force_test_std', 'speed_test',
          'lsv_ratio', 'asco_ratio', 'lsv_nonlinearity_mm', 'asco_nonlinearity_mm',
          'travel_diff_drift_mm', 'torque_test', 'torque_load']


def ratio(sensor, drum):
    """Slope of sensor travel against drum travel, and the RMS residual of that line in mm."""
    fit = np.polyfit(drum, sensor, 1)
    return fit[0], 1e3 * np.sqrt(np.mean((sensor - np.polyval(fit, drum)) ** 2))


def summarize(path, control, cell, direction):
    force, feed = map(int, re.fullmatch(r'WF(\d+)_F(\d+)', cell).groups())
    s = read_scope(path).recorded('bTrigger_Record')
    drum = s['dPosition_test']
    lsv, lsv_rms = ratio(s['dPosition_lsv_enc'], drum)
    asco, asco_rms = ratio(s['dPosition_asco_enc'], drum)
    diff = s['dPosition_diff']
    return dict(
        control=control, force_target=force, speed=feed / 60000, direction=direction,
        export=int(re.search(r'(\d+)', path.stem).group(1)), duration=s.t[-1],
        force_test=s['dForce_test'].mean(), force_load=s['dForce_load'].mean(),
        force_diff=s['dForce_diff'].mean(), force_test_std=s['dForce_test'].std(),
        speed_test=np.abs(s['dVelocity_test']).mean(),
        lsv_ratio=lsv, asco_ratio=asco, lsv_nonlinearity_mm=lsv_rms, asco_nonlinearity_mm=asco_rms,
        travel_diff_drift_mm=1e3 * (diff[-1] - diff[0]),
        # The rig software logs torques at 1/10 scale (see the authors' catanalyze.m).
        torque_test=10 * s['dTorque_test'].mean(), torque_load=10 * s['dTorque_load'].mean())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    root = dataset_root('3600') / 'Data'
    paths = sorted(root.glob('*_based/WF*_F*/*wind/*.csv'))
    with args.output.open('w', newline='') as f:
        writer = csv.DictWriter(f, FIELDS)
        writer.writeheader()
        for i, path in enumerate(paths, 1):
            control, cell, direction = path.parts[-4], path.parts[-3], path.parts[-2]
            writer.writerow(summarize(path, control.split('_')[0], cell, direction))
            print(f'[{i}/{len(paths)}] {control}/{cell}/{direction}/{path.name}', flush=True)


if __name__ == '__main__':
    main()
