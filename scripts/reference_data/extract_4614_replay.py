"""Rebuild the 13.9 kg PlaneA replay and FARO overlay from DaRUS-4614."""
import csv
import sys
import tempfile
from pathlib import Path

import numpy as np

from summarize_4614 import P0_TAR, P_TARGETS, dataset_root, load_registration, read_scope, read_tracker

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'RRF'))
from csv2can import convert_csv_to_highly_compressed

# Centerline height of the 45x45 mm bottom -Y aluminum rail (z in [0, 0.045] m in cad/platform.stl)
# where the FARO SMR nest is bolted, aligned with AttachA..D (z = 0.0225 m).
REFLECTOR_LOCAL_Z = 0.0225


def crossing_times(time, points):
    """Times at which the measured displacement crosses each PlaneA leg's midplane."""
    poses = np.vstack([P0_TAR, P_TARGETS])
    position = points - points[0] + P0_TAR
    start = 0
    crossings = []
    for a, b in zip(poses, poses[1:]):
        direction = b - a
        progress = (position - a) @ direction / (direction @ direction)
        candidates = np.flatnonzero(progress[start+1:] >= .5)
        if not len(candidates):
            raise ValueError('Incomplete PlaneA cycle')
        i = start + 1 + candidates[0]
        crossings.append(np.interp(.5, progress[i-1:i+1], time[i-1:i+1]))
        start = i
    return np.array(crossings)


def write_commands(time, targets, name):
    # Equivalent direct-drive, 49 mm drum, 200 steps/rev at 16 microsteps.
    # Round accumulated payout before differencing to avoid quantization drift.
    steps = np.rint((targets - targets[0]) * 3200 / (2 * np.pi * .049)).astype(int)
    ticks = np.rint(time * 750000).astype(int)
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / 'planeA.csv'
        with path.open('w', newline='') as stream:
            rows = csv.writer(stream)
            for n, delta in enumerate(np.diff(steps, axis=0)):
                for i, value in enumerate(delta):
                    rows.writerow([8*n+i, 40+i, ticks[n], 0, ticks[n+1]-ticks[n], 0, value, 0, 0])
        convert_csv_to_highly_compressed(path)
        (ROOT / f'public/RRF_CAN_commands/copacabana_planeA_{name}.can').write_bytes(path.with_suffix('.can').read_bytes())


def main():
    base = dataset_root('4614') / 'exp-apc_darus-upload/experiments'
    exp = '20_Std_Load13900'
    scope = read_scope(base / 'robot' / exp / 'PlaneA/Export 1.csv')
    time = scope.t - scope.t[0]
    for channel, name in [('x_tar', 'targets'), ('x', 'encoders')]:
        positions = np.column_stack([scope[f'{channel}[{i}]'] for i in range(8)])
        write_commands(time, positions, name)

    pos, tracker_time = read_tracker(base / 'tracker' / exp / 'PlaneA.csv')
    rotation, translation = load_registration(base / 'tracker')
    tracker = pos @ rotation.T + translation - P0_TAR
    command = np.column_stack([scope[f'p_tar[{i}]'] for i in range(3)])
    motor_pose = np.column_stack([scope[f'p[{i}]'] for i in range(3)])
    tracker_time -= tracker_time[0]
    offsets = crossing_times(tracker_time, tracker) - crossing_times(time, motor_pose)
    aligned = tracker_time - np.median(offsets)
    print('Tracker minus robot clock at the five midplanes (s):', offsets.round(4))
    stop = np.searchsorted(aligned, time[-1]) + 1
    if aligned[0] > 0 or aligned[-1] < time[-1] or np.any(np.diff(aligned[:stop]) <= 0):
        raise ValueError('Tracker does not cover the first robot cycle continuously')
    sample = time[::2]
    cycle = np.column_stack([np.interp(sample, aligned, xyz) for xyz in tracker.T])
    # COP_Ref.csv has its Z origin ~0.50 m below the XML homing origin; shift Z so the home SMR sits
    # at its physical bottom-rail height (REFLECTOR_LOCAL_Z) instead of near the top of the platform.
    z_origin_offset = cycle[0, 2] - REFLECTOR_LOCAL_Z
    cycle[:, 2] -= z_origin_offset
    print(f'Removed tracker-vs-XML Z-origin offset {z_origin_offset:.6f} m -> P0 reflector {cycle[0].round(7)}')
    force = np.column_stack([scope[f'f[{i}]'][::2] for i in range(8)])
    header = 't,x,y,z,' + ','.join(f'f{i}' for i in range(8)) + ',target_x,target_y,target_z'
    np.savetxt(ROOT / 'tests/fixtures/reference_data/darus4614_planeA_cycle.csv',
               np.column_stack([sample, cycle, force, command[::2] - P0_TAR]), delimiter=',', header=header, comments='', fmt='%.7f')
    # Reference path only. Sample at 50 Hz for display, in home-relative mm.
    with (ROOT / 'public/gcode/copacabana_planeA_tracker.gcode').open('w') as f:
        print('; FARO measured trajectory, first PlaneA cycle, STD 13.9 kg\nG21\nG90', file=f)
        x, y, z = cycle[0] * 1000
        print(f'G92 X{x:.3f} Y{y:.3f} Z{z:.3f}', file=f)
        for x, y, z in cycle * 1000:
            print(f'G1 X{x:.3f} Y{y:.3f} Z{z:.3f}', file=f)


if __name__ == '__main__':
    main()
