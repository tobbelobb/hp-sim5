"""Offline range-fit counterfactual on the same sampled physical poses.

The geometry substitution and near-truth initialization are diagnostics only.
No physical position or reference anchor is passed to production planning.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

ROOT = next(p for p in Path(__file__).resolve().parents if (p / 'autocal/pose_fit.py').exists())
sys.path.insert(0, str(ROOT))
from autocal.pose_fit import fit_pose_least_squares, HOME_OFFSET_PRIOR_SIGMA_MM
from autocal.tools.sanity_battery import anchor_errors


def point_dataset(lengths, machine):
    # These one-point groups preserve all four input lengths. They are valid
    # range observations, not ellipse sweeps or a collection protocol.
    return {'machine_type': machine, 'num_anchors': 4, 'dimensions': 3, 'sweeps': [
        {'fixed_anchors': [0, 3], 'fixed_lengths': row[[0, 3]].tolist(),
         'drive_anchor': 1, 'sensor_anchor': 2,
         'data_points': [{'l_drive': float(row[1]), 'l_sensor': float(row[2])}]}
        for row in lengths]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path)
    args = parser.parse_args()
    args.folder = args.folder.resolve()
    manifest = json.loads((args.folder / 'sweeps.headless/manifest.json').read_text())
    data = json.loads((args.folder / 'sweeps.json').read_text())
    sampled = np.load(args.folder / 'physics-samples.npz')
    truth = np.array(anchor_errors(manifest['firmware_initial']['M669'], manifest['firmware_initial']['M669'])['reference_mm'])
    geometry = np.linalg.norm(truth[None] - sampled['positions_mm'][:, None], axis=2) - np.linalg.norm(truth, axis=1)
    seed = truth + np.array([[0, 30, 10], [-20, 15, 10], [20, -15, 10], [0, 0, 30.]])
    results = {}
    for name, lengths in [('recorded_encoders', sampled['encoder_lengths_mm']), ('point_geometry', geometry)]:
        fit = fit_pose_least_squares(point_dataset(lengths, data['machine_type']), seed,
            fit_home_offsets=True, home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM)
        error = fit['anchors'] - truth
        results[name] = {'absolute_mm': float(np.abs(error).max()),
            'aligned_mm': float(np.abs(error - error.mean(0)).max()),
            'mean_translation_mm': error.mean(0).tolist(), 'rms_mm': float(fit['rms_mm']),
            'home_offsets_mm': fit['home_offsets_mm'].tolist(), 'success': bool(fit['success'])}
    output = {'scope': 'Offline local point-fit diagnostic with a fixed near-truth seed. Geometry uses latest-at point-effector poses. It checks measurement/model bias and local identifiability, not blind or hardware accuracy.',
        'source_revision': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'inputs': [{'path': str(path.relative_to(ROOT)), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
                   for path in [args.folder / 'sweeps.json', args.folder / 'physics-samples.npz']],
        'points': len(geometry), 'results': results}
    (args.folder / 'point-fit-counterfactual.json').write_text(json.dumps(output, indent=2) + '\n')
    print(json.dumps(output, indent=2))


if __name__ == '__main__':
    main()
