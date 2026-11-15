"""Test the recorded sensor-force error against the simulator's 50-pole phase.

Diagnostic only: physical strand-force samples never enter autocal planning or
fitting. Coefficients train on even sweeps and predict separate odd sweeps.
"""
import argparse
import json
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path)
    args = parser.parse_args()
    data = json.loads((args.folder / 'sweeps.json').read_text())
    sampled = np.load(args.folder / 'physics-samples.npz')
    observations = [(i, sweep, p) for i, sweep in enumerate(data['sweeps']) for p in sweep['data_points']]
    sensors = np.array([p.get('source_sensor_anchor', sweep['sensor_anchor']) for _, sweep, p in observations])
    angles = np.array([(p.get('mu') or p['raw_angles_deg'])[axis]
                       for (_, _, p), axis in zip(observations, sensors)])
    error = sampled['effective_force_proxy_n'][np.arange(len(sensors)), sensors] - np.array([p['sensor_force_n'] for _, _, p in observations])
    sweep_index = np.array([i for i, _, _ in observations])
    phase = 50 * angles * np.pi / 180
    design = np.column_stack([np.sin(phase), np.cos(phase), np.ones(len(phase))])
    rows = []
    for axis in np.unique(sensors):
        train = (sensors == axis) & (sweep_index % 2 == 0)
        test = (sensors == axis) & (sweep_index % 2 == 1)
        if train.sum() < 3 or test.sum() < 2:
            continue
        coef = np.linalg.lstsq(design[train], error[train], rcond=None)[0]
        residual = error[test] - design[test] @ coef
        rows.append({'sensor_anchor': int(axis), 'train_points': int(train.sum()), 'test_points': int(test.sum()),
            'force_sine_coefficient_n': float(coef[0]), 'force_cosine_coefficient_n': float(coef[1]),
            'force_offset_n': float(coef[2]), 'amplitude_n': float(np.linalg.norm(coef[:2])),
            'heldout_raw_rms_n': float(np.sqrt(np.mean(error[test]**2))),
            'heldout_phase_residual_rms_n': float(np.sqrt(np.mean(residual**2))),
            'heldout_explained_variance': float(1 - np.var(residual) / np.var(error[test]))})
    output = {'scope': 'Recorded-physics diagnostic with simulator-specified pole count 50; force proxies and fitted phase are not production inputs. Entire odd sweeps are held out. This tests the force discrepancy, not the causal share of anchor error.',
        'model_source': 'src/wasm/cable_solver/src/engine.rs:1617 and hp-sim-3d/app/torqueModeSystem.js:36',
        'rows': rows}
    (args.folder / 'force-phase.json').write_text(json.dumps(output, indent=2) + '\n')
    print(json.dumps(output, indent=2))


if __name__ == '__main__':
    main()
