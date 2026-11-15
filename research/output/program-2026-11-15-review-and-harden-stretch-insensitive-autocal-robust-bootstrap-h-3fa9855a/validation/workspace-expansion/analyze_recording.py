"""Read sampled physics to assess achieved poses and forces, never to plan them.

Existing RRD components are inspected with RrdReader; no new recording or
resampling is made. Point timestamps use latest-at physics observations.
"""
import argparse
import json
from pathlib import Path
import re
import sys

import numpy as np
from rerun.chunk import RrdReader

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "autocal/pose_fit.py").exists())
sys.path.insert(0, str(ROOT))
from autocal.pose_fit import dataset_line_lengths
from autocal.tools.sanity_battery import anchor_errors


def load_series(path):
    buffers = {}
    chunks = RrdReader(path).stream().filter(content=['/world/**', '/cable_forces/**']).to_chunks()
    for chunk in chunks:
        entity = str(chunk.entity_path)
        body = re.search(r'/bodies/Effector_[^/]+$', entity)
        force = re.search(r'/CablePath(AL|AR|BL|BR|CL|CR|D0|D1|D2)_', entity)
        if not body and not force:
            continue
        batch = chunk.to_record_batch()
        if 'sim_time' not in batch.schema.names:
            continue
        column = 'Transform3D:translation' if body else 'Scalars:scalars'
        if column not in batch.schema.names:
            continue
        times = np.asarray(batch.column('sim_time').cast('int64').to_pylist()) / 1e9
        rows = batch.column(column).to_pylist()
        valid = np.array([row is not None for row in rows])
        if not valid.any():
            continue
        values = np.array([np.ravel(row) for row in rows if row is not None], dtype=float)
        buffers.setdefault('effector' if body else force[1], []).append((times[valid], values))
    series = {}
    for key, pieces in buffers.items():
        times, values = np.concatenate([p[0] for p in pieces]), np.concatenate([p[1] for p in pieces])
        order = np.argsort(times, kind='stable')
        series[key] = times[order], values[order]
    return series


def latest(series, times):
    t, values = series
    index = np.searchsorted(t, times, side='right') - 1
    if np.any(index < 0):
        raise ValueError('A measurement precedes its recorded physics series')
    return values[index], times - t[index]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path)
    args = parser.parse_args()
    data = json.loads((args.folder / 'sweeps.json').read_text())
    manifest = json.loads((args.folder / 'sweeps.headless/manifest.json').read_text())
    points = [p for sweep in data['sweeps'] for p in sweep['data_points']]
    if any(p.get('clock_source') != 'simulation' for p in points):
        raise ValueError('This analysis requires absolute simulation point timestamps')
    files = list((args.folder / 'sweeps.headless').glob('*.rrd'))
    if len(files) != 1:
        raise ValueError('Expected exactly one finalized physics recording')
    series = load_series(files[0])
    times = np.array([(p['noise_clock_start_ms'] + p['noise_clock_end_ms']) / 2
                      if 'noise_clock_start_ms' in p else p['clock_time_ms'] for p in points]) / 1000
    positions, lag = latest(series['effector'], times)
    # First captured physical pose defines this diagnostic coordinate origin.
    origin = series['effector'][1][0]
    positions = (positions - origin) * 1000
    forces = []
    force_lags = []
    for group in [('AL', 'AR'), ('BL', 'BR'), ('CL', 'CR'), ('D0', 'D1', 'D2')]:
        samples = [latest(series[key], times) for key in group]
        forces.append(sum(sample[0][:, 0] for sample in samples))
        force_lags.extend(sample[1] for sample in samples)
    forces = np.array(forces).T
    anchors = np.array(anchor_errors(manifest['firmware_initial']['M669'], manifest['firmware_initial']['M669'])['reference_mm'])
    to_anchor = anchors[None] - positions[:, None]
    lengths = np.linalg.norm(to_anchor, axis=2)
    directions = to_anchor / lengths[:, :, None]
    null = np.linalg.svd(np.transpose(directions, (0, 2, 1)))[2][:, -1]
    null *= np.sign(np.sum(null, axis=1))[:, None]
    line = data['config']['m666_before_data_collection']
    advantage, strands = np.array(line['U']), np.array(line['N'])
    # Path scalars are strand forces; resultant anchor force includes reeving.
    # This is a proxy for a point effector, not an exact frictional-pulley sum.
    forces *= advantage
    weights = null * (lengths + np.array(line['Y']) / advantage)
    actual_strand_target = np.sum(weights * forces / (strands * advantage), axis=1) / weights.sum(axis=1)
    target = (data['sweeps'][0].get('metadata') or {}).get('strand_tension_n')
    if target is None and np.all(advantage == 1):
        target = 5 * data['config']['force_tuning']['force_mid_n'] / min(strands)
    geometry = lengths - np.linalg.norm(anchors, axis=1)
    encoder = dataset_line_lengths(data)['lengths_mm']
    error = encoder - geometry
    sensor = np.array([p.get('source_sensor_anchor', sweep['sensor_anchor'])
                       for sweep in data['sweeps'] for p in sweep['data_points']])
    sensor_error = forces[np.arange(len(points)), sensor] - np.array([p['sensor_force_n'] for p in points])
    motion = np.zeros(len(points))
    offset = 0
    for sweep in data['sweeps']:
        indices = np.arange(offset, offset + len(sweep['data_points']))
        for axis in np.unique(sensor[indices]):
            group = indices[sensor[indices] == axis]
            if len(group) > 1:
                changes = np.diff(encoder[group, axis])
                motion[group] = np.sign(np.r_[changes[0], changes])
        offset += len(indices)
    motion_errors = {name: {'points': int(np.sum(motion == sign)),
        'sensor_actual_minus_command_percentiles_n': np.percentile(sensor_error[motion == sign], [0, 5, 50, 95, 100]).tolist()}
        for sign, name in [(-1, 'rewinding'), (1, 'paying_out'), (0, 'unchanged')]
        if np.any(motion == sign)}
    load = np.array([0., 0., line['W'] * 9.81])
    matrix = np.transpose(directions, (0, 2, 1))
    multiplier = np.linalg.solve(matrix @ directions, np.broadcast_to(load, (len(points), 3))[:, :, None])[:, :, 0]
    carrying = np.einsum('pnd,pd->pn', directions, multiplier)
    amount = (np.array([p['sensor_force_n'] for p in points]) - carrying[np.arange(len(points)), sensor]) / null[np.arange(len(points)), sensor]
    commanded_forces = carrying + amount[:, None] * null
    commanded_strand_target = np.sum(weights * commanded_forces / (strands * advantage), axis=1) / weights.sum(axis=1)
    balance = np.einsum('pdn,pn->pd', matrix, forces) - load
    np.savez_compressed(args.folder / 'physics-samples.npz', positions_mm=positions, times_s=times,
                        effective_force_proxy_n=forces, encoder_lengths_mm=encoder,
                        commanded_weighted_strand_target_n=commanded_strand_target)
    result = {
        'scope': 'Sampled physics diagnostic; reference geometry is never a calibration/planning input. Geometric lengths use a point-effector model, so discrepancies include stretch, reference and routing/model errors.',
        'points': len(points), 'recording': str(files[0]), 'reference_origin_world_m': origin.tolist(),
        'height_range_mm': [float(positions[:, 2].min()), float(positions[:, 2].max())],
        'height_percentiles_mm': np.percentile(positions[:, 2], [0, 25, 50, 75, 100]).tolist(),
        'max_pose_latest_at_lag_s': float(lag.max()),
        'max_force_latest_at_lag_s': float(np.max(force_lags)),
        'declared_strand_target_n': target,
        'command_implied_weighted_strand_target_percentiles_n': np.percentile(commanded_strand_target, [0, 5, 50, 95, 100]).tolist(),
        'realized_weighted_strand_target_percentiles_n': np.percentile(actual_strand_target, [0, 5, 50, 95, 100]).tolist(),
        'force_balance_proxy_rms_n': float(np.sqrt(np.mean(balance**2))),
        'sensor_actual_minus_command_percentiles_n': np.percentile(sensor_error, [0, 5, 50, 95, 100]).tolist(),
        'force_error_by_observed_sensor_motion': motion_errors,
        'encoder_minus_point_geometry_max_abs_by_axis_mm': np.max(np.abs(error), axis=0).tolist(),
        'encoder_minus_point_geometry_rms_by_axis_mm': np.sqrt(np.mean(error**2, axis=0)).tolist(),
    }
    output = args.folder / 'physics-diagnostics.json'
    output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
