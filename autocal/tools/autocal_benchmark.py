"""Score one finished headless full-auto autocal run.

Metrics: total simulated time, simulated time until the selected fit first has
every anchor coordinate within CONVERGED_MM of truth, split-fit precision and
applied-anchor accuracy. Truth is the simulator's firmware config and is used
only for scoring. Tolerates outputs of older autocal revisions: a metric that
cannot be derived is None.
"""
import json
import re

import numpy as np

from autocal.dataset_roles import measurement_groups, normalize_dataset_point_roles
from autocal.pose_fit import fit_pose_least_squares, shift_dataset_home_offsets, HOME_OFFSET_PRIOR_SIGMA_MM
from autocal.tools.anchor_errors import anchor_errors

CONVERGED_MM = 2.0


def split_fit_difference_mm(data, seed):
    """Refit disjoint alternating points from a common seed; largest anchor disagreement."""
    normalize_dataset_point_roles(data)
    estimates = []
    options = dict(home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM)
    for parity in (0, 1):
        subset = dict(data, sweeps=[dict(s, data_points=[p for p in s['data_points']
                              if p['step_index'] % 2 == parity]) for s in data['sweeps']])
        first = fit_pose_least_squares(subset, seed, fit_home_offsets=True, **options)
        shifted = shift_dataset_home_offsets(subset, first['home_offsets_mm'])
        fit = fit_pose_least_squares(shifted, first['anchors'], fit_home_offsets=False, **options)
        if not (first['success'] and fit['success']):
            return None
        estimates.append(fit['anchors'])
    return float(np.max(np.abs(estimates[0] - estimates[1])))


def _config_m669(path):
    return next(line.split(';')[0] for line in path.read_text().splitlines()
                if re.match(r'M669\s+A', line))


def _convergence_sim_s(data, rows, truth, *, use_convention_frame=False):
    """Simulated time when the first selected fit reached CONVERGED_MM. num_sweeps counts
    collection units (sweeps or whole sphere meshes); older logs lack it, and each later
    iteration then follows exactly one newly collected unit."""
    fits = [r for r in rows if 'selected_run' in r]
    if not fits:
        return None
    if use_convention_frame:
        from autocal.tools.imperfect_machine import convention_frame
        truth_ref = convention_frame(truth)[0]
    else:
        truth_ref = truth
    last = fits[-1]['iteration']
    for row in fits:
        run = next(r for r in row['runs'] if r['id'] == row['selected_run'])
        anchors = run['metrics'].get('pose_fit_anchors')
        if anchors is None:
            return None
        cand = convention_frame(anchors)[0] if use_convention_frame else np.asarray(anchors)
        if np.max(np.abs(cand - truth_ref)) <= CONVERGED_MM:
            # One collection unit (sweep or sphere mesh) per iteration.
            groups = measurement_groups(data)
            units = row.get('num_sweeps', len(groups) - (last - row['iteration']))
            times = [p.get('clock_time_ms') for group in groups[:units]
                     for index in group for p in data['sweeps'][index]['data_points']]
            # Before 2026-11-15 points carry only per-collection timestamps.
            return None if None in times else max(times) / 1000
    return None


def benchmark_metrics(dataset, repo_root, *, truth_mm=None):
    data = json.loads(dataset.read_text())
    manifest = json.loads(dataset.with_suffix('.headless').joinpath('manifest.json').read_text())
    rows = [json.loads(line) for line in
            dataset.with_suffix('.full_auto_log.jsonl').read_text().splitlines()]
    initial = manifest.get('firmware_initial', {}).get('M669') or _config_m669(
        repo_root / 'RRF/run/vsd/sys' / manifest['firmware_config'].split('/')[-1])
    final = (manifest.get('firmware_final', {}).get('M669')
             or next((c for c in manifest.get('applied_parameters', []) if c.startswith('M669 ')), None)
             or next(r['m669'] for r in rows if r.get('event') == 'summary'))
    error = anchor_errors(initial, final)
    fitted = np.asarray(error['fitted_mm'], dtype=float)
    if truth_mm is not None:
        from autocal.tools.imperfect_machine import convention_frame
        truth_arr = np.asarray(list(truth_mm.values()) if isinstance(truth_mm, dict) else truth_mm, dtype=float)
        truth_frame, _ = convention_frame(truth_arr)
        fitted_frame, _ = convention_frame(fitted)
        accuracy_mm = float(np.max(np.abs(fitted_frame - truth_frame)))
        conv_s = _convergence_sim_s(data, rows, truth_arr, use_convention_frame=True)
    else:
        truth_arr = np.asarray(error['reference_mm'], dtype=float)
        accuracy_mm = error['max_abs_coordinate_mm']
        conv_s = _convergence_sim_s(data, rows, truth_arr)
    try:
        precision = split_fit_difference_mm(data, fitted)
    except (KeyError, ValueError, np.linalg.LinAlgError):
        precision = None
    return dict(total_sim_s=manifest.get('simulated_s'), convergence_sim_s=conv_s,
                precision_mm=precision, accuracy_mm=accuracy_mm,
                sweeps=len(measurement_groups(data)), stop_reason=manifest.get('stop_reason'),
                wall_s=manifest.get('wall_s'))
