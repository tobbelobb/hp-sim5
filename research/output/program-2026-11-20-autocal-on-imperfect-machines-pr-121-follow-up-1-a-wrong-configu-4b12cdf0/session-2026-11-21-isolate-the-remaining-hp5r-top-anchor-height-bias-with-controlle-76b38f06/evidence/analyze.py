"""HP5r replay attribution using the production robust point fitter at fixed point masks."""
import argparse
from dataclasses import replace
import gzip
import json
import re
from pathlib import Path
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

from autocal import pose_fit as pf
from autocal.dataset_roles import normalize_dataset_point_roles
from autocal.tools.imperfect_machine import convention_frame

AXES = 'ABCDI'


def read_json(path):
    if path.exists():
        return json.loads(path.read_text())
    with gzip.open(path.with_suffix(path.suffix + '.gz'), 'rt') as source:
        return json.load(source)


def cable_values(record, advantage):
    """Lengths in effector-distance units. Wrap excludes the motor spool."""
    values = []
    for letter, ratio in zip(AXES, advantage):
        n_paths = 4 if letter == 'I' else 2
        matching = re.compile(rf'^Joint({letter})([LR0-3])_(.+)$')
        joints = [(m[2], m[3], np.array(v)) for name, v in record['joints'].items()
                  if (m := matching.match(name))]
        wraps = sum(value for name, value in record['wraps'].items()
                    if re.fullmatch(rf'CablePath{letter}[LR0-3]', name))
        assert len(joints) == n_paths * (5 if letter == 'I' else 6)
        long = [v for _, suffix, v in joints if suffix != '0' and
                (letter == 'I' or suffix in ('anchor_to_pinhole', 'pinhole_to_anchor'))]
        assert len(long) == ratio * n_paths
        sums = np.sum([v for _, _, v in joints], axis=0)
        stretch = sums[2] - sums[1]
        elastic = sum(max(v[2] - v[1], 0) for _, _, v in joints)
        f, fz = np.sum(long, axis=0)[[0, 4]]
        geometric, frozen = (sums[2:4] + wraps) * 1000 / (ratio * n_paths)
        values.append([geometric, frozen, stretch * 1000 / (ratio * n_paths),
                       elastic * 1000 / (ratio * n_paths), f, fz])
    return np.array(values)


def samples(dataset, replay, dt):
    reads = [json.loads(line) for line in replay.open()]
    steps = np.array([r['step'] for r in reads])
    windows = []
    for s in dataset['sweeps']:
        for p in s['data_points']:
            start = round(p['noise_clock_start_ms'] / (dt * 1000))
            end = round(p['noise_clock_end_ms'] / (dt * 1000))
            rows = [reads[i] for i in np.flatnonzero((steps >= start) & (steps <= end))]
            # Start can also be the previously collected raw read. The last sample_count
            # reads align to the end of the collector's measurement window.
            count = p['sample_count']
            windows.append((p, rows[-count:]))
    return reads[0], windows


def describe(array, measured):
    return dict(rms_by_line=np.sqrt(np.nanmean(np.where(measured, array**2, np.nan), axis=0)).tolist(),
                max_abs=float(np.abs(array[measured]).max()))


def run(run_dir, replay, reduced_path, locked):
    data = read_json(run_dir / 'sweeps.json')
    normalize_dataset_point_roles(data)
    config = data['config']
    imperfection = read_json(run_dir / 'imperfection.json')
    live = read_json(run_dir / 'result.json')
    truth = np.array([imperfection['truth_mm'][letter] for letter in AXES])
    truth_frame = convention_frame(truth)[0]
    seed = np.array(live['fitted_mm'])
    m666 = config['m666_before_data_collection']
    m666['S'] *= imperfection['stiffness_scale']
    advantage = m666['U']
    if replay is not None:
        dt = read_json(run_dir / 'sweeps.headless/manifest.json')['dt_s']
        home, windows = samples(data, replay, dt)
        angle_error = np.array([np.max(np.abs(np.mean([r['angles_deg'] for r in rows], axis=0) - p['mu']))
                                if rows else np.inf for p, rows in windows])
        keep = np.array([len(rows) == p['sample_count'] for p, rows in windows])
        if not locked:
            keep &= angle_error < 0.0051
        if keep.sum() < 0.8 * len(keep):
            raise ValueError(f'{keep.sum()} of {len(keep)} matched; errors {angle_error.tolist()}')
        accepted = [rows for (_, rows), matched in zip(windows, keep) if matched]
        state = dict(keep=keep.tolist(), matched_max_angle_error_deg=float(angle_error[keep].max()),
                     home_pos=home['pos'], home_q=home['q'], home_cables=cable_values(home, advantage).tolist(),
                     pos=[np.mean([r['pos'] for r in rows], axis=0).tolist() for rows in accepted],
                     q=[rows[-1]['q'] for rows in accepted],
                     cables=[np.mean([cable_values(r, advantage) for r in rows], axis=0).tolist()
                             for rows in accepted])
        if locked:
            state['locked_angles_deg'] = [np.mean([r['angles_deg'] for r in rows], axis=0).tolist()
                                          for rows in accepted]
        if reduced_path is not None:
            with gzip.open(reduced_path, 'wt') as output:
                json.dump(state, output)
    else:
        with gzip.open(reduced_path, 'rt') as source:
            state = json.load(source)
    keep = np.array(state['keep'])
    i = 0
    for sweep in data['sweeps']:
        count = len(sweep['data_points'])
        sweep['data_points'] = [p for p, matched in zip(sweep['data_points'], keep[i:i + count]) if matched]
        i += count
    assert i == len(keep)
    lengths = pf.dataset_line_lengths(data)['lengths_mm']
    measured = np.isfinite(lengths)
    if 'locked_angles_deg' in state:
        lengths = np.where(measured, np.asarray(state['locked_angles_deg']) * config['mm_per_degree'], np.nan)
    pos = (np.array(state['pos']) - state['home_pos']) * 1000
    rot = Rotation.from_quat(state['q']) * Rotation.from_quat(state['home_q']).inv()
    geometry, frozen, stretch, elastic, force, fz = np.moveaxis(state['cables'], -1, 0)
    home_g, home_frozen, home_stretch, home_elastic, _, _ = np.array(state['home_cables']).T
    point = np.linalg.norm(truth - pos[:, None, :], axis=-1) - np.linalg.norm(truth, axis=-1)
    dg, dg_frozen = geometry - home_g, frozen - home_frozen
    ds, de = stretch - home_stretch, elastic - home_elastic
    roll = dg - dg_frozen
    model = pf.dataset_stretch(data)
    absolute = np.linalg.norm(truth, axis=1) + np.where(measured, lengths, 0.)
    flex_eq = pf.stretch_corrected_lengths(truth, pos, absolute, measured, model) - absolute
    def flex_with_measured_force(flex_model):
        return np.column_stack([flex_model.corrected_distance_mm(absolute[:, i], force[:, i], axis=i)
                                for i in range(len(AXES))]) - absolute

    flex_force = flex_with_measured_force(model['model'])
    guy = np.array(model['model'].guy_wire_lengths)
    guy[:4] += 85.512  # Straight effector-side reeving omitted by Y (prior probe study).
    flex_reeving = flex_with_measured_force(replace(model['model'], guy_wire_lengths=guy.tolist()))
    to_print_frame = np.linalg.lstsq(truth, truth_frame, rcond=None)[0]
    flex_convention_gravity = pf.stretch_corrected_lengths(
        truth_frame, pos @ to_print_frame, absolute, measured, model) - absolute

    def score(result):
        error = convention_frame(result['anchors'])[0] - truth_frame
        return dict(error_mm=error.tolist(), max_mm=float(np.abs(error).max()), top_z_mm=float(error[-1, 2]),
                    rms_mm=result['rms_mm'], n_rejected=len(result['rejected_points']),
                    offsets_mm=result['home_offsets_mm'].tolist(), offsets_skipped=result['home_offsets_skipped'])

    def fit(ranges, correction):
        measured_ranges = {'lengths_mm': np.where(measured, ranges, np.nan)}
        with (patch.object(pf, 'dataset_line_lengths', return_value=measured_ranges),
              patch.object(pf, 'dataset_stretch', return_value=correction)):
            return pf.fit_pose_least_squares(data, seed, fit_home_offsets=False,
                                             home_offset_prior_sigma_mm=pf.HOME_OFFSET_PRIOR_SIGMA_MM)

    if 'locked_angles_deg' not in state:
        production = pf.fit_pose_least_squares(data, seed, fit_home_offsets=True,
                                               home_offset_prior_sigma_mm=pf.HOME_OFFSET_PRIOR_SIGMA_MM)
    else:
        production = None
    variants = {
        'equilibrium_flex_no_offsets': (lengths, model),
        'realised_force_flex': (lengths + flex_force, None),
        'realised_force_flex_extra_reeving': (lengths + flex_reeving, None),
        'realised_force_flex_home_extension': (lengths + flex_force - home_elastic, None),
        'force_balance_physical_gravity_at_truth': (lengths + flex_eq, None),
        'force_balance_print_frame_gravity_at_truth': (lengths + flex_convention_gravity, None),
        'realised_force_flex_remove_all_geometry': (lengths + flex_force - (dg - point), None),
        'realised_force_flex_remove_roll': (lengths + flex_force - roll, None),
        'realised_force_flex_remove_frozen_routing': (lengths + flex_force - (dg_frozen - point), None),
        'replay_elastic_and_slack': (lengths + ds, None),
        'replay_elastic_only': (lengths + de, None),
        'replay_elastic_and_slack_remove_roll': (lengths + ds - roll, None),
        'replay_elastic_and_slack_remove_frozen_routing': (lengths + ds - (dg_frozen - point), None),
        'replay_elastic_and_slack_remove_all_geometry': (lengths + ds - (dg - point), None),
        'forward_routing_measured_attitude': (dg, None),
        'forward_routing_home_attitude': (dg_frozen, None),
        'forward_point_control': (point, None),
    }
    low_rotation = roll.copy()
    low_rotation[:, -1] = 0
    variants['forward_rotate_low_routes_only'] = (dg_frozen + low_rotation, None)
    variants['forward_rotate_top_route_only'] = (dg - low_rotation, None)
    variants['forward_reverse_rotation_chords'] = (dg_frozen - roll, None)
    components = {
        'encoder_closure': lengths - (dg - ds),
        'rotation_chords': roll,
        'frozen_fixture_geometry': dg_frozen - point,
        'full_geometry': dg - point,
        'equilibrium_flex_minus_replay': flex_eq - ds,
        'force_flex_minus_replay': flex_force - ds,
        'equilibrium_minus_realised_flex': flex_eq - flex_force,
        'print_gravity_minus_physical': flex_convention_gravity - flex_eq,
        'slack_delta': ds - de,
    }
    return dict(
        points=len(keep), matched=int(keep.sum()), matched_max_angle_error_deg=state['matched_max_angle_error_deg'],
        rotation_median_deg=float(np.degrees(np.median(rot.magnitude()))),
        rotation_max_deg=float(np.degrees(rot.magnitude().max())), home_pos=state['home_pos'],
        home_stretch_mm=home_stretch.tolist(), home_elastic_mm=home_elastic.tolist(),
        range_count_histogram={int(n): int(np.sum(measured.sum(axis=1) == n)) for n in (4, 5)},
        vertical_force_n=[float(fz.sum(axis=1).mean()), float(fz.sum(axis=1).std())],
        components={name: describe(value, measured) for name, value in components.items()},
        production_true_ea=score(production) if production is not None else None,
        fits={name: score(fit(ranges, correction)) for name, (ranges, correction) in variants.items()},
    )


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('run', type=Path)
    p.add_argument('--replay', type=Path)
    p.add_argument('--reduced', type=Path, help='Save window means with --replay, otherwise read these means')
    p.add_argument('--make-windows', action='store_true', help='Print replay sampling windows')
    p.add_argument('--locked', action='store_true', help='Replay dynamics changed, match windows by clock only')
    args = p.parse_args()
    if args.make_windows:
        data = read_json(args.run / 'sweeps.json')
        dt = read_json(args.run / 'sweeps.headless/manifest.json')['dt_s']
        print(json.dumps([[round(p['noise_clock_start_ms'] / (dt * 1000)) - 1,
                           round(p['noise_clock_end_ms'] / (dt * 1000)) + 1]
                          for s in data['sweeps'] for p in s['data_points']]))
        raise SystemExit
    if args.replay is None and args.reduced is None:
        p.error('provide --replay or --reduced')
    print(json.dumps(run(args.run, args.replay, args.reduced, args.locked), indent=2))
