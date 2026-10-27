"""Cheap analytical HP3 encoder screens; these are not dynamics simulations.

The fitter receives encoder-like length deltas and its ordinary data-derived seed.
World truth lives in a separate file and only scores returned anchors. Compliance
and reversal offsets are deliberately simple model errors, not calibrated friction.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import re
from pathlib import Path
import subprocess
import time

import numpy as np

from autocal.calibrate import build_anchor_initial_guess
from autocal.ellipse_cost import EllipseCostFunction
from autocal.ellipse_solver import solve_anchors
from autocal.theoretical_ellipse import (
    anchors_matrix_to_opt_vec, canonicalize_anchor_gauge, compute_constraint_circle_3d,
)

BASE_ANCHORS = np.asarray([[0., -1900., -280.], [1645.45, 950., -280.],
                           [-1645.45, 950., -280.], [0., 0., 1900.]])
LIMITATION = ('Analytical circular constraints with prescribed tension errors. No force '
              'equilibrium, motor dynamics, settling, rotation, line sag or hardware '
              'validation. Point thinning is offline and cannot measure collection savings.')


def default_worlds():
    """Predeclared development/holdout conditions, not a fitted distribution."""
    return [
        dict(id='D0-clean', split='development', seed=100, scale=1., noise_mm=.02),
        dict(id='D1-compliant', split='development', seed=101, scale=1., noise_mm=.02,
             axial_stiffness_n=20000., tension_span_n=5.),
        dict(id='D2-reversal', split='development', seed=102, scale=1., noise_mm=.1,
             reversal_offset_mm=.6),
        dict(id='D3-large', split='development', seed=103, scale=1.2, noise_mm=.05,
             top_shift_mm=[100., -60.]),
        dict(id='H0-small', split='holdout', seed=200, scale=.8, noise_mm=.05),
        dict(id='H1-soft', split='holdout', seed=201, scale=1., noise_mm=.1,
             axial_stiffness_n=8000., tension_span_n=8., reversal_offset_mm=.3),
        dict(id='H2-asymmetric', split='holdout', seed=202, scale=1., noise_mm=.15,
             lower_xy_factors=[1., .85, 1.15], top_shift_mm=[-180., 120.]),
        dict(id='H3-short-noisy', split='holdout', seed=203, scale=1., noise_mm=.4,
             arc_half_width_rad=.09, reversal_offset_mm=.8),
    ]


def json_bytes(value):
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + '\n').encode()


def digest(value):
    return hashlib.sha256(json_bytes(value)).hexdigest()


def write_json(path, value):
    path.write_bytes(json_bytes(value))


def generate_world(spec, *, points_per_direction=10):
    """Return observations and separate evaluation truth, with a fixed 30 mm radius.

    A fixed motor holds material length. Geometric distance uses the linear
    material-length law l = d * (1 - F/S). Drive/sensor tensions vary along a
    prescribed circle, while held tensions stay at their reference values.
    Direction offsets emulate hysteresis. They do not reproduce native friction.
    """
    if points_per_direction < 5:
        raise ValueError('At least five points per direction are required')
    anchors = BASE_ANCHORS * float(spec.get('scale', 1.))
    anchors[:3, :2] *= np.asarray(spec.get('lower_xy_factors', [1., 1., 1.]))[:, None]
    anchors[3, :2] += np.asarray(spec.get('top_shift_mm', [0., 0.]))
    anchors = canonicalize_anchor_gauge('hangprinter_4', anchors)
    stiffness = float(spec.get('axial_stiffness_n', 1e30))
    span = float(spec.get('tension_span_n', 0.))
    if not np.isfinite(stiffness) or stiffness <= 12. + span:
        raise ValueError('Axial stiffness must exceed all prescribed tensions')
    rng = np.random.default_rng(int(spec['seed']))
    reference_tension = np.asarray([4., 4., 4., 12.])
    origin_lengths = np.linalg.norm(anchors, axis=1)
    origin_material = origin_lengths * (1. - reference_tension / stiffness)
    sweeps = []
    for delta in (0., 120., -120., 240.):
        for fixed in range(3):
            held = [fixed, 3]
            material = origin_material[held] + [delta, 0.]
            lengths = material / (1. - reference_tension[held] / stiffness)
            circle = compute_constraint_circle_3d(anchors, held, lengths.tolist())
            phi0 = np.arctan2(-circle.center @ circle.v, -circle.center @ circle.u)
            half = float(spec.get('arc_half_width_rad', .18))
            arc = np.linspace(phi0-half, phi0+half, points_per_direction)
            phi = np.concatenate([arc, arc[::-1]])
            direction = np.repeat([1., -1.], points_per_direction)
            positions = circle.center + circle.radius * (
                np.cos(phi)[:, None]*circle.u + np.sin(phi)[:, None]*circle.v)
            distances = np.linalg.norm(positions[:, None, :] - anchors[None, :, :], axis=2)
            drive, sensor = [axis for axis in range(3) if axis != fixed]
            tensions = np.tile(reference_tension, (len(phi), 1))
            tensions[:, drive] += span * (.5 + .5*np.sin(phi - phi0))
            tensions[:, sensor] += span * (.5 + .5*np.cos(phi - phi0))
            observations = distances*(1.-tensions/stiffness) - origin_material
            observations[:, held] = np.asarray([delta, 0.])
            observations[:, drive] += direction*float(spec.get('reversal_offset_mm', 0.))
            observations[:, sensor] -= direction*float(spec.get('reversal_offset_mm', 0.))
            noise = rng.normal(0., float(spec.get('noise_mm', .02)), observations.shape)
            noise[:, held] = 0.
            observations += noise
            points = [dict(l_drive=float(row[drive]), l_sensor=float(row[sensor]),
                           raw_angles_deg=(row / (30.*np.pi/180.)).tolist())
                      for row in observations]
            sweeps.append(dict(id=f'sweep_{len(sweeps)+1:03d}', fixed_anchors=held,
                               fixed_lengths=[delta, 0.], drive_anchor=drive,
                               sensor_anchor=sensor, data_points=points))
    dataset = dict(version='2', machine_type='hangprinter_4', num_anchors=4,
                   dimensions=3, sweeps=sweeps,
                   config=dict(buildup_factor=0., base_radii=[30.]*4))
    truth = dict(spec=spec, anchors_mm=anchors.tolist(), limitation=LIMITATION)
    return dataset, truth


def thin_dataset(dataset):
    """Keep identical endpoint-preserving observations from each direction."""
    result = copy.deepcopy(dataset)
    for sweep in result['sweeps']:
        points = sweep['data_points']
        half = len(points)//2
        indices = np.linspace(0, half-1, max(5, half//2)).round().astype(int)
        sweep['data_points'] = [points[int(i)] for i in np.r_[indices, indices+half]]
    return result


def prediction(dataset, anchors):
    cost = EllipseCostFunction(dataset, pointwise_filtering=False,
                              sweep_wise_filtering=False, noise_normalized=False)
    rows = cost.pointwise_residual_rows(anchors_matrix_to_opt_vec(anchors, 'hangprinter_4'))
    expected = sum(len(s['data_points']) for s in dataset['sweeps'])
    values = [float(row['residual_mm_signed']) for row in rows]
    return dict(expected_points=expected, valid_points=len(values),
                rms_mm=float(np.sqrt(np.mean(np.square(values)))) if values else None)


def row_is_complete(row, expected_train_sweeps=9):
    """Optimizer status, finite outputs and full coverage are separate requirements."""
    unseen = row.get('unseen_prediction', {})
    numbers = [row.get('cost'), row.get('anchor_rmse_mm'), unseen.get('rms_mm')]
    return bool(
        not row.get('error') and row.get('optimizer_success')
        and all(isinstance(value, (int, float)) and np.isfinite(value) for value in numbers)
        and np.asarray(row.get('anchors_mm', [])).shape == (4, 3)
        and np.all(np.isfinite(row['anchors_mm']))
        and row.get('invalid_sweeps') == 0
        and row.get('valid_sweeps') == expected_train_sweeps
        and unseen.get('expected_points', 0) > 0
        and unseen.get('valid_points') == unseen.get('expected_points'))


def summarize(rows, worlds):
    """Keep failed results in the comparison, and never rank by ground truth online."""
    summary = dict(groups=[], comparisons=[], complete_rows=sum(row_is_complete(r) for r in rows),
                   total_rows=len(rows), limitation=LIMITATION)
    for split in ('development', 'holdout'):
        for mode in ('full', 'half'):
            for method in ('lbfgsb', 'trf'):
                selected = [r for r in rows if (r['split'], r['point_mode'], r['method']) == (split, mode, method)]
                errors = [r['anchor_rmse_mm'] for r in selected if isinstance(r.get('anchor_rmse_mm'), (int, float))]
                summary['groups'].append(dict(split=split, point_mode=mode, method=method,
                    rows=len(selected), complete_rows=sum(row_is_complete(r) for r in selected),
                    mean_rmse_mm=float(np.mean(errors)) if errors else None,
                    max_rmse_mm=float(max(errors)) if errors else None,
                    errors=[r.get('error') for r in selected if r.get('error')]))
    # Compare every frozen candidate on the same complete unseen observations.
    disagreements = []
    for world in worlds:
        candidates = [r for r in rows if r['world']==world['id'] and 'anchor_rmse_mm' in r]
        for index, first in enumerate(candidates):
            for second in candidates[index+1:]:
                a, b = first.get('unseen_prediction', {}), second.get('unseen_prediction', {})
                if not (a.get('valid_points') == a.get('expected_points', 0) > 0
                        and b.get('valid_points') == b.get('expected_points', 0) > 0):
                    continue
                if (a['rms_mm']-b['rms_mm'])*(first['anchor_rmse_mm']-second['anchor_rmse_mm']) < 0:
                    disagreements.append(dict(world=world['id'],
                        first=[first['method'],first['point_mode']], second=[second['method'],second['point_mode']]))
    summary['unseen_prediction_geometry_disagreements'] = disagreements
    lookup = {(r['world'], r['point_mode'], r['method']):r for r in rows}
    holdout = [w['id'] for w in worlds if w['split']=='holdout']
    for name, baseline_key, candidate_key, ratio, allowance in [
        ('trf_full_vs_lbfgsb_full', ('full','lbfgsb'), ('full','trf'), .8, 1.),
        ('half_vs_full_lbfgsb', ('full','lbfgsb'), ('half','lbfgsb'), 1.1, .5),
        ('half_vs_full_trf', ('full','trf'), ('half','trf'), 1.1, .5),
    ]:
        pairs = [(lookup.get((world,*baseline_key)), lookup.get((world,*candidate_key))) for world in holdout]
        available = [(a,b) for a,b in pairs if a and b and 'anchor_rmse_mm' in a and 'anchor_rmse_mm' in b]
        base = [a['anchor_rmse_mm'] for a,b in available]
        candidate = [b['anchor_rmse_mm'] for a,b in available]
        delta = [b-a for a,b in zip(base,candidate)]
        coverage = bool(holdout) and len(available)==len(holdout) and all(row_is_complete(a) and row_is_complete(b) for a,b in available)
        ratio_pass = bool(base) and np.mean(candidate) <= ratio*np.mean(base)
        maximum_pass = bool(delta) and max(delta) <= allowance
        summary['comparisons'].append(dict(name=name, paired_worlds=len(available),
            expected_worlds=len(holdout), full_coverage=coverage,
            mean_rmse_ratio=float(np.mean(candidate)/np.mean(base)) if base and np.mean(base)>0 else None,
            maximum_regression_mm=float(max(delta)) if delta else None,
            mean_threshold_pass=bool(ratio_pass), individual_threshold_pass=bool(maximum_pass),
            accepted=bool(coverage and ratio_pass and maximum_pass)))
    return summary


def run(output, *, worlds=None, max_fits=32, iterations=400, restarts=4, max_wall_seconds=600.):
    worlds = default_worlds() if worlds is None else worlds
    if len(worlds)*4 > max_fits:
        raise ValueError(f'{len(worlds)*4} fits exceed explicit max_fits={max_fits}')
    if iterations < 1 or restarts < 1 or not np.isfinite(max_wall_seconds) or max_wall_seconds <= 0:
        raise ValueError('Iterations, restarts and wall budget must be positive and finite')
    if len({w['id'] for w in worlds}) != len(worlds) or any(
        not re.fullmatch(r'[A-Za-z0-9_-]+',w['id']) or w['split'] not in ('development','holdout') for w in worlds):
        raise ValueError('World IDs must be unique safe names and splits development or holdout')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    contract = dict(format_version=2, worlds=worlds, max_fits=max_fits,
                    iterations=iterations, restarts=restarts, limitation=LIMITATION,
                    max_wall_seconds=max_wall_seconds,
                    wall_budget='Checked between fits; an in-progress fit can overrun this soft wall budget',
                    timing='generation_seconds measures geometry/observations only; fit_seconds includes solve, prediction and truth scoring; elapsed_seconds also includes input/result IO',
                    methods=['lbfgsb', 'trf'], point_modes=['full', 'half'],
                    train_sweep_count=9, unseen_sweep_count=3,
                    acceptance=dict(solver='holdout mean RMSE at least 20% lower, no world >1 mm worse, no missing/invalid rows',
                                    thinning='holdout mean RMSE at most 10% higher, no world >0.5 mm worse, no missing/invalid rows',
                                    complete_row='successful optimizer, finite output, all nine training sweeps and all unseen points valid'))
    write_json(output/'contract.json', contract)
    source = Path(__file__)
    write_json(output/'provenance.json', dict(script_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
               source_revision=subprocess.check_output(['git','rev-parse','HEAD'], text=True).strip(),
               source_dirty=bool(subprocess.check_output(['git','status','--porcelain'], text=True).strip())))
    rows = []
    started = time.perf_counter()
    generation_seconds = 0.
    def save(complete, stop_reason):
        result = dict(contract_sha256=digest(contract), rows=rows,
                      elapsed_seconds=time.perf_counter()-started,
                      generation_seconds=generation_seconds, complete=complete,
                      stop_reason=stop_reason, summary=summarize(rows,worlds))
        write_json(output/'results.json', result)
        return result
    for spec in worlds:
        before = time.perf_counter()
        full, truth = generate_world(spec)
        generation_seconds += time.perf_counter()-before
        folder = output/spec['id']
        folder.mkdir()
        write_json(folder/'observations.json', full)
        write_json(folder/'evaluation-truth.json', truth)
        for mode, data in [('full',full), ('half',thin_dataset(full))]:
            train = dict(data, sweeps=data['sweeps'][:9])
            # Every candidate sees the same unthinned, unseen measurements.
            unseen = dict(full, sweeps=full['sweeps'][9:])
            seed = build_anchor_initial_guess(train, base_radii_mm=[30.]*4)
            for method in ('lbfgsb','trf'):
                if time.perf_counter()-started >= max_wall_seconds:
                    return save(False,'wall_budget_before_next_fit')
                row = dict(world=spec['id'], split=spec['split'], method=method,
                           point_mode=mode, input_sha256=digest(train),
                           truth_sha256=digest(truth), seed_anchors_mm=seed.tolist() if seed is not None else None,
                           points=sum(len(s['data_points']) for s in train['sweeps']))
                before = time.perf_counter()
                try:
                    fit = solve_anchors(train, initial_guess=seed, method=method,
                                        max_iterations=iterations, num_restarts=restarts,
                                        use_parallel=False, robust_loss=True)
                    row['solver_seconds'] = time.perf_counter()-before
                    anchors = np.asarray(fit['anchors'])
                    if anchors.shape != (4,3) or not np.all(np.isfinite(anchors)) or not np.isfinite(fit['cost']):
                        raise ValueError('Nonfinite or invalid fitted anchors/cost')
                    detail = fit['details']
                    prediction_result = prediction(unseen,anchors)
                    if prediction_result['rms_mm'] is not None and not np.isfinite(prediction_result['rms_mm']):
                        raise ValueError('Nonfinite unseen residual')
                    row.update(cost=float(fit['cost']), optimizer_success=bool(fit['success']),
                               anchors_mm=anchors.tolist(),
                               anchor_rmse_mm=float(np.sqrt(np.mean(np.sum((anchors-np.asarray(truth['anchors_mm']))**2,axis=1)))),
                               invalid_sweeps=int(detail.num_invalid_sweeps),
                               valid_sweeps=int(detail.num_valid_sweeps),
                               unseen_prediction=prediction_result)
                except Exception as error:
                    row['error'] = f'{type(error).__name__}: {error}'
                row['fit_seconds'] = time.perf_counter()-before
                row['complete_coverage'] = row_is_complete(row)
                rows.append(row)
                save(False,'in_progress')
                print(json.dumps(row, allow_nan=False), flush=True)
    return save(True,'all_fits_completed')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-fits',type=int,default=32)
    parser.add_argument('--iterations',type=int,default=400)
    parser.add_argument('--restarts',type=int,default=4)
    parser.add_argument('--max-wall-seconds',type=float,default=600.,help='Soft budget checked between fits')
    args = parser.parse_args(argv)
    result = run(args.output, max_fits=args.max_fits, iterations=args.iterations,
                 restarts=args.restarts,max_wall_seconds=args.max_wall_seconds)
    return 0 if result['complete'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
