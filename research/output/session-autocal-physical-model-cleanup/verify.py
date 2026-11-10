"""Fit-free objective equivalence and saved-sentinel probe for the flex cleanup."""
from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from autocal.ellipse_cost import EllipseCostFunction
from autocal.stage_artifacts import read_stage_artifact
from autocal.theoretical_ellipse import anchors_matrix_to_opt_vec
from autocal.tools.regress_calibration_logs import DATASETS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--baseline-revision', required=True)
    parser.add_argument('--lane-stages', type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    args.baseline_revision = subprocess.check_output(
        ['git', 'rev-parse', args.baseline_revision], cwd=ROOT, text=True).strip()
    old_source = subprocess.check_output(
        ['git', 'show', f'{args.baseline_revision}:autocal/ellipse_cost.py'], cwd=ROOT)
    old_path = args.output / 'ellipse_cost_before.py'
    old_path.write_bytes(old_source)
    spec = importlib.util.spec_from_file_location('ellipse_cost_before', old_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    before_class = module.EllipseCostFunction
    rows = []
    for fixture in DATASETS:
        path = ROOT / 'autocal/data/references' / f'{fixture.name}.json'
        dataset = json.loads(path.read_text())
        anchors = np.asarray(fixture.true_anchors)[:, :int(dataset['dimensions'])]
        tensions = sum(
            'assumed_tension_drive_n' in p or 'assumed_tension_sensor_n' in p
            for sweep in dataset['sweeps'] for p in sweep['data_points'])
        for point_filter in (False, True):
            settings = dict(pointwise_filtering=point_filter, pointwise_filter_stage=2)
            before = before_class(copy.deepcopy(dataset), use_flex=False, **settings)
            inert = before_class(copy.deepcopy(dataset), use_flex=True, **settings)
            after = EllipseCostFunction(copy.deepcopy(dataset), **settings)
            for scale in (0.99, 1.0, 1.01):
                vector = anchors_matrix_to_opt_vec(anchors * scale, fixture.machine_type)
                old = before.evaluate_detailed(vector)
                new = after.evaluate_detailed(vector)
                active = inert.evaluate_detailed(vector)
                row = dict(fixture=fixture.name, point_filtering=point_filter, scale=scale,
                           tension_points=tensions, old_cost=old.total_cost, new_cost=new.total_cost,
                           old_valid=old.num_valid_sweeps, new_valid=new.num_valid_sweeps,
                           cost_difference=new.total_cost-old.total_cost,
                           flex_on_difference=active.total_cost-old.total_cost,
                           residual_vector_max_difference=float(np.max(np.abs(
                               after.anchor_residual_vector(vector)-before.anchor_residual_vector(vector)))))
                assert np.isfinite(old.total_cost) and row['cost_difference'] == 0
                assert row['flex_on_difference'] == 0 and row['residual_vector_max_difference'] == 0
                assert row['old_valid'] == row['new_valid']
                rows.append(row)
    sentinel = []
    if args.lane_stages:
        # This probes saved geometries, not the original optimizer trajectory.
        for path in sorted(args.lane_stages.glob('fit-00[3456]-default.json')):
            payload = read_stage_artifact(path)['payload']
            fit = payload['fit']
            vector = anchors_matrix_to_opt_vec(np.asarray(fit['anchors']), fit['machine_type'])
            settings = payload['fit_options']
            for sweep_filter in (False, True):
                cost = EllipseCostFunction(fit['dataset_for_estimation'], pointwise_filter_stage=2,
                    pointwise_filtering=settings['pointwise_filtering'],
                    pointwise_global_mad=settings['pointwise_global_mad'],
                    sweep_wise_filtering=sweep_filter, sweep_metric=settings['sweep_metric'],
                    use_noise_mean=settings['use_noise_mean'], sigma_source=settings['sigma_source'])
                detail = cost.evaluate_detailed(vector)
                gradient = cost.gradient_numerical(vector)
                sentinel.append(dict(artifact=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                    sweeps=len(fit['dataset']['sweeps']), sweep_filtering=sweep_filter,
                    saved_fit_cost=fit['cost'], recomputed_cost=detail.total_cost,
                    retained_sweeps=detail.num_valid_sweeps, invalid_sweeps=detail.num_invalid_sweeps,
                    minimum=cost._min_sweeps_after_trim,
                    geometry_invalid_sweeps=sum(
                        cost._pointwise_sweep_residuals(sweep, np.asarray(fit['anchors']))['residuals'] is None
                        for sweep in cost.sweeps),
                    gradient_norm=float(np.linalg.norm(gradient))))
    result = dict(baseline_revision=args.baseline_revision,
                  before_sha256=hashlib.sha256(old_source).hexdigest(),
                  after_sha256=hashlib.sha256((ROOT/'autocal/ellipse_cost.py').read_bytes()).hexdigest(),
                  objective_checks=rows, saved_geometry_probe=sentinel)
    (args.output/'results.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({'objective_checks':len(rows), 'fixtures':len(DATASETS),
                      'max_cost_difference':max(abs(r['cost_difference']) for r in rows),
                      'saved_geometry_probe':sentinel}, indent=2))


if __name__ == '__main__':
    main()
