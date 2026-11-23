"""Kinematic-only imperfect machines: rewrite a recorded sweep dataset as if its anchors
were perturbed, then refit with the default two-stage point fit.

Each point keeps its held and drive line records (position-controlled lines); the
effector pose is re-solved on the perturbed anchors from those, and the sensor record
becomes the exact range on the perturbed geometry. No stretch, no noise, no force
effects: this isolates what the fit's geometry model does with uneven low anchors and
an off-centre top anchor.

usage: geom_synthetic.py DATASET MACHINE [--seeds 0-9] > out.json
"""
import argparse
import copy
import json
import sys

import numpy as np

from autocal.pose_fit import dataset_line_lengths
from autocal.tests.test_disturbed_robustness import _fit
from autocal.tools.disturb_dataset import _point_lines, _record_keys, _solve_position
from autocal.tools.imperfect_machine import draw, gauge_truth, nominal_anchors


def synthesize(dataset, anchors):
    data = copy.deepcopy(dataset)
    home = np.linalg.norm(anchors, axis=1)
    for sweep in data['sweeps']:
        for point in sweep['data_points']:
            held, pulled = _point_lines(sweep, point)
            row = dataset_line_lengths(dict(data, sweeps=[dict(sweep, data_points=[point])]))['lengths_mm'][0]
            pose = _solve_position(anchors, np.asarray(held), home[held] + row[held], np.zeros(3))
            for anchor in (a for a in pulled if np.isfinite(row[a])):
                change = np.linalg.norm(anchors[anchor] - pose) - home[anchor] - row[anchor]
                keys = _record_keys(sweep, anchor)
                target = point if keys else next(e for e in point['extra_sensor_ranges'] if int(e['anchor']) == anchor)
                for name in keys or ('length_mm', 'length_mu_mm'):
                    if target.get(name) is not None:
                        target[name] = float(target[name]) + change
    return data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('dataset')
    parser.add_argument('machine')
    parser.add_argument('--seeds', default='0-9')
    args = parser.parse_args()
    first, last = map(int, args.seeds.split('-'))
    dataset = json.load(open(args.dataset))
    nominal = np.array(list(nominal_anchors(args.machine).values()))
    results = []
    base = _fit(synthesize(dataset, nominal))['anchors'] - nominal
    results.append(dict(case='nominal-synthetic', error_mm=base.round(3).tolist()))
    for seed in range(first, last + 1):
        full = draw(args.machine, seed)
        for case, overrides in (('all', {}), ('low_dz_only', dict(top_offset_mm=[0, 0])),
                                ('top_offset_only', dict(low_dz_mm=[0] * (len(nominal) - 1)))):
            imperfection = draw(args.machine, seed, stiffness_scale=1, **overrides)
            truth = nominal + np.array(list(imperfection['shifts_mm'].values()))
            fit = _fit(synthesize(dataset, truth))
            error = fit['anchors'] - truth
            gauge, tilt = gauge_truth(truth)
            results.append(dict(seed=seed, case=case, shifts_mm=imperfection['shifts_mm'],
                                error_mm=error.round(3).tolist(), max_abs_error_mm=float(np.abs(error).max()),
                                rms_mm=fit.get('rms_mm'), rejected=len(fit['rejected_points']),
                                tilt_deg=tilt, max_abs_gauge_error_mm=float(np.abs(fit['anchors'] - gauge).max()),
                                gauge_error_mm=(fit['anchors'] - gauge).round(3).tolist()))
        print(seed, [(round(r['max_abs_error_mm'], 2), round(r['max_abs_gauge_error_mm'], 2), round(r['rms_mm'], 4), r['rejected']) for r in results[-3:]], file=sys.stderr)
    json.dump(results, sys.stdout, indent=1)


if __name__ == '__main__':
    main()
