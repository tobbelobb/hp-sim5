"""Compare final results and same-budget prefixes with a common robust HP5r point fit."""
import argparse
import copy
import gzip
import json
from pathlib import Path

import numpy as np

from autocal.anchor_seed import build_anchor_initial_guess
from autocal.dataset_roles import mesh_metadata, normalize_dataset_point_roles
from autocal.ellipsoid_autocal import SEED_SCALES
from autocal.pose_fit import HOME_OFFSET_PRIOR_SIGMA_MM, fit_pose_least_squares
from autocal.tools.imperfect_machine import convention_frame, nominal_anchors


def load(path):
    if path.exists():
        return json.loads(path.read_text())
    with gzip.open(path.with_suffix(path.suffix + '.gz'), 'rt') as source:
        return json.load(source)


def rows_before(data, budget):
    def completed(sweep):
        return sweep['data_points'] and max((p.get('noise_clock_end_ms') or p['clock_time_ms']) / 1000
                                            for p in sweep['data_points']) <= budget
    rows = [s for s in data['sweeps'] if completed(s)]
    if any(mesh_metadata(s) is not None for s in data['sweeps']):
        unfinished = {mesh_metadata(s)['id'] for s in data['sweeps'] if not completed(s)}
        rows = [s for s in rows if mesh_metadata(s)['id'] not in unfinished]
    return rows


def record(run):
    data = load(run / 'sweeps.json')
    normalize_dataset_point_roles(data)
    try:
        imp = load(run / 'imperfection.json')
    except FileNotFoundError:
        imp = dict(truth_mm=nominal_anchors(data['machine_type']), stiffness_scale=1.0)
    manifest = load(run / 'sweeps.headless/manifest.json')
    summaries = [json.loads(row) for row in (run / 'sweeps.full_auto_log.jsonl').read_text().splitlines()]
    live = next((row for row in reversed(summaries) if row.get('event') == 'summary'), {})
    truth = convention_frame(np.array(list(imp['truth_mm'].values())))[0]
    return data, imp, manifest, live, truth


def score(anchors, truth):
    error = convention_frame(np.array(anchors))[0] - truth
    return dict(max_mm=float(np.abs(error).max()), top_z_mm=float(error[-1, 2]), error_mm=error.tolist())


def common_fit(data, ea, truth):
    data = copy.deepcopy(data)
    data['config']['m666_before_data_collection']['S'] *= ea
    start = build_anchor_initial_guess(data, machine_type=data['machine_type'])
    result = fit_pose_least_squares(data, start, extra_initial_anchors=[start * s for s in SEED_SCALES[1:]],
                                   fit_home_offsets=True, home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM)
    return dict(score(result['anchors'], truth), success=result['success'], rms_mm=result['rms_mm'],
                points=result['num_points'], rejected=len(result['rejected_points']),
                offsets_mm=result['home_offsets_mm'].tolist(), offsets_skipped=result['home_offsets_skipped'])


def compare(sweep, mesh):
    pairs = {key: record(directory) for key, directory in (('sweep', sweep), ('ellipsoid', mesh))}
    assert np.array_equal(pairs['sweep'][-1], pairs['ellipsoid'][-1])
    assert pairs['sweep'][1]['stiffness_scale'] == pairs['ellipsoid'][1]['stiffness_scale']
    budget = min(pair[2]['simulated_s'] for pair in pairs.values())
    output = dict(budget_sim_s=budget, true_ea_scale=pairs['sweep'][1]['stiffness_scale'])
    for shape, (dataset, imp, manifest, live, truth) in pairs.items():
        final_rows = dataset['sweeps']
        rows = rows_before(dataset, budget)
        fits = {'final_configured': common_fit(dataset, 1.0, truth)}
        if imp['stiffness_scale'] != 1:
            fits['final_true_ea'] = common_fit(dataset, imp['stiffness_scale'], truth)
        if len(rows) >= 3:
            if len(rows) == len(final_rows):
                fits['prefix_configured'] = fits['final_configured']
                if 'final_true_ea' in fits:
                    fits['prefix_true_ea'] = fits['final_true_ea']
            else:
                prefix = dict(dataset, sweeps=rows)
                fits['prefix_configured'] = common_fit(prefix, 1.0, truth)
                if imp['stiffness_scale'] != 1:
                    fits['prefix_true_ea'] = common_fit(prefix, imp['stiffness_scale'], truth)
        output[shape] = dict(scene_sha256=manifest.get('scene_sha256'),
                             rows=len(final_rows), prefix_rows=len(rows),
                             last_point_s=max((p.get('noise_clock_end_ms') or p['clock_time_ms']) / 1000
                                              for s in final_rows for p in s['data_points']),
                             revision=manifest['git_revision'], status=manifest['status'],
                             normal_completion=manifest.get('normal_completion'),
                             simulated_s=manifest['simulated_s'], stop_reason=manifest.get('stop_reason'),
                             live_fit=score(live['anchors'], truth) if 'anchors' in live else None,
                             fits=fits)
    return output


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('sweep', type=Path)
    p.add_argument('ellipsoid', type=Path)
    args = p.parse_args()
    print(json.dumps(compare(args.sweep, args.ellipsoid), indent=2))
