"""Paired HP5 bootstrap replay. Run from the repo root with PYTHONPATH=. and one BLAS thread."""
import argparse
import copy
import gzip
import hashlib
import json
from pathlib import Path

import numpy as np

from autocal.rough_bootstrap import assess_rough_geometry
from autocal.dataset_roles import normalize_dataset_point_roles
from autocal.pose_fit import fit_pose_least_squares, shift_dataset_home_offsets
from autocal.predictive_validation import score_future_sweeps
from autocal.tools.sanity_battery import anchor_errors


def pack(directory, dest):
    sources = {}
    def load(name):
        raw = (directory / name).read_bytes()
        sources[name] = hashlib.sha256(raw).hexdigest()
        return json.loads(raw)
    manifest = load('sweeps.headless/manifest.json')
    reference = manifest['firmware_initial']['M669']
    case = dict(reference_mm=anchor_errors(reference, reference)['reference_mm'],
                scene_sha256=manifest['initial_baked_scene_sha256'], revision=manifest['git_revision'],
                rough=load('sweeps.rough_bootstrap.json'), final=load('sweeps.json'), sources=sources)
    dest.write_bytes(gzip.compress(json.dumps(case, separators=(',', ':')).encode(), mtime=0))


def fit(training, seed):
    options = dict(home_offset_prior_sigma_mm=5.)
    first = fit_pose_least_squares(training, seed, fit_home_offsets=True, **options)
    shifted = shift_dataset_home_offsets(training, first['home_offsets_mm'])
    final = fit_pose_least_squares(shifted, first['anchors'], fit_home_offsets=False, **options)
    if not (first['success'] and final['success']):
        raise ValueError('Bootstrap fit failed to converge')
    return final['anchors'], first['home_offsets_mm']


def compare(case, physical_stiffness):
    rough, full = case['rough'], case['final']
    for data in (rough, full):
        normalize_dataset_point_roles(data)
    truth = np.asarray(case['reference_mm'])
    n = len(rough['sweeps'])
    profiled = full['sweeps'][:n]
    future = full['sweeps'][n:]
    assert len(profiled) == n
    for a, b in zip(rough['sweeps'], profiled):
        for key in ('fixed_anchors', 'drive_anchor', 'sensor_anchor'):
            assert a[key] == b[key]
        np.testing.assert_allclose(a['fixed_lengths'], b['fixed_lengths'], atol=.1, rtol=0)
        assert not a['metadata']['sensor_force_profile'] and b['metadata']['sensor_force_profile']
    seed, check = assess_rough_geometry(rough)
    assert check['ready']
    def last_sample(sweeps):
        return max(p['clock_time_ms'] for s in sweeps for p in s['data_points']) / 1000
    output = dict(bootstrap_configurations=n, rough_last_sample_s=last_sample(rough['sweeps']),
                  profiled_last_sample_s=last_sample(profiled), rows=[])
    for policy, base in [('rough', rough), ('profiled', full)]:
        for stiffness in dict.fromkeys((None, float(base['config']['m666_before_data_collection']['S']),
                                        physical_stiffness)):
            for appended in range(min(1, len(future)) + 1):
                training = copy.deepcopy(base)
                training['sweeps'] = copy.deepcopy(base['sweeps'][:n] + future[:appended])
                training['config']['m666_before_data_collection']['S'] = stiffness
                try:
                    anchors, offsets = fit(training, seed)
                except (ValueError, np.linalg.LinAlgError) as exc:
                    output['rows'].append(dict(policy=policy, stiffness=stiffness, appended=appended,
                                               failure=str(exc)))
                    continue
                delta = anchors - truth
                validation = dict(training, sweeps=future[appended:])
                score = score_future_sweeps(validation, anchors, [], home_offsets_mm=offsets)
                output['rows'].append(dict(policy=policy, stiffness=stiffness, appended=appended,
                    points=sum(len(s['data_points']) for s in training['sweeps']),
                    max_error_mm=float(np.max(abs(delta))),
                    aligned_mm=float(np.max(abs(delta-delta.mean(axis=0)))),
                    heldout_constraints_rms_mm=None if score is None else float(np.sqrt(score[0]))))
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('case', type=Path, help='Portable gzipped input')
    parser.add_argument('result', type=Path)
    parser.add_argument('--pack-from', type=Path, help='First archive this completed production run')
    parser.add_argument('--physical-stiffness', type=float, required=True)
    args = parser.parse_args()
    if args.pack_from:
        pack(args.pack_from, args.case)
    result = compare(json.loads(gzip.decompress(args.case.read_bytes())), args.physical_stiffness)
    args.result.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
