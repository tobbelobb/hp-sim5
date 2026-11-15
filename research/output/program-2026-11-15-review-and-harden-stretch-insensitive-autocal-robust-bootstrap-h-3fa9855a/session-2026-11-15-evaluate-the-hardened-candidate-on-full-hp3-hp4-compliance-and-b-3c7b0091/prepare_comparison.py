"""Prepare a paired fit replay from one recorded dataset; no production edits.

Usage from repo root: PYTHONPATH=. .venv/bin/python prepare_comparison.py TEMPLATE DATASET OUTPUT [--corrupt]
Run compare_fit_replays with the resulting stage, reference and four options files.
"""
import argparse
import hashlib
import json
from pathlib import Path
import random
from autocal.stage_artifacts import read_stage_artifact, write_stage_artifact

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('template', type=Path)
p.add_argument('dataset', type=Path)
p.add_argument('output', type=Path)
p.add_argument('--corrupt', action='store_true')
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=False)
data = json.loads(a.dataset.read_text())
points = [point for sweep in data['sweeps'] for point in sweep['data_points']]
changed = []
if a.corrupt:
    rng = random.Random(9212026)
    for point in points:
        for key in ('l_drive', 'l_sensor'):
            delta = rng.gauss(0, 0.03)
            point[key] += delta
            if key + '_mu' in point:
                point[key + '_mu'] += delta
    for index in rng.sample(range(len(points)), min(3, len(points))):
        points[index]['l_sensor'] += 1.0
        if 'l_sensor_mu' in points[index]:
            points[index]['l_sensor_mu'] += 1.0
        changed.append(index)
artifact = read_stage_artifact(a.template, expected_kind='fit')
artifact['payload']['fit']['dataset'] = data
write_stage_artifact(a.output / 'stage.json', 'fit', artifact['payload'])
ref = data['config']['m669']
(a.output / 'reference.txt').write_text(' '.join(k + ':'.join(map(str, ref[k])) for k in 'ABCD') + '\n')
for offsets in (False, True):
    for filtering in (False, True):
        opts = {'fit_home_offsets': offsets, 'pointwise_filtering': filtering,
                'sweep_wise_filtering': filtering}
        (a.output / f'offset-{str(offsets).lower()}-filter-{str(filtering).lower()}.json').write_text(json.dumps(opts) + '\n')
(a.output / 'input.json').write_text(json.dumps({'dataset': str(a.dataset.resolve()), 'dataset_sha256': hashlib.sha256(a.dataset.read_bytes()).hexdigest(), 'template': str(a.template.resolve()), 'template_sha256': hashlib.sha256(a.template.read_bytes()).hexdigest(), 'points': len(points), 'sweeps': len(data['sweeps']), 'corrupt': a.corrupt, 'seed': 9212026 if a.corrupt else None, 'noise_sigma_mm': 0.03 if a.corrupt else None, 'sensor_outliers_mm': 1.0 if a.corrupt else None, 'outlier_point_indices': changed}, indent=2) + '\n')
