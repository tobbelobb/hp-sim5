import json
from pathlib import Path
import numpy as np
from autocal.rough_bootstrap import assess_rough_geometry

base = Path('research/output/program-2026-11-15-review-and-harden-stretch-insensitive-autocal-robust-bootstrap-h-3fa9855a/validation/hp3-adaptive-2')
rough = json.loads((base / 'sweeps.rough_bootstrap.json').read_text())
for count in (3, 4):
    data = {**rough, 'sweeps': rough['sweeps'][:count]}
    _, check = assess_rough_geometry(data)
    print('rough', count, json.dumps(check, sort_keys=True))
rows = [json.loads(line) for line in (base / 'sweeps.full_auto_log.jsonl').read_text().splitlines()]
last = [row for row in rows if row.get('iteration') == 5][0]
metrics = last['runs'][0]['metrics']
print('iteration 5 production point-fit anchors', json.dumps(metrics['pose_fit_anchors']))
print('point-fit rms', metrics['pose_fit_rms_mm'], 'max difference from corresponding ellipse fit', metrics['pose_fit_max_diff_mm'])
print('Caution: point-fit and ellipse share data, initialization and fitted offsets; this is not independent LS.')
