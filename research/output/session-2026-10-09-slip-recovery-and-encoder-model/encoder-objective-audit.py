import copy
import json
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from autocal.ellipse_cost import EllipseCostFunction
from autocal.spool_model import build_spool_model_params, dataset_with_modeled_lengths
from autocal.tools.regress_calibration_logs import SIM_3D_TRUE_ANCHORS

HERE = Path(__file__).resolve().parent
def cost_signature(dataset):
    cost = EllipseCostFunction(dataset, use_flex=False, pointwise_filtering=False, sweep_wise_filtering=False)
    entries = [cost._pointwise_sweep_residuals(sweep, np.asarray(SIM_3D_TRUE_ANCHORS)) for sweep in cost.sweeps]
    assert all(e['residuals'] is not None and np.isfinite(e['residuals']).all() for e in entries)
    return [np.asarray(e['residuals']).tolist() for e in entries]

trial5 = json.loads((ROOT / 'output/extended-autocal/trial-005/sweeps.json').read_text())
perturbed = copy.deepcopy(trial5)
for sweep in perturbed['sweeps']:
    for i, point in enumerate(sweep['data_points']):
        for axis in sweep['fixed_anchors']:
            point['raw_angles_deg'][axis] += .7 if i % 2 else -.7
raw_equal = cost_signature(trial5) == cost_signature(perturbed)

trial4 = json.loads((ROOT / 'output/extended-autocal/trial-004/sweeps.json').read_text())
reordered = copy.deepcopy(trial4)
for sweep in reordered['sweeps']:
    for axis in sweep['fixed_anchors']:
        values = [p['raw_angles_deg'][axis] for p in sweep['data_points']][::-1]
        for point, value in zip(sweep['data_points'], values):
            point['raw_angles_deg'][axis] = value
params = build_spool_model_params(trial4, base_radii_mm=[30]*4, modeled_radii_mm=[39.1845]*4,
    modeled_buildup_factor=[.636619]*4, spool_to_motor_gearing_factor=[1]*4,
    mechanical_advantage=[1]*4, lines_per_spool=[1]*4)
modeled = dataset_with_modeled_lengths(trial4, params)
modeled_reordered = dataset_with_modeled_lengths(reordered, params)
modeled_equal = cost_signature(modeled) == cost_signature(modeled_reordered)
metrics = []
for sweep in trial5['sweeps']:
    points = sweep['data_points']
    metrics.append({'id': sweep['id'], 'points': len(points), 'fixed_encoder_span_deg': {
        str(a): max(p['raw_angles_deg'][a] for p in points)-min(p['raw_angles_deg'][a] for p in points)
        for a in sweep['fixed_anchors']}})
result = {'trial005_fixed_perturbation_deg': .7, 'raw_residuals_identical': raw_equal,
    'trial004_fixed_angles_reversed': True, 'modeled_residuals_identical': modeled_equal,
    'trial005_sweeps': metrics}
(HERE / 'encoder-objective-audit.json').write_text(json.dumps(result, indent=2))
print(json.dumps(result, indent=2))
