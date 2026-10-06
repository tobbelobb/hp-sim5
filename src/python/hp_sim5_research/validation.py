"""Load actual collector output through autocal's measurement and residual pipeline."""
from pathlib import Path

import numpy as np


def validate_collection(path, anchors=None):
    from autocal.json_schema import load_json_file
    from autocal.calibrate import _validate_sweep_roles
    from autocal.dataset_roles import normalize_dataset_point_roles
    from autocal.spool_model import validate_dataset_has_raw_angles
    from autocal.ellipse_cost import EllipseCostFunction
    from autocal.theoretical_ellipse import anchors_matrix_to_opt_vec

    dataset = load_json_file(Path(path), schema='sweep_dataset')
    validate_dataset_has_raw_angles(dataset)
    _validate_sweep_roles(dataset)
    if normalize_dataset_point_roles(dataset):
        raise ValueError('Collector points were not already in canonical drive/sensor orientation')
    scale = np.asarray(dataset['config']['mm_per_degree'])
    spans = []
    sensor_spans = []
    count = 0
    for sweep in dataset['sweeps']:
        points = sweep['data_points']
        count += len(points)
        for point in points:
            lengths = np.asarray(point['raw_angles_deg']) * scale
            np.testing.assert_allclose([point['l_drive'], point['l_sensor']],
                                       lengths[[sweep['drive_anchor'], sweep['sensor_anchor']]], rtol=1e-10, atol=1e-8)
            if not point.get('sample_count') or len(point.get('sigma', [])) != 4:
                raise ValueError('Missing collector noise statistics')
        spans.append(float(np.ptp([point['l_drive'] for point in points])))
        directions = {}
        for point in points:
            drive = point.get('source_drive_anchor', sweep['drive_anchor'])
            sensor = point.get('source_sensor_anchor', sweep['sensor_anchor'])
            directions.setdefault((drive, sensor), []).append(float(point['raw_angles_deg'][sensor] * scale[sensor]))
        sensor_spans.extend(float(np.ptp(values)) for values in directions.values())
    if not spans or min(spans) < .01:
        raise ValueError('Collection did not produce a measurable drive movement (0.01 mm minimum)')
    if not sensor_spans or min(sensor_spans) < .01:
        raise ValueError('A physical sub-sweep sensor did not respond (0.01 mm minimum); increase sensorCollectionForce')
    config = dataset['config']['m669']
    anchors = np.asarray([config[axis] for axis in 'ABCD'] if anchors is None else anchors, dtype=float)
    if anchors.shape != (4, 3) or not np.isfinite(anchors).all():
        raise ValueError('Collection must preserve all three firmware coordinates for each HP4 anchor')
    objective = EllipseCostFunction(dataset, min_points=5, use_flex=False,
                                   pointwise_filtering=False, sweep_wise_filtering=False)
    vector = anchors_matrix_to_opt_vec(anchors, 'hangprinter_4')
    rows = objective.pointwise_residual_rows(vector)
    residuals = np.asarray([row['residual_mm'] for row in rows])
    if len(rows) != count or not np.isfinite(residuals).all():
        raise ValueError('Autocal did not evaluate every collected measurement')
    return {'schema': dataset['version'], 'point_count': count, 'drive_span_mm': spans, 'physical_sensor_span_mm': sensor_spans,
            'autocal_residual_count': len(rows), 'firmware_anchor_residual_rms_mm': float(np.sqrt(np.mean(residuals**2))),
            'calibration_accuracy': 'Short preflight proves measurement ingestion; anchor fitting requires multiple held-out sweeps'}
