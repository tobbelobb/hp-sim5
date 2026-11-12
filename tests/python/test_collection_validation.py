"""Validate measured point roles without launching firmware or physics."""
import json

import numpy as np
import pytest

from autocal.theoretical_ellipse import compute_constraint_circle_3d
from hp_sim5_research.validation import validate_collection


@pytest.fixture
def collection():
    anchors = np.array([[0, -1900, -280], [1645.45, 950, -280],
                        [-1645.45, 950, -280], [0, 0, 1900]], dtype=float)
    fixed = [2, 3]
    seed = np.array([50, 60, 100])
    fixed_lengths = np.linalg.norm(anchors[fixed] - seed, axis=1)
    circle = compute_constraint_circle_3d(anchors, fixed, fixed_lengths.tolist())
    assert circle is not None
    scale = np.array([.5, .6, .7, .8])
    points = []
    for phi in np.linspace(.1, 1.1, 6):
        position = circle.center + circle.radius * (np.cos(phi) * circle.u + np.sin(phi) * circle.v)
        lengths = np.linalg.norm(anchors - position, axis=1) - np.linalg.norm(anchors, axis=1)
        points.append({'l_drive': lengths[0], 'l_sensor': lengths[1],
                       'raw_angles_deg': (lengths / scale).tolist(),
                       'sample_count': 4, 'sigma': [.01] * 4,
                       'source_drive_anchor': 1, 'source_sensor_anchor': 0,
                       # The command can be closer to the other axis's length.
                       'drive_setpoint_mm': lengths[0]})
    return {'version': '1.0', 'machine_type': 'hangprinter_3', 'num_anchors': 4, 'dimensions': 3,
            'config': {'mm_per_degree': scale.tolist(),
                       'm669': dict(zip('ABCD', anchors.tolist()))},
            'sweeps': [{'id': 'reversed', 'fixed_anchors': fixed,
                        'fixed_lengths': (fixed_lengths - np.linalg.norm(anchors[fixed], axis=1)).tolist(),
                        'drive_anchor': 0, 'sensor_anchor': 1, 'data_points': points}]}


@pytest.mark.parametrize('machine_type', ['hangprinter_3', 'hangprinter_4', 'skycam'])
def test_validation_uses_measured_orientation_and_catalogued_machine(tmp_path, collection, machine_type):
    collection['machine_type'] = machine_type
    path = tmp_path / 'sweeps.json'
    path.write_text(json.dumps(collection))
    result = validate_collection(path)
    assert result['point_count'] == result['autocal_residual_count'] == 6
    assert np.isfinite(result['firmware_anchor_residual_rms_mm'])


def test_validation_rejects_lengths_in_physical_instead_of_canonical_roles(tmp_path, collection):
    point = collection['sweeps'][0]['data_points'][0]
    point['l_drive'], point['l_sensor'] = point['l_sensor'], point['l_drive']
    path = tmp_path / 'sweeps.json'
    path.write_text(json.dumps(collection))
    with pytest.raises(AssertionError):
        validate_collection(path)
