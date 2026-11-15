import copy

from autocal.active_learning import generate_candidate_sweeps
from autocal.workspace import exploration_grid
from autocal._autocal_common import _merge_sweep_datasets


def _data():
    return {"num_anchors": 4, "config": {"max_travel_mm": 390.}, "sweeps": [
        {"fixed_anchors": [0, 3], "fixed_lengths": [390., -390.],
         "data_points": [{"l_drive": 0., "l_sensor": 0.}]}]}


def test_successive_completed_probes_expand_beyond_initial_travel_without_a_geometry_input():
    data = _data()
    grid, bounds = exploration_grid(data, 9)
    assert bounds[3] == (-780., 390.)
    candidates = generate_candidate_sweeps(num_anchors=4, dimensions=3,
        fixed_delta_values_mm=grid, fixed_delta_bounds_by_anchor=bounds, machine_type="hangprinter_3")
    assert any(cfg.fixed_deltas_mm == (780., -780.) for cfg in candidates)
    assert all(-780. <= cfg.fixed_deltas_mm[-1] <= 0. for cfg in candidates)
    assert any(cfg.fixed_deltas_mm[0] < 0. for cfg in candidates)
    new = copy.deepcopy(data["sweeps"][0])
    new["fixed_lengths"] = [0., -780.]
    data["sweeps"].append(new)
    assert exploration_grid(data, 9)[1][3][0] == -1170.


def test_empty_probe_does_not_expand_observed_workspace():
    data = _data()
    data["sweeps"].append({"fixed_anchors": [0, 3], "fixed_lengths": [5000., -5000.], "data_points": []})
    assert exploration_grid(data, 9)[1][3][0] == -780.


def test_probe_excursion_can_seed_exploration_without_historical_max_travel():
    data = _data()
    data["config"].clear()
    assert exploration_grid(data, 9)[1][3][0] == -780.


def test_recorded_free_line_motion_can_seed_a_legacy_zero_fixed_target_dataset():
    data = _data()
    data["config"].clear()
    data["sweeps"][0]["fixed_lengths"] = [0., 0.]
    data["sweeps"][0]["data_points"][0]["l_drive"] = 100.
    assert exploration_grid(data, 9)[1][3][0] == -100.


def test_failed_probe_records_boundary_without_changing_measurements_or_force_reference():
    base = {**_data(), 'machine_type': 'hangprinter_3', 'dimensions': 3}
    rejected = {**copy.deepcopy(base), 'sweeps': [], 'config': {'force_tuning': {'force_mid_n': 99.}},
                'acquisition_rejections': [{'fixed_anchors': [0,3], 'fixed_lengths': [0.,-780.],
                                            'drive_anchor': 1, 'sensor_anchor': 2, 'reason': 'uncoupled'}]}
    merged = _merge_sweep_datasets(base, rejected)
    assert merged['sweeps'] == base['sweeps']
    assert merged['config'] == base['config']
    assert len(merged['acquisition_rejections']) == 1
    assert exploration_grid(merged, 9)[1] == exploration_grid(base, 9)[1]
