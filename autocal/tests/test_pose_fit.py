import numpy as np
import pytest

from autocal.pose_fit import (
    HOME_OFFSET_PRIOR_SIGMA_MM,
    dataset_line_lengths,
    fit_pose_least_squares,
    shift_dataset_home_offsets,
)

ANCHORS = np.array([[0.0, -1900.0, -280.0], [1645.4, 950.0, -280.0], [-1645.4, 950.0, -280.0], [0.0, 0.0, 1900.0]])
SEED = ANCHORS + np.array([[0.0, 40.0, 15.0], [-30.0, 20.0, 15.0], [25.0, -20.0, 15.0], [10.0, -10.0, 40.0]])


def _dataset(home_offsets=(0.0, 0.0, 0.0, 0.0)):
    """Sweeps at several heights; every length change is exact geometry minus a per-line home offset."""
    home = np.linalg.norm(ANCHORS, axis=1)
    sweeps = []
    for height in (0.0, 400.0, 800.0):
        for low in range(3):
            drive, sensor = [anchor for anchor in range(3) if anchor != low]
            start = np.array([0.0, 0.0, height]) + 250.0 * ANCHORS[low] / home[low] * np.array([1.0, 1.0, 0.0])
            direction = np.cross(ANCHORS[low] - start, ANCHORS[3] - start)
            direction /= np.linalg.norm(direction)
            points = []
            for step in np.linspace(-500.0, 500.0, 9):
                position = start + step * direction
                lengths = np.linalg.norm(ANCHORS - position, axis=1) - home - np.asarray(home_offsets)
                points.append({"l_drive": lengths[drive], "l_sensor": lengths[sensor],
                               "fixed": [lengths[low], lengths[3]]})
            # Each point lies on its own pair of fixed lengths, so every point is its own one-point sweep.
            for point in points:
                sweeps.append({"fixed_anchors": [low, 3], "fixed_lengths": point.pop("fixed"),
                               "drive_anchor": drive, "sensor_anchor": sensor, "data_points": [point]})
    return {"machine_type": "hangprinter_4", "num_anchors": 4, "dimensions": 3, "config": {}, "sweeps": sweeps}


def _fit(dataset, fit_home_offsets):
    return fit_pose_least_squares(dataset, SEED, fit_home_offsets=fit_home_offsets,
                                  home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM)


def test_rigid_data_recovers_the_anchors_from_a_distant_seed():
    result = _fit(_dataset(), fit_home_offsets=False)
    assert np.max(np.abs(result["anchors"] - ANCHORS)) < 1e-3
    assert result["rms_mm"] < 1e-6
    assert result["num_points"] == 81


def test_home_offset_biases_the_rigid_fit_and_is_recovered_when_fitted():
    dataset = _dataset(home_offsets=(0.0, 0.0, 0.0, 1.5))
    assert np.max(np.abs(_fit(dataset, fit_home_offsets=False)["anchors"] - ANCHORS)) > 1.0
    result = _fit(dataset, fit_home_offsets=True)
    assert np.max(np.abs(result["anchors"] - ANCHORS)) < 0.05
    assert result["home_offsets_mm"] == pytest.approx([0.0, 0.0, 0.0, 1.5], abs=0.01)


def test_one_bad_point_does_not_move_the_anchors():
    dataset = _dataset()
    dataset["sweeps"][40]["data_points"][0]["l_sensor"] += 5.0
    result = _fit(dataset, fit_home_offsets=False)
    assert np.max(np.abs(result["anchors"] - ANCHORS)) < 0.1
    assert int(np.argmax(result["point_residuals_mm"])) == 40


def test_shifted_dataset_contains_the_offsets_on_every_line_role():
    dataset = _dataset()
    shifted = shift_dataset_home_offsets(dataset, [1.0, 2.0, 3.0, 4.0])
    difference = dataset_line_lengths(shifted)["lengths_mm"] - dataset_line_lengths(dataset)["lengths_mm"]
    assert difference == pytest.approx(np.tile([1.0, 2.0, 3.0, 4.0], (81, 1)))
    assert dataset_line_lengths(dataset)["lengths_mm"][0, 3] == dataset["sweeps"][0]["fixed_lengths"][1]


def test_fixed_line_lengths_include_the_measured_holding_error():
    dataset = _dataset()
    dataset["config"]["mm_per_degree"] = [0.5, 0.5, 0.5, 0.5]
    sweep = dataset["sweeps"][0]
    sweep["data_points"][0]["fixed_error_angles_mu_deg"] = [0.2, None, None, -0.4]
    lengths = dataset_line_lengths(dataset)["lengths_mm"][0]
    assert lengths[0] == pytest.approx(sweep["fixed_lengths"][0] + 0.1)
    assert lengths[3] == pytest.approx(sweep["fixed_lengths"][1] - 0.2)


def test_per_pose_stretch_uses_actual_reversed_sensor_and_reeving():
    from autocal.pose_fit import consistency_residuals, dataset_stretch
    anchors = np.array([[0.,-1900.,-280.], [1900.,0.,-280.], [0.,1900.,-280.],
                        [-1900.,0.,-280.], [0.,0.,1900.]])
    ma, strands, guy = np.array([2,2,2,2,4]), np.array([2,2,2,2,4]), np.array([429.1]*4+[352.3])
    dataset = dict(num_anchors=5, dimensions=3, config={
        'm666_before_data_collection': {'S':20000., 'W':.081, 'U':ma.tolist(), 'N':strands.tolist(), 'Y':guy.tolist()},
        'force_tuning': {'force_low_n':.05}}, sweeps=[])
    for sensor in (1, 3):
        for position in ([0.,-100.,0.], [50.,-100.,100.], [-50.,-100.,200.]):
            vector = anchors - position
            distance = np.linalg.norm(vector, axis=1)
            directions = (vector / distance[:,None]).T
            active = [0,1,3,4]
            matrix = np.vstack([directions[:,active], np.array(active)==sensor])
            tensions = np.zeros(5)
            tensions[2] = .05
            tensions[active] = np.linalg.solve(matrix, np.r_[[0.,0.,.081*9.81]-.05*directions[:,2], 4.])
            assert np.all(tensions > 0)
            # Independent firmware equation, including guy wires and MA squared.
            material = distance - tensions*(ma*distance+guy)/(20000*ma**2*strands)
            delta = material - np.linalg.norm(anchors,axis=1)
            point = dict(l_drive=delta[1], l_sensor=delta[3], sensor_force_n=4., source_sensor_anchor=sensor)
            dataset['sweeps'].append(dict(fixed_anchors=[0,4], fixed_lengths=delta[[0,4]].tolist(),
                                           drive_anchor=1, sensor_anchor=3, metadata={'sensor_force_profile':True}, data_points=[point]))
    lengths = dataset_line_lengths(dataset)['lengths_mm']
    rigid = consistency_residuals(anchors, lengths, np.zeros(5))
    corrected = consistency_residuals(anchors, lengths, np.zeros(5), stretch=dataset_stretch(dataset))
    assert np.linalg.norm(rigid) > .01
    assert np.max(abs(corrected)) < 1e-8


def test_missing_or_projected_sensor_force_keeps_historical_rigid_fit():
    from autocal.pose_fit import dataset_stretch
    dataset = dict(num_anchors=5, dimensions=3, config={
        'm666_before_data_collection': {'S':20000., 'U':[2]*5, 'N':[2]*5}},
        sweeps=[dict(fixed_anchors=[0,4], fixed_lengths=[0.,0.], drive_anchor=1,
                     sensor_anchor=3, data_points=[dict(l_drive=0.,l_sensor=0.)])])
    assert dataset_stretch(dataset) is None
    for sweep in dataset['sweeps']:
        sweep['metadata'] = {'sensor_force_profile':True}
        sweep['data_points'][0]['sensor_force_n'] = 2.
    assert dataset_stretch(dataset) is not None
    dataset['sweeps'][0]['data_points'][0]['sensor_force_n'] = None
    assert dataset_stretch(dataset) is None
    point = dataset['sweeps'][0]['data_points'][0]
    point['sensor_force_n'] = 2.
    point['raw_angles_zero_tension_deg'] = [0.]*5
    assert dataset_stretch(dataset) is None


def test_nonredundant_ellipse_fit_keeps_its_rigid_offset_model():
    from autocal.pose_fit import dataset_stretch
    dataset = _dataset()
    dataset['config']['m666_before_data_collection'] = {'S':20000., 'U':[2]*4, 'N':[2]*4}
    for sweep in dataset['sweeps']:
        sweep['metadata'] = {'sensor_force_profile':True}
        sweep['data_points'][0]['sensor_force_n'] = 2.
    assert dataset_stretch(dataset) is None


def test_stretched_hp5_fit_recovers_geometry_from_a_distant_seed():
    anchors = np.array([[0.,-1900.,-280.], [1900.,0.,-280.], [0.,1900.,-280.],
                        [-1900.,0.,-280.], [0.,0.,1900.]])
    ma, strands, guy = np.array([2,2,2,2,4]), np.array([2,2,2,2,4]), np.array([429.1]*4+[352.3])
    dataset = dict(machine_type='hangprinter_5_pulley_reduced', num_anchors=5, dimensions=3, config={
        'm666_before_data_collection': {'S':20000., 'W':.081, 'U':ma.tolist(), 'N':strands.tolist(), 'Y':guy.tolist()},
        'force_tuning': {'force_low_n':.05}}, sweeps=[])
    rng = np.random.default_rng(541)
    for low in range(4):
        drive, sensor, passive = (low+1)%4, (low+3)%4, (low+2)%4
        active = [low,4,drive,sensor]
        for index in range(12):
            position = anchors[low]*[.1,.1,0.] + rng.uniform([-100.,-100.,0.],[100.,100.,500.])
            vector = anchors-position
            distance = np.linalg.norm(vector,axis=1)
            directions = (vector/distance[:,None]).T
            actual_sensor = sensor if index%2 else drive
            matrix = np.vstack([directions[:,active], np.array(active)==actual_sensor])
            tension = np.zeros(5); tension[passive]=.05
            tension[active] = np.linalg.solve(matrix,np.r_[[0.,0.,.081*9.81]-.05*directions[:,passive],4.])
            assert np.all(tension>0)
            delta = distance-tension*(ma*distance+guy)/(20000*ma**2*strands)-np.linalg.norm(anchors,axis=1)
            dataset['sweeps'].append(dict(fixed_anchors=[low,4], fixed_lengths=delta[[low,4]].tolist(),
                drive_anchor=drive,sensor_anchor=sensor,metadata={'sensor_force_profile':True},
                data_points=[dict(l_drive=delta[drive],l_sensor=delta[sensor],
                                  sensor_force_n=4.,source_sensor_anchor=actual_sensor)]))
    fit = fit_pose_least_squares(dataset, anchors*1.1+[0.,0.,20.], fit_home_offsets=False,
                                home_offset_prior_sigma_mm=5.)
    assert fit['success'] and fit['stretch_corrected']
    assert np.max(np.linalg.norm(fit['anchors']-anchors,axis=1)) < .001
