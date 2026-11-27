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


@pytest.mark.parametrize('accepted', [True, False])
@pytest.mark.parametrize('profiled', [True, False])
def test_extra_range_force_survives_rejection_and_reversed_roles(accepted, profiled):
    from autocal.pose_fit import consistency_residuals, dataset_stretch
    anchors = np.array([[0.,-1900.,-280.], [1900.,0.,-280.], [0.,1900.,-280.],
                        [-1900.,0.,-280.], [0.,0.,1900.]])
    ma, strands, guy = np.array([2,2,2,2,4]), np.array([2,2,2,2,4]), np.array([429.1]*4+[352.3])
    dataset = dict(num_anchors=5, dimensions=3, config={
        'm666_before_data_collection': {'S':20000., 'W':.081, 'U':ma.tolist(), 'N':strands.tolist(), 'Y':guy.tolist()},
        'force_tuning': {'force_low_n':.05}}, sweeps=[])
    for sensor in (1, 3):
        for position in ([0.,-100.,0.], [50.,-100.,100.]):
            vector = anchors - position
            distance = np.linalg.norm(vector, axis=1)
            direction = (vector / distance[:,None]).T
            tensions = np.linalg.solve(np.vstack([direction, np.eye(5)[sensor], np.eye(5)[2]]),
                                       [0.,0.,.081*9.81,4.,3.])
            assert np.all(tensions > 0)
            delta = distance-tensions*(ma*distance+guy)/(20000*ma**2*strands)-np.linalg.norm(anchors,axis=1)
            # Mix accepted/rejected points so the measured row count varies.
            extra = dict(anchor=2, force_n=3., length_mm=delta[2], length_mu_mm=delta[2],
                         accepted=accepted and sensor == 1)
            point = dict(l_drive=delta[1], l_sensor=delta[3], sensor_force_n=4.,
                         source_sensor_anchor=sensor, extra_sensor_ranges=[extra])
            dataset['sweeps'].append(dict(fixed_anchors=[0,4], fixed_lengths=delta[[0,4]].tolist(),
                drive_anchor=1, sensor_anchor=3, metadata={'sensor_force_profile':profiled}, data_points=[point]))
    lengths = dataset_line_lengths(dataset)['lengths_mm']
    corrected = consistency_residuals(anchors, lengths, np.zeros(5), stretch=dataset_stretch(dataset))
    assert np.max(abs(corrected)) < 1e-8
    shifted = shift_dataset_home_offsets(dataset, [0,0,1.25,0,0])
    before = dataset['sweeps'][0]['data_points'][0]['extra_sensor_ranges'][0]
    after = shifted['sweeps'][0]['data_points'][0]['extra_sensor_ranges'][0]
    assert after['length_mm'] - before['length_mm'] == pytest.approx(1.25)
    assert after['length_mu_mm'] - before['length_mu_mm'] == pytest.approx(1.25)
    assert after['force_n'] == before['force_n']


@pytest.mark.parametrize("machine_type", ["hangprinter_3", "hangprinter_4"])
def test_equal_strain_home_reference_reaches_the_second_fit_once(machine_type):
    target = 2.0
    stiffness = 20000.0
    guy = np.array([500., 400., 300., 200.])
    advantage = np.array([2., 2., 2., 4.])
    offsets = target / stiffness * (np.linalg.norm(ANCHORS, axis=1) + guy / advantage)
    offsets[3] = 1.5  # The weight-bearing top line has an independent fitted offset.
    data = {**_dataset(offsets), "machine_type": machine_type}
    data["config"]["m666_before_data_collection"] = {
        "S": stiffness, "U": advantage.tolist(), "Y": guy.tolist(),
    }
    for sweep in data["sweeps"]:
        sweep["metadata"] = {"sensor_force_profile": True, "strand_tension_n": target}

    result = _fit(data, fit_home_offsets=True)
    assert result["home_offsets_mm"] == pytest.approx(offsets, abs=0.01)
    refit = _fit(shift_dataset_home_offsets(data, result["home_offsets_mm"]), fit_home_offsets=False)
    assert np.max(np.abs(refit["anchors"] - ANCHORS)) < 0.05


def test_rejected_point_preserves_physical_spool_winding_history_under_varying_tension():
    from autocal.flex import FlexModel, extract_spool_history, spool_rest_correction_mm
    from autocal.pose_fit import consistency_residuals, dataset_stretch, fit_point_positions

    anchors = ANCHORS
    m666 = {
        "S": 20000.0,
        "W": 0.081,
        "U": [1.0, 1.0, 1.0, 1.0],
        "N": [1.0, 1.0, 1.0, 1.0],
        "Y": [500.0, 500.0, 500.0, 400.0],
        "R": [30.0, 30.0, 30.0, 30.0],
        "K": 0.015,
        "V": 1,
    }
    home = np.linalg.norm(anchors, axis=1)
    # Four sweep points with strongly nonlinearly varying sensor tension (2 -> 18 -> 3 -> 14 N).
    # Rejecting point 1 from the geometric fit must NOT alter the LIFO spool winding integral
    # for subsequent points 2 and 3 because point 1 still physically occurred on the machine.
    sensor_forces = [2.0, 18.0, 3.0, 14.0]
    positions = [
        np.array([-200.0, 150.0, 100.0]),
        np.array([-100.0, 120.0, 150.0]),
        np.array([50.0, 90.0, 200.0]),
        np.array([200.0, 60.0, 250.0]),
    ]
    raw_lengths = np.array([np.linalg.norm(anchors - p, axis=1) - home for p in positions], dtype=float)
    points = []
    for k, (row, force) in enumerate(zip(raw_lengths, sensor_forces)):
        points.append({
            "l_drive": float(row[1]),
            "l_sensor": float(row[2]),
            "sensor_force_n": float(force),
        })
    dataset = {
        "machine_type": "hangprinter_3",
        "num_anchors": 4,
        "dimensions": 3,
        "config": {
            "m666_before_data_collection": m666,
            "force_tuning": {"force_low_n": 0.2, "pullout_force_n": 4.0},
        },
        "sweeps": [{
            "id": "sweep_001",
            "fixed_anchors": [0, 3],
            "fixed_lengths": [float(raw_lengths[0, 0]), float(raw_lengths[0, 3])],
            "drive_anchor": 1,
            "sensor_anchor": 2,
            "metadata": {
                "spool_reserves": [{"sensor_payout_mm": float(np.max(raw_lengths[:, 2]) + 300.0)}],
                "direction_forces": [{"pullout_force_n": 4.0}],
            },
            "data_points": points,
        }],
    }
    stretch = dataset_stretch(dataset)
    assert stretch is not None and stretch["model"].has_spool_physics
    lengths = dataset_line_lengths(dataset)["lengths_mm"]
    kept = np.array([True, False, True, True])
    masked = np.where(kept[:, None], lengths, np.nan)

    # First verify that naively skipping row 1 (without history_lengths_mm) DOES change rows 2..3
    # because sensor tension varies nonlinearly across row 1.
    _, abs_full = fit_point_positions(anchors, lengths, np.zeros(4), stretch=stretch)
    _, abs_naive = fit_point_positions(anchors, masked, np.zeros(4), stretch=stretch)
    assert not np.allclose(abs_naive[[2, 3], 2], abs_full[[2, 3], 2], atol=1e-6)

    # With physical history preserved via history_lengths_mm, kept rows 0, 2, 3 are bit-for-bit identical.
    _, abs_preserved = fit_point_positions(
        anchors, masked, np.zeros(4), stretch=stretch, history_lengths_mm=lengths
    )
    np.testing.assert_allclose(abs_preserved[[0, 2, 3], :], abs_full[[0, 2, 3], :], atol=1e-12)

    res_full = consistency_residuals(anchors, lengths, np.zeros(4), stretch=stretch)
    res_preserved = consistency_residuals(
        anchors, masked, np.zeros(4), stretch=stretch, history_lengths_mm=lengths
    )
    assert np.all(res_preserved[1] == 0.0)
    np.testing.assert_allclose(res_preserved[[0, 2, 3], :], res_full[[0, 2, 3], :], atol=1e-12)
