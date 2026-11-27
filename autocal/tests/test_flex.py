import numpy as np

from autocal.flex import FlexModel, extract_spool_history, slice_spool_history, spool_rest_correction_mm


def test_flex_model_corrects_length_monotonically():
    m666 = {"S": 20000.0, "U": [1.0], "Y": [500.0]}
    model = FlexModel.from_m666(m666, num_axes=1)
    assert model is not None

    l = np.array([2000.0], dtype=float)
    d0 = model.corrected_distance_mm(l, np.array([0.0]), axis=0)[0]
    d1 = model.corrected_distance_mm(l, np.array([10.0]), axis=0)[0]

    assert d1 > d0
    assert np.isclose(d0, 2000.0)


def test_flex_model_handles_nan_tension_as_zero():
    m666 = {"S": 20000.0, "U": [1.0], "Y": [0.0]}
    model = FlexModel.from_m666(m666, num_axes=1)
    assert model is not None

    l = np.array([1234.0], dtype=float)
    out = model.corrected_distance_mm(l, np.array([np.nan]), axis=0)[0]
    assert np.isclose(out, 1234.0)


def test_flex_inverse_matches_firmware_ma_squared_and_strand_scaling():
    for ma, strands in ((1., 2.), (2., 2.), (4., 4.)):
        model = FlexModel.from_m666({'S': 20000., 'U': [ma], 'N': [strands], 'Y': [429.1]}, num_axes=1)
        distance, force = np.array([1000., 1800., 2200.]), np.array([2., 4., 12.])
        material = distance - force * (ma * distance + 429.1) / (20000. * ma ** 2 * strands)
        assert np.allclose(model.corrected_distance_mm(material, force, axis=0), distance, atol=1e-10)


def test_spool_rho_matches_flattening_and_strained_storage_and_disables_with_q():
    m666 = {"S": 20000.0, "U": [2.0], "N": [2.0], "R": [75.0], "K": 0.015, "V": 1, "Q": 0.0}
    model = FlexModel.from_m666(m666, num_axes=1)
    assert model is not None
    assert model.has_spool_physics

    t = np.array([0.0, 20.0, 40.0])
    # Strand tension per strand = T / (U * N) = T / 4, flattening dr/R = K * (T/4) / R
    # Strain = (T/4) / (S_eff / N) = T / (U * N * S)
    t_line = t / 4.0
    expected = (1.0 - 0.015 * t_line / (75.0 ** 2)) / (1.0 + t_line / 20000.0)
    np.testing.assert_allclose(model.spool_rho(t, axis=0), expected, atol=1e-12)

    # Flattening-only mode (V=0) clamps radius contraction at 0.5.
    flat_only = FlexModel.from_m666({**m666, "V": 0}, num_axes=1)
    assert flat_only is not None and flat_only.has_spool_physics
    assert np.isclose(flat_only.spool_rho(np.array([1e7]), axis=0)[0], 0.5)

    # Nonzero buildup Q disables K and V to match firmware mutual exclusion.
    layered = FlexModel.from_m666({**m666, "Q": 0.6366}, num_axes=1)
    assert layered is not None
    assert not layered.has_spool_physics
    np.testing.assert_allclose(layered.spool_rho(t, axis=0), 1.0, atol=1e-12)


def test_home_tension_shifts_corrected_distance_exactly():
    m666 = {"S": 118500.0, "U": [1.0], "N": [1.0], "Y": [200.0], "T": [90.0]}
    model = FlexModel.from_m666(m666, num_axes=1)
    assert model is not None and model.has_home_tension
    d_home = 3800.0
    d_true = np.array([3500.0, 4100.0])
    tension = np.array([60.0, 120.0])
    home_flex = 90.0 * (d_home + 200.0) / 118500.0
    assert np.isclose(model.home_flex_mm(d_home, axis=0), home_flex)
    # Forward firmware flex: l_meas = d - F*(d + Y)/EA + home_flex
    l_meas = d_true - tension * (d_true + 200.0) / 118500.0 + home_flex
    np.testing.assert_allclose(
        model.corrected_distance_mm(l_meas, tension, axis=0, home_distance_mm=d_home),
        d_true,
        atol=1e-10,
    )


def test_spool_rest_correction_tracks_lifo_sensor_rewind_and_drive_pullout():
    m666 = {"S": 20000.0, "U": [1.0, 1.0], "N": [1.0, 1.0], "R": [30.0, 30.0], "K": 0.015, "V": 1}
    model = FlexModel.from_m666(m666, num_axes=2)
    # One pass: drive=0 winds to -100..-300 at pullout_force=4 N;
    # sensor=1 paid out to +200 mm (pre-wound at rho_init=1), then rewound to +100, 0, -100 at T=[2, 6, 10] N.
    dataset = {
        "config": {"force_tuning": {"pullout_force_n": 4.0}},
        "sweeps": [
            {
                "drive_anchor": 0,
                "sensor_anchor": 1,
                "metadata": {
                    "spool_reserves": [{"sensor_payout_mm": 200.0}],
                    "direction_forces": [{"pullout_force_n": 4.0}],
                },
                "data_points": [
                    {"l_drive": -100.0, "l_sensor": 100.0},
                    {"l_drive": -200.0, "l_sensor": 0.0},
                    {"l_drive": -300.0, "l_sensor": -100.0},
                ],
            }
        ],
    }
    history = extract_spool_history(dataset)
    lengths = np.array([[-100.0, 100.0], [-200.0, 0.0], [-300.0, -100.0]])
    tensions = np.array([[10.0, 2.0], [10.0, 6.0], [10.0, 10.0]])
    corr = spool_rest_correction_mm(lengths, tensions, model, history)

    # Drive coils were wound at pullout force (4 N), not current tension (10 N).
    drho_pull = 1.0 - float(model.spool_rho(np.array([4.0]), axis=0)[0])
    np.testing.assert_allclose(corr[:, 0], np.array([100.0, 200.0, 300.0]) * drho_pull, atol=1e-12)

    # Sensor integrated LIFO rewind from payout=200 mm:
    drho_s = 1.0 - model.spool_rho(tensions[:, 1], axis=1)
    c0 = 100.0 * drho_s[0]
    c1 = c0 + 100.0 * 0.5 * (drho_s[0] + drho_s[1])
    c2 = c1 + 100.0 * 0.5 * (drho_s[1] + drho_s[2])
    np.testing.assert_allclose(corr[:, 1], np.array([c0, c1, c2]), atol=1e-12)

    # Slicing spool history keeps valid remapped indices without crashing.
    sliced = slice_spool_history(history, np.array([True, False, True]))
    assert sliced is not None and sliced["num_points"] == 2
    assert sliced["passes"][0]["rows"].tolist() == [0, 1]
