import numpy as np

from autocal.active_learning import SweepConfig
from autocal.spool_model import (
    WinchSpoolModel,
    build_spool_model_params,
    dataset_with_modeled_lengths,
    sweep_configs_with_modeled_lengths,
)


def _sample_dataset() -> dict:
    mm_per_deg_base = (2.0 * np.pi * 10.0) / 360.0
    fixed_delta_base = 10.0
    fixed_theta_delta = fixed_delta_base / mm_per_deg_base
    theta0 = (100.0, 200.0, 300.0)

    return {
        "version": "1.0",
        "machine_type": "slideprinter",
        "num_anchors": 3,
        "dimensions": 2,
        "config": {
            "m666": {
                "R": [10.0, 10.0, 10.0],
                "H": [1.0, 1.0, 1.0],
                "L": [1.0, 1.0, 1.0],
                "U": [1.0, 1.0, 1.0],
                "O": [1.0, 1.0, 1.0],
            }
        },
        "sweeps": [
            {
                "id": "sweep_001",
                "fixed_anchors": [0],
                "fixed_lengths": [fixed_delta_base],
                "drive_anchor": 1,
                "sensor_anchor": 2,
                "data_points": [
                    {
                        "l_drive": 0.0,
                        "l_sensor": 0.0,
                        "raw_angles_deg": [theta0[0] + fixed_theta_delta, theta0[1], theta0[2]],
                    },
                    {
                        "l_drive": 36.0 * mm_per_deg_base,
                        "l_sensor": 90.0 * mm_per_deg_base,
                        "raw_angles_deg": [theta0[0] + fixed_theta_delta, theta0[1] + 36.0, theta0[2] + 90.0],
                    },
                ],
            }
        ],
    }


def test_nonlinear_delta_uses_absolute_thetas():
    model = WinchSpoolModel.from_firmware(
        base_radius=20.0,
        buildup_factor=0.01,
        spool_to_motor_gearing_factor=1.0,
        mechanical_advantage=1.0,
        lines_per_spool=1.0,
    )
    delta = model.delta_linepos_mm(40.0, 20.0)
    wrong = model.theta_deg_to_linepos_mm(20.0)
    assert not np.isclose(float(delta), float(wrong), atol=1e-9)
    assert np.isclose(
        float(delta),
        float(model.theta_deg_to_linepos_mm(40.0) - model.theta_deg_to_linepos_mm(20.0)),
        atol=1e-12,
    )


def test_build_spool_model_params_defaults_theta0_to_zero():
    dataset = _sample_dataset()
    params = build_spool_model_params(
        dataset,
        base_radii_mm=[10.0, 10.0, 10.0],
        modeled_radii_mm=[10.0, 10.0, 10.0],
        modeled_buildup_factor=[0.0, 0.0, 0.0],
        spool_to_motor_gearing_factor=[1.0, 1.0, 1.0],
        mechanical_advantage=[1.0, 1.0, 1.0],
        lines_per_spool=[1.0, 1.0, 1.0],
    )
    assert tuple(float(v) for v in params.theta0_deg) == (0.0, 0.0, 0.0)


def test_build_spool_model_params_can_infer_theta0():
    dataset = _sample_dataset()
    params = build_spool_model_params(
        dataset,
        base_radii_mm=[10.0, 10.0, 10.0],
        modeled_radii_mm=[10.0, 10.0, 10.0],
        modeled_buildup_factor=[0.0, 0.0, 0.0],
        spool_to_motor_gearing_factor=[1.0, 1.0, 1.0],
        mechanical_advantage=[1.0, 1.0, 1.0],
        lines_per_spool=[1.0, 1.0, 1.0],
        theta0_mode="infer",
    )
    assert np.allclose(np.asarray(params.theta0_deg, dtype=float), np.asarray([100.0, 200.0, 300.0]))


def test_dataset_with_modeled_lengths_rewrites_lengths_and_preserves_base_fields():
    dataset = _sample_dataset()
    params = build_spool_model_params(
        dataset,
        base_radii_mm=[10.0, 10.0, 10.0],
        modeled_radii_mm=[20.0, 10.0, 5.0],
        modeled_buildup_factor=[0.0, 0.0, 0.0],
        spool_to_motor_gearing_factor=[1.0, 1.0, 1.0],
        mechanical_advantage=[1.0, 1.0, 1.0],
        lines_per_spool=[1.0, 1.0, 1.0],
        theta0_mode="infer",
    )
    rewritten = dataset_with_modeled_lengths(dataset, params)

    sweep = rewritten["sweeps"][0]
    point0 = sweep["data_points"][0]
    point1 = sweep["data_points"][1]

    assert np.isclose(float(sweep["fixed_lengths_base"][0]), 10.0, atol=1e-9)
    assert np.isclose(float(sweep["fixed_lengths"][0]), 20.0, atol=1e-9)

    assert np.isclose(float(point0["l_drive_base"]), 0.0, atol=1e-9)
    assert np.isclose(float(point0["l_sensor_base"]), 0.0, atol=1e-9)
    assert np.isclose(float(point1["l_drive"]), float(point1["l_drive_base"]), atol=1e-9)
    assert np.isclose(float(point1["l_sensor"]), 0.5 * float(point1["l_sensor_base"]), atol=1e-9)

    # Verify source dataset was not modified in-place.
    assert "l_drive_base" not in dataset["sweeps"][0]["data_points"][0]
    assert "fixed_lengths_base" not in dataset["sweeps"][0]


def test_sweep_configs_with_modeled_lengths_scales_from_base_coordinate():
    dataset = _sample_dataset()
    params = build_spool_model_params(
        dataset,
        base_radii_mm=[10.0, 10.0, 10.0],
        modeled_radii_mm=[20.0, 10.0, 5.0],
        modeled_buildup_factor=[0.0, 0.0, 0.0],
        spool_to_motor_gearing_factor=[1.0, 1.0, 1.0],
        mechanical_advantage=[1.0, 1.0, 1.0],
        lines_per_spool=[1.0, 1.0, 1.0],
        theta0_mode="infer",
    )
    configs = [
        SweepConfig(
            fixed_anchors=(0, 2),
            fixed_deltas_mm=(10.0, 10.0),
            drive_anchor=1,
            sensor_anchor=2,
        )
    ]
    transformed = sweep_configs_with_modeled_lengths(configs, params)
    assert len(transformed) == 1
    assert np.isclose(float(transformed[0].fixed_deltas_mm[0]), 20.0, atol=1e-9)
    assert np.isclose(float(transformed[0].fixed_deltas_mm[1]), 5.0, atol=1e-9)


def _reference_theta_to_line(theta_deg, *, r0, q, gear, ma, lines):
    theta = float(theta_deg)
    if abs(float(q)) <= 1e-12:
        deg_per_unit_times_r = (float(gear) * float(ma) * 360.0) / (2.0 * np.pi)
        return (theta * float(r0)) / deg_per_unit_times_r
    k2 = -(float(ma) * float(lines)) * float(q)
    deg_per_unit_times_r = (float(gear) * float(ma) * 360.0) / (2.0 * np.pi)
    k0 = 2.0 * deg_per_unit_times_r / k2
    term = theta / k0 + float(r0)
    return (term * term - float(r0) * float(r0)) / k2


def test_winch_spool_model_matches_firmware_formula_with_multiple_lines():
    r0 = 40.184
    q = 0.31
    gear = 1.25
    ma = 1.7
    lines = 3.0
    model = WinchSpoolModel.from_firmware(
        base_radius=r0,
        buildup_factor=q,
        spool_to_motor_gearing_factor=gear,
        mechanical_advantage=ma,
        lines_per_spool=lines,
    )
    for theta in (-2000.0, -750.0, 0.0, 750.0, 2000.0):
        expected = _reference_theta_to_line(theta, r0=r0, q=q, gear=gear, ma=ma, lines=lines)
        got = model.theta_deg_to_linepos_mm(theta)
        assert np.isclose(float(got), float(expected), atol=1e-9)


def test_constant_spool_model_mm_per_degree_is_independent_of_lines_per_spool():
    kwargs = {
        "base_radius": 35.0,
        "buildup_factor": 0.0,
        "spool_to_motor_gearing_factor": 1.0,
        "mechanical_advantage": 2.0,
    }
    model_l1 = WinchSpoolModel.from_firmware(lines_per_spool=1.0, **kwargs)
    model_l3 = WinchSpoolModel.from_firmware(lines_per_spool=3.0, **kwargs)
    for theta in (-1000.0, -250.0, 0.0, 250.0, 1000.0):
        assert np.isclose(
            float(model_l1.theta_deg_to_linepos_mm(theta)),
            float(model_l3.theta_deg_to_linepos_mm(theta)),
            atol=1e-9,
        )
