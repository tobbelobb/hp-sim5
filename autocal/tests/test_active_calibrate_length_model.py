import numpy as np

from autocal.active_calibrate import (
    _delta_base_to_model,
    _transform_dataset_lengths_to_model,
    _transform_sweep_configs_to_model,
)
from autocal.active_learning import SweepConfig


def _unit_kinematics(num_axes: int = 3):
    base_r = np.array([10.0, 10.0, 10.0], dtype=float)[:num_axes]
    eff_r = np.array([20.0, 10.0, 5.0], dtype=float)[:num_axes]
    gear = np.ones(num_axes, dtype=float)
    mech = np.ones(num_axes, dtype=float)
    lines = np.ones(num_axes, dtype=float)
    return base_r, eff_r, gear, mech, lines


def test_delta_base_to_model_k0_scales_by_radius_ratio():
    base_r, eff_r, gear, mech, lines = _unit_kinematics()
    out0 = _delta_base_to_model(
        10.0,
        0,
        base_radii_mm=base_r,
        effective_radii_mm=eff_r,
        buildup_factor=0.0,
        spool_to_motor_gearing_factor=gear,
        mechanical_advantage=mech,
        lines_per_spool=lines,
    )
    out2 = _delta_base_to_model(
        10.0,
        2,
        base_radii_mm=base_r,
        effective_radii_mm=eff_r,
        buildup_factor=0.0,
        spool_to_motor_gearing_factor=gear,
        mechanical_advantage=mech,
        lines_per_spool=lines,
    )
    assert np.isclose(float(out0), 20.0, atol=1e-9)
    assert np.isclose(float(out2), 5.0, atol=1e-9)


def test_transform_dataset_lengths_to_model_updates_sweep_lengths():
    base_r, eff_r, gear, mech, lines = _unit_kinematics()
    dataset = {
        "version": "1.0",
        "machine_type": "slideprinter",
        "num_anchors": 3,
        "dimensions": 2,
        "config": {"mm_per_degree": [1.0, 1.0, 1.0]},
        "sweeps": [
            {
                "id": "sweep_001",
                "fixed_anchors": [0],
                "fixed_lengths": [10.0],
                "drive_anchor": 1,
                "sensor_anchor": 2,
                "data_points": [
                    {
                        "l_drive": 6.0,
                        "l_sensor": 8.0,
                        "l_drive_mu": 7.0,
                        "l_sensor_mu": 9.0,
                    }
                ],
            }
        ],
    }

    transformed = _transform_dataset_lengths_to_model(
        dataset,
        base_radii_mm=base_r,
        effective_radii_mm=eff_r,
        buildup_factor=0.0,
        spool_to_motor_gearing_factor=gear,
        mechanical_advantage=mech,
        lines_per_spool=lines,
    )

    sweep = transformed["sweeps"][0]
    point = sweep["data_points"][0]
    assert np.isclose(float(sweep["fixed_lengths"][0]), 20.0, atol=1e-9)
    assert np.isclose(float(point["l_drive"]), 6.0, atol=1e-9)
    assert np.isclose(float(point["l_sensor"]), 4.0, atol=1e-9)
    assert np.isclose(float(point["l_drive_mu"]), 7.0, atol=1e-9)
    assert np.isclose(float(point["l_sensor_mu"]), 4.5, atol=1e-9)

    mm_per = np.asarray(transformed["config"]["mm_per_degree"], dtype=float)
    assert mm_per.shape[0] == 3
    assert np.isclose(mm_per[0], 2.0 * np.pi * 20.0 / 360.0, atol=1e-12)
    assert np.isclose(mm_per[2], 2.0 * np.pi * 5.0 / 360.0, atol=1e-12)

    original_point = dataset["sweeps"][0]["data_points"][0]
    assert np.isclose(float(original_point["l_sensor"]), 8.0, atol=1e-9)


def test_transform_sweep_configs_to_model_uses_axis_specific_scaling():
    base_r, eff_r, gear, mech, lines = _unit_kinematics()
    configs = [
        SweepConfig(
            fixed_anchors=(0, 2),
            fixed_deltas_mm=(10.0, 10.0),
            drive_anchor=1,
            sensor_anchor=2,
        )
    ]
    transformed = _transform_sweep_configs_to_model(
        configs,
        base_radii_mm=base_r,
        effective_radii_mm=eff_r,
        buildup_factor=0.0,
        spool_to_motor_gearing_factor=gear,
        mechanical_advantage=mech,
        lines_per_spool=lines,
    )
    assert len(transformed) == 1
    cfg = transformed[0]
    assert tuple(cfg.fixed_anchors) == (0, 2)
    assert np.isclose(float(cfg.fixed_deltas_mm[0]), 20.0, atol=1e-9)
    assert np.isclose(float(cfg.fixed_deltas_mm[1]), 5.0, atol=1e-9)
