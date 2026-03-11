import numpy as np
import pytest

from autocal.sweep_config_generator import generate_sweep_configs
from autocal.sweep_io import load_sweep_dataset, save_sweep_dataset
from autocal.sweep_types import (
    DataPoint,
    EllipseCoefficients,
    MachineConfig,
    MachineType,
    Sweep,
    SweepDataset,
)


def test_machine_config_slideprinter():
    config = MachineConfig.from_type(MachineType.SLIDEPRINTER)
    assert config.num_anchors == 3
    assert config.dimensions == 2
    assert config.constraints_for_1dof == 1


def test_machine_config_hangprinter_4():
    config = MachineConfig.from_type(MachineType.HANGPRINTER_4)
    assert config.num_anchors == 4
    assert config.dimensions == 3
    assert config.constraints_for_1dof == 2
    assert config.must_be_fixed_anchors == [3]
    assert config.fixed_anchor_delta_bounds[3] == (None, 0.0)


def test_sweep_validation_valid():
    config = MachineConfig.from_type(MachineType.SLIDEPRINTER)
    sweep = Sweep(
        id="test",
        fixed_anchors=[0],
        fixed_lengths=[10.0],
        drive_anchor=1,
        sensor_anchor=2,
        data_points=[DataPoint(i, i + 100) for i in range(10)],
    )
    errors = sweep.validate(config)
    assert errors == []


def test_sweep_validation_wrong_constraints():
    config = MachineConfig.from_type(MachineType.SLIDEPRINTER)
    sweep = Sweep(
        id="test",
        fixed_anchors=[0, 1],
        fixed_lengths=[10.0, 11.0],
        drive_anchor=2,
        sensor_anchor=0,
        data_points=[DataPoint(i, i + 100) for i in range(10)],
    )
    errors = sweep.validate(config)
    assert errors


def test_sweep_validation_blocks_carrying_sensor():
    config = MachineConfig.from_type(MachineType.HANGPRINTER_4)
    sweep = Sweep(
        id="bad_sensor",
        fixed_anchors=[0, 1],
        fixed_lengths=[10.0, 11.0],
        drive_anchor=2,
        sensor_anchor=3,  # Carrying anchor must never be Sensor
        data_points=[DataPoint(i, i + 100) for i in range(10)],
    )
    errors = sweep.validate(config)
    assert any("Carrying" in err for err in errors)


def test_ellipse_coefficients_normalization():
    coeffs = EllipseCoefficients(A=3.0, B=0.0, C=4.0, D=10, E=20, F=100)
    arr = coeffs.to_array()
    assert abs(np.linalg.norm(arr[:3]) - 1.0) < 1e-10


def test_sweep_config_generator_slideprinter():
    config = MachineConfig.from_type(MachineType.SLIDEPRINTER)
    configs = generate_sweep_configs(config)
    assert len(configs) == 3


def test_sweep_config_generator_hangprinter_4():
    config = MachineConfig.from_type(MachineType.HANGPRINTER_4)
    configs = generate_sweep_configs(config)
    assert len(configs) == 3
    assert all(3 in cfg["fixed_anchors"] for cfg in configs)
    assert all(cfg["drive_anchor"] != 3 for cfg in configs)
    assert all(cfg["sensor_anchor"] != 3 for cfg in configs)


def test_roundtrip_serialization(tmp_path):
    config = MachineConfig.from_type(MachineType.SLIDEPRINTER)
    dataset = SweepDataset(
        version="1.0",
        machine_config=config,
        timestamp="2024-01-15T10:00:00Z",
        sweeps=[
            Sweep(
                id="sweep_001",
                fixed_anchors=[0],
                fixed_lengths=[10.0],
                drive_anchor=1,
                sensor_anchor=2,
                data_points=[DataPoint(i * 10, i * 12 + 50) for i in range(20)],
            )
        ],
    )

    path = tmp_path / "test_dataset.json"
    save_sweep_dataset(dataset, path)
    loaded = load_sweep_dataset(path)

    assert loaded.version == dataset.version
    assert loaded.machine_config.machine_type == config.machine_type
    assert len(loaded.sweeps) == 1
    assert loaded.sweeps[0].id == "sweep_001"
    assert loaded.sweeps[0].fixed_lengths == [10.0]
    assert loaded.sweeps[0].data_points[0].l_drive == 0
