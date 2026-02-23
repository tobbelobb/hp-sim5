import numpy as np
import json

from autocal import calibrate as calibrate_module
from autocal.calibrate import _extract_motor_samples_from_sweep_dataset, calibrate_elliptical, main
from autocal.json_schema import write_json_file


def _tiny_slideprinter_delta_dataset(tmp_path):
    phi = np.linspace(0, np.pi, 12)
    l_drive_abs = np.sqrt(10000 + 1000 * np.cos(phi))
    l_sensor_abs = np.sqrt(12000 + 800 * np.cos(phi))

    base = 100.0
    dataset = {
        "version": "1.0",
        "machine_type": "slideprinter",
        "num_anchors": 3,
        "dimensions": 2,
        "sweeps": [
            {
                "id": "sweep_001",
                "fixed_anchors": [0],
                "fixed_lengths": [10.0],
                "drive_anchor": 1,
                "sensor_anchor": 2,
                "data_points": [
                    {"l_drive": float(ld - base), "l_sensor": float(ls - base)}
                    for ld, ls in zip(l_drive_abs + base, l_sensor_abs + base)
                ],
            }
        ],
    }

    path = tmp_path / "tiny.json"
    write_json_file(path, dataset, schema="sweep_dataset")
    return path


def test_calibrate_elliptical_runs(tmp_path):
    input_path = _tiny_slideprinter_delta_dataset(tmp_path)
    result = calibrate_elliptical(
        input_path,
        output_path=None,
        residual_threshold=1e9,
        num_restarts=1,
        max_iterations=2,
        method="SLSQP",
        verbose=False,
        generate_report=False,
    )
    anchors = np.asarray(result["anchors"], dtype=float)
    assert anchors.shape == (3, 2)
    assert "gcode" in result


def test_calibrate_elliptical_accepts_in_memory_dataset(tmp_path):
    input_path = _tiny_slideprinter_delta_dataset(tmp_path)
    dataset = json.loads(input_path.read_text(encoding="utf-8"))
    result = calibrate_elliptical(
        dataset,
        output_path=None,
        residual_threshold=1e9,
        num_restarts=1,
        max_iterations=2,
        method="SLSQP",
        verbose=False,
        generate_report=False,
    )
    anchors = np.asarray(result["anchors"], dtype=float)
    assert anchors.shape == (3, 2)
    assert result.get("input_file") == "<in-memory>"


def test_calibrate_elliptical_corrects_reversed_point_roles(tmp_path):
    input_path = _tiny_slideprinter_delta_dataset(tmp_path)
    dataset = json.loads(input_path.read_text(encoding="utf-8"))
    sweep = dataset["sweeps"][0]
    point = sweep["data_points"][0]
    drive_anchor = int(sweep["drive_anchor"])
    sensor_anchor = int(sweep["sensor_anchor"])
    original_drive = float(point["l_drive"])
    original_sensor = float(point["l_sensor"])

    point["source_drive_anchor"] = sensor_anchor
    point["source_sensor_anchor"] = drive_anchor
    point["drive_setpoint_mm"] = original_sensor
    point["l_drive"], point["l_sensor"] = original_sensor, original_drive

    result = calibrate_elliptical(
        dataset,
        output_path=None,
        residual_threshold=1e9,
        num_restarts=1,
        max_iterations=2,
        method="SLSQP",
        verbose=False,
        generate_report=False,
    )

    assert point["l_drive"] == original_drive
    assert point["l_sensor"] == original_sensor
    assert result["success"] is True


def test_calibrate_elliptical_keeps_canonical_points_with_reversed_source_metadata(tmp_path):
    input_path = _tiny_slideprinter_delta_dataset(tmp_path)
    dataset = json.loads(input_path.read_text(encoding="utf-8"))
    sweep = dataset["sweeps"][0]
    point = sweep["data_points"][0]
    drive_anchor = int(sweep["drive_anchor"])
    sensor_anchor = int(sweep["sensor_anchor"])
    original_drive = float(point["l_drive"])
    original_sensor = float(point["l_sensor"])

    point["source_drive_anchor"] = sensor_anchor
    point["source_sensor_anchor"] = drive_anchor
    # drive_setpoint_mm still refers to physical source drive, so for an already
    # canonical point it is expected to match l_sensor better than l_drive.
    point["drive_setpoint_mm"] = original_sensor

    result = calibrate_elliptical(
        dataset,
        output_path=None,
        residual_threshold=1e9,
        num_restarts=1,
        max_iterations=2,
        method="SLSQP",
        verbose=False,
        generate_report=False,
    )

    assert point["l_drive"] == original_drive
    assert point["l_sensor"] == original_sensor
    assert result["success"] is True


def test_build_anchor_initial_guess_slideprinter_uses_raw_angle_travel():
    dataset = {
        "machine_type": "slideprinter",
        "num_anchors": 3,
        "dimensions": 2,
        "config": {
            "m666": {
                "R": [30.0, 30.0, 30.0],
                "U": [1.0, 1.0, 1.0],
                "L": [1.0, 1.0, 1.0],
                "H": [1.0, 1.0, 1.0],
            }
        },
        "sweeps": [
            {
                "data_points": [
                    {"raw_angles_deg": [-100.0, -10.0, -5.0]},
                    {"raw_angles_deg": [-90.0, -8.0, -4.0]},
                ]
            }
        ],
    }
    guess = calibrate_module.build_anchor_initial_guess(dataset)
    assert isinstance(guess, np.ndarray)
    assert guess.shape == (3, 2)

    a = float(100.0 * (2.0 * np.pi * 30.0) / 360.0)
    base = np.asarray([0.0, -a], dtype=float)
    c120, s120 = np.cos(2.0 * np.pi / 3.0), np.sin(2.0 * np.pi / 3.0)
    c240, s240 = np.cos(4.0 * np.pi / 3.0), np.sin(4.0 * np.pi / 3.0)
    expected = np.asarray(
        [
            base,
            [c120 * base[0] - s120 * base[1], s120 * base[0] + c120 * base[1]],
            [c240 * base[0] - s240 * base[1], s240 * base[0] + c240 * base[1]],
        ],
        dtype=float,
    )
    assert np.allclose(guess, expected, atol=1e-6)


def test_build_anchor_initial_guess_hangprinter_5_layout():
    dataset = {
        "machine_type": "hangprinter_5",
        "num_anchors": 5,
        "dimensions": 3,
        "config": {
            "m666": {
                "R": [30.0] * 5,
                "U": [1.0] * 5,
                "L": [1.0] * 5,
                "H": [1.0] * 5,
            }
        },
        "sweeps": [{"data_points": [{"raw_angles_deg": [-90.0, -5.0, -4.0, -3.0, -2.0]}]}],
    }
    guess = calibrate_module.build_anchor_initial_guess(dataset)
    assert isinstance(guess, np.ndarray)
    assert guess.shape == (5, 3)

    a = float(90.0 * (2.0 * np.pi * 30.0) / 360.0)
    low_base = np.asarray([0.0, -a * np.cos(np.pi / 10.0), -a * np.sin(np.pi / 10.0)], dtype=float)
    expected_top = np.asarray([0.0, 0.0, a], dtype=float)
    assert np.allclose(guess[0], low_base, atol=1e-6)
    assert np.allclose(guess[-1], expected_top, atol=1e-6)


def test_calibrate_elliptical_forwards_initial_guess(tmp_path, monkeypatch):
    input_path = _tiny_slideprinter_delta_dataset(tmp_path)
    dataset = json.loads(input_path.read_text(encoding="utf-8"))
    initial_guess = np.asarray(
        [
            [0.0, -1000.0],
            [866.0, 500.0],
            [-866.0, 500.0],
        ],
        dtype=float,
    )
    seen = {"initial_guess": None}

    def fake_solve_anchors(dataset_arg, **kwargs):
        _ = dataset_arg
        seen["initial_guess"] = kwargs.get("initial_guess")
        return {
            "anchors": np.asarray(initial_guess, dtype=float),
            "cost": 0.0,
            "success": True,
            "details": {},
            "raw_result": {},
        }

    monkeypatch.setattr(calibrate_module, "solve_anchors", fake_solve_anchors)

    calibrate_module.calibrate_elliptical(
        dataset,
        output_path=None,
        residual_threshold=1e9,
        num_restarts=1,
        max_iterations=2,
        method="L-BFGS-B",
        verbose=False,
        generate_report=False,
        initial_guess=initial_guess,
    )

    got = np.asarray(seen["initial_guess"], dtype=float)
    assert got.shape == initial_guess.shape
    assert np.allclose(got, initial_guess)


def test_cli_subcommand_does_not_collide_with_optimizer_flag(tmp_path, capsys):
    input_path = _tiny_slideprinter_delta_dataset(tmp_path)
    rc = main(
        [
            "ellipse",
            str(input_path),
            "--no-report",
            "--optimizer",
            "SLSQP",
            "--iterations",
            "2",
            "--restarts",
            "1",
            "--threshold",
            "1000000000",
            "--spring-k-multiplier",
            "1.0",
            "--debug",
        ]
    )
    assert rc == 0
    out = capsys.readouterr().out
    assert "M669" in out or "Anchors" in out


def test_extract_motor_samples_from_sweep_dataset():
    dataset = {
        "version": "1.0",
        "machine_type": "slideprinter",
        "num_anchors": 3,
        "dimensions": 2,
        "sweeps": [
            {
                "id": "sweep_001",
                "fixed_anchors": [0],
                "fixed_lengths": [0.0],
                "drive_anchor": 1,
                "sensor_anchor": 2,
                "data_points": [
                    {"l_drive": 0.0, "l_sensor": 0.0, "raw_angles_deg": [0.0, 0.0, 0.0]},
                    {"l_drive": 1.0, "l_sensor": 2.0, "raw_angles_deg": [10.0, 20.0, 30.0]},
                    {"l_drive": 2.0, "l_sensor": 3.0, "raw_angles_deg": [20.0, 40.0, 60.0]},
                ],
            }
        ],
    }
    motor_samp, dims = _extract_motor_samples_from_sweep_dataset(dataset, max_samples=2)
    assert dims == 2
    assert motor_samp.shape == (2, 3)
