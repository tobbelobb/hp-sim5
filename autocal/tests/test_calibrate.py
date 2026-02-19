import numpy as np

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
