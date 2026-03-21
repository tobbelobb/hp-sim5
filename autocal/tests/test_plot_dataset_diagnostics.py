import json

import numpy as np

from scripts.plot_dataset_diagnostics import _sweep_fixed_angle_series, main


def _tiny_dataset() -> dict:
    return {
        "machine_type": "hangprinter_4",
        "num_anchors": 4,
        "dimensions": 3,
        "sweeps": [
            {
                "id": "sweep_001",
                "fixed_anchors": [0, 3],
                "fixed_lengths": [483.0, 0.0],
                "drive_anchor": 1,
                "sensor_anchor": 2,
                "data_points": [
                    {"raw_angles_deg": [10.0, 20.0, 30.0, 40.0]},
                    {"raw_angles_deg": [10.1, 21.0, 31.0, 39.9]},
                    {"raw_angles_deg": [9.9, 22.0, 32.0, 40.2]},
                ],
            },
            {
                "id": "sweep_002",
                "fixed_anchors": [1, 3],
                "fixed_lengths": [1.0, 2.0],
                "drive_anchor": 0,
                "sensor_anchor": 2,
                "data_points": [
                    {"raw_angles_deg": [50.0, 60.0, 70.0, 80.0]},
                    {"raw_angles_deg": [51.0, 60.5, 71.0, 79.5]},
                ],
            },
        ],
    }


def test_sweep_fixed_angle_series_uses_fixed_anchor_indices():
    dataset = _tiny_dataset()
    labels = ["A", "B", "C", "D"]
    series = _sweep_fixed_angle_series(dataset["sweeps"][0], anchor_labels=labels)

    assert [entry["anchor_idx"] for entry in series] == [0, 3]
    assert [entry["anchor_label"] for entry in series] == ["A", "D"]
    np.testing.assert_allclose(series[0]["raw_angles_deg"], [10.0, 10.1, 9.9])
    np.testing.assert_allclose(series[1]["raw_angles_deg"], [40.0, 39.9, 40.2])


def test_main_writes_raw_angle_diagnostics(tmp_path):
    dataset_path = tmp_path / "dataset.json"
    dataset_path.write_text(json.dumps(_tiny_dataset()), encoding="utf-8")

    out_dir = tmp_path / "plots"
    exit_code = main([str(dataset_path), "--out-dir", str(out_dir), "--label", "tiny"])

    assert exit_code == 0
    raw_png = out_dir / "tiny.fixed_anchor_raw_angles.svg"
    delta_png = out_dir / "tiny.fixed_anchor_angle_deltas.svg"
    assert raw_png.exists()
    assert delta_png.exists()
    assert raw_png.stat().st_size > 0
    assert delta_png.stat().st_size > 0
