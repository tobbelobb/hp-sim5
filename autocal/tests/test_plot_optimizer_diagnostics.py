import json

import numpy as np

from scripts.plot_optimizer_diagnostics import (
    _json_ready,
    _sample_theoretical_raw_curve,
    _signed_fill_rgb,
    _svg_scatter_plot,
)


def test_signed_fill_rgb_uses_consistent_sign_convention():
    assert _signed_fill_rgb(10.0, limit=10.0) == "rgb(220,0,0)"
    assert _signed_fill_rgb(0.0, limit=10.0) == "rgb(220,220,220)"
    assert _signed_fill_rgb(-10.0, limit=10.0) == "rgb(0,0,220)"


def test_svg_scatter_plot_keeps_legend_labels_and_point_colors_aligned(tmp_path):
    out_path = tmp_path / "residual_plot.svg"
    rows = [
        {
            "sweep_id": "sweep_a",
            "point_idx": 0,
            "l_drive_mm": 100.0,
            "residual_mm_signed": 10.0,
            "residual_mm": 10.0,
        },
        {
            "sweep_id": "sweep_b",
            "point_idx": 1,
            "l_drive_mm": 200.0,
            "residual_mm_signed": -10.0,
            "residual_mm": 10.0,
        },
    ]

    _svg_scatter_plot(
        title="demo",
        rows=rows,
        x_key="l_drive_mm",
        y_key="residual_mm_signed",
        x_label="l_drive [mm]",
        y_label="signed residual [mm]",
        out_path=out_path,
        annotate_points=False,
    )

    text = out_path.read_text(encoding="utf-8")
    assert ">+10<" in text
    assert ">0<" in text
    assert ">-10<" in text
    assert 'fill="rgb(220,0,0)"' in text
    assert 'fill="rgb(0,0,220)"' in text


def test_json_ready_converts_numpy_scalars_and_arrays():
    payload = {
        "arr": np.asarray([1.0, 2.0]),
        "nested": {"value": np.float64(3.5), "flag": np.bool_(True)},
        "count": np.int64(7),
    }

    normalized = _json_ready(payload)
    encoded = json.dumps(normalized, sort_keys=True)

    assert encoded == '{"arr": [1.0, 2.0], "count": 7, "nested": {"flag": true, "value": 3.5}}'


def test_sample_theoretical_raw_curve_returns_finite_points_for_slideprinter_sweep():
    anchors = np.asarray(
        [[0.0, -1900.0], [1645.44826719, 950.0], [-1645.44826719, 950.0]],
        dtype=float,
    )
    sweep = {
        "id": "sweep_demo",
        "fixed_anchors": [0],
        "fixed_lengths": [0.0],
        "drive_anchor": 1,
        "sensor_anchor": 2,
    }

    curve = _sample_theoretical_raw_curve(
        anchors=anchors,
        sweep=sweep,
        dimensions=2,
        num_samples=64,
    )

    assert len(curve) >= 32
    assert all(np.isfinite(x) and np.isfinite(y) for x, y in curve)
    assert all(x > 0.0 and y > 0.0 for x, y in curve)
