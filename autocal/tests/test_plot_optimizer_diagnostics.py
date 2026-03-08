import json

import numpy as np

from scripts.plot_optimizer_diagnostics import (
    _parse_iteration_models_from_log_text,
    _parse_summary_model_from_log_text,
    _json_ready,
    _sample_theoretical_raw_curve,
    _signed_fill_rgb,
    _svg_per_sweep_residual_order_plot,
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


def test_parse_iteration_models_accepts_selected_run_without_console_score():
    text = """
; Anchors: [[0.0, -1900.0], [1645.4, 950.0], [-1645.4, 950.0]]
; line_model: planning=L_base_mm estimation=L_model_mm find_radii=global find_buildup_factor=off k=0.6366 base=[30,30,30] effective=[39.1,39.1,39.1]
; selected run=default fit_score_ui=0.4983 score_basis=layered-calibrated cost=111.6 rel_std=4.536 max_std=1.036e+04mm rank_score=4.427 iteration_adjust=-0.2544 coverage_adjust=-0.1532 history_rank_score=4.142
""".strip()

    items = _parse_iteration_models_from_log_text(text)

    assert len(items) == 1
    assert np.allclose(items[0]["anchors"], np.asarray([[0.0, -1900.0], [1645.4, 950.0], [-1645.4, 950.0]]))
    assert np.allclose(items[0]["radii"], np.asarray([39.1, 39.1, 39.1]))
    assert np.allclose(items[0]["buildup"], np.asarray([0.6366, 0.6366, 0.6366]))


def test_parse_summary_model_accepts_new_anchor_and_spool_labels():
    text = """
Wrote to console:
Wrote to console: == Calibration summary ==
Wrote to console: Found parameters of good quality
Wrote to console: Fit/UI quality score (lower is better): 4.037
Wrote to console: Anchors (M669): M669 A0.00:-1901.21:0.00 B1647.09:951.03:0.00 C-1647.44:950.90:0.00
Wrote to console: Spools (M666): M666 R39.13:39.13:39.13 Q0.636619
""".strip()

    parsed = _parse_summary_model_from_log_text(text)

    assert parsed is not None
    assert np.allclose(parsed["anchors"], np.asarray([[0.0, -1901.21], [1647.09, 951.03], [-1647.44, 950.90]]))
    assert np.allclose(parsed["radii"], np.asarray([39.13, 39.13, 39.13]))
    assert np.allclose(parsed["buildup"], np.asarray([0.636619, 0.636619, 0.636619]))
    assert parsed["fit_score_ui"] == 4.037


def test_svg_per_sweep_residual_order_plot_marks_midpoint_split(tmp_path):
    out_path = tmp_path / "sweep_demo.residual_order.svg"
    sweep = {"id": "sweep_demo"}
    rows = [
        {"point_idx": idx, "residual_mm_signed": resid, "residual_mm": abs(resid)}
        for idx, resid in enumerate([0.4, 0.2, -0.3, -0.5])
    ]

    _svg_per_sweep_residual_order_plot(
        label="demo",
        sweep=sweep,
        rows=rows,
        out_path=out_path,
        color_limit=1.0,
    )

    text = out_path.read_text(encoding="utf-8")
    assert "signed residual [mm]" in text
    assert "measured point order" in text
    assert "sub-sweep split" in text


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
