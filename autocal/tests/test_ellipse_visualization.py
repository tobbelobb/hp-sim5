import matplotlib

matplotlib.use("Agg")  # Non-interactive backend for testing

import matplotlib.pyplot as plt
import numpy as np
import pytest

from autocal.ellipse_visualization import (
    create_calibration_report,
    plot_anchors_2d,
    plot_anchors_3d,
    plot_cost_convergence,
    plot_ellipse_fit,
    plot_sampling_density,
    plot_sweep_data,
)


@pytest.fixture
def sample_sweep() -> dict:
    phi = np.linspace(0, np.pi, 50)
    l_drive = np.sqrt(10000 + 2000 * np.cos(phi))
    l_sensor = np.sqrt(12000 + 1800 * np.cos(phi))
    return {
        "id": "test_sweep",
        "fixed_anchors": [0],
        "fixed_lengths": [600.0],
        "drive_anchor": 1,
        "sensor_anchor": 2,
        "data_points": [{"l_drive": float(ld), "l_sensor": float(ls)} for ld, ls in zip(l_drive, l_sensor)],
    }


@pytest.fixture
def sample_fitted_ellipse(sample_sweep: dict) -> dict:
    # Simple non-degenerate coefficients for testing plot paths.
    return {
        "sweep_id": sample_sweep["id"],
        "coefficients": {
            "A": 0.6,
            "B": 0.1,
            "C": 0.8,
            "D": -100.0,
            "E": -120.0,
            "F": 5000.0,
        },
        "residual_rms": 0.005,
        "residual_max": 0.01,
        "residual_series": [0.0 for _ in sample_sweep["data_points"]],
        "valid": True,
        "sweep_config": {
            "fixed_anchors": sample_sweep["fixed_anchors"],
            "fixed_lengths": sample_sweep["fixed_lengths"],
            "drive_anchor": sample_sweep["drive_anchor"],
            "sensor_anchor": sample_sweep["sensor_anchor"],
        },
    }


class TestPlotting:
    def test_plot_sweep_data(self, sample_sweep):
        fig, ax = plt.subplots()
        result_ax = plot_sweep_data(sample_sweep, ax=ax)
        assert result_ax is ax
        plt.close(fig)

    def test_plot_ellipse_fit(self, sample_sweep, sample_fitted_ellipse):
        fig, ax = plt.subplots()
        result_ax = plot_ellipse_fit(sample_sweep, sample_fitted_ellipse, ax=ax)
        assert result_ax is ax
        plt.close(fig)

    def test_plot_sampling_density(self, sample_sweep, sample_fitted_ellipse):
        fig, ax = plt.subplots()
        result_ax = plot_sampling_density(sample_sweep, sample_fitted_ellipse, ax=ax)
        assert result_ax is ax
        plt.close(fig)

    def test_plot_cost_convergence(self):
        fig, ax = plt.subplots()
        history = [10.0, 5.0, 1.0]
        result_ax = plot_cost_convergence(history, ax=ax)
        assert result_ax is ax
        plt.close(fig)

    def test_plot_anchors_2d(self):
        anchors = np.array([[-500, 400], [500, 400], [0, -500]])
        fig, ax = plt.subplots()
        result_ax = plot_anchors_2d(anchors, ax=ax)
        assert result_ax is ax
        plt.close(fig)

    def test_plot_anchors_3d(self):
        anchors = np.array([[-500, 400, -100], [500, 400, -100], [0, -500, -100], [0, 0, 1000]])
        fig = plot_anchors_3d(anchors)
        assert fig is not None
        plt.close(fig)

    def test_create_calibration_report_smoke(self, tmp_path, sample_sweep, sample_fitted_ellipse):
        dataset = {"machine_type": "slideprinter", "num_anchors": 3, "dimensions": 2, "sweeps": [sample_sweep]}
        solution = {"anchors": np.array([[0, 0], [1, 0], [0, 1]]), "cost": 1.23e-3, "success": True}
        out = tmp_path / "report.png"
        fig = create_calibration_report(dataset, solution, ellipse_fits=[sample_fitted_ellipse], output_path=str(out))
        assert fig is not None
        assert out.exists()
        plt.close(fig)

