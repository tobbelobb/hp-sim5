import numpy as np
import pytest

from autocal.ellipse_fitting import (
    EllipseFitResult,
    ellipse_geometric_params,
    fit_all_sweeps,
    fit_ellipse_from_sweep,
    fit_ellipse_maini_stefano,
)


def generate_ellipse_points(
    center: tuple,
    semi_axes: tuple,
    rotation: float,
    n_points: int,
    noise_std: float = 0,
) -> tuple:
    """Generate points on an ellipse with optional noise."""
    cx, cy = center
    a, b = semi_axes

    theta = np.linspace(0, 2 * np.pi, n_points, endpoint=False)

    x_ell = a * np.cos(theta)
    y_ell = b * np.sin(theta)

    cos_r = np.cos(rotation)
    sin_r = np.sin(rotation)
    x = cx + x_ell * cos_r - y_ell * sin_r
    y = cy + x_ell * sin_r + y_ell * cos_r

    if noise_std > 0:
        x += np.random.normal(0, noise_std, n_points)
        y += np.random.normal(0, noise_std, n_points)

    return x, y


class TestMainiStefanoFitting:
    def test_perfect_circle(self):
        """Fit a perfect circle (special case of ellipse)."""
        x, y = generate_ellipse_points(
            center=(100, 200),
            semi_axes=(50, 50),
            rotation=0,
            n_points=100,
        )

        coeffs = fit_ellipse_maini_stefano(x, y)
        center, semi_axes, rotation = ellipse_geometric_params(coeffs)

        assert abs(center[0] - 100) < 1e-6
        assert abs(center[1] - 200) < 1e-6
        assert abs(semi_axes[0] - 50) < 1e-6
        assert abs(semi_axes[1] - 50) < 1e-6

    def test_rotated_ellipse(self):
        """Fit a rotated ellipse."""
        x, y = generate_ellipse_points(
            center=(500, 800),
            semi_axes=(100, 60),
            rotation=np.pi / 6,  # 30 degrees
            n_points=100,
        )

        coeffs = fit_ellipse_maini_stefano(x, y)
        center, semi_axes, rotation = ellipse_geometric_params(coeffs)

        assert abs(center[0] - 500) < 1e-3
        assert abs(center[1] - 800) < 1e-3
        assert abs(semi_axes[0] - 100) < 1e-3
        assert abs(semi_axes[1] - 60) < 1e-3

    def test_noisy_data(self):
        """Fit ellipse with Gaussian noise."""
        np.random.seed(42)
        x, y = generate_ellipse_points(
            center=(1000, 2000),
            semi_axes=(200, 100),
            rotation=0.5,
            n_points=200,
            noise_std=5.0,
        )

        coeffs = fit_ellipse_maini_stefano(x, y)
        center, semi_axes, rotation = ellipse_geometric_params(coeffs)

        assert abs(center[0] - 1000) < 10
        assert abs(center[1] - 2000) < 10
        assert abs(semi_axes[0] - 200) < 10
        assert abs(semi_axes[1] - 100) < 10

    def test_minimum_points(self):
        """Fitting with exactly 5 points (minimum)."""
        x = np.array([0, 1, 0, -1, 0.5])
        y = np.array([1, 0, -1, 0, 0.866])

        coeffs = fit_ellipse_maini_stefano(x, y)
        assert len(coeffs) == 6

    def test_insufficient_points(self):
        """Fitting with fewer than 5 points should raise."""
        x = np.array([0, 1, 0, -1])
        y = np.array([1, 0, -1, 0])

        with pytest.raises(ValueError, match="At least 5 points"):
            fit_ellipse_maini_stefano(x, y)


class TestSweepFitting:
    def test_valid_sweep(self):
        """Fit from simulated sweep data."""
        phi = np.linspace(0, np.pi, 50)  # Half rotation

        K_d, M_d, N_d = 10000, 2000, 1500
        K_s, M_s, N_s = 12000, 1800, -1200

        L_drive_sq = K_d + M_d * np.cos(phi) + N_d * np.sin(phi)
        L_sensor_sq = K_s + M_s * np.cos(phi) + N_s * np.sin(phi)

        l_drive = np.sqrt(L_drive_sq)
        l_sensor = np.sqrt(L_sensor_sq)

        result = fit_ellipse_from_sweep(l_drive, l_sensor, residual_threshold=0.01)

        assert isinstance(result, EllipseFitResult)
        assert result.valid
        assert result.residual_rms < 0.01
        assert result.num_points == 50

    def test_insufficient_points_sweep(self):
        """Sweep with too few points should be rejected."""
        l_drive = np.array([100, 101, 102])
        l_sensor = np.array([95, 96, 97])

        result = fit_ellipse_from_sweep(l_drive, l_sensor, min_points=10)

        assert not result.valid
        assert "Insufficient points" in result.rejection_reason

    def test_high_noise_rejection(self):
        """Highly noisy data should be rejected."""
        np.random.seed(123)
        l_drive = np.random.uniform(80, 120, 50)
        l_sensor = np.random.uniform(90, 110, 50)

        result = fit_ellipse_from_sweep(l_drive, l_sensor, residual_threshold=0.001)

        assert not result.valid
        assert "RMS residual too high" in (result.rejection_reason or "")


class TestBatchFitting:
    def test_fit_all_sweeps(self):
        """Test batch fitting of multiple sweeps."""
        phi = np.linspace(0, np.pi, 30)
        dataset = {
            "sweeps": [
                {
                    "id": "sweep_001",
                    "fixed_anchors": [0],
                    "fixed_lengths": [500.0],
                    "drive_anchor": 1,
                    "sensor_anchor": 2,
                    "data_points": [
                        {
                            "l_drive": np.sqrt(10000 + 2000 * np.cos(p)),
                            "l_sensor": np.sqrt(12000 + 1800 * np.cos(p)),
                        }
                        for p in phi
                    ],
                },
                {
                    "id": "sweep_002",
                    "fixed_anchors": [1],
                    "fixed_lengths": [520.0],
                    "drive_anchor": 0,
                    "sensor_anchor": 2,
                    "data_points": [
                        {
                            "l_drive": np.sqrt(9000 + 1500 * np.cos(p)),
                            "l_sensor": np.sqrt(11000 + 2000 * np.cos(p)),
                        }
                        for p in phi
                    ],
                },
            ]
        }

        results = fit_all_sweeps(dataset["sweeps"], residual_threshold=0.01)

        assert len(results) == 2
        assert results[0]["sweep_id"] == "sweep_001"
        assert results[1]["sweep_id"] == "sweep_002"
        assert results[0]["sweep_config"]["drive_anchor"] == 1
        assert results[1]["sweep_config"]["fixed_anchors"] == [1]
