"""Reference shifts must preserve physical lengths and invert without truth."""
import numpy as np

from autocal.tools.rebase_dataset import rebase_dataset, restore_anchors
from autocal.theoretical_ellipse import canonicalize_anchor_gauge


def test_shift_preserves_lengths_and_restores_original_frame():
    anchors = np.array([[0., -1900., -280.], [1645.45, 950., -280.],
                        [-1645.45, 950., -280.], [0., 0., 1900.]])
    position = np.array([120., -170., 330.])
    offsets = np.linalg.norm(anchors - position, axis=1) - np.linalg.norm(anchors, axis=1)
    data = {"num_anchors": 4, "sweeps": [{"fixed_anchors": [0, 3], "fixed_lengths": [200., -50.],
        "drive_anchor": 1, "sensor_anchor": 2, "data_points": [
            {"l_drive": 100., "l_sensor": -80., "l_drive_mu": 99., "l_sensor_mu": -79.,
             "source_drive_anchor": 2, "source_sensor_anchor": 1, "drive_setpoint_mm": -80.,
             "raw_angles_deg": [1., 2., 3., 4.], "sigma": [0.1] * 4}]}]}
    rebased = rebase_dataset(data, offsets)
    point = rebased['sweeps'][0]['data_points'][0]
    np.testing.assert_allclose(point['l_drive'] + np.linalg.norm(anchors[1] - position),
                               100 + np.linalg.norm(anchors[1]))
    np.testing.assert_allclose(point['drive_setpoint_mm'], -80 - offsets[2])
    assert point['raw_angles_deg'] == [1., 2., 3., 4.]
    assert data['sweeps'][0]['data_points'][0]['l_drive'] == 100.
    shifted = canonicalize_anchor_gauge('hangprinter_4', anchors - position)
    restored = restore_anchors(shifted, offsets)
    np.testing.assert_allclose(restored['anchors'], anchors, atol=1e-8)
    assert restored['closure_rms_mm'] < 1e-8


def test_noisy_reference_reports_nonzero_closure():
    anchors = np.array([[0., -1900., -280.], [1645.45, 950., -280.],
                        [-1645.45, 950., -280.], [0., 0., 1900.]])
    restored = restore_anchors(anchors, [1., 1., 1., 1.])
    assert restored['closure_rms_mm'] > 0.5


def test_exact_translation_preserves_model_residuals():
    from autocal.ellipse_cost import EllipseCostFunction

    anchors = np.array([[0., -1900., -280.], [1645.45, 950., -280.],
                        [-1645.45, 950., -280.], [0., 0., 1900.]])
    position = np.array([120., -170., 330.])
    offsets = np.linalg.norm(anchors - position, axis=1) - np.linalg.norm(anchors, axis=1)
    data = {"machine_type": "hangprinter_4", "num_anchors": 4, "dimensions": 3, "sweeps": []}
    for fixed in range(3):
        drive, sensor = [a for a in range(3) if a != fixed]
        data["sweeps"].append({"id": str(fixed), "fixed_anchors": [fixed, 3],
            "fixed_lengths": [100., 0.], "drive_anchor": drive, "sensor_anchor": sensor,
            "data_points": [{"l_drive": float(x), "l_sensor": float(80 * np.cos(x / 200))}
                            for x in np.linspace(-200, 200, 20)]})

    def residuals(dataset, a):
        cost = EllipseCostFunction(dataset, use_flex=False, pointwise_filtering=False,
                                   sweep_wise_filtering=False)
        return np.array([r["residual_z_signed"] for r in cost.pointwise_residual_rows(a.ravel())])

    shifted = canonicalize_anchor_gauge('hangprinter_4', anchors - position)
    original = residuals(data, anchors)
    assert len(original) == 60
    np.testing.assert_allclose(residuals(rebase_dataset(data, offsets), shifted), original,
                               rtol=1e-8, atol=1e-8)
