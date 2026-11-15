import numpy as np
import pytest

import autocal.rough_bootstrap as bootstrap
from autocal.active_learning import SweepConfig
from autocal.pose_planning import predict_sweep_positions, SWEEP_PRETENSION_SHARE

ANCHORS = np.array([[0., -1900., -280.], [1645.4, 950., -280.],
                    [-1645.4, 950., -280.], [0., 0., 1900.]])


def _sweep(cfg):
    positions = predict_sweep_positions(ANCHORS, cfg, 20, SWEEP_PRETENSION_SHARE)
    lengths = np.linalg.norm(ANCHORS[None] - positions[:, None], axis=2) - np.linalg.norm(ANCHORS, axis=1)
    return {"fixed_anchors": list(cfg.fixed_anchors), "fixed_lengths": list(cfg.fixed_deltas_mm),
            "drive_anchor": cfg.drive_anchor, "sensor_anchor": cfg.sensor_anchor,
            "data_points": [{"l_drive": row[cfg.drive_anchor], "l_sensor": row[cfg.sensor_anchor]} for row in lengths]}


def _dataset():
    return {"machine_type": "hangprinter_4", "num_anchors": 4, "dimensions": 3,
            "config": {"max_travel_mm": 390.0},
            "sweeps": [_sweep(SweepConfig((low, 3), (390., 0.), *[a for a in range(3) if a != low]))
                       for low in range(3)]}


def test_weak_bootstrap_is_rejected_despite_exact_residuals_and_improves_with_height(monkeypatch):
    monkeypatch.setattr(bootstrap, "build_anchor_initial_guess", lambda *a, **kw: ANCHORS * 1.05)
    dataset = _dataset()
    anchors, check = bootstrap.assess_rough_geometry(dataset)
    assert check["rms_mm"] < 1e-6
    assert not check["ready"]
    assert check["max_anchor_std_mm"] > bootstrap.MAX_ROUGH_ANCHOR_STD_MM
    cfg = bootstrap.next_rough_sweep(dataset, anchors)
    assert cfg.fixed_deltas_mm[cfg.fixed_anchors.index(3)] < 0.0
    dataset["sweeps"].append(_sweep(cfg))
    _, check = bootstrap.assess_rough_geometry(dataset)
    assert check["ready"]


def test_no_encoder_travel_cannot_silently_produce_a_force_profile():
    with pytest.raises(ValueError, match="raw encoder travel"):
        bootstrap.assess_rough_geometry(_dataset())


def test_repeating_one_sweep_does_not_hide_unobservable_geometry(monkeypatch):
    monkeypatch.setattr(bootstrap, "build_anchor_initial_guess", lambda *a, **kw: ANCHORS)
    dataset = _dataset()
    dataset["sweeps"] = [dataset["sweeps"][0]] * 6
    _, check = bootstrap.assess_rough_geometry(dataset)
    assert not check["ready"]
