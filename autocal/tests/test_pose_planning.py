import numpy as np
import pytest
import autocal.planning_pass as planning

from autocal.active_learning import SweepConfig, generate_candidate_sweeps
from autocal.pose_planning import SWEEP_PRETENSION_SHARE, predict_sweep_positions, rank_candidates_pose_information

ANCHORS = np.array([[0.0, -1900.0, -280.0], [1645.4, 950.0, -280.0], [-1645.4, 950.0, -280.0], [0.0, 0.0, 1900.0]])
HOME = np.linalg.norm(ANCHORS, axis=1)


def _lengths(configs):
    positions = np.concatenate([predict_sweep_positions(ANCHORS, cfg, 20, SWEEP_PRETENSION_SHARE) for cfg in configs])
    return np.linalg.norm(ANCHORS[None, :, :] - positions[:, None, :], axis=2) - HOME


def test_predicted_positions_keep_the_fixed_lengths_and_stay_between_the_anchors():
    cfg = SweepConfig((0, 3), (390.0, -150.0), 1, 2)
    positions = predict_sweep_positions(ANCHORS, cfg, 20, SWEEP_PRETENSION_SHARE)
    assert positions.shape == (20, 3)
    lengths = np.linalg.norm(ANCHORS[None, :, :] - positions[:, None, :], axis=2) - HOME
    assert np.allclose(lengths[:, 0], 390.0) and np.allclose(lengths[:, 3], -150.0)
    assert np.all(np.abs(positions[:, 0]) < 900.0) and np.all(positions[:, 2] > -280.0)
    # The stretch is symmetric about the plane through A and D, like the recorded sweeps.
    assert np.allclose(positions[0] * [-1.0, 1.0, 1.0], positions[-1], atol=30.0)


def test_unreachable_fixed_lengths_give_no_positions():
    assert predict_sweep_positions(ANCHORS, SweepConfig((0, 3), (-1500.0, -1500.0), 1, 2), 20, SWEEP_PRETENSION_SHARE).shape[0] == 0


def test_data_on_one_d_sphere_makes_a_shortened_d_line_the_best_next_sweep():
    observed = [SweepConfig((low, 3), (390.0, 0.0), *[a for a in range(3) if a != low]) for low in range(3)]
    candidates = generate_candidate_sweeps(
        num_anchors=4, dimensions=3, fixed_delta_values_mm=[-390.0, -195.0, 0.0, 195.0, 390.0],
        machine_type="hangprinter_3",
    )
    ranked = rank_candidates_pose_information(
        ANCHORS, machine_type="hangprinter_3", low_anchor_z=None, observed_lengths_mm=_lengths(observed),
        candidates=candidates, points_per_sweep=20, sigma_mm=0.05, regularization=1e-6, top_k=3,
    )
    assert len(ranked) == 3
    assert ranked[0][0] >= ranked[1][0] >= ranked[2][0]
    assert all(cfg.fixed_deltas_mm[cfg.fixed_anchors.index(3)] < 0.0 for _, cfg in ranked)


@pytest.mark.parametrize("equal_strain", [False, True])
def test_profiled_data_cannot_silently_collect_a_constant_force_sweep(tmp_path, monkeypatch, equal_strain):
    cfg = SweepConfig((0, 3), (390.0, -150.0), 1, 2)
    dataset = {"machine_type": "hangprinter_4", "num_anchors": 4, "dimensions": 3,
               "config": {}, "sweeps": [{"fixed_anchors": [0, 3], "fixed_lengths": [390., 0.],
                   "drive_anchor": 1, "sensor_anchor": 2, "data_points": [{"l_drive": 0., "l_sensor": 0.}],
                   "metadata": {"sensor_force_profile": True}}]}
    fit = {"dataset": dataset, "dataset_for_estimation": dataset, "anchors": ANCHORS,
           "machine_type": "hangprinter_4", "num_anchors": 4, "dimensions": 3,
           "spool_model_params": None, "sweep_configs_for_info": [], "info": np.eye(9),
           "pose_fit": {"robust_scale_mm": .02},
           "information_settings": {"fd_eps_mm": 1., "regularization": 1e-6},
           "collection_settings": {"find_radii_mode": "off", "find_buildup_mode": "off",
                                   "base_radii": None, "buildup_factor": None}}
    monkeypatch.setattr(planning, "generate_candidate_sweeps", lambda **kw: [cfg])
    monkeypatch.setattr(planning, "rank_candidates_pose_information", lambda *a, **kw: [(1., cfg)])
    plan = planning.plan_ellipse_sweep(fit, tmp_path / "sweeps.json", equal_strain=equal_strain)
    assert plan["collect_command"] is None
    assert plan["collection_error"]
