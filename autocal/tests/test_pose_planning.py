import numpy as np

from autocal.active_learning import SweepConfig, generate_candidate_sweeps
from autocal.pose_planning import predict_sweep_positions, rank_candidates_pose_information

ANCHORS = np.array([[0.0, -1900.0, -280.0], [1645.4, 950.0, -280.0], [-1645.4, 950.0, -280.0], [0.0, 0.0, 1900.0]])
HOME = np.linalg.norm(ANCHORS, axis=1)


def _lengths(configs):
    positions = np.concatenate([predict_sweep_positions(ANCHORS, cfg, 20) for cfg in configs])
    return np.linalg.norm(ANCHORS[None, :, :] - positions[:, None, :], axis=2) - HOME


def test_predicted_positions_keep_the_fixed_lengths_and_stay_between_the_anchors():
    cfg = SweepConfig((0, 3), (390.0, -150.0), 1, 2)
    positions = predict_sweep_positions(ANCHORS, cfg, 20)
    assert positions.shape == (20, 3)
    lengths = np.linalg.norm(ANCHORS[None, :, :] - positions[:, None, :], axis=2) - HOME
    assert np.allclose(lengths[:, 0], 390.0) and np.allclose(lengths[:, 3], -150.0)
    assert np.all(np.abs(positions[:, 0]) < 900.0) and np.all(positions[:, 2] > -280.0)
    # The stretch is symmetric about the plane through A and D, like the recorded sweeps.
    assert np.allclose(positions[0] * [-1.0, 1.0, 1.0], positions[-1], atol=30.0)


def test_unreachable_fixed_lengths_give_no_positions():
    assert predict_sweep_positions(ANCHORS, SweepConfig((0, 3), (-1500.0, -1500.0), 1, 2), 20).shape[0] == 0


def test_data_on_one_d_sphere_makes_a_shortened_d_line_the_best_next_sweep():
    observed = [SweepConfig((low, 3), (390.0, 0.0), *[a for a in range(3) if a != low]) for low in range(3)]
    candidates = generate_candidate_sweeps(
        num_anchors=4, dimensions=3, fixed_delta_values_mm=[-390.0, -195.0, 0.0, 195.0, 390.0],
        machine_type="hangprinter_3", max_total_fixed_delta_mm=390.0,
    )
    ranked = rank_candidates_pose_information(
        ANCHORS, machine_type="hangprinter_3", low_anchor_z=None, observed_lengths_mm=_lengths(observed),
        candidates=candidates, points_per_sweep=20, sigma_mm=0.05, regularization=1e-6, top_k=3,
    )
    assert len(ranked) == 3
    assert ranked[0][0] >= ranked[1][0] >= ranked[2][0]
    assert all(cfg.fixed_deltas_mm[cfg.fixed_anchors.index(3)] < 0.0 for _, cfg in ranked)
