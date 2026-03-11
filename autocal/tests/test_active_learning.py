import numpy as np

from autocal.active_learning import (
    SweepConfig,
    generate_candidate_sweeps,
    l2_scale_for_machine,
    rank_candidates_d_optimal,
    sweep_information_matrix,
)


def test_sweep_information_matrix_shape_and_symmetry():
    anchors = np.array(
        [
            [0.0, -1000.0],
            [866.0, 500.0],
            [-866.0, 500.0],
        ],
        dtype=float,
    )
    cfg = SweepConfig(fixed_anchors=(0,), fixed_deltas_mm=(200.0,), drive_anchor=1, sensor_anchor=2)
    l2_scale = l2_scale_for_machine("slideprinter", num_anchors=3, dimensions=2)
    info = sweep_information_matrix(
        anchors,
        cfg,
        machine_type="slideprinter",
        num_anchors=3,
        dimensions=2,
        l2_scale=l2_scale,
        fd_eps_mm=1.0,
    )
    assert info is not None
    assert info.shape == (6, 6)
    assert np.allclose(info, info.T, atol=1e-9)


def test_rank_candidates_skips_existing_when_requested():
    anchors = np.array(
        [
            [0.0, -1000.0],
            [866.0, 500.0],
            [-866.0, 500.0],
        ],
        dtype=float,
    )
    observed = [SweepConfig(fixed_anchors=(0,), fixed_deltas_mm=(0.0,), drive_anchor=1, sensor_anchor=2)]
    candidates = [
        SweepConfig(fixed_anchors=(0,), fixed_deltas_mm=(0.0,), drive_anchor=1, sensor_anchor=2),
        SweepConfig(fixed_anchors=(0,), fixed_deltas_mm=(300.0,), drive_anchor=1, sensor_anchor=2),
    ]
    l2_scale = l2_scale_for_machine("slideprinter", num_anchors=3, dimensions=2)
    ranked = rank_candidates_d_optimal(
        anchors,
        machine_type="slideprinter",
        num_anchors=3,
        dimensions=2,
        l2_scale=l2_scale,
        observed=observed,
        candidates=candidates,
        exclude_existing=True,
        top_k=10,
    )
    assert ranked
    assert ranked[0][1].fixed_deltas_mm != (0.0,)


def test_generate_candidate_sweeps_slideprinter_count():
    cands = generate_candidate_sweeps(num_anchors=3, dimensions=2, fixed_delta_values_mm=[-10.0, 10.0])
    # Slideprinter: choose 1 fixed anchor (3 choices), remaining 2 anchors form a single pair -> 3 * 2 = 6.
    assert len(cands) == 6


def test_generate_candidate_sweeps_respects_hangprinter_4_fixed_anchor_rules():
    cands = generate_candidate_sweeps(
        num_anchors=4,
        dimensions=3,
        fixed_delta_values_mm=[-10.0, 0.0, 10.0],
        machine_type="hangprinter_4",
    )
    assert cands
    assert len(cands) == 18
    assert all(3 in cfg.fixed_anchors for cfg in cands)
    assert all(cfg.drive_anchor != 3 for cfg in cands)
    assert all(cfg.sensor_anchor != 3 for cfg in cands)
    assert all(len(cfg.fixed_anchors) == 2 for cfg in cands)
    assert all(dict(zip(cfg.fixed_anchors, cfg.fixed_deltas_mm))[3] <= 0.0 for cfg in cands)
    assert any(
        any(delta_mm > 0.0 for anchor_idx, delta_mm in zip(cfg.fixed_anchors, cfg.fixed_deltas_mm) if anchor_idx != 3)
        for cfg in cands
    )
