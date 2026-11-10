"""History selection: geometric consensus of fitted candidates."""
import numpy as np

from autocal._autocal_common import _m669_from_plan
from autocal.history_selection import (
    SPREAD_STOP_FRAC,
    apply_consensus_plan,
    consensus_anchor_estimate,
    consensus_spread_hit,
    rank_history_candidates,
    select_history_plan,
)


def _item(iteration, anchors, *, prediction=1.0, sort_key=None):
    plan = {"anchors": None if anchors is None else np.asarray(anchors, dtype=float)}
    candidate = {"iteration": iteration, "run_id": "default", "plan": plan}
    info = {"prediction_score": prediction}
    return ((sort_key if sort_key is not None else float(iteration),), candidate, info)


def _anchors(offset):
    return [[0.0, -1900.0 + offset, -280.0], [1645.4, 950.0, -280.0],
            [-1645.4, 950.0, -280.0], [0.0, 0.0, 1900.0]]


def test_consensus_returns_nothing_without_anchor_geometry():
    assert consensus_anchor_estimate([]) == (None, (), None)
    assert consensus_anchor_estimate([_item(1, None)]) == (None, (), None)
    non_finite = _anchors(0.0)
    non_finite[0][1] = float("nan")
    assert consensus_anchor_estimate([_item(1, non_finite)]) == (None, (), None)


def test_consensus_passes_through_a_single_candidate():
    anchors, members, spread = consensus_anchor_estimate([_item(7, _anchors(0.0))])
    assert np.allclose(anchors, _anchors(0.0))
    assert members == (7,)
    assert spread == 0.0


def test_consensus_merges_restarts_that_converged_to_the_same_anchors():
    scored = [_item(2, _anchors(0.0)), _item(5, _anchors(0.005))]
    anchors, members, spread = consensus_anchor_estimate(scored)
    assert members == (2, 5)
    assert np.allclose(anchors, _anchors(0.0025))
    assert spread == 0.0


def test_consensus_keeps_a_pair_inside_the_spread_gate():
    anchors, members, spread = consensus_anchor_estimate(
        [_item(1, _anchors(0.0)), _item(9, _anchors(-4.0))]
    )
    assert members == (1, 9)
    assert np.allclose(anchors, _anchors(-2.0))
    assert np.isclose(spread, 4.0)


def test_consensus_refuses_a_pair_separated_wider_than_the_spread_gate():
    # Two clusters 6 mm apart are two hypotheses, not one tight fit, and their
    # midpoint can land further from the truth than either member. Report the
    # separation and apply nothing so the caller keeps the best-ranked fit.
    scored = [_item(1, _anchors(0.0)), _item(2, _anchors(0.0)), _item(8, _anchors(-6.0))]
    assert consensus_anchor_estimate(scored) == (None, (), 6.0)
    # An experiment that wants the raw midpoint can still lift the gate.
    anchors, members, spread = consensus_anchor_estimate(scored, max_spread_mm=6.0)
    assert members == (1, 2, 8)
    assert np.allclose(anchors, _anchors(-3.0))
    assert np.isclose(spread, 6.0)


def test_consensus_averages_the_nearest_distinct_pair_not_the_ranking():
    # The best-ranked candidate (iteration 3) is the outlier, so consensus must
    # ignore the ranking and average the two mutually consistent fits.
    scored = [
        _item(3, _anchors(40.0), prediction=0.0),
        _item(4, _anchors(0.0), prediction=9.0),
        _item(6, _anchors(-2.0), prediction=9.0),
    ]
    anchors, members, spread = consensus_anchor_estimate(scored)
    assert members == (4, 6)
    assert np.allclose(anchors, _anchors(-1.0))
    assert np.isclose(spread, 2.0)
    assert rank_history_candidates(scored)[0][1]["iteration"] == 3


def test_consensus_skips_candidates_with_a_different_anchor_shape():
    scored = [_item(1, _anchors(0.0)), _item(2, [[0.0, 0.0, 0.0]]), _item(3, _anchors(-4.0))]
    anchors, members, spread = consensus_anchor_estimate(scored)
    assert members == (1, 3)
    assert np.allclose(anchors, _anchors(-2.0))
    assert np.isclose(spread, 4.0)


def test_apply_consensus_plan_rebuilds_geometry_instead_of_reusing_cached_gcode():
    plan = {
        "anchors": np.asarray(_anchors(0.0), dtype=float),
        "machine_type": "hangprinter_4",
        "calibration": {"gcode": "M669 A9.99:9.99:9.99"},
    }
    applied = apply_consensus_plan(plan, _anchors(-1.0))
    assert "calibration" not in applied
    assert np.allclose(applied["anchors"], _anchors(-1.0))
    assert _m669_from_plan(applied).startswith("M669 A0.00:-1901.00:-280.00")
    # The best-ranked plan itself is left untouched.
    assert _m669_from_plan(plan).startswith("M669 A9.99:9.99:9.99")


def test_apply_consensus_plan_without_anchors_keeps_the_best_ranked_plan():
    plan = {"anchors": np.asarray(_anchors(0.0), dtype=float), "machine_type": "hangprinter_4"}
    applied = apply_consensus_plan(plan, None)
    assert np.allclose(applied["anchors"], _anchors(0.0))
    assert applied is not plan


def test_spread_hit_needs_two_distinct_fits_and_stays_scale_relative():
    truth = _anchors(0.0)
    # A single fit has nothing to disagree with. The loop also requires four
    # candidates, but identical fits must not satisfy the distinct-fit test.
    anchors, _members, spread = consensus_anchor_estimate([_item(1, truth)])
    assert consensus_spread_hit(anchors, spread)[0] is False
    anchors, _members, spread = consensus_anchor_estimate(
        [_item(index, truth) for index in range(1, 5)]
    )
    assert spread == 0.0
    assert consensus_spread_hit(anchors, spread)[0] is False
    assert consensus_spread_hit(None, None) == (False, None, None)
    assert consensus_spread_hit(truth, None) == (False, None, None)
    # Two fits 1 mm apart on a 3.3 m machine: far below the default fraction.
    anchors, _members, spread = consensus_anchor_estimate(
        [_item(1, truth), _item(2, _anchors(1.0))]
    )
    def extent(array):
        array = np.asarray(array, dtype=float)
        return float(np.sqrt(((array[:, None] - array[None, :]) ** 2).sum(-1)).max())

    hit, spread_mm, limit_mm = consensus_spread_hit(anchors, spread)
    assert hit is True
    assert np.isclose(spread_mm, 1.0)
    assert np.isclose(limit_mm, SPREAD_STOP_FRAC * extent(anchors))
    # The same absolute 1 mm disagreement on a ten-times smaller machine is not
    # agreement: the tolerance is relative to the machine, not absolute.
    small = (np.asarray(truth, dtype=float) * 0.1).tolist()
    small_offset = (np.asarray(truth, dtype=float) * 0.1).tolist()
    small_offset[0][1] += 1.0
    anchors, _members, spread = consensus_anchor_estimate(
        [_item(1, small), _item(2, small_offset)]
    )
    hit, _spread_mm, limit_mm = consensus_spread_hit(anchors, spread)
    assert hit is False
    assert np.isclose(limit_mm, SPREAD_STOP_FRAC * extent(anchors))


def test_spread_hit_rejects_degenerate_geometry():
    flat = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    assert consensus_spread_hit(flat, 0.0) == (False, 0.0, None)
    assert consensus_spread_hit([[1.0, 2.0]], 0.0)[0] is False
    nan = _anchors(0.0)
    nan[1][0] = float("nan")
    assert consensus_spread_hit(nan, 0.1)[0] is False
    assert consensus_spread_hit(_anchors(0.0), float("nan"))[0] is False


def test_spread_hit_fraction_can_be_tightened_or_loosened():
    truth = _anchors(0.0)
    anchors, _members, spread = consensus_anchor_estimate(
        [_item(1, truth), _item(2, _anchors(8.0))], max_spread_mm=10.0
    )
    # 8 mm separation is loose for the default and tight for a 1% tolerance.
    assert consensus_spread_hit(anchors, spread)[0] is False
    assert consensus_spread_hit(anchors, spread, frac=1e-2)[0] is True


def test_selected_plan_keeps_single_fit_scores_and_all_candidate_consensus():
    best = _item(1, _anchors(0.0), prediction=0.1)
    newest = _item(2, _anchors(2.0), prediction=None)
    best[1]['summary_meta'] = {'score_ui': 7.0}
    best[1]['plan']['calibration'] = {'anchors': 'cached single fit'}
    best[2].update(anchor_comparison_score=0.2, prediction_sweeps=3.0)

    plan, meta, ranked = select_history_plan([best, newest])

    assert ranked == [best]
    np.testing.assert_allclose(plan['anchors'], _anchors(1.0))
    assert 'calibration' not in plan
    assert meta['score_ui'] == 7.0
    assert meta['heldout_prediction_score'] == 0.1
    assert meta['anchor_comparison_score'] == 0.2
    assert meta['history_consensus_members'] == [1, 2]
    assert meta['history_consensus_spread_mm'] == 2.0
    # Summary construction must leave the saved candidates intact for replay.
    assert best[1]['summary_meta'] == {'score_ui': 7.0}
    assert 'calibration' in best[1]['plan']
    np.testing.assert_allclose(best[1]['plan']['anchors'], _anchors(0.0))


def test_selected_plan_preserves_prediction_veto_and_no_holdout_fallback():
    assert select_history_plan([]) == (None, None, [])
    failed = _item(1, _anchors(0.0), prediction=float('inf'))
    unvalidated = _item(2, _anchors(1.0), prediction=None)
    assert select_history_plan([failed, unvalidated]) == (None, None, [])
    plan, meta, ranked = select_history_plan([unvalidated])
    np.testing.assert_allclose(plan['anchors'], _anchors(1.0))
    assert meta['history_consensus_members'] == [2]
    assert meta['heldout_prediction_score'] is None
    assert ranked == [unvalidated]


def test_final_heldout_selection_is_distinct_from_running_training_best():
    training_best = _item(2, _anchors(20.0), prediction=5.0, sort_key=5.0)
    validated_best = _item(4, _anchors(0.0), prediction=1.0, sort_key=1.0)
    training_best[1]["rank_score"] = 1.0
    validated_best[1]["rank_score"] = 2.0
    # Even incomplete/stale summary metadata must identify the final candidate.
    validated_best[1]["summary_meta"] = {"iteration": 2, "run_id": "old"}
    plan, meta, ranked = select_history_plan([training_best, validated_best])
    assert ranked[0][1]["iteration"] == 4
    assert meta["iteration"] == 4
    assert meta["run_id"] == "default"
    assert meta["history_consensus_members"] == []
    np.testing.assert_array_equal(plan["anchors"], _anchors(0.0))
