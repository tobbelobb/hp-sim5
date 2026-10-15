"""History selection: geometric consensus of fitted candidates."""
import numpy as np

from autocal._autocal_common import _m669_from_plan
from autocal.history_selection import (
    SPREAD_STOP_FRAC,
    apply_consensus_plan,
    consensus_anchor_estimate,
    consensus_spread_hit,
    rank_history_candidates,
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


def test_consensus_of_a_bimodal_field_reports_the_cluster_midpoint():
    # Two separated clusters are an ambiguous fit, not a wrong one: the applied
    # estimate is their midpoint and `spread_mm` states how far apart they were.
    anchors, members, spread = consensus_anchor_estimate(
        [_item(1, _anchors(0.0)), _item(2, _anchors(0.0)), _item(8, _anchors(-6.0))]
    )
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
        [_item(1, truth), _item(2, _anchors(8.0))]
    )
    # 8 mm separation is loose for the default and tight for a 1% tolerance.
    assert consensus_spread_hit(anchors, spread)[0] is False
    assert consensus_spread_hit(anchors, spread, frac=1e-2)[0] is True
