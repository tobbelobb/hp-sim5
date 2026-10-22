"""Frozen prediction evaluation and pure history ordering, without refitting."""
from __future__ import annotations

from autocal._autocal_common import _arg_has_flag, _float_or_none, _full_auto_history_selection_score
from autocal.predictive_validation import score_future_sweeps
from typing import Dict, List, Optional, Tuple
import numpy as np


def evaluate_history_candidates(
    history_candidates, *, collector_args=(), use_noise_mean=True,
    pointwise_residual_mode="sampson", sigma_source="auto", log_line=lambda _line: None,
):
    """Evaluate saved models on future sweeps, keeping every model frozen."""
    validation_dataset = next(
        (
            candidate["plan"].get("dataset")
            for candidate in reversed(history_candidates)
            if isinstance(candidate.get("plan"), dict)
            and isinstance(candidate["plan"].get("dataset"), dict)
        ),
        None,
    )
    comparison_dataset = next(
        (
            candidate["plan"].get("dataset_for_estimation")
            for candidate in reversed(history_candidates)
            if isinstance(candidate.get("plan"), dict)
            and isinstance(candidate["plan"].get("dataset_for_estimation"), dict)
        ),
        None,
    )
    scored: List[
        Tuple[
            Tuple[float, ...],
            Dict[str, object],
            Dict[str, Optional[float]],
        ]
    ] = []
    for candidate in history_candidates:
        candidate_rank = _float_or_none(candidate.get("rank_score"))
        selection_score, selection_info = _full_auto_history_selection_score(
            candidate_rank,
            iteration_index=int(candidate.get("iteration", 1)),
            coverage_adjust=_float_or_none(candidate.get("rank_coverage_adjust")),
        )
        rel_std = _float_or_none(candidate.get("rel_std"))
        cost = _float_or_none(candidate.get("cost"))
        prediction = None
        comparison = None
        candidate_plan = candidate.get("plan")
        if isinstance(validation_dataset, dict) and isinstance(candidate_plan, dict):
            settings = candidate.get("settings") or {}
            prediction = score_future_sweeps(
                validation_dataset,
                np.asarray(candidate_plan.get("anchors"), dtype=float),
                candidate.get("training_sweep_ids", ()),
                spool_params=candidate_plan.get("spool_model_params"),
                prefer_zero_tension_angles=_arg_has_flag(
                    collector_args, "--project-zero-tension"
                ),
                use_noise_mean=bool(settings.get("use_noise_mean", use_noise_mean)),
                pointwise_residual_mode=str(
                    settings.get("pointwise_residual_mode", pointwise_residual_mode)
                ),
                sigma_source=str(settings.get("sigma_source", sigma_source)),
            )
            if prediction is not None and not np.isfinite(prediction[0]):
                log_line(
                    f"; history_validate: iter={candidate.get('iteration')} "
                    f"run={candidate.get('run_id')} heldout_sweeps={prediction[1]} "
                    "failed_prediction=True"
                )
            if isinstance(comparison_dataset, dict):
                # Keep the common length model for comparing anchors.
                # A lower own-model residual can instead reflect radius/
                # elasticity compensation; it is a veto and tie-breaker.
                comparison = score_future_sweeps(
                    comparison_dataset,
                    np.asarray(candidate_plan.get("anchors"), dtype=float),
                    candidate.get("training_sweep_ids", ()),
                )
        sort_key = (
            float(comparison[0]) if comparison is not None else float("inf"),
            float(prediction[0]) if prediction is not None else float("inf"),
            float(selection_score),
            float(candidate_rank) if candidate_rank is not None else float("inf"),
            float(rel_std) if rel_std is not None else float("inf"),
            float(cost) if cost is not None else float("inf"),
            -float(candidate.get("iteration", 0)),
        )
        selection_info = dict(selection_info)
        selection_info["prediction_score"] = prediction[0] if prediction else None
        selection_info["prediction_sweeps"] = float(prediction[1]) if prediction else None
        selection_info["anchor_comparison_score"] = comparison[0] if comparison else None
        scored.append((sort_key, candidate, selection_info))

    return scored


def rank_history_candidates(scored):
    """Order evaluated candidates; an empty list means all predictions failed.

    This inexpensive step is separate so ranking hypotheses need neither
    optimization nor prediction evaluation. No held-out data falls back to the
    same history ordering used by the full-auto loop.
    """
    validated = [item for item in scored
                 if item[2].get("prediction_score") is not None
                 and np.isfinite(item[2]["prediction_score"])]
    if validated:
        scored = validated
    elif any(item[2].get("prediction_score") is not None for item in scored):
        return []
    return sorted(scored, key=lambda item: item[0])


HISTORY_CONSENSUS_MERGE_MM = 0.01
# Provisional guard from an 11-dataset replay: refusing the two pairs wider
# than this lowered summed anchor error by 11.6 mm. A separate stop-chain replay
# was mixed (+0.8 mm net), so re-derive this threshold on a larger corpus.
HISTORY_CONSENSUS_MAX_SPREAD_MM = 5.0


def consensus_anchor_estimate(scored, *, max_spread_mm=HISTORY_CONSENSUS_MAX_SPREAD_MM):
    """Most mutually-consistent pair of fitted anchor sets, averaged.

    The held-out ordering reduces every candidate to one noisy scalar measured
    on a different, nested fold, so it ranks candidates only weakly: it can
    prefer an anchor set that is ~2 mm worse than another selectable one, and
    the newest iteration is structurally unselectable because no sweep is left
    to predict. Candidate anchor sets do carry the signal the scalar loses,
    because fits dragged off by model mismatch or a divergent local optimum
    scatter while fits near the true geometry cluster.

    Two restarts that converge to the same anchors are one hypothesis, not two,
    so candidates closer than `HISTORY_CONSENSUS_MERGE_MM` are merged first.
    The closest remaining pair is then the tightest available consensus and its
    mean is taken; averaging a pair can move the estimate by at most half their
    separation, so it stays bracketed by real fits.

    `scored` items may be the raw candidate dicts kept by the full-auto loop or
    the `(sort_key, candidate, info)` tuples of `evaluate_history_candidates`.

    A pair separated by more than `max_spread_mm` is ambiguous rather than
    tight, so no consensus is returned and the caller keeps the best-ranked
    candidate.

    Returns `(anchors, member_iterations, spread_mm)`. `anchors` is None when
    no candidate carries a usable anchor array, and also when the closest pair
    is wider than `max_spread_mm`; `spread_mm` still reports that separation.
    """
    rows = []
    shape = None
    for item in scored:
        # Items are either the raw candidate dicts kept by the full-auto loop or
        # the `(sort_key, candidate, info)` tuples built by
        # `evaluate_history_candidates`; only the candidate payload is read.
        candidate = item if isinstance(item, dict) else item[1]
        plan = candidate.get("plan")
        anchors = plan.get("anchors") if isinstance(plan, dict) else None
        if anchors is None:
            continue
        array = np.asarray(anchors, dtype=float)
        if not np.all(np.isfinite(array)):
            continue
        if shape is None:
            shape = array.shape
        elif array.shape != shape:
            continue
        rows.append((int(candidate.get("iteration", 0)), array))
    if not rows:
        return None, (), None
    if len(rows) == 1:
        return rows[0][1], (rows[0][0],), 0.0

    matrix = np.stack([array.reshape(-1) for _iteration, array in rows])
    pairwise = np.sqrt(
        np.maximum(((matrix[:, None, :] - matrix[None, :, :]) ** 2).sum(-1), 0.0)
    )
    parent = list(range(len(rows)))

    def find(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    for first in range(len(rows)):
        for second in range(first + 1, len(rows)):
            if pairwise[first, second] <= HISTORY_CONSENSUS_MERGE_MM:
                parent[find(first)] = find(second)

    groups: Dict[int, List[int]] = {}
    for index in range(len(rows)):
        groups.setdefault(find(index), []).append(index)
    merged = [
        (
            [rows[index][0] for index in indices],
            np.mean([rows[index][1] for index in indices], axis=0),
        )
        for indices in groups.values()
    ]
    if len(merged) == 1:
        return merged[0][1], tuple(merged[0][0]), 0.0

    flat = np.stack([anchors.reshape(-1) for _iterations, anchors in merged])
    distances = np.sqrt(
        np.maximum(((flat[:, None, :] - flat[None, :, :]) ** 2).sum(-1), 0.0)
    )
    np.fill_diagonal(distances, np.inf)
    first, second = np.unravel_index(np.argmin(distances), distances.shape)
    spread = float(distances[first, second])
    if spread > max_spread_mm:
        return None, (), spread
    anchors = (merged[first][1] + merged[second][1]) / 2.0
    members = tuple(merged[first][0] + merged[second][0])
    return anchors, members, spread


def apply_consensus_plan(chosen_plan, anchors):
    """`chosen_plan` with its applied geometry replaced by the consensus `anchors`.

    `anchors` is None when no candidate carried usable anchor geometry; the
    best-ranked plan is then returned unchanged. A cached `calibration` block is
    dropped so M669/M666 are rebuilt from the new anchors rather than reusing the
    best-ranked candidate's geometry.
    """
    plan = dict(chosen_plan or {})
    if anchors is not None:
        plan["anchors"] = np.asarray(anchors, dtype=float)
        plan.pop("calibration", None)
    return plan


def select_history_plan(scored):
    """Return the applied plan, single-fit diagnostics and ordered candidates.

    Held-out predictions rank individual fits. Geometric consensus separately
    uses all fitted candidates, including the newest unvalidated fit. The
    reported score therefore describes the best-ranked single fit, not the
    averaged geometry. Keep the two policies together here so offline replay
    can reproduce the calibration actually applied by the loop.

    Failed held-out predictions veto application when no validated fit remains.
    """
    ranked = rank_history_candidates(scored)
    if not ranked:
        return None, None, ranked
    _key, chosen, info = ranked[0]
    anchors, members, spread = consensus_anchor_estimate(scored)
    plan = apply_consensus_plan(chosen.get("plan"), anchors)
    meta = dict(chosen.get("summary_meta") or {})
    meta.update({
        "history_iteration_adjust": info.get("iteration_adjust"),
        "history_coverage_adjust": info.get("coverage_adjust"),
        "history_selection_score": info.get("selection_score"),
        "heldout_prediction_score": info.get("prediction_score"),
        "heldout_prediction_sweeps": info.get("prediction_sweeps"),
        "anchor_comparison_score": info.get("anchor_comparison_score"),
        "history_consensus_members": list(members),
        "history_consensus_spread_mm": spread,
    })
    return plan, meta, ranked


SPREAD_STOP_FRAC = 1.5e-3
SPREAD_STOP_MIN_CANDIDATES = 4
SPREAD_STOP_CONSECUTIVE = 2


def consensus_spread_hit(consensus_anchors, consensus_spread, *, frac=SPREAD_STOP_FRAC):
    """Whether the closest two distinct fits agree to within `frac` of machine size.

    The shipped consensus averages the two mutually most consistent candidate
    anchor sets and reports their separation as `spread_mm`.  Once that
    separation is a small fraction of the applied geometry's own extent, the
    independent fits have converged: more sweeps move the applied estimate far
    less than the machine's own scale, so collecting them only spends move and
    settle time.  The test is fractional, so it transfers to a smaller machine.

    This is deliberately *not* a stability test on the running estimate.  A
    stable estimate is not evidence of a correct one: adding a candidate whose
    anchors sit inside the current consensus pair leaves the estimate unmoved
    while telling us nothing new.  `spread_mm` measures a distance *between*
    distinct fits, so it cannot fire while the best available consensus is
    still loose.

    Returns `(hit, spread_mm, limit_mm)`.  `hit` is False whenever the inputs
    cannot be evaluated, so this predicate alone can never stop a run; the
    caller additionally requires a minimum candidate count and a consecutive
    streak.
    """
    if consensus_anchors is None or consensus_spread is None:
        return False, None, None
    array = np.asarray(consensus_anchors, dtype=float)
    if array.ndim != 2 or len(array) < 2 or array.shape[1] < 1:
        return False, None, None
    if not np.all(np.isfinite(array)):
        return False, None, None
    pairwise = np.sqrt(
        np.maximum(((array[:, None, :] - array[None, :, :]) ** 2).sum(-1), 0.0)
    )
    extent = float(pairwise.max())
    spread = float(consensus_spread)
    if not np.isfinite(spread) or not np.isfinite(extent) or extent <= 0.0:
        return False, (spread if np.isfinite(spread) else None), None
    limit = float(frac) * extent
    # `consensus_anchor_estimate` reports zero spread when all candidates merge
    # into one hypothesis. That is not agreement between distinct fits.
    has_distinct_fits = spread > HISTORY_CONSENSUS_MERGE_MM
    return bool(has_distinct_fits and spread <= limit), spread, limit
