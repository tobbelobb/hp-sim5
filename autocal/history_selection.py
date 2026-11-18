"""Held-out selection for spool-fitting runs, without refitting.

With radii or buildup fitted, adding sweeps does not reliably improve the fit: on
the replayed legacy datasets per-iteration anchor error moved like 1.5, 10, 17.5,
5.8, 27 mm. Applying the newest fit there summed to ~380 mm more true error over
12 datasets than this selection, so spool runs keep it. Runs without spool fitting
apply the newest fit instead (see autocal.anchor_uncertainty).
"""
from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import numpy as np

from autocal._autocal_common import _float_or_none
from autocal.predictive_validation import score_future_sweeps


def evaluate_history_candidates(history_candidates, *, use_noise_mean=True,
                                pointwise_residual_mode="sampson", sigma_source="auto",
                                log_line=lambda _line: None):
    """Score each saved model, frozen, on the sweeps that arrived after its fit."""
    def newest(key):
        return next((c["plan"][key] for c in reversed(history_candidates)
                     if isinstance(c["plan"].get(key), dict)), None)
    validation_dataset = newest("dataset")
    comparison_dataset = newest("dataset_for_estimation")
    scored: List[Tuple[Tuple[float, ...], Dict[str, object], Dict[str, Optional[float]]]] = []
    for candidate in history_candidates:
        plan = candidate["plan"]
        anchors = np.asarray(plan["anchors"], dtype=float)
        training = candidate.get("training_sweep_ids", ())
        settings = candidate.get("settings") or {}
        prediction = comparison = None
        if validation_dataset is not None:
            prediction = score_future_sweeps(
                validation_dataset, anchors, training, spool_params=plan.get("spool_model_params"),
                use_noise_mean=bool(settings.get("use_noise_mean", use_noise_mean)),
                pointwise_residual_mode=str(settings.get("pointwise_residual_mode", pointwise_residual_mode)),
                sigma_source=str(settings.get("sigma_source", sigma_source)),
            )
            if prediction is not None and not np.isfinite(prediction[0]):
                log_line(f"; history_validate: iter={candidate.get('iteration')} "
                         f"run={candidate.get('run_id')} heldout_sweeps={prediction[1]} failed_prediction=True")
        if comparison_dataset is not None:
            # Compare anchors under one common (newest) length model. A lower own-model
            # residual can instead reflect radius compensation; it only breaks ties.
            comparison = score_future_sweeps(comparison_dataset, anchors, training)
        cost = _float_or_none(candidate.get("cost"))
        sort_key = (
            float(comparison[0]) if comparison is not None else float("inf"),
            float(prediction[0]) if prediction is not None else float("inf"),
            float(cost) if cost is not None else float("inf"),
            -float(candidate.get("iteration", 0)),
        )
        scored.append((sort_key, candidate, {
            "prediction_score": prediction[0] if prediction else None,
            "prediction_sweeps": float(prediction[1]) if prediction else None,
            "anchor_comparison_score": comparison[0] if comparison else None,
        }))
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

    The applied anchors are the geometric consensus of the fitted candidates; the
    metadata describes the best-ranked single fit, not the mean geometry.

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
        "iteration": chosen.get("iteration"),
        "run_id": chosen.get("run_id"),
        "heldout_prediction_score": info.get("prediction_score"),
        "heldout_prediction_sweeps": info.get("prediction_sweeps"),
        "anchor_comparison_score": info.get("anchor_comparison_score"),
        "history_consensus_members": list(members),
        "history_consensus_spread_mm": spread,
    })
    return plan, meta, ranked

