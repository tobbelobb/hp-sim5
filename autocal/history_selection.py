"""Frozen prediction evaluation and pure history ordering, without refitting."""
from __future__ import annotations

from autocal._autocal_common import _arg_has_flag, _float_or_none, _full_auto_history_selection_score
from autocal.predictive_validation import score_future_sweeps
from typing import Dict, List, Optional, Tuple
import numpy as np


def evaluate_history_candidates(
    history_candidates, *, collector_args=(), use_flex=False,
    spring_k_multiplier=1.0, use_noise_mean=True,
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
                use_flex=bool(settings.get("use_flex", use_flex)),
                spring_k_multiplier=float(
                    settings.get("spring_k_multiplier", spring_k_multiplier)
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
