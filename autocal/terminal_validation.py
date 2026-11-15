"""Compare frozen candidates on one common set of new, unfitted observations."""
import numpy as np

from autocal.predictive_validation import score_future_sweeps
from autocal.history_selection import rank_history_candidates, select_history_plan


def evaluate_terminal_candidates(candidates, validation_dataset, *, use_noise_mean=True,
                                 pointwise_residual_mode="sampson", sigma_source="auto"):
    sweeps = validation_dataset.get("sweeps", [])
    ids = {sweep.get("id") for sweep in sweeps}
    if not sweeps or None in ids or "" in ids or len(ids) != len(sweeps):
        raise ValueError("Terminal validation requires distinct, named fresh sweeps")
    if any(ids.intersection(candidate.get("training_sweep_ids", ())) for candidate in candidates):
        raise ValueError("Terminal observations overlap a candidate's training data")
    scored = []
    for candidate in candidates:
        plan = candidate["plan"]
        prediction = score_future_sweeps(
            validation_dataset, np.asarray(plan["anchors"], dtype=float), (),
            spool_params=plan.get("spool_model_params"), home_offsets_mm=plan.get("home_offsets_mm"),
            use_noise_mean=use_noise_mean, pointwise_residual_mode=pointwise_residual_mode,
            sigma_source=sigma_source,
        )
        score = float(prediction[0]) if prediction is not None else float("inf")
        # One common prediction protocol; training rank only breaks exact ties.
        rank = candidate.get("rank_score", float("inf"))
        key = (score, float(rank) if rank is not None else float("inf"), -candidate.get("iteration", 0))
        scored.append((key, candidate, {
            "prediction_score": score, "prediction_sweeps": len(sweeps),
            "selection_score": score, "anchor_comparison_score": None,
        }))
    return scored


def select_terminal_plan(scored):
    ranked = rank_history_candidates(scored)
    if not ranked:
        return None, None, ranked
    # Apply an actually validated frozen model; do not form an unscored average.
    plan, meta, _ = select_history_plan([ranked[0]])
    meta["terminal_validation"] = True
    return plan, meta, ranked
