from __future__ import annotations

from typing import Dict, Iterable, Optional, Tuple

import numpy as np

from autocal.ellipse_cost import EllipseCostFunction


def score_future_sweeps(
    dataset: Dict[str, object], anchors: np.ndarray, training_sweep_ids: Iterable[str]
) -> Optional[Tuple[float, int]]:
    """Score anchor predictions on whole sweeps absent from the candidate's fit."""
    sweeps = dataset.get("sweeps")
    if not isinstance(sweeps, list):
        return None
    training_ids = {str(sweep_id) for sweep_id in training_sweep_ids}
    held_out = [
        sweep
        for sweep in sweeps
        if isinstance(sweep, dict) and str(sweep.get("id", "")) not in training_ids
    ]
    if not held_out:
        return None

    validation_data = dict(dataset)
    validation_data["sweeps"] = held_out
    try:
        cost = EllipseCostFunction(
            validation_data,
            pointwise_filtering=False,
            sweep_wise_filtering=False,
            robust_loss=True,
        )
        # A validation fold may intentionally contain only one complete sweep.
        cost._min_sweeps_after_trim = 1
        value = float(cost.evaluate(np.asarray(anchors, dtype=float).reshape(-1)))
    except (TypeError, ValueError, FloatingPointError, np.linalg.LinAlgError):
        return None
    if not np.isfinite(value):
        return None
    return value, len(held_out)
