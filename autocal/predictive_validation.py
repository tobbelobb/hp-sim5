from __future__ import annotations

from typing import Dict, Iterable, Optional, Tuple

import numpy as np

from autocal.ellipse_cost import EllipseCostFunction
from autocal.spool_model import SpoolModelParams, dataset_with_modeled_lengths


def score_future_sweeps(
    dataset: Dict[str, object], anchors: np.ndarray, training_sweep_ids: Iterable[str],
    *,
    spool_params: Optional[SpoolModelParams] = None,
    prefer_zero_tension_angles: bool = False,
    use_flex: bool = True,
    spring_k_multiplier: float = 1.0,
    use_noise_mean: bool = True,
    pointwise_residual_mode: str = "sampson",
    sigma_source: str = "auto",
) -> Optional[Tuple[float, int]]:
    """Score a frozen model on raw, whole sweeps absent from its fit.

    None means no validation movement is available; infinity means validation
    failed. Never rank a geometry that cannot predict all held-out observations
    by the fitting objective's finite underconstrained sentinel.
    """
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
        if spool_params is not None:
            validation_data = dataset_with_modeled_lengths(
                validation_data, spool_params,
                prefer_zero_tension_angles=prefer_zero_tension_angles,
            )
        cost = EllipseCostFunction(
            validation_data,
            pointwise_filtering=False,
            sweep_wise_filtering=False,
            robust_loss=True,
            use_flex=use_flex,
            spring_k_multiplier=spring_k_multiplier,
            use_noise_mean=use_noise_mean,
            pointwise_residual_mode=pointwise_residual_mode,
            sigma_source=sigma_source,
        )
        # A validation fold may intentionally contain only one complete sweep.
        cost._min_sweeps_after_trim = 1
        vector = np.asarray(anchors, dtype=float).reshape(-1)
        result = cost.evaluate_detailed(vector)
        rows = cost.pointwise_residual_rows(vector)
        expected = sum(len(sweep.get("data_points", [])) for sweep in held_out)
        if result.num_invalid_sweeps or not expected or len(rows) != expected:
            return float("inf"), len(held_out)
        value = float(result.total_cost)
    except (TypeError, ValueError, FloatingPointError, np.linalg.LinAlgError):
        return float("inf"), len(held_out)
    if not np.isfinite(value):
        return float("inf"), len(held_out)
    return value, len(held_out)
