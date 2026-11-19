from __future__ import annotations

from typing import Dict, Iterable, Optional, Tuple

import numpy as np

from autocal.slideprinter_2d.ellipse_cost import EllipseCostFunction
from autocal.projection_mode import resolve_projection_mode
from autocal.spool_model import SpoolModelParams, dataset_with_modeled_lengths
from autocal.pose_fit import (shift_dataset_home_offsets,
                              dataset_line_lengths, dataset_stretch, consistency_residuals)


def score_future_sweeps(
    dataset: Dict[str, object], anchors: np.ndarray, training_sweep_ids: Iterable[str],
    *,
    spool_params: Optional[SpoolModelParams] = None,
    home_offsets_mm=None,
    prefer_zero_tension_angles: bool = False,
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
        projected = resolve_projection_mode(
            dataset, requested=bool(prefer_zero_tension_angles)
        ) is True
        if spool_params is not None:
            validation_data = dataset_with_modeled_lengths(
                validation_data, spool_params,
                prefer_zero_tension_angles=projected,
            )
        if home_offsets_mm is not None:
            validation_data = shift_dataset_home_offsets(validation_data, home_offsets_mm)
        if str(validation_data["machine_type"]).startswith("hangprinter_") and not projected:
            lengths = dataset_line_lengths(validation_data, use_noise_mean=use_noise_mean)["lengths_mm"]
            measured = np.isfinite(lengths)
            dimensions = int(validation_data["dimensions"])
            if not lengths.size or not np.all(measured.sum(axis=1) > dimensions):
                return float("inf"), len(held_out)
            residuals = consistency_residuals(anchors, lengths, np.zeros(int(validation_data["num_anchors"])),
                                             stretch=dataset_stretch(validation_data))
            if not np.all(np.isfinite(residuals)):
                return float("inf"), len(held_out)
            # Each pose consumes d nuisance coordinates. Score only its remaining
            # range consistency constraints, with offsets frozen from training.
            dof = int(measured.sum()) - dimensions * len(lengths)
            return float(np.sum(residuals ** 2) / dof), len(held_out)
        cost = EllipseCostFunction(
            validation_data,
            pointwise_filtering=False,
            sweep_wise_filtering=False,
            robust_loss=True,
            use_noise_mean=bool(use_noise_mean) and not projected,
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
