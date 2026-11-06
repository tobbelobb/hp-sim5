"""Resolve projected-point semantics without trusting inconsistent metadata."""
from __future__ import annotations

import math
from typing import Optional


def resolve_projection_mode(dataset: object, *, requested: bool = False) -> Optional[bool]:
    """Return the point mode, or None for an empty/uninspectable dataset.

    Legacy projected files have no mode marker, but every projected point has
    raw_angles_zero_tension_deg. Reject mixed records or a mode marker that
    contradicts the records rather than silently choosing one length source.
    """
    if not isinstance(dataset, dict):
        if requested:
            raise ValueError("Projected angle mode requires a dictionary dataset")
        return None
    config = dataset.get("config")
    model = config.get("noise_model") if isinstance(config, dict) else None
    declared = model.get("project_zero_tension") if isinstance(model, dict) else None
    if declared is not None and not isinstance(declared, bool):
        raise ValueError("project_zero_tension metadata must be boolean")
    sweeps = dataset.get("sweeps")
    points = []
    if isinstance(sweeps, list):
        for sweep in sweeps:
            if not isinstance(sweep, dict):
                continue
            raw_points = sweep.get("data_points")
            if isinstance(raw_points, list):
                points.extend(point for point in raw_points if isinstance(point, dict))
    point_modes = {"raw_angles_zero_tension_deg" in point for point in points}
    if len(point_modes) > 1:
        raise ValueError("Dataset contains mixed projected and nonprojected points")
    observed = next(iter(point_modes)) if point_modes else None
    if declared is not None and observed is not None and declared != observed:
        raise ValueError("Projection metadata contradicts point fields")
    if requested and (declared is False or observed is False):
        raise ValueError("Projection requested for nonprojected point records")
    projected = bool(requested or declared is True or observed is True)
    if projected and points:
        try:
            count = int(dataset.get("num_anchors", 0))
        except (TypeError, ValueError):
            count = 0
        if count <= 0:
            raise ValueError("Projected dataset missing num_anchors")
        for point in points:
            values = point.get("raw_angles_zero_tension_deg")
            if not isinstance(values, list) or len(values) < count:
                raise ValueError("raw_angles_zero_tension_deg missing or wrong length")
            try:
                valid = all(math.isfinite(float(value)) for value in values[:count])
            except (TypeError, ValueError):
                valid = False
            if not valid:
                raise ValueError("raw_angles_zero_tension_deg contains non-finite values")
    return projected if (requested or declared is not None or observed is not None) else None
