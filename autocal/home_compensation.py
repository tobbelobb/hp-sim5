"""Recover the physical home only from independently measured line extensions.

The point fit chooses a translation gauge: at its virtual origin its geometric
ranges are |a| + k, where k is its fitted home-length offset.  At the physical
encoder zero, the corresponding geometric ranges are |a-q|.  If e_measure and
 e_home are extensions in the virtual measurement state and physical home state,
material-length conservation gives |a-q| = |a| + k - e_measure + e_home.
All extensions are effective carriage-distance extensions, in millimetres.
"""
from __future__ import annotations

import numpy as np


def compensate_home(anchors, offsets, evidence):
    """Return (corrected anchors, shift), or raise ValueError for unusable evidence.

    The evidence must be independently measured for these exact reference states;
    nominal firmware stiffness and commanded forces are not such measurements.
    """
    a = np.asarray(anchors, dtype=float)
    k = np.asarray(offsets, dtype=float)
    if a.ndim != 2 or a.shape[1] != 3 or a.shape[0] < 4 or k.shape != (a.shape[0],):
        raise ValueError("a 3D anchor fit and one fitted offset per anchor are required")
    if (not isinstance(evidence, dict) or not isinstance(evidence.get("provenance"), str)
            or not evidence["provenance"].strip()):
        raise ValueError("independent extension evidence and provenance are required")
    try:
        em = np.asarray(evidence["measurement_extension_mm"], dtype=float)
        eh = np.asarray(evidence["home_extension_mm"], dtype=float)
        uncertainty = float(evidence["range_uncertainty_mm"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("extension vectors and range uncertainty are required") from exc
    if (em.shape != k.shape or eh.shape != k.shape or not np.all(np.isfinite(a))
            or not np.all(np.isfinite(k)) or not np.all(np.isfinite(em))
            or not np.all(np.isfinite(eh)) or np.any(em < 0) or np.any(eh < 0)
            or not np.isfinite(uncertainty) or uncertainty <= 0):
        raise ValueError("extension evidence must be finite, nonnegative, and complete")
    # Firmware references the geometric origin to unstretched material length.
    # A loaded encoder zero needs an additional firmware reference correction.
    if np.any(eh > uncertainty):
        raise ValueError("loaded encoder home is not supported by the firmware reference")
    ranges = np.linalg.norm(a, axis=1) + k - em + eh
    if np.any(ranges <= 0):
        raise ValueError("invalid home ranges")
    # Differencing squared range equations removes the quadratic |q|² term.
    b = ranges**2 - np.sum(a*a, axis=1)
    matrix = -2 * (a[1:] - a[0])
    q, _, rank, _ = np.linalg.lstsq(matrix, b[1:] - b[0], rcond=None)
    if rank < 3:
        raise ValueError("home position is ambiguous for this anchor geometry")
    vectors = q - a
    distances = np.linalg.norm(vectors, axis=1)
    if np.any(distances == 0):
        raise ValueError("home coincides with an anchor")
    singular = np.linalg.svd(vectors / distances[:, None], compute_uv=False)
    if singular[-1] < 1e-6 or uncertainty / singular[-1] > 0.5:
        raise ValueError("home position is too uncertain for compensation")
    if np.max(np.abs(distances - ranges)) > 3 * uncertainty:
        raise ValueError("home extension evidence is inconsistent with the fitted geometry")
    return a - q, q
