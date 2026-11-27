"""Vectorized swivel-pulley line geometry matching firmware SwivelPulley.h.

A swivel pulley's fork turns about a fixed unit axis through the anchor entry point.
The line arrives along the axis, wraps the wheel of radius R, and leaves from a
tangent point (the exit) toward the effector. When R <= 0 the pulley reduces to a
point pinhole at the anchor entry.
"""
from __future__ import annotations

from typing import Optional, Tuple

import numpy as np


def swivel_geometry_from_m669(
    m669: Optional[dict],
    num_anchors: int,
) -> Optional[dict]:
    """Parse swivel pulley radii (R) and unit axes (V) from a parsed M669 dict.

    Returns None when no anchor has a positive swivel pulley radius.
    """
    if not isinstance(m669, dict) or num_anchors <= 0:
        return None
    raw_r = m669.get("R")
    raw_v = m669.get("V")
    if raw_r is None or raw_v is None:
        return None
    try:
        radii = np.broadcast_to(np.asarray(raw_r, dtype=float), (num_anchors,)).copy()
        axes_flat = np.asarray(raw_v, dtype=float).reshape(-1)
    except (TypeError, ValueError):
        return None
    if axes_flat.size != 3 * num_anchors or not np.all(np.isfinite(radii)) or not np.all(np.isfinite(axes_flat)):
        return None
    if not np.any(radii > 0.0):
        return None
    axes = axes_flat.reshape(num_anchors, 3)
    norms = np.linalg.norm(axes, axis=1)
    if np.any((radii > 0.0) & (norms <= 1e-9)):
        return None
    unit_axes = np.where((norms > 1e-9)[:, None], axes / np.maximum(norms, 1e-9)[:, None], 0.0)
    return {
        "radii_mm": np.maximum(radii, 0.0),
        "axes": unit_axes,
    }


def swivel_length_and_exit(
    entries: np.ndarray,
    axes: np.ndarray,
    radii: np.ndarray,
    positions: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Compute swivel-pulley line length (wrap + free span) and tangent exit point.

    Parameters
    ----------
    entries : (N, 3)
        Anchor entry points in mm.
    axes : (N, 3)
        Unit swivel axes (direction of incoming line travel).
    radii : (N,)
        Pulley radii in mm (<= 0 behaves as a point pinhole at `entries`).
    positions : (..., 3)
        Effector position(s) in mm. Leading batch dimensions are preserved.

    Returns
    -------
    lengths : (..., N)
        Total line length from entry through wrap to effector position, in mm.
    exits : (..., N, 3)
        Tangent exit point on each pulley wheel (or `entries` when radius <= 0).
    """
    entries = np.asarray(entries, dtype=float)
    axes = np.asarray(axes, dtype=float)
    radii = np.asarray(radii, dtype=float)
    positions = np.asarray(positions, dtype=float)
    lead_shape = positions.shape[:-1]
    pos_flat = positions.reshape(-1, 3)

    # v = p - entry, shape (P, N, 3)
    v = pos_flat[:, None, :] - entries[None, :, :]
    a = axes[None, :, :]
    r = radii[None, :]
    active = r > 0.0

    # Axial and radial components relative to the swivel axis
    z = np.sum(v * a, axis=2)  # (P, N)
    e_vec = v - z[:, :, None] * a  # (P, N, 3)
    rho = np.linalg.norm(e_vec, axis=2)  # (P, N)

    # Fallback wheel plane when p lies on the swivel axis (rho < 1e-9), matching SwivelPulley.h
    k_idx = np.where(
        np.abs(axes[:, 0]) < 0.6,
        0,
        np.where(np.abs(axes[:, 1]) < 0.6, 1, 2),
    )  # (N,)
    basis_k = np.eye(3)[k_idx]  # (N, 3)
    fallback = basis_k - axes[np.arange(len(axes)), k_idx][:, None] * axes
    fallback /= np.maximum(np.linalg.norm(fallback, axis=1, keepdims=True), 1e-12)

    on_axis = rho < 1e-9
    e_unit = np.where(
        on_axis[:, :, None],
        fallback[None, :, :],
        e_vec / np.maximum(rho, 1e-12)[:, :, None],
    )

    c = rho - r
    d = np.hypot(c, z)
    outside = d > r
    ratio = np.where(outside, r / np.maximum(d, 1e-12), 1.0)
    theta = np.arctan2(z, c) + np.arccos(np.clip(ratio, -1.0, 1.0))
    span = np.where(outside, np.sqrt(np.maximum(d * d - r * r, 0.0)), 0.0)

    swivel_exit = entries[None, :, :] + r[:, :, None] * (
        e_unit * (1.0 + np.cos(theta))[:, :, None] + a * np.sin(theta)[:, :, None]
    )
    swivel_len = span + r * (np.pi - theta)

    pinhole_len = np.linalg.norm(v, axis=2)
    pinhole_exit = np.broadcast_to(entries[None, :, :], v.shape)

    lengths = np.where(active, swivel_len, pinhole_len)
    exits = np.where(active[:, :, None], swivel_exit, pinhole_exit)
    return lengths.reshape(*lead_shape, len(entries)), exits.reshape(*lead_shape, len(entries), 3)


def line_geometry(
    anchors: np.ndarray,
    positions: np.ndarray,
    swivel: Optional[dict] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """Return `(distances, dir_to_anchor)` for pinholes or swivel pulleys.

    By the envelope theorem, the gradient of swivel line length with respect to
    effector position is `-dir_to_anchor` where `dir_to_anchor = (exit - pos) / ||exit - pos||`,
    which is also the unit direction of tension pulled on the effector.
    """
    anchors = np.asarray(anchors, dtype=float)
    positions = np.asarray(positions, dtype=float)
    if swivel is None or anchors.shape[1] != 3:
        to_anchor = anchors[None, :, :] - positions[:, None, :]
        dist = np.maximum(np.linalg.norm(to_anchor, axis=2), 1e-9)
        return dist, to_anchor / dist[:, :, None]

    lengths, exits = swivel_length_and_exit(
        anchors, swivel["axes"], swivel["radii_mm"], positions
    )
    span_vec = exits - positions[:, None, :]
    span_norm = np.linalg.norm(span_vec, axis=2)
    fallback = anchors[None, :, :] - positions[:, None, :]
    fallback_norm = np.maximum(np.linalg.norm(fallback, axis=2), 1e-9)
    dir_to_anchor = np.where(
        (span_norm > 1e-9)[:, :, None],
        span_vec / np.maximum(span_norm, 1e-9)[:, :, None],
        fallback / fallback_norm[:, :, None],
    )
    return np.maximum(lengths, 1e-9), dir_to_anchor


def home_line_lengths(
    anchors: np.ndarray,
    swivel: Optional[dict] = None,
) -> np.ndarray:
    """Line lengths at the home origin `p = 0`, in mm."""
    anchors = np.asarray(anchors, dtype=float)
    if swivel is None or anchors.shape[1] != 3:
        return np.linalg.norm(anchors, axis=1)
    lengths, _ = swivel_length_and_exit(
        anchors, swivel["axes"], swivel["radii_mm"], np.zeros((1, 3), dtype=float)
    )
    return lengths[0]
