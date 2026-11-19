"""Anchor matrices as plain text and as M669 G-code."""
from __future__ import annotations

from typing import Union

import numpy as np

from autocal.sweep_types import MachineType


def format_anchor_matrix_plain(anchors: np.ndarray, *, decimals: int = 2) -> str:
    """Render anchors as nested lists with fixed decimal precision."""
    anchors_arr = np.asarray(anchors, dtype=float)
    if anchors_arr.ndim != 2:
        raise ValueError("Anchors must be a 2D matrix for plain formatting")
    rows = []
    for anchor in anchors_arr:
        coords = ", ".join(f"{float(value):.{int(decimals)}f}" for value in anchor)
        rows.append(f"[{coords}]")
    return "[" + ", ".join(rows) + "]"


def format_anchors_gcode(anchors: np.ndarray, machine_type: Union[str, MachineType]) -> str:
    """Render anchors as a G-code M669 string."""
    machine_type_str = machine_type.value if isinstance(machine_type, MachineType) else str(machine_type)
    plain_anchors = format_anchor_matrix_plain(anchors)

    if machine_type_str in ("hangprinter_3", "hangprinter_4", "hangprinter_5", "hangprinter_5_pulley_reduced"):
        labels = ["A", "B", "C", "D", "I"]
        parts = []
        for idx, anchor in enumerate(anchors):
            label = labels[idx] if idx < len(labels) else f"P{idx}"
            if anchor.shape[0] < 3:
                raise ValueError("Hangprinter anchors must be 3D for G-code formatting")
            parts.append(f"{label}{anchor[0]:.2f}:{anchor[1]:.2f}:{anchor[2]:.2f}")
        return "M669 " + " ".join(parts)

    if machine_type_str == "slideprinter":
        return f"; Anchors: {plain_anchors}"

    return f"; Anchors ({machine_type_str}): {plain_anchors}"
