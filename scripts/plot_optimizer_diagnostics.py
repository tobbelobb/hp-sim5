#!/usr/bin/env python3
from __future__ import annotations

"""
Generate optimizer residual diagnostics as simple SVG scatter plots.

Examples:

1. From explicit parameters:
python scripts/plot_optimizer_diagnostics.py \
  --dataset autocal/data/.regress_parallel_runs/flexible_lines_2000_d7ca8507/flexible_lines_2000.json \
  --limit-sweeps 3 \
  --anchors "[[0.0,-1900.0],[1645.44826719,950.0],[-1645.44826719,950.0]]" \
  --radii "39.184,39.184,39.184" \
  --buildup "0.636619" \
  --label flexible_lines_2000_first3

2. From a full-auto log iteration:
python scripts/plot_optimizer_diagnostics.py \
  --dataset autocal/data/.regress_parallel_runs/flexible_lines_2000_d7ca8507/flexible_lines_2000.json \
  --limit-sweeps 4 \
  --log autocal/data/.regress_parallel_runs/flexible_lines_2000_d7ca8507/flexible_lines_2000.full_auto.log \
  --log-iteration 2 \
  --label flexible_lines_2000_first4_iter2
"""

import argparse
import ast
import copy
import csv
import json
import math
import re
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from autocal import active_calibrate as ac
from autocal.spool_model import build_spool_model_params, dataset_with_modeled_lengths
from autocal.theoretical_ellipse import (
    compute_constraint_circle_2d,
    compute_constraint_circle_3d,
    squared_length_coefficients,
)

_RE_ANCHORS = re.compile(r"Anchors:\s*(\[\[.*\]\])")
_RE_EFFECTIVE = re.compile(r"effective=\[([^\]]+)\]")
_RE_SCORE = re.compile(r"\bScore:\s*([0-9.+-eE]+)\b")
_RE_K = re.compile(r"\bk=([^\s;]+)")
_RE_SUMMARY_FIT_SCORE = re.compile(r"Fit(?:/UI)? quality score.*?:\s*([0-9.+-eE]+)\s*$", re.IGNORECASE)
_RE_M669_PARTS = re.compile(
    r"([ABC])\s*([+-]?\d+(?:\.\d+)?(?:e[+-]?\d+)?)\:([+-]?\d+(?:\.\d+)?(?:e[+-]?\d+)?)\:([+-]?\d+(?:\.\d+)?(?:e[+-]?\d+)?)",
    re.IGNORECASE,
)
_RE_M666_R = re.compile(
    r"R\s*([+-]?\d+(?:\.\d+)?(?:e[+-]?\d+)?):([+-]?\d+(?:\.\d+)?(?:e[+-]?\d+)?):([+-]?\d+(?:\.\d+)?(?:e[+-]?\d+)?)",
    re.IGNORECASE,
)
_RE_M666_Q = re.compile(r"Q([^\s]+)", re.IGNORECASE)

_SWEEP_PALETTE = (
    "#202020",
    "#0b6e4f",
    "#8a2d3b",
    "#6153cc",
    "#946300",
    "#0085a1",
    "#7c3aed",
    "#b45309",
)


def _parse_csv_floats(text: str) -> List[float]:
    return [float(part.strip()) for part in str(text).split(",") if str(part).strip()]


def _parse_anchors_text(text: str) -> np.ndarray:
    arr = np.asarray(ast.literal_eval(text), dtype=float)
    if arr.ndim != 2 or arr.shape[1] != 2:
        raise ValueError("--anchors must be a 2D list like [[x,y],[x,y],[x,y]]")
    return np.asarray(arr, dtype=float)


def _parse_buildup_text(text: str, *, num_anchors: int) -> np.ndarray:
    raw = str(text).strip()
    if raw.startswith("["):
        vals = np.asarray(ast.literal_eval(raw), dtype=float).reshape(-1)
    elif "," in raw:
        vals = np.asarray(_parse_csv_floats(raw), dtype=float).reshape(-1)
    else:
        vals = np.full(int(num_anchors), float(raw), dtype=float)
    if vals.size == 1:
        vals = np.full(int(num_anchors), float(vals[0]), dtype=float)
    if vals.size != int(num_anchors):
        raise ValueError("buildup must be scalar or one value per anchor")
    return np.asarray(vals, dtype=float)


def _subset_dataset(dataset: dict, *, limit_sweeps: Optional[int]) -> dict:
    subset = copy.deepcopy(dataset)
    if limit_sweeps is None:
        return subset
    sweeps = subset.get("sweeps")
    if isinstance(sweeps, list):
        subset["sweeps"] = sweeps[: max(0, int(limit_sweeps))]
    return subset


def _fmt_num(value: float, decimals: int = 3) -> str:
    if not np.isfinite(value):
        return "nan"
    if abs(float(value)) >= 1e5:
        return f"{float(value):.3e}"
    text = f"{float(value):.{int(decimals)}f}"
    if "." in text:
        text = text.rstrip("0").rstrip(".")
    return text


def _signed_fill_rgb(value: float, *, limit: float) -> str:
    lim = float(max(limit, 1e-9))
    t = max(-1.0, min(1.0, float(value) / lim))
    if t >= 0.0:
        r = 220
        g = int(round(220.0 * (1.0 - t)))
        b = int(round(220.0 * (1.0 - t)))
    else:
        r = int(round(220.0 * (1.0 + t)))
        g = int(round(220.0 * (1.0 + t)))
        b = 220
    return f"rgb({r},{g},{b})"


def _svg_scatter_plot(
    *,
    title: str,
    rows: Sequence[dict],
    x_key: str,
    y_key: str,
    x_label: str,
    y_label: str,
    out_path: Path,
    annotate_points: bool,
) -> None:
    width = 960
    height = 620
    left = 80
    right = 920
    top = 50
    bottom = 545

    points = [
        row
        for row in rows
        if isinstance(row, dict)
        and ac._float_or_none(row.get(x_key)) is not None
        and ac._float_or_none(row.get(y_key)) is not None
    ]
    if not points:
        raise ValueError(f"no finite rows for {x_key} vs {y_key}")

    x_vals = np.asarray([float(row[x_key]) for row in points], dtype=float)
    y_vals = np.asarray([float(row[y_key]) for row in points], dtype=float)
    residuals = np.asarray(
        [float(row.get("residual_mm_signed", 0.0)) for row in points],
        dtype=float,
    )
    color_limit = float(max(np.max(np.abs(residuals)), 1.0))

    x_pad = 0.05 * max(float(np.max(x_vals) - np.min(x_vals)), 1.0)
    y_pad = 0.05 * max(float(np.max(y_vals) - np.min(y_vals)), 1.0)
    x_min = float(np.min(x_vals) - x_pad)
    x_max = float(np.max(x_vals) + x_pad)
    y_min = float(np.min(y_vals) - y_pad)
    y_max = float(np.max(y_vals) + y_pad)
    if not np.isfinite(x_min) or not np.isfinite(x_max) or x_max <= x_min:
        x_min, x_max = 0.0, 1.0
    if not np.isfinite(y_min) or not np.isfinite(y_max) or y_max <= y_min:
        y_min, y_max = -1.0, 1.0

    def sx(value: float) -> float:
        return left + (float(value) - x_min) * (right - left) / (x_max - x_min)

    def sy(value: float) -> float:
        return bottom - (float(value) - y_min) * (bottom - top) / (y_max - y_min)

    sweep_ids = []
    for row in points:
        sweep_id = str(row.get("sweep_id", ""))
        if sweep_id not in sweep_ids:
            sweep_ids.append(sweep_id)
    sweep_strokes = {
        sweep_id: _SWEEP_PALETTE[idx % len(_SWEEP_PALETTE)]
        for idx, sweep_id in enumerate(sweep_ids)
    }

    svg: List[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{0.5 * width:.1f}" y="24" text-anchor="middle" font-family="monospace" font-size="17">{title}</text>',
        f'<line x1="{left}" y1="{bottom}" x2="{right}" y2="{bottom}" stroke="black" stroke-width="1"/>',
        f'<line x1="{left}" y1="{top}" x2="{left}" y2="{bottom}" stroke="black" stroke-width="1"/>',
    ]

    for tick_idx in range(6):
        t = tick_idx / 5.0
        x_val = x_min + t * (x_max - x_min)
        y_val = y_min + t * (y_max - y_min)
        x_pos = left + t * (right - left)
        y_pos = bottom - t * (bottom - top)
        svg.append(f'<line x1="{x_pos:.2f}" y1="{bottom}" x2="{x_pos:.2f}" y2="{bottom + 6}" stroke="black" stroke-width="1"/>')
        svg.append(
            f'<text x="{x_pos:.2f}" y="{bottom + 22}" text-anchor="middle" font-family="monospace" font-size="11">{_fmt_num(x_val, 1)}</text>'
        )
        svg.append(f'<line x1="{left - 6}" y1="{y_pos:.2f}" x2="{left}" y2="{y_pos:.2f}" stroke="black" stroke-width="1"/>')
        svg.append(
            f'<text x="{left - 10}" y="{y_pos + 4:.2f}" text-anchor="end" font-family="monospace" font-size="11">{_fmt_num(y_val, 1)}</text>'
        )

    if y_min < 0.0 < y_max:
        y_zero = sy(0.0)
        svg.append(
            f'<line x1="{left}" y1="{y_zero:.2f}" x2="{right}" y2="{y_zero:.2f}" stroke="#bbbbbb" stroke-width="1" stroke-dasharray="4,4"/>'
        )
        svg.append(
            f'<text x="{right - 4}" y="{y_zero - 6:.2f}" text-anchor="end" font-family="monospace" font-size="10" fill="#666666">y=0</text>'
        )

    svg.append(
        f'<text x="{0.5 * (left + right):.1f}" y="{height - 34}" text-anchor="middle" font-family="monospace" font-size="13">{x_label}</text>'
    )
    svg.append(
        f'<text x="24" y="{0.5 * (top + bottom):.1f}" text-anchor="middle" transform="rotate(-90 24 {0.5 * (top + bottom):.1f})" font-family="monospace" font-size="13">{y_label}</text>'
    )

    for row in points:
        x_val = float(row[x_key])
        y_val = float(row[y_key])
        resid = float(row.get("residual_mm_signed", 0.0))
        cutoff = ac._float_or_none(row.get("cutoff_mm"))
        resid_abs = abs(float(ac._float_or_none(row.get("residual_mm")) or resid))
        clipped = cutoff is not None and resid_abs > (float(cutoff) + 1e-12)
        sweep_id = str(row.get("sweep_id", ""))
        px = sx(x_val)
        py = sy(y_val)
        fill = _signed_fill_rgb(resid, limit=color_limit)
        stroke = sweep_strokes.get(sweep_id, "#202020")
        radius = 4.0 if clipped else 3.1
        stroke_width = 1.2 if clipped else 0.8
        svg.append(
            f'<circle cx="{px:.2f}" cy="{py:.2f}" r="{radius:.1f}" fill="{fill}" fill-opacity="0.88" stroke="{stroke}" stroke-width="{stroke_width:.1f}"/>'
        )
        if annotate_points:
            label = f'{sweep_id}:{int(row.get("point_idx", 0))}'
            svg.append(
                f'<text x="{px + 6:.2f}" y="{py - 4:.2f}" font-family="monospace" font-size="9" fill="#333333">{label}</text>'
            )

    legend_x = right - 165
    legend_y = top + 8
    svg.append(
        f'<text x="{legend_x}" y="{legend_y}" font-family="monospace" font-size="11">fill = signed residual [mm]</text>'
    )
    color_levels = [1.0, 0.66, 0.33, 0.0, -0.33, -0.66, -1.0]
    for idx, frac in enumerate(color_levels):
        y0 = legend_y + 10 + idx * 12
        val = frac * color_limit
        svg.append(
            f'<rect x="{legend_x}" y="{y0}" width="18" height="10" fill="{_signed_fill_rgb(val, limit=color_limit)}" stroke="#666666" stroke-width="0.3"/>'
        )
    svg.append(
        f'<text x="{legend_x + 24}" y="{legend_y + 20}" font-family="monospace" font-size="10">+{_fmt_num(color_limit, 1)}</text>'
    )
    svg.append(
        f'<text x="{legend_x + 24}" y="{legend_y + 56}" font-family="monospace" font-size="10">0</text>'
    )
    svg.append(
        f'<text x="{legend_x + 24}" y="{legend_y + 92}" font-family="monospace" font-size="10">-{_fmt_num(color_limit, 1)}</text>'
    )
    svg.append(
        f'<text x="{legend_x}" y="{legend_y + 116}" font-family="monospace" font-size="11">stroke = sweep_id</text>'
    )
    for idx, sweep_id in enumerate(sweep_ids):
        y0 = legend_y + 126 + idx * 14
        svg.append(
            f'<line x1="{legend_x}" y1="{y0}" x2="{legend_x + 18}" y2="{y0}" stroke="{sweep_strokes[sweep_id]}" stroke-width="2"/>'
        )
        svg.append(
            f'<text x="{legend_x + 24}" y="{y0 + 4}" font-family="monospace" font-size="10">{sweep_id}</text>'
        )
    svg.append(
        f'<text x="{legend_x}" y="{legend_y + 126 + len(sweep_ids) * 14 + 16}" font-family="monospace" font-size="10">thick outline = clipped by cutoff</text>'
    )
    svg.append("</svg>")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("\n".join(svg), encoding="utf-8")


def _write_rows_csv(rows: Sequence[dict], out_path: Path) -> None:
    fieldnames = [
        "sweep_id",
        "point_idx",
        "drive_anchor",
        "sensor_anchor",
        "l_drive_mm",
        "l_sensor_mm",
        "avg_len_mm",
        "residual_mm_signed",
        "residual_mm",
        "residual_z_signed",
        "cutoff_mm",
        "clipped",
    ]
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            l_drive = float(row.get("l_drive_mm", 0.0))
            l_sensor = float(row.get("l_sensor_mm", 0.0))
            residual_mm = float(row.get("residual_mm", 0.0))
            cutoff = ac._float_or_none(row.get("cutoff_mm"))
            writer.writerow(
                {
                    "sweep_id": row.get("sweep_id"),
                    "point_idx": row.get("point_idx"),
                    "drive_anchor": row.get("drive_anchor"),
                    "sensor_anchor": row.get("sensor_anchor"),
                    "l_drive_mm": _fmt_num(l_drive, 6),
                    "l_sensor_mm": _fmt_num(l_sensor, 6),
                    "avg_len_mm": _fmt_num(0.5 * (l_drive + l_sensor), 6),
                    "residual_mm_signed": _fmt_num(float(row.get("residual_mm_signed", 0.0)), 6),
                    "residual_mm": _fmt_num(residual_mm, 6),
                    "residual_z_signed": _fmt_num(float(row.get("residual_z_signed", 0.0)), 6),
                    "cutoff_mm": "" if cutoff is None else _fmt_num(float(cutoff), 6),
                    "clipped": bool(cutoff is not None and residual_mm > (float(cutoff) + 1e-12)),
                }
            )


def _json_ready(value: object) -> object:
    if isinstance(value, dict):
        return {str(k): _json_ready(v) for k, v in value.items()}
    if isinstance(value, list):
        return [_json_ready(v) for v in value]
    if isinstance(value, tuple):
        return [_json_ready(v) for v in value]
    if isinstance(value, np.ndarray):
        return _json_ready(value.tolist())
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, np.bool_):
        return bool(value)
    return value


def _write_json(data: object, out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(
        json.dumps(_json_ready(data), indent=2, sort_keys=True),
        encoding="utf-8",
    )


def _safe_filename(text: object) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(text or "").strip())
    return cleaned.strip("._") or "sweep"


def _rects_overlap(a: Tuple[float, float, float, float], b: Tuple[float, float, float, float], *, pad: float = 2.0) -> bool:
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b
    return not (ax1 + pad < bx0 or bx1 + pad < ax0 or ay1 + pad < by0 or by1 + pad < ay0)


def _place_label(
    *,
    px: float,
    py: float,
    text: str,
    plot_left: float,
    plot_right: float,
    plot_top: float,
    plot_bottom: float,
    used_boxes: List[Tuple[float, float, float, float]],
) -> Tuple[float, float, Tuple[float, float, float, float]]:
    width = max(16.0, 5.4 * float(len(text)))
    height = 10.0
    offsets = [
        (6.0, -4.0),
        (6.0, -16.0),
        (6.0, 10.0),
        (-width - 6.0, -4.0),
        (-width - 6.0, -16.0),
        (-width - 6.0, 10.0),
        (12.0, -28.0),
        (-width - 12.0, -28.0),
        (12.0, 22.0),
        (-width - 12.0, 22.0),
    ]
    for dx, dy in offsets:
        tx = px + dx
        ty = py + dy
        tx = min(max(tx, plot_left + 2.0), plot_right - width - 2.0)
        ty = min(max(ty, plot_top + height), plot_bottom - 2.0)
        box = (tx, ty - height + 2.0, tx + width, ty + 2.0)
        if not any(_rects_overlap(box, other) for other in used_boxes):
            used_boxes.append(box)
            return tx, ty, box
    tx = min(max(px + 6.0, plot_left + 2.0), plot_right - width - 2.0)
    ty = min(max(py - 4.0, plot_top + height), plot_bottom - 2.0)
    box = (tx, ty - height + 2.0, tx + width, ty + 2.0)
    used_boxes.append(box)
    return tx, ty, box


def _sample_theoretical_raw_curve(
    *,
    anchors: np.ndarray,
    sweep: dict,
    dimensions: int,
    num_samples: int = 361,
) -> List[Tuple[float, float]]:
    fixed_anchor_indices = [int(v) for v in (sweep.get("fixed_anchors", []) or [])]
    fixed_deltas = [float(v) for v in (sweep.get("fixed_lengths", []) or [])]
    if not fixed_anchor_indices or len(fixed_anchor_indices) != len(fixed_deltas):
        return []

    fixed_lengths_abs = [
        float(np.linalg.norm(np.asarray(anchors[idx], dtype=float)) + delta)
        for idx, delta in zip(fixed_anchor_indices, fixed_deltas)
    ]
    drive_idx = int(sweep.get("drive_anchor", 0))
    sensor_idx = int(sweep.get("sensor_anchor", 0))

    dims = int(dimensions)
    if dims == 2:
        if len(fixed_anchor_indices) != 1:
            return []
        circle = compute_constraint_circle_2d(
            np.asarray(anchors, dtype=float),
            fixed_anchor_indices[0],
            fixed_lengths_abs[0],
        )
    else:
        if len(fixed_anchor_indices) != 2:
            return []
        circle = compute_constraint_circle_3d(
            np.asarray(anchors, dtype=float),
            fixed_anchor_indices,
            fixed_lengths_abs,
        )
    if circle is None:
        return []

    k_drive, m_drive, n_drive = squared_length_coefficients(circle, np.asarray(anchors[drive_idx], dtype=float))
    k_sensor, m_sensor, n_sensor = squared_length_coefficients(circle, np.asarray(anchors[sensor_idx], dtype=float))

    phi = np.linspace(0.0, 2.0 * np.pi, int(max(num_samples, 32)), endpoint=True)
    l_drive_sq = k_drive + m_drive * np.cos(phi) + n_drive * np.sin(phi)
    l_sensor_sq = k_sensor + m_sensor * np.cos(phi) + n_sensor * np.sin(phi)
    mask = (
        np.isfinite(l_drive_sq)
        & np.isfinite(l_sensor_sq)
        & (l_drive_sq >= 0.0)
        & (l_sensor_sq >= 0.0)
    )
    if not np.any(mask):
        return []
    return [
        (float(np.sqrt(ld2)), float(np.sqrt(ls2)))
        for ld2, ls2 in zip(l_drive_sq[mask], l_sensor_sq[mask])
    ]


def _svg_per_sweep_fit_plot(
    *,
    label: str,
    sweep: dict,
    rows: Sequence[dict],
    anchors: np.ndarray,
    dimensions: int,
    out_path: Path,
    color_limit: float,
    annotate_points: bool,
) -> None:
    width = 980
    height = 720
    left = 80
    right = 780
    top = 60
    bottom = 650

    ordered_rows = sorted(rows, key=lambda row: int(row.get("point_idx", 0)))
    if not ordered_rows:
        raise ValueError("per-sweep plot requires at least one row")

    measured = [
        (float(row["l_drive_mm"]), float(row["l_sensor_mm"]))
        for row in ordered_rows
        if ac._float_or_none(row.get("l_drive_mm")) is not None
        and ac._float_or_none(row.get("l_sensor_mm")) is not None
    ]
    if not measured:
        raise ValueError("per-sweep plot requires finite measured lengths")

    theory = _sample_theoretical_raw_curve(
        anchors=np.asarray(anchors, dtype=float),
        sweep=sweep,
        dimensions=int(dimensions),
    )

    x_vals = [pt[0] for pt in measured]
    y_vals = [pt[1] for pt in measured]
    if theory:
        x_vals.extend(pt[0] for pt in theory)
        y_vals.extend(pt[1] for pt in theory)

    x_min = float(min(x_vals))
    x_max = float(max(x_vals))
    y_min = float(min(y_vals))
    y_max = float(max(y_vals))
    span = max(x_max - x_min, y_max - y_min, 1.0)
    pad = 0.08 * span
    cx = 0.5 * (x_min + x_max)
    cy = 0.5 * (y_min + y_max)
    x_min = cx - 0.5 * span - pad
    x_max = cx + 0.5 * span + pad
    y_min = cy - 0.5 * span - pad
    y_max = cy + 0.5 * span + pad

    def sx(value: float) -> float:
        return left + (float(value) - x_min) * (right - left) / (x_max - x_min)

    def sy(value: float) -> float:
        return bottom - (float(value) - y_min) * (bottom - top) / (y_max - y_min)

    svg: List[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{0.5 * width:.1f}" y="24" text-anchor="middle" font-family="monospace" font-size="17">{label}: {sweep.get("id", "")} raw-length fit</text>',
        f'<line x1="{left}" y1="{bottom}" x2="{right}" y2="{bottom}" stroke="black" stroke-width="1"/>',
        f'<line x1="{left}" y1="{top}" x2="{left}" y2="{bottom}" stroke="black" stroke-width="1"/>',
    ]

    for tick_idx in range(6):
        t = tick_idx / 5.0
        x_val = x_min + t * (x_max - x_min)
        y_val = y_min + t * (y_max - y_min)
        x_pos = left + t * (right - left)
        y_pos = bottom - t * (bottom - top)
        svg.append(f'<line x1="{x_pos:.2f}" y1="{bottom}" x2="{x_pos:.2f}" y2="{bottom + 6}" stroke="black" stroke-width="1"/>')
        svg.append(
            f'<text x="{x_pos:.2f}" y="{bottom + 22}" text-anchor="middle" font-family="monospace" font-size="11">{_fmt_num(x_val, 1)}</text>'
        )
        svg.append(f'<line x1="{left - 6}" y1="{y_pos:.2f}" x2="{left}" y2="{y_pos:.2f}" stroke="black" stroke-width="1"/>')
        svg.append(
            f'<text x="{left - 10}" y="{y_pos + 4:.2f}" text-anchor="end" font-family="monospace" font-size="11">{_fmt_num(y_val, 1)}</text>'
        )

    svg.append(
        f'<text x="{0.5 * (left + right):.1f}" y="{height - 34}" text-anchor="middle" font-family="monospace" font-size="13">l_drive [mm]</text>'
    )
    svg.append(
        f'<text x="24" y="{0.5 * (top + bottom):.1f}" text-anchor="middle" transform="rotate(-90 24 {0.5 * (top + bottom):.1f})" font-family="monospace" font-size="13">l_sensor [mm]</text>'
    )

    measured_points = " ".join(f"{sx(x):.2f},{sy(y):.2f}" for x, y in measured)
    if measured_points:
        svg.append(
            f'<polyline points="{measured_points}" fill="none" stroke="#a0a0a0" stroke-width="1.2"/>'
        )
    if theory:
        theory_points = " ".join(f"{sx(x):.2f},{sy(y):.2f}" for x, y in theory)
        svg.append(
            f'<polyline points="{theory_points}" fill="none" stroke="#cc0000" stroke-width="1.4" stroke-dasharray="6,4"/>'
        )

    used_label_boxes: List[Tuple[float, float, float, float]] = []
    for row in ordered_rows:
        x_val = float(row["l_drive_mm"])
        y_val = float(row["l_sensor_mm"])
        resid = float(row.get("residual_mm_signed", 0.0))
        cutoff = ac._float_or_none(row.get("cutoff_mm"))
        resid_abs = abs(float(ac._float_or_none(row.get("residual_mm")) or resid))
        clipped = cutoff is not None and resid_abs > (float(cutoff) + 1e-12)
        px = sx(x_val)
        py = sy(y_val)
        svg.append(
            f'<circle cx="{px:.2f}" cy="{py:.2f}" r="{4.0 if clipped else 3.2:.1f}" fill="{_signed_fill_rgb(resid, limit=color_limit)}" fill-opacity="0.9" stroke="#202020" stroke-width="{1.2 if clipped else 0.8:.1f}"/>'
        )
        if annotate_points:
            label_text = f'{int(row.get("point_idx", 0))}:{_fmt_num(resid, 1)}'
            tx, ty, _box = _place_label(
                px=px,
                py=py,
                text=label_text,
                plot_left=float(left),
                plot_right=float(right),
                plot_top=float(top),
                plot_bottom=float(bottom),
                used_boxes=used_label_boxes,
            )
            svg.append(
                f'<text x="{tx:.2f}" y="{ty:.2f}" font-family="monospace" font-size="9" fill="#333333">{label_text}</text>'
            )

    legend_x = 810
    legend_y = 74
    svg.append(
        f'<text x="{legend_x}" y="{legend_y}" font-family="monospace" font-size="11">fill = signed residual [mm]</text>'
    )
    color_levels = [1.0, 0.66, 0.33, 0.0, -0.33, -0.66, -1.0]
    for idx, frac in enumerate(color_levels):
        y0 = legend_y + 10 + idx * 12
        val = frac * color_limit
        svg.append(
            f'<rect x="{legend_x}" y="{y0}" width="18" height="10" fill="{_signed_fill_rgb(val, limit=color_limit)}" stroke="#666666" stroke-width="0.3"/>'
        )
    svg.append(
        f'<text x="{legend_x + 24}" y="{legend_y + 20}" font-family="monospace" font-size="10">+{_fmt_num(color_limit, 1)}</text>'
    )
    svg.append(
        f'<text x="{legend_x + 24}" y="{legend_y + 56}" font-family="monospace" font-size="10">0</text>'
    )
    svg.append(
        f'<text x="{legend_x + 24}" y="{legend_y + 92}" font-family="monospace" font-size="10">-{_fmt_num(color_limit, 1)}</text>'
    )
    svg.append(
        f'<line x1="{legend_x}" y1="{legend_y + 118}" x2="{legend_x + 18}" y2="{legend_y + 118}" stroke="#a0a0a0" stroke-width="1.2"/>'
    )
    svg.append(
        f'<text x="{legend_x + 24}" y="{legend_y + 122}" font-family="monospace" font-size="10">measured point order</text>'
    )
    svg.append(
        f'<line x1="{legend_x}" y1="{legend_y + 138}" x2="{legend_x + 18}" y2="{legend_y + 138}" stroke="#cc0000" stroke-width="1.4" stroke-dasharray="6,4"/>'
    )
    svg.append(
        f'<text x="{legend_x + 24}" y="{legend_y + 142}" font-family="monospace" font-size="10">theoretical locus</text>'
    )
    svg.append(
        f'<text x="{legend_x}" y="{legend_y + 162}" font-family="monospace" font-size="10">thick outline = clipped</text>'
    )

    fixed_text = ",".join(str(v) for v in (sweep.get("fixed_anchors", []) or []))
    fixed_len_text = ",".join(_fmt_num(float(v), 2) for v in (sweep.get("fixed_lengths", []) or []))
    svg.append(
        f'<text x="{legend_x}" y="{legend_y + 196}" font-family="monospace" font-size="10">drive={sweep.get("drive_anchor")} sensor={sweep.get("sensor_anchor")}</text>'
    )
    svg.append(
        f'<text x="{legend_x}" y="{legend_y + 212}" font-family="monospace" font-size="10">fixed={fixed_text} deltas={fixed_len_text}</text>'
    )

    svg.append("</svg>")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("\n".join(svg), encoding="utf-8")


def _svg_per_sweep_residual_order_plot(
    *,
    label: str,
    sweep: dict,
    rows: Sequence[dict],
    out_path: Path,
    color_limit: float,
) -> None:
    width = 980
    height = 520
    left = 70
    right = 920
    top = 55
    bottom = 430

    ordered_rows = sorted(rows, key=lambda row: int(row.get("point_idx", 0)))
    if not ordered_rows:
        raise ValueError("residual-order plot requires at least one row")

    residuals = np.asarray([float(row.get("residual_mm_signed", 0.0)) for row in ordered_rows], dtype=float)
    n = int(residuals.size)
    y_limit = float(max(np.max(np.abs(residuals)) * 1.15 if residuals.size else 0.0, 1.0))

    def sx(idx: float) -> float:
        denom = max(float(max(n - 1, 1)), 1.0)
        return left + float(idx) * (right - left) / denom

    def sy(value: float) -> float:
        return bottom - (float(value) + y_limit) * (bottom - top) / (2.0 * y_limit)

    zero_y = sy(0.0)
    svg: List[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{0.5 * width:.1f}" y="24" text-anchor="middle" font-family="monospace" font-size="17">{label}: {sweep.get("id", "")} residual by point order</text>',
        f'<line x1="{left}" y1="{bottom}" x2="{right}" y2="{bottom}" stroke="black" stroke-width="1"/>',
        f'<line x1="{left}" y1="{top}" x2="{left}" y2="{bottom}" stroke="black" stroke-width="1"/>',
        f'<line x1="{left}" y1="{zero_y:.2f}" x2="{right}" y2="{zero_y:.2f}" stroke="#999999" stroke-width="1" stroke-dasharray="4,4"/>',
    ]

    tick_step = 1 if n <= 20 else (2 if n <= 40 else max(5, int(round(n / 10.0))))
    for idx in range(n):
        if idx % tick_step != 0 and idx != n - 1:
            continue
        x_pos = sx(float(idx))
        svg.append(f'<line x1="{x_pos:.2f}" y1="{bottom}" x2="{x_pos:.2f}" y2="{bottom + 6}" stroke="black" stroke-width="1"/>')
        svg.append(
            f'<text x="{x_pos:.2f}" y="{bottom + 22}" text-anchor="middle" font-family="monospace" font-size="11">{idx}</text>'
        )

    for frac in (-1.0, -0.5, 0.0, 0.5, 1.0):
        y_val = frac * y_limit
        y_pos = sy(y_val)
        svg.append(f'<line x1="{left - 6}" y1="{y_pos:.2f}" x2="{left}" y2="{y_pos:.2f}" stroke="black" stroke-width="1"/>')
        svg.append(
            f'<text x="{left - 10}" y="{y_pos + 4:.2f}" text-anchor="end" font-family="monospace" font-size="11">{_fmt_num(y_val, 1)}</text>'
        )

    if n >= 4 and n % 2 == 0:
        split_idx = (n // 2) - 0.5
        split_x = sx(split_idx)
        svg.append(
            f'<line x1="{split_x:.2f}" y1="{top}" x2="{split_x:.2f}" y2="{bottom}" stroke="#bbbbbb" stroke-width="1" stroke-dasharray="4,4"/>'
        )
        svg.append(
            f'<text x="{split_x + 4:.2f}" y="{top + 14}" font-family="monospace" font-size="10" fill="#666666">sub-sweep split</text>'
        )

    svg.append(
        f'<text x="{0.5 * (left + right):.1f}" y="{height - 34}" text-anchor="middle" font-family="monospace" font-size="13">measured point order</text>'
    )
    svg.append(
        f'<text x="24" y="{0.5 * (top + bottom):.1f}" text-anchor="middle" transform="rotate(-90 24 {0.5 * (top + bottom):.1f})" font-family="monospace" font-size="13">signed residual [mm]</text>'
    )

    for row in ordered_rows:
        idx = int(row.get("point_idx", 0))
        resid = float(row.get("residual_mm_signed", 0.0))
        cutoff = ac._float_or_none(row.get("cutoff_mm"))
        resid_abs = abs(float(ac._float_or_none(row.get("residual_mm")) or resid))
        clipped = cutoff is not None and resid_abs > (float(cutoff) + 1e-12)
        x_pos = sx(float(idx))
        y_pos = sy(resid)
        color = _signed_fill_rgb(resid, limit=color_limit)
        svg.append(
            f'<line x1="{x_pos:.2f}" y1="{zero_y:.2f}" x2="{x_pos:.2f}" y2="{y_pos:.2f}" stroke="{color}" stroke-width="{2.2 if clipped else 1.6:.1f}"/>'
        )
        svg.append(
            f'<circle cx="{x_pos:.2f}" cy="{y_pos:.2f}" r="{4.0 if clipped else 3.2:.1f}" fill="{color}" fill-opacity="0.95" stroke="#202020" stroke-width="{1.2 if clipped else 0.8:.1f}"/>'
        )

    legend_x = 760
    legend_y = 72
    svg.append(
        f'<text x="{legend_x}" y="{legend_y}" font-family="monospace" font-size="11">color = signed residual [mm]</text>'
    )
    color_levels = [1.0, 0.66, 0.33, 0.0, -0.33, -0.66, -1.0]
    for idx, frac in enumerate(color_levels):
        y0 = legend_y + 10 + idx * 12
        val = frac * color_limit
        svg.append(
            f'<rect x="{legend_x}" y="{y0}" width="18" height="10" fill="{_signed_fill_rgb(val, limit=color_limit)}" stroke="#666666" stroke-width="0.3"/>'
        )
    svg.append(
        f'<text x="{legend_x + 24}" y="{legend_y + 20}" font-family="monospace" font-size="10">+{_fmt_num(color_limit, 1)}</text>'
    )
    svg.append(
        f'<text x="{legend_x + 24}" y="{legend_y + 56}" font-family="monospace" font-size="10">0</text>'
    )
    svg.append(
        f'<text x="{legend_x + 24}" y="{legend_y + 92}" font-family="monospace" font-size="10">-{_fmt_num(color_limit, 1)}</text>'
    )
    svg.append(
        f'<text x="{legend_x}" y="{legend_y + 116}" font-family="monospace" font-size="10">thick stem/circle = clipped</text>'
    )
    svg.append("</svg>")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("\n".join(svg), encoding="utf-8")


def _write_per_sweep_svgs(
    *,
    label: str,
    dataset: dict,
    rows: Sequence[dict],
    anchors: np.ndarray,
    out_dir: Path,
    annotate_points: bool,
) -> int:
    sweeps = dataset.get("sweeps")
    if not isinstance(sweeps, list):
        return 0
    rows_by_sweep: Dict[str, List[dict]] = {}
    for row in rows:
        sweep_id = str(row.get("sweep_id", ""))
        rows_by_sweep.setdefault(sweep_id, []).append(row)

    residuals = [abs(float(row.get("residual_mm_signed", 0.0))) for row in rows]
    color_limit = float(max(max(residuals, default=0.0), 1.0))
    count = 0
    target_dir = out_dir / f"{label}.per_sweep"
    for sweep in sweeps:
        if not isinstance(sweep, dict):
            continue
        sweep_id = str(sweep.get("id", ""))
        sweep_rows = rows_by_sweep.get(sweep_id)
        if not sweep_rows:
            continue
        _svg_per_sweep_fit_plot(
            label=label,
            sweep=sweep,
            rows=sweep_rows,
            anchors=np.asarray(anchors, dtype=float),
            dimensions=int(dataset.get("dimensions", 2) or 2),
            out_path=target_dir / f"{_safe_filename(sweep_id)}.raw_fit.svg",
            color_limit=color_limit,
            annotate_points=bool(annotate_points),
        )
        _svg_per_sweep_residual_order_plot(
            label=label,
            sweep=sweep,
            rows=sweep_rows,
            out_path=target_dir / f"{_safe_filename(sweep_id)}.residual_order.svg",
            color_limit=color_limit,
        )
        count += 1
    return count


def _parse_iteration_models_from_log_text(text: str) -> List[Dict[str, object]]:
    items: List[Dict[str, object]] = []
    cur: Dict[str, object] = {}
    for line in text.splitlines():
        if "Anchors:" in line:
            match = _RE_ANCHORS.search(line)
            if match:
                cur["anchors"] = np.asarray(ast.literal_eval(match.group(1)), dtype=float)
        if "line_model:" in line and "effective=[" in line:
            match_eff = _RE_EFFECTIVE.search(line)
            if match_eff:
                cur["radii"] = np.asarray(
                    ast.literal_eval("[" + match_eff.group(1) + "]"),
                    dtype=float,
                ).reshape(-1)
            match_k = _RE_K.search(line)
            if match_k:
                raw_k = match_k.group(1)
                cur["buildup"] = _parse_buildup_text(
                    raw_k,
                    num_anchors=int(np.asarray(cur.get("radii", [])).size or 3),
                )
        if "; selected run=" in line:
            if "anchors" in cur and "radii" in cur:
                buildup = cur.get("buildup")
                if buildup is None:
                    buildup = np.zeros(int(np.asarray(cur["radii"]).size), dtype=float)
                items.append(
                    {
                        "anchors": np.asarray(cur["anchors"], dtype=float),
                        "radii": np.asarray(cur["radii"], dtype=float).reshape(-1),
                        "buildup": np.asarray(buildup, dtype=float).reshape(-1),
                    }
                )
            cur = {}
            continue
        if "Score:" in line and _RE_SCORE.search(line):
            if "anchors" in cur and "radii" in cur:
                buildup = cur.get("buildup")
                if buildup is None:
                    buildup = np.zeros(int(np.asarray(cur["radii"]).size), dtype=float)
                items.append(
                    {
                        "anchors": np.asarray(cur["anchors"], dtype=float),
                        "radii": np.asarray(cur["radii"], dtype=float).reshape(-1),
                        "buildup": np.asarray(buildup, dtype=float).reshape(-1),
                    }
                )
            cur = {}
    return items


def _parse_summary_model_from_log_text(text: str) -> Optional[Dict[str, object]]:
    summary_start = text.rfind("== Calibration summary ==")
    if summary_start < 0:
        return None
    summary_text = text[summary_start:]
    anchors_map: Dict[str, Tuple[float, float]] = {}
    radii = None
    buildup = None
    fit_score = None
    for line in summary_text.splitlines():
        match_fit = _RE_SUMMARY_FIT_SCORE.search(line)
        if match_fit:
            fit_score = float(match_fit.group(1))
        if "Parameters (M669)" in line or "Anchors (M669)" in line:
            for label, x_str, y_str, _z_str in _RE_M669_PARTS.findall(line):
                anchors_map[str(label).upper()] = (float(x_str), float(y_str))
        if "Line model (M666)" in line or "Spools (M666)" in line:
            match_r = _RE_M666_R.search(line)
            if match_r:
                radii = np.asarray(
                    [float(match_r.group(1)), float(match_r.group(2)), float(match_r.group(3))],
                    dtype=float,
                )
            match_q = _RE_M666_Q.search(line)
            if match_q:
                buildup = match_q.group(1)
    if len(anchors_map) != 3 or radii is None:
        return None
    anchors = np.asarray(
        [anchors_map["A"], anchors_map["B"], anchors_map["C"]],
        dtype=float,
    )
    buildup_arr = (
        _parse_buildup_text(str(buildup), num_anchors=3)
        if buildup is not None
        else np.zeros(3, dtype=float)
    )
    return {
        "anchors": anchors,
        "radii": np.asarray(radii, dtype=float),
        "buildup": buildup_arr,
        "fit_score_ui": fit_score,
    }


def _resolve_model_from_args(args: argparse.Namespace, dataset: dict) -> Tuple[np.ndarray, Optional[np.ndarray], Optional[np.ndarray]]:
    num_anchors = int(dataset.get("num_anchors", 0))

    if args.log is not None:
        text = Path(args.log).read_text(encoding="utf-8", errors="replace")
        if args.log_summary:
            summary = _parse_summary_model_from_log_text(text)
            if summary is None:
                raise ValueError(f"could not parse summary model from log: {args.log}")
            return (
                np.asarray(summary["anchors"], dtype=float),
                np.asarray(summary["radii"], dtype=float),
                np.asarray(summary["buildup"], dtype=float),
            )
        items = _parse_iteration_models_from_log_text(text)
        if args.log_iteration is None:
            raise ValueError("--log requires --log-iteration or --log-summary")
        idx = int(args.log_iteration)
        if idx < 0 or idx >= len(items):
            raise ValueError(f"log iteration {idx} out of range; parsed {len(items)} iterations")
        item = items[idx]
        return (
            np.asarray(item["anchors"], dtype=float),
            np.asarray(item["radii"], dtype=float),
            np.asarray(item["buildup"], dtype=float),
        )

    if args.anchors is None:
        raise ValueError("provide --anchors or --log")
    anchors = _parse_anchors_text(str(args.anchors))
    if anchors.shape[0] != num_anchors:
        raise ValueError(f"anchor count mismatch: dataset has {num_anchors}, got {anchors.shape[0]}")

    if args.radii is None:
        return anchors, None, None
    radii = np.asarray(_parse_csv_floats(str(args.radii)), dtype=float).reshape(-1)
    if radii.size == 1:
        radii = np.full(num_anchors, float(radii[0]), dtype=float)
    if radii.size != num_anchors:
        raise ValueError("radii must be scalar or one value per anchor")

    buildup = None
    if args.buildup is not None:
        buildup = _parse_buildup_text(str(args.buildup), num_anchors=num_anchors)
    else:
        buildup = np.full(num_anchors, float(ac._resolve_buildup_factor_seed(dataset, buildup_factor_override=None)), dtype=float)
    return anchors, radii, buildup


def _build_residual_rows(
    *,
    dataset: dict,
    anchors: np.ndarray,
    modeled_radii_mm: Optional[np.ndarray],
    modeled_buildup_factor: Optional[np.ndarray],
    base_radii_override: Optional[List[float]],
    residual_threshold: float,
    pointwise_filtering: bool,
    pointwise_global_mad: bool,
    sweep_wise_filtering: bool,
    sweep_metric: str,
    use_noise_mean: bool,
    sigma_source: str,
    prefer_zero_tension_angles: bool,
    line_width: float,
    sigma_floor_mm: Optional[float],
    sigma_used_mm: Optional[float],
) -> Tuple[List[dict], dict]:
    dataset_local = copy.deepcopy(dataset)
    find_radii_mode = "off" if modeled_radii_mm is None else "global"
    find_buildup_mode = "off" if modeled_buildup_factor is None else "global"
    ac._annotate_dataset_noise_model(
        dataset_local,
        line_width_mm=float(line_width),
        sigma_floor_mm=sigma_floor_mm,
        sigma_used_mm=sigma_used_mm,
        find_radii_mode=find_radii_mode,
        find_buildup_mode=find_buildup_mode,
        project_zero_tension=bool(prefer_zero_tension_angles),
    )

    dataset_eval = dataset_local
    if modeled_radii_mm is not None and modeled_buildup_factor is not None:
        num_anchors = int(dataset_local.get("num_anchors", 0))
        lm_params = ac._resolve_length_model_base_params(
            dataset_local,
            num_anchors=num_anchors,
            base_radii_override=base_radii_override,
        )
        spool_params = build_spool_model_params(
            dataset_local,
            base_radii_mm=np.asarray(lm_params["base_radii_mm"], dtype=float),
            modeled_radii_mm=np.asarray(modeled_radii_mm, dtype=float),
            modeled_buildup_factor=np.asarray(modeled_buildup_factor, dtype=float),
            spool_to_motor_gearing_factor=np.asarray(
                lm_params["spool_to_motor_gearing_factor"],
                dtype=float,
            ),
            mechanical_advantage=np.asarray(lm_params["mechanical_advantage"], dtype=float),
            lines_per_spool=np.asarray(lm_params["lines_per_spool"], dtype=float),
            base_buildup_factor=np.zeros(num_anchors, dtype=float),
            theta0_mode="zero",
            prefer_zero_tension_angles=bool(prefer_zero_tension_angles),
        )
        dataset_eval = dataset_with_modeled_lengths(
            dataset_local,
            spool_params,
            prefer_zero_tension_angles=bool(prefer_zero_tension_angles),
        )

    cost_fn = ac._build_ellipse_cost_function(
        dataset_eval,
        residual_threshold=float(residual_threshold),
        spring_k_multiplier=1.0,
        use_flex=False,
        pointwise_residual_mode="sampson",
        pointwise_filtering=bool(pointwise_filtering),
        pointwise_global_mad=bool(pointwise_global_mad),
        sweep_wise_filtering=bool(sweep_wise_filtering),
        sweep_metric=str(sweep_metric),
        use_noise_mean=bool(use_noise_mean),
        noise_normalized=True,
        sigma_source=str(sigma_source),
    )
    rows = cost_fn.pointwise_residual_rows(np.asarray(anchors, dtype=float).ravel())
    for row in rows:
        l_drive = float(row.get("l_drive_mm", 0.0))
        l_sensor = float(row.get("l_sensor_mm", 0.0))
        row["avg_len_mm"] = 0.5 * (l_drive + l_sensor)
    return rows, dataset_eval


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Plot optimizer residual diagnostics as SVG.")
    parser.add_argument("--dataset", type=Path, required=True, help="Sweep dataset JSON")
    parser.add_argument("--out-dir", type=Path, default=Path("autocal/data/.analysis/optimizer_diagnostics"))
    parser.add_argument("--label", type=str, default=None, help="Output file prefix")
    parser.add_argument("--limit-sweeps", type=int, default=None, help="Use only the first N sweeps")
    parser.add_argument("--anchors", type=str, default=None, help="Anchors as [[x,y],[x,y],[x,y]]")
    parser.add_argument("--radii", type=str, default=None, help="Radii as 'r,r,r' or scalar")
    parser.add_argument("--buildup", type=str, default=None, help="Buildup as scalar, CSV, or [..]")
    parser.add_argument("--base-radii", type=str, default=None, help="Base radii override as CSV or scalar")
    parser.add_argument("--log", type=Path, default=None, help="Full-auto text log to parse model from")
    parser.add_argument("--log-iteration", type=int, default=None, help="Use iteration N from --log")
    parser.add_argument("--log-summary", action="store_true", help="Use the final summary model from --log")
    parser.add_argument("--threshold", type=float, default=250.0, help="Residual threshold")
    parser.add_argument("--pointwise-filtering", dest="pointwise_filtering", action="store_true")
    parser.add_argument("--no-pointwise-filtering", dest="pointwise_filtering", action="store_false")
    parser.add_argument("--pointwise-global-mad", dest="pointwise_global_mad", action="store_true")
    parser.add_argument("--pointwise-per-sweep-mad", dest="pointwise_global_mad", action="store_false")
    parser.add_argument("--sweep-wise-filtering", dest="sweep_wise_filtering", action="store_true")
    parser.add_argument("--no-sweep-wise-filtering", dest="sweep_wise_filtering", action="store_false")
    parser.add_argument("--sweep-metric", choices=["mad", "median_abs", "outlier_ratio"], default="outlier_ratio")
    parser.add_argument("--sigma-source", choices=["auto", "point", "origin", "min"], default="auto")
    parser.add_argument("--use-noise-mean", dest="use_noise_mean", action="store_true")
    parser.add_argument("--use-raw-lengths", dest="use_noise_mean", action="store_false")
    parser.add_argument("--prefer-zero-tension-angles", action="store_true")
    parser.add_argument("--line-width", type=float, default=1.0)
    parser.add_argument("--sigma-floor-mm", type=float, default=None)
    parser.add_argument("--sigma-used-mm", type=float, default=None)
    parser.add_argument("--annotate-points", action="store_true")
    parser.add_argument("--per-sweep-fits", action="store_true", help="Write one raw-length fit SVG per sweep")
    parser.set_defaults(
        pointwise_filtering=True,
        pointwise_global_mad=True,
        sweep_wise_filtering=True,
        use_noise_mean=True,
    )
    args = parser.parse_args(argv)

    dataset = json.loads(args.dataset.read_text(encoding="utf-8"))
    dataset = _subset_dataset(dataset, limit_sweeps=args.limit_sweeps)
    label = args.label or args.dataset.stem
    if args.limit_sweeps is not None and args.label is None:
        label = f"{label}_first{int(args.limit_sweeps)}"

    anchors, radii, buildup = _resolve_model_from_args(args, dataset)
    base_radii_override = None
    if args.base_radii is not None:
        base_vals = _parse_csv_floats(str(args.base_radii))
        if len(base_vals) == 1:
            base_vals = base_vals * int(dataset.get("num_anchors", 0))
        base_radii_override = [float(v) for v in base_vals]

    rows, dataset_eval = _build_residual_rows(
        dataset=dataset,
        anchors=np.asarray(anchors, dtype=float),
        modeled_radii_mm=None if radii is None else np.asarray(radii, dtype=float),
        modeled_buildup_factor=None if buildup is None else np.asarray(buildup, dtype=float),
        base_radii_override=base_radii_override,
        residual_threshold=float(args.threshold),
        pointwise_filtering=bool(args.pointwise_filtering),
        pointwise_global_mad=bool(args.pointwise_global_mad),
        sweep_wise_filtering=bool(args.sweep_wise_filtering),
        sweep_metric=str(args.sweep_metric),
        use_noise_mean=bool(args.use_noise_mean),
        sigma_source=str(args.sigma_source),
        prefer_zero_tension_angles=bool(args.prefer_zero_tension_angles),
        line_width=float(args.line_width),
        sigma_floor_mm=args.sigma_floor_mm,
        sigma_used_mm=args.sigma_used_mm,
    )

    out_dir = Path(args.out_dir)
    _write_rows_csv(rows, out_dir / f"{label}.residual_rows.csv")
    _write_json(dataset_eval, out_dir / f"{label}.modeled_dataset.json")
    _svg_scatter_plot(
        title=f"{label}: l_drive vs signed residual",
        rows=rows,
        x_key="l_drive_mm",
        y_key="residual_mm_signed",
        x_label="l_drive [mm]",
        y_label="signed residual [mm]",
        out_path=out_dir / f"{label}.drive_len_vs_residual.svg",
        annotate_points=bool(args.annotate_points),
    )
    _svg_scatter_plot(
        title=f"{label}: l_sensor vs signed residual",
        rows=rows,
        x_key="l_sensor_mm",
        y_key="residual_mm_signed",
        x_label="l_sensor [mm]",
        y_label="signed residual [mm]",
        out_path=out_dir / f"{label}.sensor_len_vs_residual.svg",
        annotate_points=bool(args.annotate_points),
    )
    _svg_scatter_plot(
        title=f"{label}: avg length vs signed residual",
        rows=rows,
        x_key="avg_len_mm",
        y_key="residual_mm_signed",
        x_label="0.5*(l_drive+l_sensor) [mm]",
        y_label="signed residual [mm]",
        out_path=out_dir / f"{label}.avg_len_vs_residual.svg",
        annotate_points=bool(args.annotate_points),
    )
    _svg_scatter_plot(
        title=f"{label}: l_drive / l_sensor colored by signed residual",
        rows=rows,
        x_key="l_drive_mm",
        y_key="l_sensor_mm",
        x_label="l_drive [mm]",
        y_label="l_sensor [mm]",
        out_path=out_dir / f"{label}.drive_sensor_colored.svg",
        annotate_points=bool(args.annotate_points),
    )
    per_sweep_count = 0
    if args.per_sweep_fits:
        per_sweep_count = _write_per_sweep_svgs(
            label=label,
            dataset=dataset_eval,
            rows=rows,
            anchors=np.asarray(anchors, dtype=float),
            out_dir=out_dir,
            annotate_points=bool(args.annotate_points),
        )

    print(f"Wrote {out_dir / f'{label}.residual_rows.csv'}")
    print(f"Wrote {out_dir / f'{label}.modeled_dataset.json'}")
    print(f"Wrote {out_dir / f'{label}.drive_len_vs_residual.svg'}")
    print(f"Wrote {out_dir / f'{label}.sensor_len_vs_residual.svg'}")
    print(f"Wrote {out_dir / f'{label}.avg_len_vs_residual.svg'}")
    print(f"Wrote {out_dir / f'{label}.drive_sensor_colored.svg'}")
    if per_sweep_count > 0:
        print(f"Wrote {per_sweep_count} per-sweep SVG sets under {out_dir / f'{label}.per_sweep'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
