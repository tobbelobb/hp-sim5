#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import List, Optional, Sequence, Tuple
from xml.sax.saxutils import escape

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import autocal._autocal_common as ac

_PALETTE = (
    "#1f77b4",
    "#ff7f0e",
    "#2ca02c",
    "#d62728",
    "#9467bd",
    "#8c564b",
    "#e377c2",
    "#7f7f7f",
    "#bcbd22",
    "#17becf",
)


def _anchor_labels_for_machine_type(machine_type: str, *, num_anchors: int) -> List[str]:
    machine_type = ac._normalize_machine_type(machine_type) or str(machine_type)
    if machine_type in ("hangprinter_4", "hangprinter_5"):
        labels = ["A", "B", "C", "D", "I"]
    else:
        labels = ["A", "B", "C", "D", "I", "J", "K", "L", "O"]
    if num_anchors > len(labels):
        labels.extend(f"P{idx}" for idx in range(len(labels), int(num_anchors)))
    return labels[:num_anchors]


def _subset_dataset(dataset: dict, *, limit_sweeps: Optional[int]) -> dict:
    if limit_sweeps is None:
        return dataset
    subset = dict(dataset)
    sweeps = subset.get("sweeps")
    if isinstance(sweeps, list):
        subset["sweeps"] = sweeps[: max(0, int(limit_sweeps))]
    return subset


def _sweep_fixed_angle_series(
    sweep: dict,
    *,
    anchor_labels: Sequence[str],
) -> List[dict]:
    data_points = sweep.get("data_points", [])
    if not isinstance(data_points, list) or not data_points:
        raise ValueError(f"sweep {sweep.get('id', '<unnamed>')} has no data_points")

    fixed_anchors = [int(v) for v in (sweep.get("fixed_anchors", []) or [])]
    if not fixed_anchors:
        raise ValueError(f"sweep {sweep.get('id', '<unnamed>')} has no fixed_anchors")

    point_idx = np.arange(len(data_points), dtype=int)
    series: List[dict] = []
    for anchor_idx in fixed_anchors:
        raw_angles: List[float] = []
        for point in data_points:
            raw = point.get("raw_angles_deg")
            if not isinstance(raw, list) or anchor_idx >= len(raw):
                raw_angles.append(float("nan"))
                continue
            value = ac._float_or_none(raw[anchor_idx])
            raw_angles.append(float("nan") if value is None else float(value))
        series.append(
            {
                "anchor_idx": anchor_idx,
                "anchor_label": anchor_labels[anchor_idx] if 0 <= anchor_idx < len(anchor_labels) else f"anchor {anchor_idx}",
                "point_idx": point_idx.copy(),
                "raw_angles_deg": np.asarray(raw_angles, dtype=float),
            }
        )
    return series


def _pick_grid_shape(count: int) -> Tuple[int, int]:
    if count <= 0:
        return 0, 0
    cols = min(3, max(1, int(math.ceil(math.sqrt(count)))))
    rows = int(math.ceil(count / cols))
    return rows, cols


def _apply_series_transform(series: np.ndarray, *, mode: str) -> np.ndarray:
    if mode == "raw":
        return series
    finite = series[np.isfinite(series)]
    if finite.size == 0:
        return series
    if mode == "delta":
        return series - float(finite[0])
    raise ValueError(f"unsupported transform mode: {mode}")


def _fmt_num(value: float, decimals: int = 2) -> str:
    if not np.isfinite(value):
        return "nan"
    if abs(float(value)) >= 1e5:
        return f"{float(value):.3e}"
    text = f"{float(value):.{int(decimals)}f}"
    if "." in text:
        text = text.rstrip("0").rstrip(".")
    return text


def _series_color(anchor_idx: int) -> str:
    return _PALETTE[int(anchor_idx) % len(_PALETTE)]


def _tick_values(vmin: float, vmax: float, *, ticks: int = 5) -> List[float]:
    if not np.isfinite(vmin) or not np.isfinite(vmax) or vmax <= vmin:
        return [0.0]
    if ticks <= 1:
        return [vmin]
    return [float(vmin + (vmax - vmin) * (i / float(ticks - 1))) for i in range(ticks)]


def _collect_series(dataset: dict, *, transform: str) -> Tuple[List[dict], float, float, int]:
    sweeps = dataset.get("sweeps", [])
    if not isinstance(sweeps, list) or not sweeps:
        raise ValueError("dataset has no sweeps")

    machine_type = str(dataset.get("machine_type", ""))
    num_anchors = int(dataset.get("num_anchors", 0) or 0)
    anchor_labels = _anchor_labels_for_machine_type(machine_type, num_anchors=num_anchors)

    prepared: List[dict] = []
    all_values: List[float] = []
    max_point_index = 0
    for sweep in sweeps:
        if not isinstance(sweep, dict):
            continue
        series = _sweep_fixed_angle_series(sweep, anchor_labels=anchor_labels)
        transformed: List[dict] = []
        for entry in series:
            values = _apply_series_transform(np.asarray(entry["raw_angles_deg"], dtype=float), mode=transform)
            finite = values[np.isfinite(values)]
            all_values.extend(float(v) for v in finite)
            max_point_index = max(max_point_index, int(entry["point_idx"].size - 1))
            transformed.append({**entry, "plot_values": values})
        prepared.append({"sweep": sweep, "series": transformed})

    if not all_values:
        raise ValueError("dataset has no finite raw_angles_deg values")

    y_min = float(min(all_values))
    y_max = float(max(all_values))
    y_span = max(y_max - y_min, 1.0)
    if transform == "delta":
        y_abs = max(abs(y_min), abs(y_max), 1.0)
        y_min = -1.15 * y_abs
        y_max = 1.15 * y_abs
    else:
        y_pad = 0.08 * y_span
        y_min -= y_pad
        y_max += y_pad
    return prepared, y_min, y_max, max_point_index


def _panel_svg(
    *,
    panel_x: float,
    panel_y: float,
    panel_w: float,
    panel_h: float,
    sweep: dict,
    series: Sequence[dict],
    y_min: float,
    y_max: float,
    max_point_index: int,
    transform: str,
) -> List[str]:
    left = panel_x + 42.0
    right = panel_x + panel_w - 12.0
    top = panel_y + 22.0
    bottom = panel_y + panel_h - 36.0

    def sx(point_index: float) -> float:
        denom = max(float(max(max_point_index, 1)), 1.0)
        return left + float(point_index) * (right - left) / denom

    def sy(value: float) -> float:
        return bottom - (float(value) - y_min) * (bottom - top) / max(y_max - y_min, 1e-9)

    fixed_text = ", ".join("{} ({})".format(entry["anchor_label"], entry["anchor_idx"]) for entry in series)
    svg: List[str] = [
        f'<rect x="{panel_x:.1f}" y="{panel_y:.1f}" width="{panel_w:.1f}" height="{panel_h:.1f}" fill="#ffffff" stroke="#d0d0d0" stroke-width="1"/>',
        f'<text x="{panel_x + 8:.1f}" y="{panel_y + 14:.1f}" font-family="monospace" font-size="11" fill="#111111">{escape(str(sweep.get("id", "<unnamed>")))}</text>',
        f'<text x="{panel_x + 8:.1f}" y="{panel_y + 27:.1f}" font-family="monospace" font-size="9" fill="#444444">fixed: {escape(fixed_text)}</text>',
        f'<line x1="{left:.1f}" y1="{bottom:.1f}" x2="{right:.1f}" y2="{bottom:.1f}" stroke="#111111" stroke-width="1"/>',
        f'<line x1="{left:.1f}" y1="{top:.1f}" x2="{left:.1f}" y2="{bottom:.1f}" stroke="#111111" stroke-width="1"/>',
    ]

    x_ticks = _tick_values(0.0, float(max_point_index), ticks=min(max_point_index + 1, 5))
    for x_val in x_ticks:
        x_pos = sx(x_val)
        svg.append(f'<line x1="{x_pos:.1f}" y1="{bottom:.1f}" x2="{x_pos:.1f}" y2="{bottom + 4:.1f}" stroke="#111111" stroke-width="1"/>')
        svg.append(f'<text x="{x_pos:.1f}" y="{bottom + 16:.1f}" text-anchor="middle" font-family="monospace" font-size="8">{_fmt_num(x_val, 0)}</text>')

    for y_val in _tick_values(y_min, y_max, ticks=5):
        y_pos = sy(y_val)
        svg.append(f'<line x1="{left - 4:.1f}" y1="{y_pos:.1f}" x2="{left:.1f}" y2="{y_pos:.1f}" stroke="#111111" stroke-width="1"/>')
        svg.append(f'<text x="{left - 7:.1f}" y="{y_pos + 3.0:.1f}" text-anchor="end" font-family="monospace" font-size="8">{_fmt_num(y_val, 1)}</text>')

    if transform == "delta" and y_min <= 0.0 <= y_max:
        zero_y = sy(0.0)
        svg.append(f'<line x1="{left:.1f}" y1="{zero_y:.1f}" x2="{right:.1f}" y2="{zero_y:.1f}" stroke="#888888" stroke-width="1" stroke-dasharray="4,4"/>')

    svg.append(f'<text x="{0.5 * (left + right):.1f}" y="{panel_y + panel_h - 8:.1f}" text-anchor="middle" font-family="monospace" font-size="9">point index</text>')
    y_axis_label = "raw angle delta [deg]" if transform == "delta" else "raw angle [deg]"
    svg.append(
        f'<text x="{panel_x + 10:.1f}" y="{0.5 * (top + bottom):.1f}" text-anchor="middle" transform="rotate(-90 {panel_x + 10:.1f} {0.5 * (top + bottom):.1f})" font-family="monospace" font-size="9">{y_axis_label}</text>'
    )

    for entry in series:
        anchor_idx = int(entry["anchor_idx"])
        values = np.asarray(entry["plot_values"], dtype=float)
        points = []
        for i, value in enumerate(values):
            if np.isfinite(value):
                points.append("{:.2f},{:.2f}".format(sx(float(i)), sy(float(value))))
        if len(points) >= 2:
            svg.append(
                f'<polyline points="{" ".join(points)}" fill="none" stroke="{_series_color(anchor_idx)}" stroke-width="1.4"/>'
            )
        for i, value in enumerate(values):
            if not np.isfinite(value):
                continue
            svg.append(
                f'<circle cx="{sx(float(i)):.2f}" cy="{sy(float(value)):.2f}" r="2.4" fill="{_series_color(anchor_idx)}" stroke="#222222" stroke-width="0.5"/>'
            )

    legend_x = right - 76.0
    legend_y = panel_y + 39.0
    for idx, entry in enumerate(series):
        y0 = legend_y + idx * 14.0
        label_text = "{} ({})".format(entry["anchor_label"], entry["anchor_idx"])
        svg.append(f'<line x1="{legend_x:.1f}" y1="{y0:.1f}" x2="{legend_x + 14:.1f}" y2="{y0:.1f}" stroke="{_series_color(int(entry["anchor_idx"]))}" stroke-width="2"/>')
        svg.append(f'<text x="{legend_x + 18:.1f}" y="{y0 + 3.0:.1f}" font-family="monospace" font-size="8">{escape(label_text)}</text>')

    return svg


def _plot_series_grid(*, dataset: dict, label: str, out_path: Path, transform: str) -> None:
    prepared, y_min, y_max, max_point_index = _collect_series(dataset, transform=transform)
    rows, cols = _pick_grid_shape(len(prepared))

    panel_w = 360.0
    panel_h = 240.0
    margin_x = 18.0
    margin_y = 18.0
    title_h = 30.0
    width = margin_x * 2 + cols * panel_w
    height = margin_y * 2 + title_h + rows * panel_h

    svg: List[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width:.1f}" height="{height:.1f}" viewBox="0 0 {width:.1f} {height:.1f}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        f'<text x="{0.5 * width:.1f}" y="22" text-anchor="middle" font-family="monospace" font-size="16">{escape(f"{label}: fixed-anchor raw angles ({transform})")}</text>',
    ]

    for idx, entry in enumerate(prepared):
        row = idx // cols
        col = idx % cols
        panel_x = margin_x + col * panel_w
        panel_y = margin_y + title_h + row * panel_h
        svg.extend(
            _panel_svg(
                panel_x=panel_x,
                panel_y=panel_y,
                panel_w=panel_w,
                panel_h=panel_h,
                sweep=entry["sweep"],
                series=entry["series"],
                y_min=y_min,
                y_max=y_max,
                max_point_index=max_point_index,
                transform=transform,
            )
        )

    for idx in range(len(prepared), rows * cols):
        row = idx // cols
        col = idx % cols
        panel_x = margin_x + col * panel_w
        panel_y = margin_y + title_h + row * panel_h
        svg.append(
            f'<rect x="{panel_x:.1f}" y="{panel_y:.1f}" width="{panel_w:.1f}" height="{panel_h:.1f}" fill="#ffffff" stroke="#d0d0d0" stroke-width="1"/>'
        )

    svg.append("</svg>")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("\n".join(svg), encoding="utf-8")


def _write_outputs(dataset: dict, *, label: str, out_dir: Path) -> List[Path]:
    outputs = []
    raw_path = out_dir / f"{label}.fixed_anchor_raw_angles.svg"
    delta_path = out_dir / f"{label}.fixed_anchor_angle_deltas.svg"
    _plot_series_grid(dataset=dataset, label=label, out_path=raw_path, transform="raw")
    _plot_series_grid(dataset=dataset, label=label, out_path=delta_path, transform="delta")
    outputs.extend([raw_path, delta_path])
    return outputs


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Plot dataset diagnostics for fixed-anchor raw angles.")
    parser.add_argument("dataset", type=Path, help="Sweep dataset JSON")
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("autocal/data/.analysis/dataset_diagnostics"),
        help="Directory for output SVGs",
    )
    parser.add_argument("--label", type=str, default=None, help="Output file prefix")
    parser.add_argument("--limit-sweeps", type=int, default=None, help="Use only the first N sweeps")
    args = parser.parse_args(argv)

    dataset = json.loads(args.dataset.read_text(encoding="utf-8"))
    dataset = _subset_dataset(dataset, limit_sweeps=args.limit_sweeps)
    label = args.label or args.dataset.stem
    if args.limit_sweeps is not None and args.label is None:
        label = f"{label}_first{int(args.limit_sweeps)}"

    out_dir = Path(args.out_dir)
    outputs = _write_outputs(dataset, label=label, out_dir=out_dir)
    for path in outputs:
        print(f"Wrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
