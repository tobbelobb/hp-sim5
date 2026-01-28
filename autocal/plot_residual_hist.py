#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def _parse_float(value: object) -> float | None:
    if value is None:
        return None
    try:
        val = float(value)
    except (TypeError, ValueError):
        return None
    if not np.isfinite(val):
        return None
    return float(val)


def _parse_int(value: object) -> int | None:
    if value is None:
        return None
    try:
        val = int(value)
    except (TypeError, ValueError):
        return None
    return int(val)


def _load_rows(csv_path: Path, *, column: str) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    with csv_path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if column not in row:
                continue
            val = _parse_float(row.get(column))
            if val is None:
                continue
            row["_value"] = val
            rows.append(row)
    return rows


def _sweep_top_values(rows: list[dict[str, object]], *, top_n: int = 2) -> list[float]:
    by_sweep: dict[str, list[float]] = {}
    for row in rows:
        sweep_id = row.get("sweep_id")
        if sweep_id is None:
            continue
        val = row.get("_value")
        if not isinstance(val, (int, float)):
            continue
        by_sweep.setdefault(str(sweep_id), []).append(float(val))
    top_values: list[float] = []
    for values in by_sweep.values():
        if not values:
            continue
        values_sorted = sorted(values, reverse=True)
        top_values.extend(values_sorted[: max(1, int(top_n))])
    return top_values


def _parse_points_arg(raw: str | None) -> list[int]:
    if not raw:
        return []
    out: list[int] = []
    for chunk in str(raw).split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        try:
            out.append(int(chunk))
        except ValueError:
            continue
    return out


def _point_values(rows: list[dict[str, object]], point_idx: int) -> list[float]:
    values: list[float] = []
    for row in rows:
        idx = _parse_int(row.get("point_idx"))
        if idx is None or idx != point_idx:
            continue
        values.append(float(row["_value"]))
    return values


def _cutoff_stats(rows: list[dict[str, object]]) -> dict[str, float] | None:
    values = [_parse_float(row.get("cutoff_mm")) for row in rows]
    cutoffs = [val for val in values if val is not None]
    if not cutoffs:
        return None
    arr = np.asarray(cutoffs, dtype=float)
    return {
        "median": float(np.median(arr)),
        "p10": float(np.percentile(arr, 10)),
        "p90": float(np.percentile(arr, 90)),
    }


def _subsweep_marker_values(
    rows: list[dict[str, object]],
    *,
    sweep_points: int | None,
) -> dict[str, list[float]]:
    by_sweep: dict[str, dict[int, float]] = {}
    max_idx: dict[str, int] = {}
    for row in rows:
        sweep_id = row.get("sweep_id")
        idx = _parse_int(row.get("point_idx"))
        if sweep_id is None or idx is None:
            continue
        sid = str(sweep_id)
        by_sweep.setdefault(sid, {})[idx] = float(row["_value"])
        if sid not in max_idx or idx > max_idx[sid]:
            max_idx[sid] = idx

    results = {
        "first_sub1": [],
        "last_sub1": [],
        "first_sub2": [],
        "last_sub2": [],
    }

    for sid, values in by_sweep.items():
        total_points = max_idx.get(sid, -1) + 1
        if total_points <= 0:
            continue
        if sweep_points is not None and sweep_points > 0:
            points = int(sweep_points)
        elif total_points % 2 == 0 and total_points >= 2:
            points = total_points // 2
        else:
            points = total_points

        idx0 = 0
        idx_last1 = points - 1
        if idx0 in values:
            results["first_sub1"].append(values[idx0])
        if idx_last1 in values:
            results["last_sub1"].append(values[idx_last1])

        if points > 0 and total_points >= 2 * points:
            idx_first2 = points
            idx_last2 = (2 * points) - 1
            if idx_first2 in values:
                results["first_sub2"].append(values[idx_first2])
            if idx_last2 in values:
                results["last_sub2"].append(values[idx_last2])

    return results


def _gamma_fit(values: list[float]) -> tuple[float, float] | None:
    arr = np.asarray(values, dtype=float)
    arr = arr[np.isfinite(arr)]
    if arr.size == 0:
        return None
    arr = arr[arr > 0.0]
    if arr.size == 0:
        return None
    mean = float(np.mean(arr))
    var = float(np.var(arr, ddof=0))
    if mean <= 0.0 or var <= 0.0:
        return None
    k = (mean * mean) / var
    theta = var / mean
    if not np.isfinite(k) or not np.isfinite(theta) or k <= 0.0 or theta <= 0.0:
        return None
    return k, theta


def _gamma_pdf(x: np.ndarray, k: float, theta: float) -> np.ndarray:
    x = np.asarray(x, dtype=float)
    coef = 1.0 / (math.gamma(k) * (theta ** k))
    with np.errstate(divide="ignore", invalid="ignore", over="ignore", under="ignore"):
        y = coef * np.power(x, k - 1.0) * np.exp(-x / theta)
    y = np.where(np.isfinite(y), y, 0.0)
    y = np.where(x <= 0.0, 0.0, y)
    return y


def _unique_path(path: Path) -> Path:
    if not path.exists():
        return path
    stem = path.stem
    suffix = path.suffix
    for idx in range(1, 10_000):
        candidate = path.with_name(f"{stem}_{idx}{suffix}")
        if not candidate.exists():
            return candidate
    raise RuntimeError(f"Could not find available filename for {path}")


def _overlay_hist(
    ax: plt.Axes,
    values: list[float],
    *,
    bins: np.ndarray,
    label: str,
    color: str,
    linestyle: str = "-",
    linewidth: float = 1.6,
    density: bool,
    total_count: int,
) -> None:
    if not values:
        return
    if density:
        counts, _ = np.histogram(values, bins=bins)
        widths = np.diff(bins)
        scale = float(total_count) if total_count > 0 else 1.0
        dens = counts / (scale * widths)
        ax.stairs(
            dens,
            bins,
            fill=False,
            color=color,
            linestyle=linestyle,
            linewidth=linewidth,
            label=label,
        )
    else:
        ax.hist(
            values,
            bins=bins,
            histtype="step",
            linewidth=linewidth,
            color=color,
            linestyle=linestyle,
            label=label,
            density=False,
        )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Plot a residual histogram from CSV output.")
    parser.add_argument("csv_path", type=Path, help="CSV file produced by --residuals-csv")
    parser.add_argument("--column", default="residual_mm", help="Column to plot (default: residual_mm)")
    parser.add_argument("--bins", type=int, default=60, help="Histogram bins (default: 60)")
    parser.add_argument("--max-mm", type=float, default=None, help="Optional max residual to include (mm)")
    parser.add_argument(
        "--sweep-points",
        type=int,
        default=None,
        help="Points per sub-sweep (used to label subsweep endpoints).",
    )
    parser.add_argument(
        "--density",
        dest="density",
        action="store_true",
        help="Plot a density histogram (default).",
    )
    parser.add_argument(
        "--counts",
        dest="density",
        action="store_false",
        help="Plot histogram counts instead of density.",
    )
    parser.add_argument(
        "--highlight",
        choices=("subsweeps", "first-last", "top2", "both", "none"),
        default="subsweeps",
        help="How to highlight per-sweep points in the histogram",
    )
    parser.add_argument(
        "--highlight-points",
        default=None,
        help="Comma-separated point_idx values to highlight (optional).",
    )
    parser.add_argument(
        "--gamma-fit",
        dest="gamma_fit",
        action="store_true",
        help="Fit and overlay a gamma distribution (default).",
    )
    parser.add_argument(
        "--no-gamma-fit",
        dest="gamma_fit",
        action="store_false",
        help="Disable the gamma fit overlay.",
    )
    parser.add_argument("--output", type=Path, default=None, help="Optional output PNG path")
    parser.set_defaults(density=True, gamma_fit=True)
    args = parser.parse_args(argv)

    rows = _load_rows(args.csv_path, column=args.column)
    if args.max_mm is not None:
        rows = [row for row in rows if float(row.get("_value", 0.0)) <= args.max_mm]

    if not rows:
        raise SystemExit("No residuals to plot.")

    values = [float(row["_value"]) for row in rows]
    bins = np.histogram_bin_edges(values, bins=int(args.bins))

    subsweep_values = {"first_sub1": [], "last_sub1": [], "first_sub2": [], "last_sub2": []}
    top2_values: list[float] = []
    highlight_points = _parse_points_arg(args.highlight_points)
    highlight_values: list[tuple[int, list[float]]] = []
    if highlight_points:
        for point_idx in highlight_points:
            vals = _point_values(rows, point_idx)
            if vals:
                highlight_values.append((point_idx, vals))

    if args.highlight in ("subsweeps", "first-last", "both"):
        subsweep_values = _subsweep_marker_values(rows, sweep_points=args.sweep_points)

    if args.highlight in ("top2", "both"):
        top2_values = _sweep_top_values(rows, top_n=2)

    fig, ax = plt.subplots(figsize=(8.5, 5.0))
    ax.hist(
        values,
        bins=bins,
        color="#4B5563",
        alpha=0.6,
        label=f"all points (n={len(values)})",
        density=bool(args.density),
    )

    total_count = len(values)
    if subsweep_values["first_sub1"]:
        _overlay_hist(
            ax,
            subsweep_values["first_sub1"],
            bins=bins,
            label=f"first point subsweep 1 (n={len(subsweep_values['first_sub1'])})",
            color="#D1495B",
            density=bool(args.density),
            total_count=total_count,
        )
    if subsweep_values["last_sub1"]:
        _overlay_hist(
            ax,
            subsweep_values["last_sub1"],
            bins=bins,
            label=f"last point subsweep 1 (n={len(subsweep_values['last_sub1'])})",
            color="#2E86AB",
            density=bool(args.density),
            total_count=total_count,
        )
    if subsweep_values["first_sub2"]:
        _overlay_hist(
            ax,
            subsweep_values["first_sub2"],
            bins=bins,
            label=f"first point subsweep 2 (n={len(subsweep_values['first_sub2'])})",
            color="#7C3AED",
            density=bool(args.density),
            total_count=total_count,
        )
    if subsweep_values["last_sub2"]:
        _overlay_hist(
            ax,
            subsweep_values["last_sub2"],
            bins=bins,
            label=f"last point subsweep 2 (n={len(subsweep_values['last_sub2'])})",
            color="#0EA5E9",
            density=bool(args.density),
            total_count=total_count,
        )
    if top2_values:
        _overlay_hist(
            ax,
            top2_values,
            bins=bins,
            label=f"top-2 residuals per sweep (n={len(top2_values)})",
            color="#E17C05",
            linewidth=1.8,
            density=bool(args.density),
            total_count=total_count,
        )
    if highlight_values:
        palette = ["#16A34A", "#7C3AED", "#F97316", "#0EA5E9", "#C026D3", "#4B5563"]
        for idx, (point_idx, vals) in enumerate(highlight_values):
            color = palette[idx % len(palette)]
            _overlay_hist(
                ax,
                vals,
                bins=bins,
                label=f"point {point_idx} per sweep (n={len(vals)})",
                color=color,
                linestyle="--",
                density=bool(args.density),
                total_count=total_count,
            )

    gamma_fit = _gamma_fit(values) if args.gamma_fit else None
    if gamma_fit is not None:
        k, theta = gamma_fit
        positive = [v for v in values if v > 0.0]
        max_val = max(positive) if positive else max(values)
        if positive:
            x_min = max(min(positive) * 0.5, max_val * 1e-6, 1e-6)
        else:
            x_min = max_val * 1e-6
        if x_min >= max_val:
            x_min = max_val * 1e-6
        x = np.linspace(x_min, max_val, 240)
        y = _gamma_pdf(x, k, theta)
        if not args.density:
            bin_width = float(np.mean(np.diff(bins))) if len(bins) > 1 else 1.0
            y = y * len(values) * bin_width
        ax.plot(x, y, color="#F97316", linewidth=2.0, label=f"gamma fit (k={k:.2f}, theta={theta:.3f} mm)")

    cutoff_stats = _cutoff_stats(rows)
    if cutoff_stats is not None:
        cutoff_med = cutoff_stats["median"]
        cutoff_p10 = cutoff_stats["p10"]
        cutoff_p90 = cutoff_stats["p90"]
        if cutoff_p90 > cutoff_p10:
            ax.axvspan(
                cutoff_p10,
                cutoff_p90,
                color="#10B981",
                alpha=0.12,
                label=f"cutoff 10-90% ({cutoff_p10:.4g}-{cutoff_p90:.4g} mm)",
            )
        ax.axvline(
            cutoff_med,
            color="#047857",
            linestyle="-",
            linewidth=2.0,
            label=f"cutoff median ({cutoff_med:.4g} mm)",
        )

    x_label = "Residual error (mm)" if str(args.column).endswith("_mm") else str(args.column)
    ax.set_xlabel(x_label)
    ax.set_ylabel("Density" if args.density else "Count of points")
    if gamma_fit is not None:
        k, theta = gamma_fit
        ax.set_title(f"Gamma fit (k={k:.2f}, theta={theta:.3f} mm)")
    else:
        ax.set_title(f"Residual histogram ({args.column})")
    ax.grid(axis="y", alpha=0.25)
    ax.legend(frameon=False, fontsize=9)
    fig.tight_layout()

    if args.output:
        output_path = _unique_path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_path, dpi=150)
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
