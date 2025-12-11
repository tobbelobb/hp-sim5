#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Optional

import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from autocal.ellipse_fitting import ellipse_algebraic_distance, fit_ellipse_from_sweep


def _load_obs(path: Optional[Path]) -> Dict[str, list]:
    if not path:
        return {}
    with path.open("r", encoding="utf-8") as f:
        obs = json.load(f)
    return {s["id"]: s.get("histogram", []) for s in obs.get("sweeps", [])}


def _inflate_lengths(raw_points: Iterable[dict], base_length: float) -> tuple[np.ndarray, np.ndarray]:
    l_drive = np.array([p["l_drive"] + base_length for p in raw_points], dtype=float)
    l_sensor = np.array([p["l_sensor"] + base_length for p in raw_points], dtype=float)
    return l_drive, l_sensor


def _plot_single_sweep(
    sweep: dict,
    base_length: float,
    residual_threshold: float,
    obs_bins: Optional[list],
    out_path: Path,
) -> None:
    l_drive, l_sensor = _inflate_lengths(sweep["data_points"], base_length)
    result = fit_ellipse_from_sweep(l_drive, l_sensor, residual_threshold=residual_threshold)

    x = l_drive**2
    y = l_sensor**2

    if np.isnan(result.residual_rms):
        norm_residuals = np.zeros_like(x)
    else:
        scale = float(np.sqrt(np.mean(x**2 + y**2)))
        scale = 1.0 if scale < 1e-10 else scale
        norm_residuals = ellipse_algebraic_distance(result.coefficients, x, y) / scale

    colors = np.abs(norm_residuals)
    fig, (ax_xy, ax_res) = plt.subplots(1, 2, figsize=(12, 5))
    fig.suptitle(
        f"{sweep.get('id', '<unnamed>')} | fixed {sweep.get('fixed_anchors')}@{sweep.get('fixed_lengths')} | "
        f"drive {sweep.get('drive_anchor')} sensor {sweep.get('sensor_anchor')}"
    )

    # Squared-length plane with ellipse
    scatter = ax_xy.scatter(x, y, c=colors, cmap="magma", s=40, edgecolor="k", linewidth=0.3)
    ax_xy.plot(x, y, color="gray", alpha=0.4, linestyle="--", linewidth=1)

    cx, cy = result.center
    a, b = result.semi_axes
    theta = result.rotation_angle
    if a > 0 and b > 0:
        t = np.linspace(0, 2 * np.pi, 600)
        xe = cx + a * np.cos(t) * np.cos(theta) - b * np.sin(t) * np.sin(theta)
        ye = cy + a * np.cos(t) * np.sin(theta) + b * np.sin(t) * np.cos(theta)
        ax_xy.plot(xe, ye, color="tab:red", alpha=0.7, label="fit ellipse")
        ax_xy.legend(loc="best")

    x_margin = (x.max() - x.min()) * 0.05 if len(x) else 1
    y_margin = (y.max() - y.min()) * 0.05 if len(y) else 1
    ax_xy.set_xlim(x.min() - x_margin, x.max() + x_margin)
    ax_xy.set_ylim(y.min() - y_margin, y.max() + y_margin)
    ax_xy.set_xlabel("l_drive^2 (mm^2)")
    ax_xy.set_ylabel("l_sensor^2 (mm^2)")

    status = "VALID" if result.valid else "REJECTED"
    text_lines = [
        f"{status} (th={residual_threshold})",
        f"RMS={result.residual_rms:.3f}",
        f"max={result.residual_max:.3f}",
    ]
    if result.rejection_reason:
        text_lines.append(result.rejection_reason)
    ax_xy.text(
        0.02,
        0.98,
        "\n".join(text_lines),
        transform=ax_xy.transAxes,
        ha="left",
        va="top",
        bbox=dict(facecolor="white", alpha=0.8, edgecolor="gray"),
    )
    cbar = fig.colorbar(scatter, ax=ax_xy, shrink=0.8, pad=0.02)
    cbar.set_label("|normalized residual|")

    # Raw length plane with per-point residual colors
    ax_res.scatter(l_drive, l_sensor, c=colors, cmap="magma", s=40, edgecolor="k", linewidth=0.3)
    ax_res.plot(l_drive, l_sensor, color="gray", alpha=0.4, linestyle="--", linewidth=1)
    ax_res.set_xlabel("l_drive (mm, abs)")
    ax_res.set_ylabel("l_sensor (mm, abs)")
    ax_res.set_title("Raw lengths")

    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=200)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="Plot sweep data with fitted ellipses.")
    parser.add_argument("dataset", type=Path, help="Sweep dataset JSON")
    parser.add_argument("--obs", type=Path, help="Optional observability histogram JSON")
    parser.add_argument(
        "--base-length",
        type=float,
        default=0.0,
        help="Add this to l_drive/l_sensor before fitting (e.g. 1900 for slideprinter)",
    )
    parser.add_argument(
        "--residual-threshold",
        type=float,
        default=0.01,
        help="Threshold passed to fit_ellipse_from_sweep",
    )
    parser.add_argument(
        "--outdir",
        type=Path,
        default=Path("plots") / "sweep_fits",
        help="Directory for output PNGs",
    )
    args = parser.parse_args()

    with args.dataset.open("r", encoding="utf-8") as f:
        data = json.load(f)

    obs_by_id = _load_obs(args.obs)

    for sweep in data.get("sweeps", []):
        sid = sweep.get("id", "sweep")
        out_path = args.outdir / f"{sid}.png"
        _plot_single_sweep(
            sweep,
            base_length=args.base_length,
            residual_threshold=args.residual_threshold,
            obs_bins=obs_by_id.get(sid),
            out_path=out_path,
        )
        print(f"Wrote {out_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
