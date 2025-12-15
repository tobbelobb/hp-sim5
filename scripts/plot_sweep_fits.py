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

from autocal.ellipse_fitting import EllipseFitResult, ellipse_algebraic_distance, fit_ellipse_from_sweep
from autocal.flex import FlexModel


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
    flex_model: Optional[FlexModel],
    out_path: Path,
) -> "EllipseFitResult":
    l_drive, l_sensor = _inflate_lengths(sweep["data_points"], base_length)
    if flex_model is not None:
        drive_idx = int(sweep.get("drive_anchor", 0))
        sensor_idx = int(sweep.get("sensor_anchor", 0))
        t_drive = np.array(
            [p.get("assumed_tension_drive_n", 0.0) for p in sweep.get("data_points", [])], dtype=float
        )
        t_sensor = np.array(
            [p.get("assumed_tension_sensor_n", 0.0) for p in sweep.get("data_points", [])], dtype=float
        )
        if t_drive.shape == l_drive.shape and t_sensor.shape == l_sensor.shape:
            l_drive = flex_model.corrected_distance_mm(l_drive, t_drive, axis=drive_idx)
            l_sensor = flex_model.corrected_distance_mm(l_sensor, t_sensor, axis=sensor_idx)
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
    return result


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
    parser.add_argument(
        "--no-flex",
        action="store_true",
        help="Disable spring-based flex correction (auto-enabled if dataset includes config.m666 and tensions).",
    )
    parser.add_argument(
        "--spring-k-multiplier",
        type=float,
        default=1.0,
        help="Multiply M666 S by this factor (e.g. 2.0 for two parallel lines per axis).",
    )
    args = parser.parse_args()

    with args.dataset.open("r", encoding="utf-8") as f:
        data = json.load(f)

    obs_by_id = _load_obs(args.obs)

    flex_model: Optional[FlexModel] = None
    if not args.no_flex:
        m666 = (data.get("config") or {}).get("m666")
        if isinstance(m666, dict):
            num_axes = int(data.get("num_anchors", 0) or 0)
            flex_model = FlexModel.from_m666(
                m666,
                num_axes=num_axes,
                spring_k_multiplier=float(args.spring_k_multiplier),
            )

    for sweep in data.get("sweeps", []):
        sid = sweep.get("id", "sweep")
        out_path = args.outdir / f"{sid}.png"
        result = _plot_single_sweep(
            sweep,
            base_length=args.base_length,
            residual_threshold=args.residual_threshold,
            obs_bins=obs_by_id.get(sid),
            flex_model=flex_model,
            out_path=out_path,
        )
        status = "VALID" if result.valid else "REJECTED"
        reason = f" | {result.rejection_reason}" if result.rejection_reason else ""
        print(f"{sid} {status} rms={result.residual_rms:.6g} max={result.residual_max:.6g}{reason}")
        print(f"Wrote {out_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
