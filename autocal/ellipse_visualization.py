from __future__ import annotations

"""Visualization helpers for elliptical feature calibration.

These plots are intended for debugging and monitoring sweep quality, ellipse
fits, and optimization progress. They operate on the sweep JSON/dict format
and on sidecar fitted-ellipse dicts produced by `autocal.fit_ellipses`.
"""

from dataclasses import asdict
from typing import Dict, Iterable, List, Optional, Tuple, Union

import numpy as np

try:
    import matplotlib.pyplot as plt
except Exception as exc:  # pragma: no cover
    raise RuntimeError("matplotlib is required for ellipse visualization") from exc

from autocal.ellipse_fitting import ellipse_geometric_params
from autocal.sweep_types import Sweep


def _extract_points(
    sweep: Union[Sweep, dict],
) -> Tuple[np.ndarray, np.ndarray]:
    if isinstance(sweep, Sweep):
        l_drive = np.array([p.l_drive for p in sweep.data_points], dtype=float)
        l_sensor = np.array([p.l_sensor for p in sweep.data_points], dtype=float)
    else:
        data_points = sweep.get("data_points", [])
        l_drive = np.array([p.get("l_drive") for p in data_points], dtype=float)
        l_sensor = np.array([p.get("l_sensor") for p in data_points], dtype=float)
    return l_drive, l_sensor


def _extract_id_and_cfg(sweep: Union[Sweep, dict]) -> Tuple[str, Dict]:
    if isinstance(sweep, Sweep):
        return sweep.id, asdict(sweep)
    return sweep.get("id", ""), sweep


def plot_sweep_data(
    sweep: Union[Sweep, dict],
    ax: Optional["plt.Axes"] = None,
    title: Optional[str] = None,
    show_squared: bool = True,
) -> "plt.Axes":
    """Plot raw sweep data points.

    If `show_squared` is True, plots L² vs L²; else L vs L.
    Points are colored by index to show arc progression.
    """
    if ax is None:
        _, ax = plt.subplots(figsize=(8, 6))

    sweep_id, cfg = _extract_id_and_cfg(sweep)
    l_drive, l_sensor = _extract_points(sweep)

    if show_squared:
        x = l_drive**2
        y = l_sensor**2
        xlabel = f"L²_drive (anchor {cfg.get('drive_anchor')})"
        ylabel = f"L²_sensor (anchor {cfg.get('sensor_anchor')})"
    else:
        x = l_drive
        y = l_sensor
        xlabel = f"L_drive (anchor {cfg.get('drive_anchor')})"
        ylabel = f"L_sensor (anchor {cfg.get('sensor_anchor')})"

    colors = np.linspace(0, 1, len(x)) if len(x) else np.array([])
    scatter = ax.scatter(x, y, c=colors, cmap="viridis", s=30, alpha=0.7)
    if len(x):
        plt.colorbar(scatter, ax=ax, label="Point index")

    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    fixed = cfg.get("fixed_anchors", [])
    ax.set_title(title or f"Sweep {sweep_id}\nFixed: {fixed}")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)

    return ax


def _coeffs_array(fitted_ellipse: dict) -> np.ndarray:
    coeffs = fitted_ellipse.get("coefficients", {})
    if isinstance(coeffs, np.ndarray):
        return coeffs.astype(float)
    return np.array(
        [
            coeffs.get("A", 0.0),
            coeffs.get("B", 0.0),
            coeffs.get("C", 0.0),
            coeffs.get("D", 0.0),
            coeffs.get("E", 0.0),
            coeffs.get("F", 0.0),
        ],
        dtype=float,
    )


def _plot_ellipse_from_coeffs(
    ax: "plt.Axes",
    coeffs: np.ndarray,
    color: str = "green",
    linestyle: str = "-",
    linewidth: float = 2.0,
    label: Optional[str] = None,
    num_points: int = 300,
    alpha: float = 1.0,
) -> None:
    center, semi_axes, theta = ellipse_geometric_params(coeffs)
    a, b = semi_axes
    if a <= 0 or b <= 0:
        return

    t = np.linspace(0, 2 * np.pi, num_points)
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    cx, cy = center

    x_ell = a * np.cos(t)
    y_ell = b * np.sin(t)
    x_plot = cx + x_ell * cos_t - y_ell * sin_t
    y_plot = cy + x_ell * sin_t + y_ell * cos_t

    ax.plot(
        x_plot,
        y_plot,
        color=color,
        linestyle=linestyle,
        linewidth=linewidth,
        label=label,
        alpha=alpha,
    )


def _plot_theoretical_arc_near_data(
    ax: "plt.Axes",
    coeffs: np.ndarray,
    x_obs: np.ndarray,
    y_obs: np.ndarray,
    color: str = "tab:red",
    linestyle: str = "--",
    linewidth: float = 2.0,
    label: Optional[str] = None,
    num_points: int = 300,
    padding_rad: float = 0.15,
) -> None:
    """Plot only the arc of an ellipse closest to observed data.

    This disambiguates short-arc sweeps where many ellipses can fit locally:
    we find the parametric angles implied by the data in ellipse-aligned
    coordinates, unwrap them, and render only that angular span.
    """
    center, semi_axes, theta = ellipse_geometric_params(coeffs)
    a, b = semi_axes
    if a <= 0 or b <= 0 or x_obs.size == 0:
        _plot_ellipse_from_coeffs(
            ax,
            coeffs,
            color=color,
            linestyle=linestyle,
            linewidth=linewidth,
            label=label,
            num_points=num_points,
        )
        return

    cx, cy = center
    cos_t = float(np.cos(theta))
    sin_t = float(np.sin(theta))

    # Transform observed points into ellipse-aligned coordinates.
    dx = x_obs - cx
    dy = y_obs - cy
    x_rot = cos_t * dx + sin_t * dy
    y_rot = -sin_t * dx + cos_t * dy

    # Estimate param angles for each observed point.
    phi = np.arctan2(y_rot / b, x_rot / a)
    phi = np.unwrap(phi)

    phi_min = float(np.min(phi) - padding_rad)
    phi_max = float(np.max(phi) + padding_rad)
    if not np.isfinite(phi_min) or not np.isfinite(phi_max):
        _plot_ellipse_from_coeffs(
            ax,
            coeffs,
            color=color,
            linestyle=linestyle,
            linewidth=linewidth,
            label=label,
            num_points=num_points,
        )
        return

    t = np.linspace(phi_min, phi_max, num_points)
    x_ell = a * np.cos(t)
    y_ell = b * np.sin(t)

    x_plot = cx + x_ell * cos_t - y_ell * sin_t
    y_plot = cy + x_ell * sin_t + y_ell * cos_t

    ax.plot(
        x_plot,
        y_plot,
        color=color,
        linestyle=linestyle,
        linewidth=linewidth,
        label=label,
    )


def plot_ellipse_fit(
    sweep: Union[Sweep, dict],
    fitted_ellipse: dict,
    theoretical_coeffs: Optional[np.ndarray] = None,
    align_theory_to_data: bool = False,
    ax: Optional["plt.Axes"] = None,
) -> "plt.Axes":
    """Plot sweep data with fitted (and optionally theoretical) ellipse overlays."""
    if ax is None:
        _, ax = plt.subplots(figsize=(10, 8))

    sweep_id, cfg = _extract_id_and_cfg(sweep)
    l_drive, l_sensor = _extract_points(sweep)
    x = l_drive**2
    y = l_sensor**2

    ax.scatter(x, y, c="tab:blue", s=30, alpha=0.6, label="Data points")

    coeffs_arr = _coeffs_array(fitted_ellipse)
    # Most sweeps trace only a short arc of the full ellipse. Rendering the whole
    # curve is visually misleading, so by default show the local arc nearest data.
    _plot_theoretical_arc_near_data(
        ax,
        coeffs_arr,
        x_obs=x,
        y_obs=y,
        color="tab:green",
        linestyle="-",
        linewidth=2.0,
        label="Fitted ellipse (nearest arc)",
        padding_rad=0.05,
    )

    if theoretical_coeffs is not None:
        theory_arr = np.asarray(theoretical_coeffs, dtype=float)
        if align_theory_to_data:
            # Show the full ellipse for global context, then highlight the local arc
            # nearest the observed data for disambiguation.
            _plot_ellipse_from_coeffs(
                ax,
                theory_arr,
                color="tab:red",
                linestyle="--",
                linewidth=1.5,
                alpha=0.35,
                label="Theoretical ellipse",
            )
            _plot_theoretical_arc_near_data(
                ax,
                theory_arr,
                x_obs=x,
                y_obs=y,
                color="tab:red",
                linestyle="-",
                linewidth=2.5,
                label="Theoretical arc (nearest)",
            )
        else:
            _plot_ellipse_from_coeffs(
                ax,
                theory_arr,
                color="tab:red",
                linestyle="--",
                label="Theoretical ellipse",
            )

    rms = float(fitted_ellipse.get("residual_rms", float("nan")))
    rmax = float(fitted_ellipse.get("residual_max", float("nan")))
    valid = bool(fitted_ellipse.get("valid", False))
    status = "VALID" if valid else "REJECTED"
    reason = fitted_ellipse.get("rejection_reason")

    text_lines = [f"{status}", f"RMS={rms:.4f}", f"max={rmax:.4f}"]
    if reason:
        text_lines.append(str(reason))

    ax.text(
        0.02,
        0.98,
        "\n".join(text_lines),
        transform=ax.transAxes,
        fontsize=10,
        va="top",
        fontfamily="monospace",
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
    )

    drive_anchor = cfg.get("drive_anchor") or fitted_ellipse.get("sweep_config", {}).get(
        "drive_anchor"
    )
    sensor_anchor = cfg.get("sensor_anchor") or fitted_ellipse.get("sweep_config", {}).get(
        "sensor_anchor"
    )

    ax.set_xlabel(f"L²_drive (anchor {drive_anchor})")
    ax.set_ylabel(f"L²_sensor (anchor {sensor_anchor})")
    ax.set_title(f"Ellipse Fit: {fitted_ellipse.get('sweep_id', sweep_id)}")
    ax.legend(loc="lower right")
    ax.grid(True, alpha=0.3)

    return ax


def plot_all_sweeps(
    dataset: dict,
    ncols: int = 3,
    figsize: Optional[Tuple[int, int]] = None,
    ellipse_fits: Optional[List[dict]] = None,
) -> "plt.Figure":
    """Plot all sweeps in a dataset as a grid, optionally overlaying fits."""
    sweeps = dataset.get("sweeps", [])
    fitted_by_id = {e.get("sweep_id", ""): e for e in (ellipse_fits or [])}

    n = len(sweeps)
    nrows = (n + ncols - 1) // ncols
    if figsize is None:
        figsize = (5 * ncols, 4 * nrows)

    fig, axes = plt.subplots(nrows, ncols, figsize=figsize)
    axes = np.atleast_1d(axes).flatten()

    for i, sweep in enumerate(sweeps):
        ax = axes[i]
        sid = sweep.get("id", "")
        if sid in fitted_by_id:
            plot_ellipse_fit(sweep, fitted_by_id[sid], ax=ax)
        else:
            plot_sweep_data(sweep, ax=ax)

    for i in range(n, len(axes)):
        axes[i].set_visible(False)

    fig.tight_layout()
    return fig


def plot_sampling_density(
    sweep: Union[Sweep, dict],
    fitted_ellipse: Optional[dict] = None,
    bins: int = 20,
    ax: Optional["plt.Axes"] = None,
) -> "plt.Axes":
    """Show sampling density along the drive arc, with residual σ overlay."""
    if ax is None:
        _, ax = plt.subplots(figsize=(8, 4))

    sweep_id, _ = _extract_id_and_cfg(sweep)
    l_drive, _l_sensor = _extract_points(sweep)

    hist, edges = np.histogram(l_drive, bins=bins)
    centers = 0.5 * (edges[:-1] + edges[1:])

    ax.bar(
        centers,
        hist,
        width=np.diff(edges),
        alpha=0.5,
        color="steelblue",
        label="Samples",
    )

    if fitted_ellipse and fitted_ellipse.get("residual_series"):
        residuals = np.asarray(fitted_ellipse["residual_series"], dtype=float)
        if len(residuals) == len(l_drive):
            res_sigma = []
            for i in range(len(edges) - 1):
                mask = (l_drive >= edges[i]) & (l_drive < edges[i + 1])
                res_sigma.append(float(np.std(residuals[mask])) if np.any(mask) else 0.0)
            ax.plot(
                centers,
                res_sigma,
                color="darkred",
                marker="o",
                label="Residual σ",
            )

    ax.set_xlabel("Drive length delta (mm)")
    ax.set_ylabel("Count")
    ax.set_title(f"Sampling density for {sweep_id}")
    ax.legend()
    ax.grid(True, alpha=0.3)
    return ax


def plot_cost_convergence(
    cost_history: List[float],
    ax: Optional["plt.Axes"] = None,
) -> "plt.Axes":
    """Plot optimization cost over iterations."""
    if ax is None:
        _, ax = plt.subplots(figsize=(10, 4))

    iterations = np.arange(len(cost_history))
    ax.semilogy(iterations, cost_history, "b-", linewidth=1.5)
    ax.set_xlabel("Iteration")
    ax.set_ylabel("Cost (log scale)")
    ax.set_title("Optimization Convergence")
    ax.grid(True, alpha=0.3)

    if cost_history:
        min_idx = int(np.argmin(cost_history))
        min_cost = float(cost_history[min_idx])
        ax.plot(min_idx, min_cost, "ro", markersize=8, label=f"Min: {min_cost:.2e}")
        ax.legend()

    return ax


def plot_anchors_2d(
    anchors: np.ndarray,
    true_anchors: Optional[np.ndarray] = None,
    ax: Optional["plt.Axes"] = None,
    labels: Optional[List[str]] = None,
) -> "plt.Axes":
    """Plot 2D anchor positions, optionally with true anchors."""
    anchors = np.asarray(anchors, dtype=float)
    if ax is None:
        _, ax = plt.subplots(figsize=(8, 8))

    n = len(anchors)
    if labels is None:
        labels = [str(i) for i in range(n)]

    ax.scatter(
        anchors[:, 0],
        anchors[:, 1],
        c="tab:blue",
        s=200,
        marker="o",
        label="Estimated",
        zorder=5,
    )
    for i, (x, y) in enumerate(anchors):
        ax.annotate(
            labels[i],
            (x, y),
            textcoords="offset points",
            xytext=(10, 10),
            fontsize=12,
            color="tab:blue",
        )

    if true_anchors is not None:
        true_anchors = np.asarray(true_anchors, dtype=float)
        ax.scatter(
            true_anchors[:, 0],
            true_anchors[:, 1],
            c="tab:red",
            s=200,
            marker="x",
            linewidths=3,
            label="True",
            zorder=4,
        )
        for i in range(min(n, len(true_anchors))):
            ax.plot(
                [anchors[i, 0], true_anchors[i, 0]],
                [anchors[i, 1], true_anchors[i, 1]],
                "r--",
                alpha=0.5,
                linewidth=1,
            )

    ax.scatter(
        [0],
        [0],
        c="black",
        s=100,
        marker="+",
        linewidths=2,
        label="Origin",
        zorder=3,
    )

    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_title("Anchor Positions")
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal")

    return ax


def plot_anchors_3d(
    anchors: np.ndarray,
    true_anchors: Optional[np.ndarray] = None,
    labels: Optional[List[str]] = None,
) -> "plt.Figure":
    """Plot 3D anchor positions, optionally with true anchors.

    If the current matplotlib installation lacks 3D projection support, this
    falls back to a 2D (x,y) plot so visualization remains usable.
    """
    anchors = np.asarray(anchors, dtype=float)
    fig = plt.figure(figsize=(10, 8))
    try:
        ax = fig.add_subplot(111, projection="3d")
        is_3d = True
    except Exception:  # pragma: no cover
        ax = fig.add_subplot(111)
        is_3d = False

    n = len(anchors)
    if labels is None:
        labels = [str(i) for i in range(n)]

    if is_3d:
        ax.scatter(
            anchors[:, 0],
            anchors[:, 1],
            anchors[:, 2],
            c="tab:blue",
            s=200,
            marker="o",
            label="Estimated",
        )
        for i, (x, y, z) in enumerate(anchors):
            ax.text(x, y, z, f"  {labels[i]}", fontsize=10, color="tab:blue")
    else:
        ax.scatter(
            anchors[:, 0],
            anchors[:, 1],
            c="tab:blue",
            s=200,
            marker="o",
            label="Estimated (xy only)",
        )
        for i, (x, y, _z) in enumerate(anchors):
            ax.annotate(labels[i], (x, y), textcoords="offset points", xytext=(10, 10))

    if true_anchors is not None:
        true_anchors = np.asarray(true_anchors, dtype=float)
        if is_3d:
            ax.scatter(
                true_anchors[:, 0],
                true_anchors[:, 1],
                true_anchors[:, 2],
                c="tab:red",
                s=200,
                marker="x",
                linewidths=3,
                label="True",
            )
            for i in range(min(n, len(true_anchors))):
                ax.plot(
                    [anchors[i, 0], true_anchors[i, 0]],
                    [anchors[i, 1], true_anchors[i, 1]],
                    [anchors[i, 2], true_anchors[i, 2]],
                    "r--",
                    alpha=0.5,
                )
        else:
            ax.scatter(
                true_anchors[:, 0],
                true_anchors[:, 1],
                c="tab:red",
                s=200,
                marker="x",
                linewidths=3,
                label="True (xy only)",
            )
            for i in range(min(n, len(true_anchors))):
                ax.plot(
                    [anchors[i, 0], true_anchors[i, 0]],
                    [anchors[i, 1], true_anchors[i, 1]],
                    "r--",
                    alpha=0.5,
                )

    if is_3d:
        ax.scatter([0], [0], [0], c="black", s=100, marker="+", linewidths=2, label="Origin")
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_zlabel("Z (mm)")
        ax.set_title("Anchor Positions")
    else:
        ax.scatter([0], [0], c="black", s=100, marker="+", linewidths=2, label="Origin")
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_title("Anchor Positions (3D unavailable)")
        ax.set_aspect("equal", adjustable="box")

    ax.legend()

    return fig


def plot_residuals_comparison(
    observed_ellipses: List[dict],
    predicted_coeffs: List[np.ndarray],
    sweep_ids: List[str],
    ax: Optional["plt.Axes"] = None,
) -> "plt.Axes":
    """Bar chart comparing observed vs predicted ellipse coefficients."""
    if ax is None:
        _, ax = plt.subplots(figsize=(12, 6))

    distances = []
    for obs, pred in zip(observed_ellipses, predicted_coeffs):
        obs_arr = _coeffs_array(obs)
        pred_arr = np.asarray(pred, dtype=float)
        obs_norm = obs_arr / (np.linalg.norm(obs_arr[:3]) or 1.0)
        pred_norm = pred_arr / (np.linalg.norm(pred_arr[:3]) or 1.0)
        dist = min(
            np.linalg.norm(obs_norm - pred_norm),
            np.linalg.norm(obs_norm + pred_norm),
        )
        distances.append(float(dist))

    colors = [
        "tab:green" if d < 0.1 else "tab:orange" if d < 0.3 else "tab:red"
        for d in distances
    ]

    x = np.arange(len(sweep_ids))
    ax.bar(x, distances, color=colors)
    ax.set_xlabel("Sweep")
    ax.set_ylabel("Coefficient Distance")
    ax.set_title("Observed vs Predicted Ellipse Mismatch")
    ax.set_xticks(x)
    ax.set_xticklabels(sweep_ids, rotation=45, ha="right")
    ax.axhline(y=0.1, color="tab:green", linestyle="--", alpha=0.5, label="Good")
    ax.axhline(y=0.3, color="tab:orange", linestyle="--", alpha=0.5, label="Acceptable")
    ax.legend()
    plt.tight_layout()
    return ax


def create_calibration_report(
    dataset: dict,
    solution: dict,
    ellipse_fits: Optional[List[dict]] = None,
    output_path: str = "calibration_report.png",
) -> "plt.Figure":
    """Create a compact multi-panel calibration report."""
    fig = plt.figure(figsize=(16, 12))
    gs = fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)

    anchors = solution.get("anchors")
    anchors_arr = None
    if anchors is not None:
        anchors_arr = np.asarray(anchors, dtype=float)
        if anchors_arr.ndim == 2 and anchors_arr.shape[1] == 2:
            ax_anchors = fig.add_subplot(gs[0, 0])
            plot_anchors_2d(anchors_arr, ax=ax_anchors)
        elif anchors_arr.ndim == 2 and anchors_arr.shape[1] >= 3:
            ax_anchors = fig.add_subplot(gs[0, 0], projection="3d")
            ax_anchors.scatter(
                anchors_arr[:, 0], anchors_arr[:, 1], anchors_arr[:, 2]
            )
            ax_anchors.set_title("Anchor Positions")

    def _inflate_sweep_for_plot(sweep: dict) -> dict:
        if anchors_arr is None or anchors_arr.ndim != 2:
            return sweep
        baselines = np.linalg.norm(anchors_arr, axis=1)
        drive_idx = int(sweep.get("drive_anchor", 0))
        sensor_idx = int(sweep.get("sensor_anchor", 0))
        fixed_anchors = list(sweep.get("fixed_anchors", []))
        fixed_lengths = list(sweep.get("fixed_lengths", []))
        fixed_lengths_abs = [
            float(baselines[idx] + delta) for idx, delta in zip(fixed_anchors, fixed_lengths)
        ]
        data_points = sweep.get("data_points", [])
        abs_points = []
        for p in data_points:
            abs_points.append(
                {
                    **p,
                    "l_drive": float(p.get("l_drive", 0.0) + baselines[drive_idx]),
                    "l_sensor": float(p.get("l_sensor", 0.0) + baselines[sensor_idx]),
                }
            )
        return {**sweep, "fixed_lengths": fixed_lengths_abs, "data_points": abs_points}

    sweeps = dataset.get("sweeps", [])[:2]
    fitted_by_id = {e.get("sweep_id", ""): e for e in (ellipse_fits or [])}
    for i, sweep in enumerate(sweeps):
        ax = fig.add_subplot(gs[0, 1 + i])
        sid = sweep.get("id", "")
        sweep_plot = _inflate_sweep_for_plot(sweep) if isinstance(sweep, dict) else sweep
        if sid in fitted_by_id:
            plot_ellipse_fit(sweep_plot, fitted_by_id[sid], ax=ax)
        else:
            plot_sweep_data(sweep_plot, ax=ax)

    ax_costs = fig.add_subplot(gs[1, :2])
    details = solution.get("details")
    per_sweep_costs = None
    if details is not None:
        per_sweep_costs = getattr(details, "per_sweep_costs", None)
        if per_sweep_costs is None and isinstance(details, dict):
            per_sweep_costs = details.get("per_sweep_costs")

    if per_sweep_costs:
        sweep_ids = list(per_sweep_costs.keys())
        costs = list(per_sweep_costs.values())
        ax_costs.bar(range(len(sweep_ids)), costs)
        ax_costs.set_xlabel("Sweep")
        ax_costs.set_ylabel("Cost")
        ax_costs.set_title("Per-Sweep Costs")
        ax_costs.set_xticks(range(len(sweep_ids)))
        ax_costs.set_xticklabels(sweep_ids, rotation=45, ha="right")
    else:
        ax_costs.text(0.5, 0.5, "No per-sweep cost data", ha="center", va="center")
        ax_costs.set_axis_off()

    ax_summary = fig.add_subplot(gs[1, 2])
    ax_summary.axis("off")
    valid_fit_count = len([e for e in (ellipse_fits or []) if e.get("valid")])
    cost_val = solution.get("cost", float("nan"))

    summary_text = (
        "Calibration Summary\n"
        "==================\n\n"
        f"Machine Type: {dataset.get('machine_type', 'unknown')}\n"
        f"Num Anchors: {dataset.get('num_anchors', 'unknown')}\n"
        f"Dimensions: {dataset.get('dimensions', 'unknown')}\n\n"
        f"Total Sweeps: {len(dataset.get('sweeps', []))}\n"
        f"Ellipse Fits Shown: {valid_fit_count}\n\n"
        f"Final Cost: {float(cost_val):.6e}\n"
        f"Success: {solution.get('success', False)}\n"
    )
    ax_summary.text(
        0.05,
        0.95,
        summary_text,
        transform=ax_summary.transAxes,
        fontsize=10,
        va="top",
        fontfamily="monospace",
        bbox=dict(boxstyle="round", facecolor="lightgray", alpha=0.5),
    )

    ax_gcode = fig.add_subplot(gs[2, :])
    ax_gcode.axis("off")
    if anchors is not None:
        from autocal.ellipse_solver import format_anchors_gcode

        try:
            gcode = format_anchors_gcode(np.asarray(anchors, dtype=float), dataset.get("machine_type", ""))
            ax_gcode.text(
                0.5,
                0.5,
                f"Generated G-code:\n\n{gcode}",
                transform=ax_gcode.transAxes,
                fontsize=12,
                ha="center",
                va="center",
                fontfamily="monospace",
                bbox=dict(boxstyle="round", facecolor="lightyellow", alpha=0.8),
            )
        except Exception as exc:  # pragma: no cover
            ax_gcode.text(0.5, 0.5, f"G-code formatting failed: {exc}", ha="center", va="center")

    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    return fig


def _load_json(path: str) -> dict:
    import json

    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def main(argv: Optional[List[str]] = None) -> int:  # pragma: no cover
    import argparse

    parser = argparse.ArgumentParser(description="Visualize calibration sweep data.")
    parser.add_argument("input", help="Input sweep dataset JSON")
    parser.add_argument("-o", "--output", help="Output image file")
    parser.add_argument("--sweep", help="Specific sweep ID to plot")
    parser.add_argument("--fits", help="Optional JSON sidecar with fitted ellipses")

    args = parser.parse_args(argv)

    dataset = _load_json(args.input)

    ellipse_fits: List[dict] = []
    if args.fits:
        sidecar = _load_json(args.fits)
        ellipse_fits = sidecar.get("fitted_ellipses", sidecar.get("ellipses", []))

    fitted_by_id = {e.get("sweep_id", ""): e for e in ellipse_fits}

    if args.sweep:
        sweep = next((s for s in dataset.get("sweeps", []) if s.get("id") == args.sweep), None)
        if sweep is None:
            raise ValueError(f"Sweep {args.sweep} not found")
        fitted = fitted_by_id.get(args.sweep)
        if fitted:
            plot_ellipse_fit(sweep, fitted)
        else:
            plot_sweep_data(sweep)
    else:
        plot_all_sweeps(dataset, ellipse_fits=ellipse_fits)

    if args.output:
        plt.savefig(args.output, dpi=150, bbox_inches="tight")
    else:
        plt.show()

    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
