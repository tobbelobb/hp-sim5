# Substep 6: Visualization Module

## Overview

Create visualization tools for debugging and monitoring the calibration process. This includes plots of raw sweep data, fitted vs theoretical ellipses, cost function landscapes, and anchor position convergence.

Raw plots visualize the encoder-relative lengths collected from origin; if you need to see absolute lengths, reconstruct them with the current anchor guess before plotting theoretical overlays so both curves share the same baseline.

## Implementation Details

All plots read the `sweep_config` snapshot shipped with each `FittedEllipse` so overlays stay consistent through the Phase 1 → Phase 2 handoff.

### 6.1 Core Visualization (`ellipse_visualization.py`)

```python
"""
ellipse_visualization.py

Visualization tools for elliptical feature calibration.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse as MplEllipse
from matplotlib.collections import LineCollection
from mpl_toolkits.mplot3d import Axes3D
from typing import List, Dict, Optional, Tuple
import json


def plot_sweep_data(
    sweep: dict,
    ax: Optional[plt.Axes] = None,
    title: Optional[str] = None,
    show_squared: bool = True
) -> plt.Axes:
    """
    Plot raw sweep data points.

    Parameters:
        sweep: Sweep dict with data_points
        ax: Matplotlib axes (created if None)
        title: Plot title
        show_squared: If True, plot L² vs L²; else L vs L

    Returns:
        Matplotlib axes
    """
    if ax is None:
        fig, ax = plt.subplots(figsize=(8, 6))

    data_points = sweep['data_points']
    l_drive = np.array([p['l_drive'] for p in data_points])
    l_sensor = np.array([p['l_sensor'] for p in data_points])

    if show_squared:
        x = l_drive**2
        y = l_sensor**2
        xlabel = f"L²_drive (Anchor {sweep['drive_anchor']})"
        ylabel = f"L²_sensor (Anchor {sweep['sensor_anchor']})"
    else:
        x = l_drive
        y = l_sensor
        xlabel = f"L_drive (Anchor {sweep['drive_anchor']})"
        ylabel = f"L_sensor (Anchor {sweep['sensor_anchor']})"

    # Color by index to show progression
    colors = np.linspace(0, 1, len(x))
    scatter = ax.scatter(x, y, c=colors, cmap='viridis', s=30, alpha=0.7)
    plt.colorbar(scatter, ax=ax, label='Point index')

    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)

    if title:
        ax.set_title(title)
    else:
        fixed = sweep.get('fixed_anchors', [])
        ax.set_title(f"Sweep {sweep['id']}\nFixed: {fixed}")

    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, alpha=0.3)

    return ax


def plot_ellipse_fit(
    sweep: dict,
    fitted_ellipse: dict,
    theoretical_coeffs: Optional[np.ndarray] = None,
    ax: Optional[plt.Axes] = None
) -> plt.Axes:
    """
    Plot sweep data with fitted ellipse overlay.

    Parameters:
        sweep: Sweep dict with data_points
        fitted_ellipse: Fitted ellipse dict with coefficients
        theoretical_coeffs: Optional predicted coefficients for comparison
        ax: Matplotlib axes

    Returns:
        Matplotlib axes
    """
    if ax is None:
        fig, ax = plt.subplots(figsize=(10, 8))

    cfg = fitted_ellipse['sweep_config']
    sweep_id = fitted_ellipse.get('sweep_id', sweep.get('id'))

    # Plot data points
    data_points = sweep['data_points']
    l_drive = np.array([p['l_drive'] for p in data_points])
    l_sensor = np.array([p['l_sensor'] for p in data_points])
    x = l_drive**2
    y = l_sensor**2

    ax.scatter(x, y, c='blue', s=30, alpha=0.6, label='Data points')

    # Plot fitted ellipse
    coeffs = fitted_ellipse['coefficients']
    coeffs_arr = np.array([
        coeffs['A'], coeffs['B'], coeffs['C'],
        coeffs['D'], coeffs['E'], coeffs['F']
    ])

    _plot_ellipse_from_coeffs(ax, coeffs_arr, color='green', linestyle='-',
                               linewidth=2, label='Fitted ellipse')

    # Plot theoretical ellipse if provided
    if theoretical_coeffs is not None:
        _plot_ellipse_from_coeffs(ax, theoretical_coeffs, color='red',
                                   linestyle='--', linewidth=2,
                                   label='Theoretical ellipse')

    # Add residual info
    rms = fitted_ellipse.get('residual_rms', 'N/A')
    valid = fitted_ellipse.get('valid', False)
    status = 'VALID' if valid else 'REJECTED'
    ax.text(0.02, 0.98, f"RMS: {rms:.4f}\nStatus: {status}",
            transform=ax.transAxes, fontsize=10,
            verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    drive_anchor = cfg.get('drive_anchor')
    sensor_anchor = cfg.get('sensor_anchor')
    ax.set_xlabel(f"L²_drive (anchor {drive_anchor})" if drive_anchor is not None else 'L²_drive')
    ax.set_ylabel(f"L²_sensor (anchor {sensor_anchor})" if sensor_anchor is not None else 'L²_sensor')
    ax.set_title(f"Ellipse Fit: {sweep_id}")
    ax.legend(loc='lower right')
    ax.grid(True, alpha=0.3)

    return ax


def _plot_ellipse_from_coeffs(
    ax: plt.Axes,
    coeffs: np.ndarray,
    color: str = 'green',
    linestyle: str = '-',
    linewidth: float = 2,
    label: Optional[str] = None,
    num_points: int = 200
):
    """
    Plot an ellipse from algebraic coefficients.
    """
    A, B, C, D, E, F = coeffs

    # Solve for points on the ellipse
    # Parametric approach: find center and axes
    discriminant = B**2 - 4*A*C

    if discriminant >= 0:
        # Not an ellipse, skip
        return

    # Center
    cx = (2*C*D - B*E) / discriminant
    cy = (2*A*E - B*D) / discriminant

    # Angle
    if abs(A - C) < 1e-10:
        theta = 0 if B >= 0 else np.pi/4
    else:
        theta = 0.5 * np.arctan2(B, A - C)

    # Semi-axes
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)

    A_rot = A * cos_t**2 + B * cos_t * sin_t + C * sin_t**2
    C_rot = A * sin_t**2 - B * cos_t * sin_t + C * cos_t**2

    F_c = A * cx**2 + B * cx * cy + C * cy**2 + D * cx + E * cy + F

    if F_c == 0 or A_rot == 0 or C_rot == 0:
        return

    a_sq = -F_c / A_rot
    b_sq = -F_c / C_rot

    if a_sq < 0 or b_sq < 0:
        return

    a = np.sqrt(a_sq)
    b = np.sqrt(b_sq)

    # Generate ellipse points
    t = np.linspace(0, 2*np.pi, num_points)
    x_ell = a * np.cos(t)
    y_ell = b * np.sin(t)

    # Rotate and translate
    x_plot = cx + x_ell * cos_t - y_ell * sin_t
    y_plot = cy + x_ell * sin_t + y_ell * cos_t

    ax.plot(x_plot, y_plot, color=color, linestyle=linestyle,
            linewidth=linewidth, label=label)


def plot_all_sweeps(
    dataset: dict,
    ncols: int = 3,
    figsize: Tuple[int, int] = None
) -> plt.Figure:
    """
    Plot all sweeps in a dataset as a grid.
    """
    sweeps = dataset.get('sweeps', [])
    fitted = {e['sweep_id']: e for e in dataset.get('fitted_ellipses', [])}

    n = len(sweeps)
    nrows = (n + ncols - 1) // ncols

    if figsize is None:
        figsize = (5 * ncols, 4 * nrows)

    fig, axes = plt.subplots(nrows, ncols, figsize=figsize)
    axes = np.atleast_2d(axes).flatten()

    for i, sweep in enumerate(sweeps):
        ax = axes[i]
        sweep_id = sweep['id']

        if sweep_id in fitted:
            plot_ellipse_fit(sweep, fitted[sweep_id], ax=ax)
        else:
            plot_sweep_data(sweep, ax=ax)

    # Hide unused axes
    for i in range(n, len(axes)):
        axes[i].set_visible(False)

    plt.tight_layout()
    return fig


def plot_cost_convergence(
    cost_history: List[float],
    ax: Optional[plt.Axes] = None
) -> plt.Axes:
    """
    Plot optimization cost over iterations.
    """
    if ax is None:
        fig, ax = plt.subplots(figsize=(10, 4))

    iterations = range(len(cost_history))
    ax.semilogy(iterations, cost_history, 'b-', linewidth=1.5)
    ax.set_xlabel('Iteration')
    ax.set_ylabel('Cost (log scale)')
    ax.set_title('Optimization Convergence')
    ax.grid(True, alpha=0.3)

    # Mark minimum
    min_idx = np.argmin(cost_history)
    min_cost = cost_history[min_idx]
    ax.plot(min_idx, min_cost, 'ro', markersize=10, label=f'Min: {min_cost:.2e}')
    ax.legend()

    return ax


def plot_anchors_2d(
    anchors: np.ndarray,
    true_anchors: Optional[np.ndarray] = None,
    ax: Optional[plt.Axes] = None,
    labels: Optional[List[str]] = None
) -> plt.Axes:
    """
    Plot 2D anchor positions.
    """
    if ax is None:
        fig, ax = plt.subplots(figsize=(8, 8))

    n = len(anchors)
    if labels is None:
        labels = [str(i) for i in range(n)]

    # Plot estimated anchors
    ax.scatter(anchors[:, 0], anchors[:, 1], c='blue', s=200, marker='o',
               label='Estimated', zorder=5)

    for i, (x, y) in enumerate(anchors):
        ax.annotate(labels[i], (x, y), textcoords="offset points",
                    xytext=(10, 10), fontsize=12, color='blue')

    # Plot true anchors if provided
    if true_anchors is not None:
        ax.scatter(true_anchors[:, 0], true_anchors[:, 1], c='red', s=200,
                   marker='x', linewidths=3, label='True', zorder=4)

        # Draw error lines
        for i in range(n):
            ax.plot([anchors[i, 0], true_anchors[i, 0]],
                    [anchors[i, 1], true_anchors[i, 1]],
                    'r--', alpha=0.5, linewidth=1)

    # Plot origin
    ax.scatter([0], [0], c='black', s=100, marker='+', linewidths=2,
               label='Origin', zorder=3)

    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_title('Anchor Positions')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    return ax


def plot_anchors_3d(
    anchors: np.ndarray,
    true_anchors: Optional[np.ndarray] = None,
    labels: Optional[List[str]] = None
) -> plt.Figure:
    """
    Plot 3D anchor positions.
    """
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    n = len(anchors)
    if labels is None:
        labels = [str(i) for i in range(n)]

    # Plot estimated anchors
    ax.scatter(anchors[:, 0], anchors[:, 1], anchors[:, 2],
               c='blue', s=200, marker='o', label='Estimated')

    for i, (x, y, z) in enumerate(anchors):
        ax.text(x, y, z, f'  {labels[i]}', fontsize=10, color='blue')

    # Plot true anchors if provided
    if true_anchors is not None:
        ax.scatter(true_anchors[:, 0], true_anchors[:, 1], true_anchors[:, 2],
                   c='red', s=200, marker='x', linewidths=3, label='True')

        # Draw error lines
        for i in range(n):
            ax.plot([anchors[i, 0], true_anchors[i, 0]],
                    [anchors[i, 1], true_anchors[i, 1]],
                    [anchors[i, 2], true_anchors[i, 2]],
                    'r--', alpha=0.5)

    # Plot origin
    ax.scatter([0], [0], [0], c='black', s=100, marker='+', linewidths=2,
               label='Origin')

    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.set_title('Anchor Positions')
    ax.legend()

    return fig


def plot_residuals_comparison(
    observed_ellipses: List[dict],
    predicted_coeffs: List[np.ndarray],
    sweep_ids: List[str],
    ax: Optional[plt.Axes] = None
) -> plt.Axes:
    """
    Bar chart comparing observed vs predicted ellipse coefficients.
    """
    if ax is None:
        fig, ax = plt.subplots(figsize=(12, 6))

    n = len(sweep_ids)
    x = np.arange(n)
    width = 0.35

    # Compute coefficient distances
    distances = []
    for obs, pred in zip(observed_ellipses, predicted_coeffs):
        obs_arr = np.array([
            obs['coefficients']['A'], obs['coefficients']['B'],
            obs['coefficients']['C'], obs['coefficients']['D'],
            obs['coefficients']['E'], obs['coefficients']['F']
        ])
        # Normalize and compute distance
        obs_norm = obs_arr / np.linalg.norm(obs_arr[:3])
        pred_norm = pred / np.linalg.norm(pred[:3])
        dist = min(np.linalg.norm(obs_norm - pred_norm),
                   np.linalg.norm(obs_norm + pred_norm))
        distances.append(dist)

    colors = ['green' if d < 0.1 else 'orange' if d < 0.3 else 'red'
              for d in distances]

    ax.bar(x, distances, color=colors)
    ax.set_xlabel('Sweep')
    ax.set_ylabel('Coefficient Distance')
    ax.set_title('Observed vs Predicted Ellipse Mismatch')
    ax.set_xticks(x)
    ax.set_xticklabels(sweep_ids, rotation=45, ha='right')

    # Add threshold line
    ax.axhline(y=0.1, color='green', linestyle='--', alpha=0.5, label='Good')
    ax.axhline(y=0.3, color='orange', linestyle='--', alpha=0.5, label='Acceptable')
    ax.legend()

    plt.tight_layout()
    return ax


def create_calibration_report(
    dataset: dict,
    solution: dict,
    output_path: str = 'calibration_report.png'
) -> plt.Figure:
    """
    Create a comprehensive calibration report figure.
    """
    fig = plt.figure(figsize=(16, 12))

    # Grid layout
    gs = fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)

    # 1. Anchor positions
    anchors = solution.get('anchors')
    if anchors is not None:
        if anchors.shape[1] == 2:
            ax_anchors = fig.add_subplot(gs[0, 0])
            plot_anchors_2d(anchors, ax=ax_anchors)
        else:
            ax_anchors = fig.add_subplot(gs[0, 0], projection='3d')
            ax_anchors.scatter(anchors[:, 0], anchors[:, 1], anchors[:, 2])
            ax_anchors.set_title('Anchor Positions')

    # 2-3. First two sweep fits
    sweeps = dataset.get('sweeps', [])[:2]
    fitted = {e['sweep_id']: e for e in dataset.get('fitted_ellipses', [])}

    for i, sweep in enumerate(sweeps):
        ax = fig.add_subplot(gs[0, 1 + i])
        if sweep['id'] in fitted:
            plot_ellipse_fit(sweep, fitted[sweep['id']], ax=ax)
        else:
            plot_sweep_data(sweep, ax=ax)

    # 4. Per-sweep costs
    ax_costs = fig.add_subplot(gs[1, :2])
    details = solution.get('details')
    if details and hasattr(details, 'per_sweep_costs'):
        sweep_ids = list(details.per_sweep_costs.keys())
        costs = list(details.per_sweep_costs.values())
        ax_costs.bar(range(len(sweep_ids)), costs)
        ax_costs.set_xlabel('Sweep')
        ax_costs.set_ylabel('Cost')
        ax_costs.set_title('Per-Sweep Costs')
        ax_costs.set_xticks(range(len(sweep_ids)))
        ax_costs.set_xticklabels(sweep_ids, rotation=45, ha='right')

    # 5. Summary text
    ax_summary = fig.add_subplot(gs[1, 2])
    ax_summary.axis('off')

    summary_text = f"""
Calibration Summary
==================

Machine Type: {dataset.get('machine_type', 'unknown')}
Num Anchors: {dataset.get('num_anchors', 'unknown')}
Dimensions: {dataset.get('dimensions', 'unknown')}

Total Sweeps: {len(dataset.get('sweeps', []))}
Valid Ellipses: {len([e for e in dataset.get('fitted_ellipses', []) if e.get('valid')])}

Final Cost: {solution.get('cost', 'N/A'):.6e}
Success: {solution.get('success', False)}
"""

    ax_summary.text(0.1, 0.9, summary_text, transform=ax_summary.transAxes,
                    fontsize=10, verticalalignment='top', fontfamily='monospace',
                    bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.5))

    # 6. G-code output
    ax_gcode = fig.add_subplot(gs[2, :])
    ax_gcode.axis('off')

    if anchors is not None:
        from ellipse_solver import format_anchors_gcode
        gcode = format_anchors_gcode(anchors, dataset.get('machine_type', ''))
        ax_gcode.text(0.5, 0.5, f"Generated G-code:\n\n{gcode}",
                      transform=ax_gcode.transAxes, fontsize=12,
                      ha='center', va='center', fontfamily='monospace',
                      bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Saved calibration report to {output_path}")

    return fig


# CLI for standalone visualization
if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Visualize calibration data')
    parser.add_argument('input', help='Input JSON file')
    parser.add_argument('-o', '--output', help='Output image file')
    parser.add_argument('--sweep', help='Specific sweep ID to plot')

    args = parser.parse_args()

    with open(args.input, 'r') as f:
        dataset = json.load(f)

    if args.sweep:
        sweep = next((s for s in dataset['sweeps'] if s['id'] == args.sweep), None)
        if sweep:
            fitted = next((e for e in dataset.get('fitted_ellipses', [])
                          if e['sweep_id'] == args.sweep), None)
            if fitted:
                plot_ellipse_fit(sweep, fitted)
            else:
                plot_sweep_data(sweep)
        else:
            print(f"Sweep {args.sweep} not found")
    else:
        plot_all_sweeps(dataset)

    if args.output:
        plt.savefig(args.output, dpi=150, bbox_inches='tight')
    else:
        plt.show()
```

## Testing

### Visual Tests

```python
# test_ellipse_visualization.py
import pytest
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend for testing
import matplotlib.pyplot as plt

from ellipse_visualization import (
    plot_sweep_data,
    plot_ellipse_fit,
    plot_anchors_2d,
    plot_anchors_3d,
    create_calibration_report
)


@pytest.fixture
def sample_sweep():
    """Create sample sweep data."""
    phi = np.linspace(0, np.pi, 50)
    l_drive = np.sqrt(10000 + 2000 * np.cos(phi))
    l_sensor = np.sqrt(12000 + 1800 * np.cos(phi))

    return {
        'id': 'test_sweep',
        'fixed_anchors': [0],
        'fixed_lengths': [600.0],
        'drive_anchor': 1,
        'sensor_anchor': 2,
        'data_points': [
            {'l_drive': ld, 'l_sensor': ls}
            for ld, ls in zip(l_drive, l_sensor)
        ]
    }


@pytest.fixture
def sample_fitted_ellipse():
    return {
        'sweep_id': 'test_sweep',
        'coefficients': {
            'A': 0.6, 'B': 0.1, 'C': 0.8,
            'D': -100, 'E': -120, 'F': 5000
        },
        'residual_rms': 0.005,
        'valid': True
    }


class TestPlotting:
    def test_plot_sweep_data(self, sample_sweep):
        fig, ax = plt.subplots()
        result_ax = plot_sweep_data(sample_sweep, ax=ax)
        assert result_ax is not None
        plt.close(fig)

    def test_plot_ellipse_fit(self, sample_sweep, sample_fitted_ellipse):
        fig, ax = plt.subplots()
        result_ax = plot_ellipse_fit(sample_sweep, sample_fitted_ellipse, ax=ax)
        assert result_ax is not None
        plt.close(fig)

    def test_plot_anchors_2d(self):
        anchors = np.array([[-500, 400], [500, 400], [0, -500]])
        fig, ax = plt.subplots()
        result_ax = plot_anchors_2d(anchors, ax=ax)
        assert result_ax is not None
        plt.close(fig)

    def test_plot_anchors_3d(self):
        anchors = np.array([
            [-500, 400, -100],
            [500, 400, -100],
            [0, -500, -100],
            [0, 0, 1000]
        ])
        fig = plot_anchors_3d(anchors)
        assert fig is not None
        plt.close(fig)
```

## Validation Criteria

1. **Plot Generation**: All plot functions produce valid matplotlib figures
2. **Data Rendering**: Data points correctly positioned in plots
3. **Ellipse Rendering**: Fitted ellipses match data visually
4. **Color Coding**: Valid/invalid sweeps clearly distinguished
5. **Report Completeness**: Calibration report includes all key information
6. **Export**: Figures save correctly to PNG/PDF

## Dependencies

- Python 3.8+
- numpy
- matplotlib
- (ellipse_fitting.py from Substep 3)
- (ellipse_cost.py from Substep 5)

## Estimated Complexity

**Effort**: Low-Medium (2-3 hours)

Matplotlib plotting is straightforward. The main complexity is in computing ellipse points from algebraic coefficients for visualization and ensuring the report layout works for different dataset sizes.

## Files to Create/Modify

| File | Action |
|------|--------|
| `autocal/ellipse_visualization.py` | Create |
| `autocal/tests/test_ellipse_visualization.py` | Create |
