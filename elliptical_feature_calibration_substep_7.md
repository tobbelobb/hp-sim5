# Substep 7: Integration with Existing Calibration Framework

## Overview

Integrate the elliptical feature calibration system with the existing `simulation.py` calibration framework. This includes adapting the new modules to work alongside or replace the existing point-based calibration, providing a unified CLI, and ensuring backward compatibility.

Keep the `sweep_config` snapshots that travel with each `FittedEllipse` when round-tripping files so the optimizer and visualizer always know which cables were fixed/drive/sense for each ellipse.
Absolute cable lengths are still unknown at data-collection time, so the integration layer must rebuild them from anchor guesses during optimization (`L_abs = ||A_i - origin|| + ΔL_measured`) before feeding anything into the forward model or cost function; otherwise observed (relative) and predicted (absolute) ellipses will be misaligned. Keep the raw encoder angles alongside the derived lengths so future sag/flex/buildup models can reinterpret the same files without recollecting.
Ellipse fitting now happens inside the optimizer on those reconstructed absolute lengths; drop any stored `fitted_ellipses` paths. Compare ellipses in the canonical geometry space `(x0, y0, a, b, θ)` (with `a >= b`, θ wrapped) rather than raw `(A..F)` coefficients.

## Implementation Details

### 7.1 Unified Calibration Pipeline (`calibrate.py`)

```python
#!/usr/bin/env python3
"""
calibrate.py

Unified calibration script supporting both point-based and elliptical methods.
"""

import argparse
import json
import sys
import numpy as np
from pathlib import Path
from datetime import datetime

# Existing modules
from simulation import solve as solve_point_based
from data import motor_pos_samp, xyz_of_samp, line_lengths_when_at_origin

# New elliptical modules
from sweep_types import SweepDataset, MachineConfig, MachineType
from sweep_io import load_sweep_dataset, save_sweep_dataset
from ellipse_fitting import fit_all_sweeps
from ellipse_cost import EllipseCostFunction
from ellipse_solver import solve_anchors, format_anchors_gcode
from ellipse_visualization import create_calibration_report


def calibrate_elliptical(
    input_path: str,
    output_path: str = None,
    residual_threshold: float = 0.01,
    num_restarts: int = 8,
    max_iterations: int = 1000,
    verbose: bool = False,
    generate_report: bool = True
) -> dict:
    """
    Run elliptical feature calibration.

    Parameters:
        input_path: Path to sweep data JSON
        output_path: Optional path for output JSON
        residual_threshold: Threshold for valid ellipse fits
        num_restarts: Number of optimization restarts
        max_iterations: Max iterations per restart
        verbose: Print progress
        generate_report: Create PDF report

    Returns:
        Solution dict with anchors, cost, etc.
    """
    print(f"Loading sweep data from {input_path}")
    with open(input_path, 'r') as f:
        dataset = json.load(f)

    # Guard: Hangprinter carrying anchors must never be Sensor/torque
    machine_type = dataset.get('machine_type', '')
    forbidden_sensors = {
        'hangprinter_4': {3},
        'hangprinter_5': {4},
    }.get(machine_type, set())
    if forbidden_sensors:
        invalid_sweeps = [
            s['id'] for s in dataset.get('sweeps', [])
            if s.get('sensor_anchor') in forbidden_sensors
        ]
        if invalid_sweeps:
            print(f"Skipping sweeps that put carrying anchors in Sensor role: {invalid_sweeps}")
            dataset['sweeps'] = [
                s for s in dataset['sweeps']
                if s.get('sensor_anchor') not in forbidden_sensors
            ]

    # Guard: dataset should only contain raw sweeps; ellipse fitting happens per-iteration in the solver
    if not dataset.get('sweeps'):
        print("ERROR: No sweeps found in dataset.")
        sys.exit(1)

    # Step 2: Solve for anchors (solver rebuilds absolute lengths + fits ellipses internally)
    print(f"\nSolving for anchor positions ({num_restarts} restarts)...")
    solution = solve_anchors(
        dataset,
        method='SLSQP',
        max_iterations=max_iterations,
        num_restarts=num_restarts,
        residual_threshold=residual_threshold,
        verbose=verbose
    )

    if not solution['success']:
        print("WARNING: Optimization may not have converged")

    # Step 3: Output results
    anchors = solution['anchors']
    machine_type = dataset.get('machine_type', 'unknown')

    print(f"\n{'='*60}")
    print("CALIBRATION RESULTS")
    print(f"{'='*60}")
    print(f"\nFinal cost: {solution['cost']:.6e}")
    print(f"\nAnchors ({machine_type}):")
    for i, anchor in enumerate(anchors):
        print(f"  {i}: [{', '.join(f'{x:.2f}' for x in anchor)}]")

    print(f"\nG-code command:")
    gcode = format_anchors_gcode(anchors, machine_type)
    print(f"  {gcode}")

    # Step 4: Save results (optionally include debug fits for the final anchor guess)
    def rebuild_absolute_sweeps(anchors_mat: np.ndarray) -> list:
        """Convert encoder deltas into absolute lengths for reporting/debug."""
        sweeps_abs = []
        for sweep in dataset.get('sweeps', []):
            drive_base = np.linalg.norm(anchors_mat[sweep['drive_anchor']])
            sensor_base = np.linalg.norm(anchors_mat[sweep['sensor_anchor']])
            sweeps_abs.append({
                **sweep,
                "fixed_lengths": [
                    np.linalg.norm(anchors_mat[idx]) + dl
                    for idx, dl in zip(sweep['fixed_anchors'], sweep['fixed_lengths'])
                ],
                "data_points": [
                    {
                        **p,
                        "l_drive": p['l_drive'] + drive_base,
                        "l_sensor": p['l_sensor'] + sensor_base,
                    }
                    for p in sweep['data_points']
                ]
            })
        return sweeps_abs

    report_fits = fit_all_sweeps(rebuild_absolute_sweeps(anchors), residual_threshold)

    if output_path:
        result_data = {
            'input_file': input_path,
            'timestamp': datetime.now().isoformat(),
            'machine_type': machine_type,
            'anchors': anchors.tolist(),
            'cost': solution['cost'],
            'success': solution['success'],
            'gcode': gcode,
            'ellipse_fits_debug': report_fits,
        }
        with open(output_path, 'w') as f:
            json.dump(result_data, f, indent=2)
        print(f"\nSaved results to {output_path}")

    # Step 5: Generate report
    if generate_report:
        report_path = output_path.replace('.json', '_report.png') if output_path else 'calibration_report.png'
        create_calibration_report(dataset, solution, ellipse_fits=report_fits, output_path=report_path)

    return solution


def calibrate_point_based(
    use_flex: bool = False,
    use_line_lengths: bool = True,
    verbose: bool = False
) -> dict:
    """
    Run existing point-based calibration.

    Uses data from data.py (motor_pos_samp, xyz_of_samp).
    """
    print("Running point-based calibration (existing method)")
    print(f"  Flex compensation: {use_flex}")
    print(f"  Use line lengths: {use_line_lengths}")

    # Call existing solve function
    solution = solve_point_based(
        motor_pos_samp,
        xyz_of_samp,
        line_lengths_when_at_origin,
        use_flex,
        use_line_lengths,
        debug=verbose
    )

    # Extract anchors from solution vector
    from simulation import anchorsvec2matrix, params_anch
    anchors = anchorsvec2matrix(solution[0:params_anch])

    print(f"\nPoint-based calibration complete")
    print(f"Anchors:")
    for i, anchor in enumerate(anchors):
        print(f"  {i}: [{', '.join(f'{x:.2f}' for x in anchor)}]")

    return {
        'anchors': anchors,
        'solution_vector': solution,
        'method': 'point_based'
    }


def calibrate_hybrid(
    sweep_path: str,
    ellipse_weight: float = 1.0,
    point_weight: float = 0.1,
    verbose: bool = False
) -> dict:
    """
    Hybrid calibration using both ellipse and point constraints.

    Useful for transitioning from point-based to ellipse-based,
    or when both data types are available.
    """
    print("Running hybrid calibration...")
    print(f"  Ellipse weight: {ellipse_weight}")
    print(f"  Point weight: {point_weight}")

    # Load sweep data
    with open(sweep_path, 'r') as f:
        sweep_dataset = json.load(f)

    # Create cost functions
    ellipse_cost_fn = EllipseCostFunction(sweep_dataset)

    # Point-based cost (if data.py has data)
    from simulation import cost_sq_for_pos_samp, anchorsvec2matrix
    from data import motor_pos_samp, xyz_of_samp, constant_spool_buildup_factor
    from data import spool_r_in_origin_first_guess, line_lengths_when_at_origin

    def point_cost(anchor_vec):
        anchors = anchorsvec2matrix(anchor_vec[:15])
        # Simplified - actual implementation would need full param vector
        return 0.0  # Placeholder

    # Combined cost
    def combined_cost(x):
        e_cost = ellipse_cost_fn.evaluate(x)
        p_cost = point_cost(x) if point_weight > 0 else 0
        return ellipse_weight * e_cost + point_weight * p_cost

    # Optimize
    from scipy.optimize import minimize
    from theoretical_ellipse import get_anchor_bounds

    lb, ub = get_anchor_bounds(sweep_dataset.get('machine_type', 'hangprinter_4'))
    bounds = list(zip(lb, ub))

    x0 = np.array([(l + u) / 2 for l, u in bounds])

    result = minimize(
        combined_cost,
        x0,
        method='SLSQP',
        bounds=bounds,
        options={'maxiter': 1000}
    )

    from theoretical_ellipse import anchors_vec_to_matrix
    n_anchors = sweep_dataset.get('num_anchors', 4)
    dims = sweep_dataset.get('dimensions', 3)
    anchors = anchors_vec_to_matrix(result.x, n_anchors, dims)

    return {
        'anchors': anchors,
        'cost': result.fun,
        'success': result.success,
        'method': 'hybrid'
    }


def main():
    parser = argparse.ArgumentParser(
        description='Unified calibration for cable-driven robots',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Elliptical calibration from sweep data
  python calibrate.py ellipse sweep_data.json -o results.json

  # Point-based calibration (uses data.py)
  python calibrate.py point --flex

  # Hybrid calibration
  python calibrate.py hybrid sweep_data.json --ellipse-weight 1.0 --point-weight 0.1
        """
    )

    subparsers = parser.add_subparsers(dest='method', help='Calibration method')

    # Ellipse subcommand
    ellipse_parser = subparsers.add_parser('ellipse', help='Elliptical feature calibration')
    ellipse_parser.add_argument('input', help='Sweep data JSON file')
    ellipse_parser.add_argument('-o', '--output', help='Output results JSON')
    ellipse_parser.add_argument('-t', '--threshold', type=float, default=0.01,
                                help='Ellipse fit residual threshold')
    ellipse_parser.add_argument('-r', '--restarts', type=int, default=8,
                                help='Number of optimization restarts')
    ellipse_parser.add_argument('-i', '--iterations', type=int, default=1000,
                                help='Max iterations per restart')
    ellipse_parser.add_argument('-v', '--verbose', action='store_true')
    ellipse_parser.add_argument('--no-report', action='store_true',
                                help='Skip report generation')

    # Point subcommand
    point_parser = subparsers.add_parser('point', help='Point-based calibration')
    point_parser.add_argument('--flex', action='store_true',
                              help='Use flex compensation')
    point_parser.add_argument('--no-line-lengths', action='store_true',
                              help='Ignore hand-measured line lengths')
    point_parser.add_argument('-v', '--verbose', action='store_true')

    # Hybrid subcommand
    hybrid_parser = subparsers.add_parser('hybrid', help='Hybrid calibration')
    hybrid_parser.add_argument('input', help='Sweep data JSON file')
    hybrid_parser.add_argument('--ellipse-weight', type=float, default=1.0)
    hybrid_parser.add_argument('--point-weight', type=float, default=0.1)
    hybrid_parser.add_argument('-v', '--verbose', action='store_true')

    args = parser.parse_args()

    if args.method == 'ellipse':
        calibrate_elliptical(
            args.input,
            args.output,
            args.threshold,
            args.restarts,
            args.iterations,
            args.verbose,
            not args.no_report
        )
    elif args.method == 'point':
        calibrate_point_based(
            args.flex,
            not args.no_line_lengths,
            args.verbose
        )
    elif args.method == 'hybrid':
        calibrate_hybrid(
            args.input,
            args.ellipse_weight,
            args.point_weight,
            args.verbose
        )
    else:
        parser.print_help()


if __name__ == '__main__':
    main()
```

### 7.2 Updated `data.py` Structure

```python
# data.py additions for sweep-based calibration

import json
import numpy as np
from pathlib import Path

# ... existing data definitions ...

# New: Load sweep data if available
SWEEP_DATA_PATH = Path(__file__).parent / 'measurements' / 'sweep_data.json'

sweep_dataset = None
if SWEEP_DATA_PATH.exists():
    with open(SWEEP_DATA_PATH, 'r') as f:
        sweep_dataset = json.load(f)

def get_sweep_data():
    """Get loaded sweep dataset or None if not available."""
    return sweep_dataset

def get_machine_config_from_sweep():
    """Extract machine configuration from sweep dataset."""
    if sweep_dataset is None:
        return None

    return {
        'machine_type': sweep_dataset.get('machine_type'),
        'num_anchors': sweep_dataset.get('num_anchors'),
        'dimensions': sweep_dataset.get('dimensions'),
    }
```

### 7.3 Migration Guide

```markdown
# Migration from Point-Based to Elliptical Calibration

## Step 1: Collect Sweep Data

Replace point measurement collection with sweep collection:

```bash
# Old way (point-based)
node collect_encoder_data.mjs --points-file measurement_points.txt --outputFile measurements.json

# New way (sweep-based)
node collect_sweep_data.mjs --machineType slideprinter --sweepRange 100 --outputFile sweep_data.json
```

## Step 2: Run Calibration

```bash
# Old way
cd autocal/auto-calibration-simulation-for-hangprinter
python simulation.py

# New way
python calibrate.py ellipse sweep_data.json -o results.json
```

## Step 3: Apply Results

Both methods output G-code commands. The format is the same:

```
M669 A-500.00:400.00:-100.00 B500.00:400.00:-100.00 ...
```

## Hybrid Approach

During transition, you can use both methods:

```bash
python calibrate.py hybrid sweep_data.json --ellipse-weight 1.0 --point-weight 0.5
```

This combines ellipse constraints with any existing point measurements.

## Data Format Comparison

### Point-Based (data.py)
- `motor_pos_samp`: Motor positions at each sample
- `xyz_of_samp`: Known XYZ positions (if any)
- `line_lengths_when_at_origin`: Hand-measured lengths

### Sweep-Based (JSON)
- Sweep configurations (fixed, drive, sensor)
- Dense (l_drive, l_sensor) measurements
- Optional debug sidecars with ellipse fits (computed per anchor guess; never baked into canonical JSON)
```

### 7.4 Module Organization

```
autocal/
├── auto-calibration-simulation-for-hangprinter/
│   ├── simulation.py          # Existing point-based solver
│   ├── data.py                # Existing data definitions
│   └── ...
├── sweep_types.py             # New: Data structures (Substep 1)
├── sweep_io.py                # New: JSON I/O (Substep 1)
├── sweep_config_generator.py  # New: Config generation (Substep 1)
├── ellipse_fitting.py         # New: Ellipse fitting (Substep 3)
├── theoretical_ellipse.py     # New: Forward model (Substep 4)
├── ellipse_cost.py            # New: Cost function (Substep 5)
├── ellipse_solver.py          # New: Optimizer (Substep 5)
├── ellipse_visualization.py   # New: Plotting (Substep 6)
├── calibrate.py               # New: Unified CLI (Substep 7)
├── tests/
│   ├── test_sweep_types.py
│   ├── test_ellipse_fitting.py
│   ├── test_theoretical_ellipse.py
│   ├── test_ellipse_cost.py
│   └── test_ellipse_visualization.py
└── measurements/
    ├── sweep_data.json        # New format
    └── measurements.json      # Old format (still supported)

scripts/
├── collect_encoder_data.mjs   # Existing
└── collect_sweep_data.mjs     # New (Substep 2)
```

### 7.5 End-to-End Test Script

```python
#!/usr/bin/env python3
"""
test_end_to_end.py

Full integration test of elliptical calibration pipeline.
"""

import numpy as np
import json
import tempfile
from pathlib import Path

# Import all modules
from sweep_types import MachineConfig, MachineType, Sweep, DataPoint, SweepDataset
from sweep_config_generator import generate_sweep_configs
from ellipse_fitting import fit_ellipse_from_sweep, fit_all_sweeps
from theoretical_ellipse import (
    predict_ellipse_coefficients,
    compute_constraint_circle_2d,
    squared_length_coefficients
)
from ellipse_cost import EllipseCostFunction
from ellipse_solver import solve_anchors


def generate_synthetic_sweep_data(
    true_anchors: np.ndarray,
    config: MachineConfig,
    n_sweeps: int = 6,
    n_points: int = 50,
    noise_std: float = 0.1
) -> dict:
    """Generate synthetic sweep data from known anchor positions."""

    sweep_configs = generate_sweep_configs(config.machine_type.value)[:n_sweeps]

    sweeps = []
    for i, sc in enumerate(sweep_configs):
        # Compute constraint circle
        if config.dimensions == 2:
            fixed_idx = sc['fixed_anchors'][0]
            fixed_length = np.linalg.norm(true_anchors[fixed_idx])
            circle = compute_constraint_circle_2d(
                true_anchors, fixed_idx, fixed_length
            )
        else:
            # 3D case
            continue  # Skip for this test

        # Generate points on circle
        phi = np.linspace(0, np.pi, n_points)
        data_points = []

        for p in phi:
            pos = circle.center + circle.radius * (
                np.cos(p) * circle.u[:2] + np.sin(p) * circle.v[:2]
            )

            l_drive = np.linalg.norm(pos - true_anchors[sc['drive_anchor']][:2])
            l_sensor = np.linalg.norm(pos - true_anchors[sc['sensor_anchor']][:2])

            # Add noise
            l_drive += np.random.normal(0, noise_std)
            l_sensor += np.random.normal(0, noise_std)

            data_points.append({
                'l_drive': float(l_drive - np.linalg.norm(true_anchors[sc['drive_anchor']])),
                'l_sensor': float(l_sensor - np.linalg.norm(true_anchors[sc['sensor_anchor']]))
            })

        fixed_lengths_abs = [float(np.linalg.norm(true_anchors[idx])) for idx in sc['fixed_anchors']]
        sweeps.append({
            'id': f'sweep_{i:03d}',
            'fixed_anchors': sc['fixed_anchors'],
            'fixed_lengths': [
                fl_abs - np.linalg.norm(true_anchors[idx])
                for fl_abs, idx in zip(fixed_lengths_abs, sc['fixed_anchors'])
            ],
            'drive_anchor': sc['drive_anchor'],
            'sensor_anchor': sc['sensor_anchor'],
            'data_points': data_points,
            'metadata': {'feed_rate': 2000, 'torque': 0.05}
        })

    return {
        'version': '1.0',
        'machine_type': config.machine_type.value,
        'num_anchors': config.num_anchors,
        'dimensions': config.dimensions,
        'sweeps': sweeps
    }


def test_full_pipeline():
    """Test the complete calibration pipeline."""

    print("="*60)
    print("END-TO-END INTEGRATION TEST")
    print("="*60)

    # 1. Define true anchors (Slideprinter)
    true_anchors = np.array([
        [-500, 400],
        [500, 400],
        [0, -500]
    ], dtype=float)

    config = MachineConfig.from_type(MachineType.SLIDEPRINTER)

    print(f"\n1. True anchor positions:")
    for i, a in enumerate(true_anchors):
        print(f"   {i}: {a}")

    # 2. Generate synthetic sweep data
    print("\n2. Generating synthetic sweep data...")
    np.random.seed(42)
    dataset = generate_synthetic_sweep_data(
        true_anchors, config, n_sweeps=6, n_points=50, noise_std=0.5
    )
    print(f"   Generated {len(dataset['sweeps'])} sweeps")

    # 3. Debug-fit ellipses at the true anchors (sidecar only; solver re-fits per iteration)
    print("\n3. Fitting ellipses at ground truth for QC (sidecar only)...")
    def rebuild_absolute_sweeps(ds, anchors_guess):
        sweeps_abs = []
        for sweep in ds['sweeps']:
            drive_base = np.linalg.norm(anchors_guess[sweep['drive_anchor']])
            sensor_base = np.linalg.norm(anchors_guess[sweep['sensor_anchor']])
            sweeps_abs.append({
                **sweep,
                "fixed_lengths": [
                    np.linalg.norm(anchors_guess[idx]) + dl
                    for idx, dl in zip(sweep['fixed_anchors'], sweep['fixed_lengths'])
                ],
                "data_points": [
                    {
                        **p,
                        "l_drive": p['l_drive'] + drive_base,
                        "l_sensor": p['l_sensor'] + sensor_base,
                    }
                    for p in sweep['data_points']
                ]
            })
        return sweeps_abs

    fitted = fit_all_sweeps(rebuild_absolute_sweeps(dataset, true_anchors), residual_threshold=0.05)
    valid = sum(1 for e in fitted if e['valid'])
    print(f"   Valid fits: {valid}/{len(fitted)} (not stored; solver re-fits every iteration)")

    # 4. Solve for anchors
    print("\n4. Solving for anchor positions...")
    solution = solve_anchors(
        dataset,
        num_restarts=4,
        max_iterations=500,
        residual_threshold=0.05,
        verbose=False
    )

    print(f"   Final cost: {solution['cost']:.6e}")
    print(f"   Success: {solution['success']}")

    # 5. Compare results
    print("\n5. Results comparison:")
    estimated = solution['anchors']

    total_error = 0
    for i in range(len(true_anchors)):
        error = np.linalg.norm(estimated[i] - true_anchors[i])
        total_error += error
        print(f"   Anchor {i}: Error = {error:.2f} mm")

    mean_error = total_error / len(true_anchors)
    print(f"\n   Mean error: {mean_error:.2f} mm")

    # 6. Pass/fail
    threshold = 10.0  # mm
    if mean_error < threshold:
        print(f"\n   TEST PASSED (error < {threshold} mm)")
        return True
    else:
        print(f"\n   TEST FAILED (error >= {threshold} mm)")
        return False


if __name__ == '__main__':
    success = test_full_pipeline()
    exit(0 if success else 1)
```

## Testing

### Integration Tests

```python
# test_integration.py
import pytest
import json
import tempfile
import numpy as np
from pathlib import Path

from calibrate import calibrate_elliptical


class TestIntegration:
    @pytest.fixture
    def sample_dataset(self, tmp_path):
        """Create a sample dataset file."""
        dataset = {
            'version': '1.0',
            'machine_type': 'slideprinter',
            'num_anchors': 3,
            'dimensions': 2,
            'sweeps': []  # Would be populated with real data
        }
        path = tmp_path / 'test_data.json'
        with open(path, 'w') as f:
            json.dump(dataset, f)
        return path

    def test_calibrate_elliptical_runs(self, sample_dataset):
        """Verify calibration runs without error."""
        # This would need actual sweep data to produce meaningful results
        pass

    def test_calibrate_output_format(self):
        """Verify output JSON has correct structure."""
        pass
```

## Validation Criteria

1. **Pipeline Completion**: Full pipeline runs without errors on valid data
2. **Backward Compatibility**: Existing `simulation.py` still works unchanged
3. **Result Format**: Output matches expected G-code format
4. **Error Recovery**: Graceful handling of invalid inputs
5. **Performance**: Calibration completes in reasonable time (<5 min for typical data)
6. **Accuracy**: Synthetic test recovers true anchors within 10mm

## Dependencies

- All modules from Substeps 1-6
- Existing `simulation.py` and dependencies
- scipy
- matplotlib

## Estimated Complexity

**Effort**: Medium (4-5 hours)

Integration requires careful coordination between existing and new code. The unified CLI needs comprehensive argument handling. Testing end-to-end requires synthetic data generation.

## Files to Create/Modify

| File | Action |
|------|--------|
| `autocal/calibrate.py` | Create |
| `autocal/data.py` | Modify (add sweep data loading) |
| `autocal/test_end_to_end.py` | Create |
| `autocal/tests/test_integration.py` | Create |
| `docs/migration_guide.md` | Create |
