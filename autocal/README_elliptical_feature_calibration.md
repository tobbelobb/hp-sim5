# Elliptical Feature Calibration — Usage Manual

This manual covers the workflow and tools added in substeps 3–6:

- **Substep 3**: ellipse fitting (`autocal/ellipse_fitting.py`, CLI `autocal/fit_ellipses.py`)
- **Substep 4**: theoretical ellipse projection (`autocal/theoretical_ellipse.py`)
- **Substep 5**: cost function + optimizer (`autocal/ellipse_cost.py`, `autocal/ellipse_solver.py`)
- **Substep 6**: visualization (`autocal/ellipse_visualization.py`, `scripts/plot_sweep_fits.py`)

The key idea:
- Your sweep dataset stores **encoder deltas from origin** (because anchors were unknown at logging time).
- During optimization, those deltas are reconstructed into **absolute lengths** using the current anchor guess.

## Prerequisites

- Python 3.8+ with `numpy`, `scipy`, `matplotlib`, `pytest`.
- Node.js only for collecting new sweep data (Phase 1).

## Data format recap

A dataset JSON has:
- Machine metadata: `machine_type`, `num_anchors`, `dimensions`.
- `sweeps[]`, each with:
  - `fixed_anchors[]`, `fixed_lengths[]` (deltas from origin).
  - `drive_anchor`, `sensor_anchor`.
  - `data_points[]` of `{l_drive, l_sensor, timestamp_ms?}` (deltas).

All `l_*` values in the canonical dataset are **deltas from origin**.

## Phase 1: Collect sweep data

Sweep data is collected by:

```bash
node scripts/collect_sweep_data.mjs \
  --sweepRange 150 \
  --superSweepRange 150 \
  --superSweepPoints 3 \
  --speedup 16 \
  --trace
```

This produces a `sweep_data_*.json` file at repo root (or wherever the script writes it).

## Phase 1 QC: Fit ellipses per sweep (Substep 3)

### CLI: `autocal/fit_ellipses.py`

`fit_ellipses.py` expects sweeps whose lengths are already reconstructed to **absolute** values.
It writes a “fits sidecar” JSON usable by the visualization tools.

```bash
python autocal/fit_ellipses.py <absolute_dataset.json> \
  -o <absolute_dataset_fits.json> \
  -t <residual_threshold> \
  --min-points 10 \
  -v
```

Notes:
- The residual threshold is in **Sampson residual units** (same as the fitter).
- For Slideprinter training data, the test uses `-t 250.0`.

### Debug plots: `scripts/plot_sweep_fits.py`

This script is a more detailed per-sweep plotter; it can add a uniform base length:

```bash
python scripts/plot_sweep_fits.py <dataset.json> \
  --base-length 1900 \
  --residual-threshold 250 \
  --outdir plots/sweep_fits
```

Use `--base-length 0` if your dataset is already absolute.

## Using your known Slideprinter dataset

The dataset you mentioned is stored as deltas:
`autocal/data/sweep_data_slideprinter_for_test_slideprinter_training_data_fits.json`.

To inspect it, first make an **absolute** copy by adding a base length.
The tests use `BASE_LENGTH = 1900.0` for Slideprinter.

### Step 1: Inflate to absolute lengths

```bash
python - <<'PY'
import json, copy
from pathlib import Path

BASE = 1900.0
src = Path("autocal/data/sweep_data_slideprinter_for_test_slideprinter_training_data_fits.json")
dst = Path("autocal/data/sweep_data_slideprinter_training_data_abs.json")

raw = json.loads(src.read_text())
abs_data = copy.deepcopy(raw)

for sweep in abs_data["sweeps"]:
    sweep["fixed_lengths"] = [fl + BASE for fl in sweep.get("fixed_lengths", [])]
    for p in sweep["data_points"]:
        p["l_drive"] += BASE
        p["l_sensor"] += BASE

dst.write_text(json.dumps(abs_data, indent=2))
print("Wrote", dst)
PY
```

### Step 2: Fit ellipses and create sidecar

```bash
python autocal/fit_ellipses.py autocal/data/sweep_data_slideprinter_training_data_abs.json \
  -o autocal/data/sweep_data_slideprinter_training_data_abs_fits.json \
  -t 250 \
  --min-points 10 \
  -v
```

### Step 3A: Grid plot of all sweeps (Substep 6)

```bash
python autocal/ellipse_visualization.py autocal/data/sweep_data_slideprinter_training_data_abs.json \
  --fits autocal/data/sweep_data_slideprinter_training_data_abs_fits.json \
  -o plots/slideprinter_sweeps_grid.png
```

### Step 3B: Single sweep plot

```bash
python autocal/ellipse_visualization.py autocal/data/sweep_data_slideprinter_training_data_abs.json \
  --fits autocal/data/sweep_data_slideprinter_training_data_abs_fits.json \
  --sweep sweep_001 \
  -o plots/sweep_001_fit.png
```

### Alternative: Use the debug sweep plotter directly

Skip the inflation step and let the script add base length:

```bash
python scripts/plot_sweep_fits.py autocal/data/sweep_data_slideprinter_for_test_slideprinter_training_data_fits.json \
  --base-length 1900 \
  --residual-threshold 250 \
  --outdir plots/sweep_fits
```

## Theoretical ellipse projection (Substep 4)

Forward predictions are in `autocal/theoretical_ellipse.py`.

You normally don’t call these directly; `EllipseCostFunction` does it per iteration.
But for debugging, you can compare one sweep’s fitted ellipse to the theoretical prediction
given an anchor guess:

```python
import json
import numpy as np
from autocal.theoretical_ellipse import predict_ellipse_coefficients
from autocal.ellipse_fitting import fit_ellipse_from_sweep

dataset = json.load(open("sweep_data.json"))
sweep = dataset["sweeps"][0]

# anchors: shape (N, D), guessed in mm, origin at [0,0,(0)]
anchors = np.array([
    [-500,  400],
    [ 500,  400],
    [   0, -500],
], dtype=float)

# reconstruct absolute fixed lengths for this guess
fixed_lengths_abs = [
    np.linalg.norm(anchors[idx]) + delta
    for idx, delta in zip(sweep["fixed_anchors"], sweep["fixed_lengths"])
]

# reconstruct absolute drive/sensor arrays for this guess
l_drive_abs = np.array([p["l_drive"] for p in sweep["data_points"]]) + np.linalg.norm(anchors[sweep["drive_anchor"]])
l_sensor_abs = np.array([p["l_sensor"] for p in sweep["data_points"]]) + np.linalg.norm(anchors[sweep["sensor_anchor"]])

obs_fit = fit_ellipse_from_sweep(l_drive_abs, l_sensor_abs, residual_threshold=250, square_inputs=True)
pred_coeffs = predict_ellipse_coefficients(
    anchors,
    sweep["fixed_anchors"],
    fixed_lengths_abs,
    sweep["drive_anchor"],
    sweep["sensor_anchor"],
    dimensions=dataset.get("dimensions", 2),
)
print("observed valid?", obs_fit.valid)
print("pred coeffs", pred_coeffs)
```

If the anchor guess is good, observed vs predicted ellipses should be close.

## Cost function + optimization (Substep 5)

### Core class: `EllipseCostFunction`

`EllipseCostFunction(dataset)`:
- Takes **raw delta dataset**.
- For each anchor guess:
  - Reconstructs absolute lengths.
  - Fits ellipses on reconstructed lengths.
  - Predicts theoretical ellipse geometry.
  - Computes mismatch cost in canonical `(x0, y0, a, b, θ)` space.

### Solver entry point: `solve_anchors`

Use `autocal/ellipse_solver.py` programmatically:

```python
import json
from autocal.ellipse_solver import solve_anchors

dataset = json.load(open("sweep_data.json"))

solution = solve_anchors(
    dataset,
    method="SLSQP",          # or "differential_evolution"
    max_iterations=2000,
    num_restarts=4,
    residual_threshold=0.01, # Sampson RMS threshold for per-iteration fits
    verbose=True,
)

print("best cost", solution["cost"])
print("anchors", solution["anchors"])
```

The solver returns:
- `anchors`: best anchor matrix (N×D)
- `cost`: final scalar cost
- `details`: per-sweep cost breakdown and valid/invalid counts
- `success`: optimizer status

## Plotting and verifying a solver result (Substep 6)

The visualization tools operate on:
1. The **raw dataset**.
2. An optional **fitted-ellipses sidecar** (list of dicts from `fit_all_sweeps` / `fit_ellipses.py`).

Because Phase 2 fitting happens per iteration, you typically create a
sidecar at the *final* anchor estimate for inspection:

```python
import json
import numpy as np
from autocal.ellipse_fitting import fit_all_sweeps
from autocal.ellipse_solver import solve_anchors

dataset = json.load(open("sweep_data.json"))
solution = solve_anchors(dataset, verbose=True)
anchors = np.asarray(solution["anchors"])

abs_sweeps = []
for sweep in dataset["sweeps"]:
    fixed_abs = [
        np.linalg.norm(anchors[idx]) + d
        for idx, d in zip(sweep["fixed_anchors"], sweep["fixed_lengths"])
    ]
    abs_points = []
    for p in sweep["data_points"]:
        abs_points.append({
            **p,
            "l_drive": p["l_drive"] + np.linalg.norm(anchors[sweep["drive_anchor"]]),
            "l_sensor": p["l_sensor"] + np.linalg.norm(anchors[sweep["sensor_anchor"]]),
        })
    abs_sweeps.append({**sweep, "fixed_lengths": fixed_abs, "data_points": abs_points})

fits = fit_all_sweeps(abs_sweeps, residual_threshold=solution["details"].num_valid_sweeps and 0.01 or 0.01, square_inputs=True)
json.dump({"fitted_ellipses": fits}, open("best_fits.json", "w"), indent=2)
print("wrote best_fits.json")
```

Then plot:

```bash
python autocal/ellipse_visualization.py sweep_data.json \
  --fits best_fits.json \
  -o plots/final_fits_grid.png
```

To focus on sweep observability:

```python
import json
import matplotlib.pyplot as plt
from autocal.ellipse_visualization import plot_sampling_density

dataset = json.load(open("sweep_data.json"))
fits = json.load(open("best_fits.json"))["fitted_ellipses"]
fits_by_id = {f["sweep_id"]: f for f in fits}

for sweep in dataset["sweeps"]:
    fig, ax = plt.subplots()
    plot_sampling_density(sweep, fits_by_id.get(sweep["id"]), ax=ax)
    fig.savefig(f"plots/sampling_{sweep['id']}.png", dpi=150)
    plt.close(fig)
```

Finally, for a compact report:

```python
import json
from autocal.ellipse_visualization import create_calibration_report

dataset = json.load(open("sweep_data.json"))
solution = json.load(open("solution.json"))  # or in-memory dict from solve_anchors
fits = json.load(open("best_fits.json"))["fitted_ellipses"]

create_calibration_report(dataset, solution, ellipse_fits=fits, output_path="plots/calibration_report.png")
```

## Tips / troubleshooting

- If a sweep is rejected in QC:
  - Inspect its `residual_rms`, `residual_max`, and `rejection_reason`.
  - Use `plot_sampling_density` to check sparse/noisy arc regions.
- `plot_anchors_3d` automatically falls back to a 2D XY view if your local matplotlib lacks 3D support.
- Keep raw datasets as deltas; only inflate to absolute lengths for *fit/plotting* outside the optimizer.

## To Just Get a Lot of Plots:
`python ../scripts/run_ellipse_calibration_debug.py data/sweep_data_slideprinter_for_test_slideprinter_training_data_fits.json --anchors "0.0,-1900.0;1645.44826719,950.0;-1645.44826719,950.0" --threshold 250 --min-points 10`


## To do a reference optimization with the old point method, including flex:
python autocal/calibrate.py point autocal/data/big_pattern.json --spring-k-multiplier 2.0 --debug
