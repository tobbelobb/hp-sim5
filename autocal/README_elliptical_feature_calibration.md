# Elliptical Feature Calibration (Auto)

This is the current, fully automated calibration pipeline. It collects sweeps, fits anchors with a pointwise robust cost, and actively chooses the next sweep to plug data gaps. The loop runs end-to-end: data collection, force auto-tuning, robust fitting, and active learning.

## Quick start (simulation)

```bash
python autocal/active_calibrate.py --semi-auto \
  --residuals-csv /tmp/residuals1.csv \
  --sim \
  --work-dataset autocal/data/test1.json \
  --robust-debug \
  --collector-args --speedup 25
```

To plot the residual histogram for the first iteration:

```bash
python scripts/plot_residual_hist.py /tmp/residuals1_001.csv --output /tmp/residuals1_001.png
```

## Data format recap (current)

A dataset JSON contains:

- Top-level metadata:
  - `machine_type`, `num_anchors`, `dimensions`, `timestamp`, `version`
  - `config` (machine settings, e.g. `m666`, `m669`, `mm_per_degree`, `force_tuning`, `max_travel_mm`)
- `sweeps[]` entries:
  - `id`
  - `fixed_anchors[]`, `fixed_lengths[]` (encoder deltas from origin)
  - `drive_anchor`, `sensor_anchor`
  - `drive_range` (optional)
  - `data_points[]` with fields like `l_drive`, `l_sensor`, `timestamp_ms`, `raw_angles_deg`, `drive_setpoint_mm`, `step_index`, `step_count`
  - `metadata` (optional)

All `l_*` values in the dataset are encoder deltas from origin. The optimizer reconstructs absolute lengths from the current anchor guess.

## Why this is fully automated now

Four ingredients make the new workflow hands-off:

1. Elliptical feature geometry
   - When you sweep along a circular sector, each drive/sensor pair traces an ellipse in the `(L_drive^2, L_sensor^2)` plane. This gives a strong geometric constraint without knowing anchors ahead of time.
2. Outlier-robust estimation
   - The optimizer uses a GNC-IRLS style pointwise loss (based on `ai_docs/Peng_IRLS/Peng_IRLS.md`) to down-weight or trim outliers as it converges.
3. Auto force tuning
   - Force ramping is now auto-tuned; you do not need to select torque/force ramps by hand for normal runs.
4. Active learning
   - The solver evaluates the current dataset, finds the most informative next sweep, and collects it automatically (see `ai_docs/Active_learning_calibration.md`).

Together these steps let the loop calibrate from scratch with minimal operator input, far beyond the old `autocal/calibrate.py point ...` workflow.

## How the active loop works

1. **Bootstrap (if the dataset does not exist)**
   - The loop auto-tunes force, auto-sizes travel, and collects an initial set of sweeps.
2. **Solve anchors (pointwise cost mode)**
   - Uses GNC stages (wide Huber -> tight Huber -> trim) on pointwise residuals.
3. **Write residuals + diagnostics**
   - The loop writes per-point residuals to CSV and prints robust filtering stats.
4. **Pick next sweep (active learning)**
   - Chooses the next sweep that improves observability and writes a sweep config.
5. **Collect and merge**
   - Calls the node collector with the chosen sweep and merges it back into the dataset.

## Pointwise cost mode (default)

The pointwise mode compares each measured point against the predicted ellipse (in the squared-length plane). It uses a robust loss with a GNC schedule:

- Stage 1: wide Huber (all points included)
- Stage 2: tight Huber (all points included)
- Stage 3: trim (hard cutoff)

Residuals are written as `residual_mm` (approximate mm via local linearization) along with `cutoff_mm`, which is the per-point trim threshold used in stage 3.

Use the histogram plotter to inspect quality without needing anchor context:

```bash
python scripts/plot_residual_hist.py /tmp/residuals1_001.csv --output /tmp/residuals1_001.png
```

If most points are far above the cutoff, the solver is likely underconstrained or the dataset is too sparse; collect more sweeps and let active learning plug the gaps.

## Understanding the output

The most important log lines (from a typical simulation run) are:

- `starting rrf_simulator ...`
  - The loop starts a local simulator when `--sim` is set.
- `bootstrapping dataset ...`
  - Initial sweep collection when the dataset does not exist yet.
- `auto-tune force ...`
  - Force tuning probes and selects a safe force envelope automatically.
- `size-tune max travel=...`
  - Auto-detects a safe travel range for sweeps.
- `Sweep 1/3 ... Collected 42 points ...`
  - Raw sweep collection. On a Slideprinter, each sweep has two sub-sweeps (drive/sensor swapped).
- `[gnc] stage 1/3 ... stage 3/3 ...`
  - The pointwise robust solver progressing through its GNC stages.
- `[robust] pointwise ...`
  - Summary of the pointwise filter settings, scale, and inlier ratios.
- `[residuals] wrote 126 points to ...`
  - A CSV file containing per-point residuals and `cutoff_mm` for plotting.
- `Active ellipse calibration ... cost=...`
  - Current anchor estimate and cost summary.
- `next_sweep ...`
  - The next most informative sweep suggested by active learning.
- `collect_command ...`
  - The exact node command the loop will run to collect that sweep.
- `Accept anchors [a], collect next sweep [c], quit [q]?`
  - Interactive prompt to accept the current anchor estimate or collect more data.


`--semi-auto` is optional; the semi-auto loop is the default mode.

## Related utilities (used by the semi-auto loop)

The semi-auto loop in `active_calibrate.py` uses the same sweep planner as `ellipse_active.py` and the same
merge logic as `merge_sweep_datasets.py`. You can invoke them manually:

- Plan a single next sweep (no collection):
  ```bash
  python autocal/ellipse_active.py autocal/data/merged.json --collector-args --return-to-origin
  ```
- Merge datasets:
  ```bash
  python autocal/merge_sweep_datasets.py autocal/data/base.json extra.json -o autocal/data/merged.json
  ```
