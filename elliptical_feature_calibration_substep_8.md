# Substep 8: Shared Logic and Comparison Harness (Ellipse vs Point-Based)

## Overview

Align the new ellipse-fitting pipeline with the existing point-based solver in `autocal/auto-calibration-simulation-for-hangprinter/simulation.py`. The goals are:
- Both solvers consume the same sweep JSON format (plus optional legacy data) via a thin adapter.
- Common geometric/physics routines live in shared modules instead of being duplicated across `simulation.py` and the new `elliptical_feature_autocal.py` (or similar).
- Side-by-side runs produce comparable outputs (anchors, cost breakdowns, plots) to validate parity and guide migration.

Ensure the ellipse side of the comparison reports Sampson (approximate geometric) residuals so “validity” and RMS figures align with the fitter used in Phase 2, and document that the point-based solver may use a different metric.

## Implementation Details

### 8.1 Shared Core Library

Create a small shared layer (e.g. `autocal/calibration_core.py`) that exposes:
- Anchor parameterization and bounds helpers (`anchors_vec_to_matrix`, `get_anchor_bounds`).
- Spool/line-length helpers reused by both solvers (`distance_samples_relative_to_origin`, flex/buildup models).
- Sweep utilities to rebuild absolute lengths from encoder deltas (`rebuild_absolute_lengths(sweep, anchors)`).
- A common `CalibrationResult` dataclass carrying anchors, residuals, and human-readable summaries.

Both `simulation.py` and `elliptical_feature_autocal.py` import from this shared layer instead of re-implementing math.

### 8.2 Data Adapters for `simulation.py`

Add an adapter that can feed the point-based solver from sweep data:
- `sweep_to_point_samples(dataset, sampling_strategy)` that replays each sweep into synthetic point samples the existing solver understands (e.g. use per-point `l_drive`/`l_sensor` + fixed anchors to reconstruct cartesian poses or line-length deltas).
- Update `simulation.py` entrypoints to accept either legacy `data.py` inputs or a `SweepDataset` path: `python simulation.py --sweeps sweep_data.json [--flex --no-line-lengths]`.
- Keep the old API intact for backwards compatibility; the adapter is opt-in and lives in a separate module (`autocal/sweep_adapter.py`) so the point-based solver is not tangled with JSON parsing.

### 8.3 Comparison Harness

Extend `calibrate.py` (from Substep 7) with a `compare` subcommand:
```bash
python calibrate.py compare sweep_data.json \
  --elliptical --point --report compare_report.png \
  --dump-json compare_results.json
```
Responsibilities:
- Run both solvers on the same sweep dataset (point-based via the adapter).
- Normalize outputs into `CalibrationResult` (anchors, cost, residual stats).
- Emit diffs: anchor-wise distance table, RMS difference, overlay plots (ellipse residuals vs point residuals), and optional QC table of which sweeps/samples each solver used.
- Optionally fit ellipses at each solver's final anchors and place both fits in the report for visual comparison.

### 8.4 Regression Tests

- Synthetic parity test: generate sweeps from known anchors; assert both solvers recover anchors within the same tolerance and their costs are monotonic when anchors are perturbed.
- Adapter smoke test: load a minimal sweep JSON, run `sweep_to_point_samples`, ensure `simulation.py` accepts the resulting samples without touching legacy globals.
- Report structure test: `compare` command produces JSON with both result blocks and diff metrics.

### 8.5 Migration Notes

- Keep sweep JSON canonical; do not persist ellipse fits. Any point-based sidecars produced by the adapter stay ephemeral (or in explicit debug files) to avoid schema drift.
- If flex/buildup options diverge between solvers, unify them in the shared core or clearly flag which model each run used in the comparison output.
- Document the comparison workflow in `README.md` or a short `docs/elliptical_vs_point.md` so users can validate new hardware data against the legacy solver before fully switching.
