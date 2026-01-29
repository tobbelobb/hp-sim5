# Full-auto calibration implementation plan (overview)

## Goal
Deliver a `--full-auto` mode that:
- collects/merges sweeps,
- runs calibrations with multiple robustness settings (including all `--sweep-metric` options),
- compares results using a noise-normalized objective,
- stops when a confidence interval and convergence criteria indicate “good enough”,
- updates machine config on success, and emits data-quality warnings on failure.

This plan assumes the existing calibration pipeline already computes Fisher information and a covariance estimate for anchors (via the information matrix and pseudo-inverse). It also assumes sweep-wise filtering options are already present (including the three `--sweep-metric` choices).

## High-level flow
1. **Collect**: For each static pose, record encoder samples per cable and store the mean + robust noise estimate.
2. **Normalize & optimize**: Use noise-normalized residuals (whitened by per-pose, per-cable `sigma_eff[i][k]`) to drive objective scores and allow cross-dataset comparison via `J(theta) = (1 / N_obs) * sum z^2`.
3. **Multi-run selection**: Try multiple robustness settings (including all `--sweep-metric` values), rank runs by a common cost + covariance scoring, and select the best run.
4. **Stop conditions**: Accept when confidence intervals and convergence meet thresholds; otherwise retry or warn.
5. **Apply configuration**: On success, update config and running machine (per existing desired behavior).

## Documents in this plan
- **Substep 1:** Add encoder noise measurements to every collected datapoint.
- **Substep 2:** Define and implement noise-normalized cost & comparable metrics.
- **Substep 3:** Construct confidence intervals using Fisher information/covariance.
- **Substep 4:** Orchestrate `--full-auto` runs across sweep-metrics and stop criteria.
