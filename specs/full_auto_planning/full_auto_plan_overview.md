# Full-auto calibration implementation plan (overview)

## Goal
Deliver a `--full-auto` mode to autocal/active_calibrate.py that extends the existing `--semi-auto` mode.
That means the full auto mode:
1. collects/merges sweeps,
2. runs calibrations with multiple robustness settings (including all `--sweep-metric` options),
3. compares results using a noise-normalized objective,
4. stops when a confidence interval and convergence criteria indicate "good enough",
5. updates machine config on success, and emits data-quality warnings on failure.

A simpler versions of step 1 is already working with `--semi-auto`.
We need to extend it with noise measurements.
Step 2 is already implemented, at least if flags are sent manually.
We need to automate the permutations of configs.
Steps 3, 4, and 5 are brand new to the implementation of `--full-auto`.

This plan uses the existing calibration pipeline that already computes Fisher information and a covariance estimate for anchors (via the information matrix and pseudo-inverse). It also uses the sweep-wise filtering options that are already present (including the three `--sweep-metric` choices).

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
