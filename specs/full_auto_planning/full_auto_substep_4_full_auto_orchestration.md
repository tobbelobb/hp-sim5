# Substep 4: Orchestrate full-auto runs across sweep-metrics and stop criteria

## Objective
Implement a deterministic orchestration layer that tries multiple robustness settings (including all sweep metrics), compares results with the same normalized objective and covariance metrics, then stops when criteria are met.

## Candidate runs
Run calibration with all `--sweep-metric` options:
- `mad`
- `median_abs`
- `outlier_ratio`

Optionally expand the matrix of runs with:
- pointwise filtering on/off,
- sweep-wise filtering on/off,
- global vs per-sweep MAD.

## Ranking strategy (per dataset)
1. **Filter invalid runs**:
   - non-finite cost,
   - covariance not finite.
2. **Select the best run** deterministically:
   - Primary: lowest `J` (or a robust variant like mean rho(z)).
   - Tie-breaker: smaller scaled covariance summary in **dimensionless** units (e.g., `max_anchor_std_mm / workspace_diag`).
3. **Select the best run** (lowest primary + tie-breaker), avoiding unit-mixing unless explicitly normalized.

## Stop criteria
Accept and update config if all conditions are true:
- Confidence intervals below threshold using absolute + relative-to-size checks (e.g., `max_anchor_std_mm < 10 mm` and `max_anchor_std_mm / workspace_diag < 0.01`).
- Cost improvement below epsilon for `K` iterations (converged).
- No data-quality warnings.

Otherwise:
- If not converged but improving, gather more sweeps.
- If not converged and data quality poor, emit warning and stop.

## Deliverables
- Full-auto orchestration CLI or entrypoint.
- Structured result log summarizing cost, covariance, rank, and run flags.
- Deterministic run selection + clear warnings.
