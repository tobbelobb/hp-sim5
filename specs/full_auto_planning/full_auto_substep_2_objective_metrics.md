# Substep 2: Define and implement noise-normalized cost & comparable metrics

## Objective
Implement a dimensionless, comparable cost that works across datasets, machines, and algorithm flags.

## Core definition (whitened residuals)
For each pose `i` and cable `k`:
- measurement mean: `mu[i][k]`
- measured noise: `sigma_eff[i][k]`
- model prediction: `e_hat[i][k](theta)`
- residual: `r[i][k] = mu[i][k] - e_hat[i][k](theta)`
- whitened residual: `z[i][k] = r[i][k] / sigma_eff[i][k]`

### Primary cost (use for optimization and comparison)
Let `N_obs` be the number of valid `(i,k)` pairs. Use the mean squared whitened residual:
```
J(theta) = (1 / N_obs) * sum_{i,k} z[i][k]^2
```
If you know parameter count `p`, optionally report a reduced chi-square variant:
```
chi2_red(theta) = (1 / (N_obs - p)) * sum_{i,k} z[i][k]^2
```

## Additional metrics for ranking
- **Covariance summary**: trace, max std, or determinant of covariance (from Fisher info).
- **Robust residual summaries**: median |z[i][k]|, 95th percentile, outlier ratio.
- **Sweep-wise indicators**: per-sweep metric derived from existing sweep filtering.

## Expected integration points
- Use existing sweep-wise filtering flags and metrics in selection runs (see sweep-metric options).
- Ensure the same residual definition is used across all algorithm/flag variants to keep comparability.

## Deliverables
- Functions to compute `J` and optional `J_red` from dataset + model predictions.
- Standardized metric reporting for full-auto logs.
- Tests for cost invariance (e.g., constant scaling with sigma).
