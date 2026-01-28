# Substep 3: Construct confidence intervals using Fisher information

## Objective
Use the Fisher information matrix to estimate parameter covariance and construct confidence intervals that enable a principled stop condition.

## Existing building blocks
The active calibration code already computes an information matrix and a covariance estimate (via pseudo-inverse), and reports the matrix rank. These provide the basis for confidence intervals and for diagnosing rank deficiencies.

## Proposed approach
1. **Compute information matrix** for the current dataset and parameterization.
2. **Estimate covariance** via pseudo-inverse with optional regularization.
3. **Scale covariance** by the observed normalized residual variance:
   - `Cov_scaled = Cov * chi2_red` (or `Cov * J` if you do not compute `chi2_red`).
   - This is standard in weighted least squares when sigmas are off by a constant factor.
4. **Extract standard deviations** from the scaled covariance diagonal.
5. **Build confidence intervals** per parameter:
   - `CI_95% = estimate ± 1.96 * std` (or 2.0 for simplicity).
6. **Use CI widths** to enforce the “good enough” threshold, but prefer absolute + relative-to-size thresholds instead of only percent-of-parameter:
   - `max_anchor_std_mm < X mm`
   - `max_anchor_std_mm / workspace_diag < Y`

## Handling rank deficiency (5/6 issue)
If the information matrix is rank-deficient:
- **Report rank** in logs (already available).
- **Investigate geometry**: insufficient excitation, symmetric anchor layout, or missing degrees of freedom.
- **Actionable responses**:
  - Trigger new sweeps that improve observability.
  - Increase sweep diversity (e.g., different fixed anchors or deltas).
  - Apply mild regularization for numerical stability (but track if this masks poor data).
- **Do not hard-fail by default**: treat rank deficiency as a strong warning and allow success only if observable parameters stabilize and unobservable ones are constrained or held fixed.

## Deliverables
- Confidence interval computation from covariance matrix.
- Rank-deficiency detection and user-facing warnings.
- CI-based stopping criterion using absolute + relative-to-size thresholds.
