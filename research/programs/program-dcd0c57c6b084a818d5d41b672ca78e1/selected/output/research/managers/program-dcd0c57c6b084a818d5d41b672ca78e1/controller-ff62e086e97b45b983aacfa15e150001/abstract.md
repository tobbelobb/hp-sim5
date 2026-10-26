# Hangprinter unknowns: discarded fixed observations and a missing length standard

Status: complete for this bounded iteration; scoped scientific finding separately reviewed.

## Abstract

This program asks which important Hangprinter uncertainties deserve the cheapest next experiment. Three fresh offline tests compare current NumPy and JAX calibration residuals, synthetic scale changes and local Jacobians. Permuting fixed-angle observations changes no residual across complete 75-point synthetic and 60-point frozen simulated conditions, while positive controls respond. Scaling geometry, radii and dimensionful buildup together preserves clean synthetic consistency below 1e-12 mm RMS. Diverse sweeps improve the weakest observable local sensitivity by 7.51 times but leave rank 13 of 14. An assumed independent radius observation raises local rank to 14. The final numerical work took 2.97 seconds after imports. The first noise-normalization control hit a production floor and was corrected; it remains preserved. The resulting decision prioritizes pointwise modeling and auditing independently measured length quantities. Broader stretch, winding, dynamics, held-out prediction and hardware-transfer questions remain open. A fresh separate reviewer supports keeping the scoped report. No fits, collection, physics advancement, production change or hardware improvement is claimed.

## Topics

Hangprinter, encoder-only calibration, fixed encoder association, scale identifiability, spool radius, buildup, experimental design, NumPy, JAX, hardware validation

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Pointwise fixed-angle association reaches the tested objective | E1 | Rejected in the unfiltered tested paths | Worker E1, complete finite arrays and active controls |
| Encoder observations fix common scale with free radii and dimensionful buildup | E2 | Rejected in the specified synthetic model | Worker E2, dimensional controls and retained initial trial |
| Diverse sweeps remove the common-scale direction | E3 | Rejected in the specified synthetic model | Worker E3, rank 13 of 14 at two derivative steps |
| One independent radius observation removes that local scale null direction | E3 | Supported locally | Worker E3, augmented rank 14 of 14 |

## Decision handoff

- Scope: frozen simulated HP3 prefix and specified HP3-like synthetic geometry at source c0f111c3. This is residual sensitivity, not measured accuracy.
- Review: manager inspected report, source provenance and numerical controls. Separate reviewer inspected fixed artifacts and hashes, supported the claim, and required no additional check. No independent numerical reproduction was claimed.
- Decision: retain calibration defaults; prototype pointwise fixed-length modeling and audit which spool quantities are measured.
- Next: proposed independent cable displacement per encoder-angle observation. It could bound scale error without another full calibration collection.

## References

- [Full report](report.md): synthesis, scope and next observations.
- [Notebook](research.md): allocation, setup failures and accepted user instructions.
- [Uncertainty map](unknowns.md): prior-report-based open questions.
- [Source identities](sources.json): inspected sources and hashes.
