# Force-varied sweeps for robust HP3 autocalibration

Status: pending startup interview.

## Abstract

This program asks whether a fresh force-varied sweep can resolve disagreement
between L-BFGS-B and TRF and improve encoder-only HP3 calibration across different
physical worlds. The startup audit found completed profiling, warm-start and
early-stopping work, mixed solver/default results, a ten-world mass/gravity
comparison, and an origin-shift study. It replaced stale starting questions in
the parameter-variation prompt with narrower unresolved directions. Program
runtime setup and independent command socket checks passed. These are tool
checks. No new fits, simulated collections or calibration-accuracy experiments
have run, and no scientific result is available. The human requested a startup
grill. Questions about practical failure thresholds and physical variation are
pending; no worker has been dispatched.

## Topics

Autocalibration, HP3, force tuning, solver disagreement, parameter variation,
encoder-only calibration, research program, startup interview.

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| A force-varied sweep improves applied geometry across worlds | None | Not tested | Proposed in [notebook](research.md) |
| Solver disagreement reveals a useful next measurement | None | Not tested | Proposed in [notebook](research.md) |

## Decision handoff

- Scope: known spool radii, zero buildup, no line layering, HP3 simulation.
- Review: manager inspected the historical reports; no separate review yet.
- Decision: preserve current defaults; interview before commissioning.
- Next: settle the practical success/failure criterion and choose the first
  bounded discriminator. No calibration result is available yet.

## References

- [Research notebook](research.md): objective, candidate experiment, budgets and pending interview.
- [Startup report](report.md): audit and tool checks.
- [Updated prompt](../../../../research/prompts/autocal_parameter_variation_focus.txt): scoped questions and historical evidence links.
- [Ten-world and fallback report](../2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9d784051/report.md): existing simulated solver and force evidence.
- [Fresh reference study](../2026-10-20-interactive-research-ae0174ce/report.md): limits of origin-shift approaches.
