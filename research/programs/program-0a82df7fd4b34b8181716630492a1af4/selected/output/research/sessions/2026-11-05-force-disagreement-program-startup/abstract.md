# Force and measurement robustness across HP3 friction and noise variation

Status: in progress, first bounded worker running.

## Abstract

This program asks whether force and measurement choices can prevent large
encoder-only HP3 calibration errors across friction and noise variation.
The startup audit found completed profiling, warm-start and
early-stopping work, mixed solver/default results, a ten-world mass/gravity
comparison, and an origin-shift study. It replaced stale starting questions in
the parameter-variation prompt with narrower unresolved directions. Program
runtime setup and independent command socket checks passed. These are tool
checks. The human ended the interview and prioritized robustness, with any anchor
above 20 mm a failure and every anchor below 5 mm good performance. Friction is
the primary variation, and up to 20% additional collection time is acceptable for
substantially fewer large errors. A bounded worker has begun fresh collection.
The first nominal run stopped automatically with maximum applied anchor error
3.841 mm and all four anchors below 5 mm. The first active-friction/noise run
failed settling before collecting any points. Its clean encoder drift was much
larger than injected noise. One bounded preload policy is under development;
no policy improvement or separate evidence review is available yet.

## Topics

Autocalibration, HP3, force tuning, solver disagreement, parameter variation,
encoder-only calibration, research program, startup interview.

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| A bounded preload improves completion across friction/noise worlds | None completed | Not tested | Worker experiment in progress |
| Current default completes the first active-friction/noise world | dev1-active-default | Rejected for this simulated collection, unreviewed | [Worker evidence](../attempt-939c073f49004eb6949910600200be51/evidence/measurements.json) |

## Decision handoff

- Scope: known spool radii, zero buildup, no line layering, HP3 simulation.
- Review: manager inspected the historical reports; no separate review yet.
- Decision: preserve current defaults pending a paired robustness experiment.
- Next: verify active friction/noise interventions, then compare a frozen policy
  on held-out worlds. Initial results remain unreviewed.

## References

- [Research notebook](research.md): objective, candidate experiment, budgets and pending interview.
- [Startup report](report.md): audit and tool checks.
- [Updated prompt](../../../../research/prompts/autocal_parameter_variation_focus.txt): scoped questions and historical evidence links.
- [Ten-world and fallback report](../2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9d784051/report.md): existing simulated solver and force evidence.
- [Fresh reference study](../2026-10-20-interactive-research-ae0174ce/report.md): limits of origin-shift approaches.
