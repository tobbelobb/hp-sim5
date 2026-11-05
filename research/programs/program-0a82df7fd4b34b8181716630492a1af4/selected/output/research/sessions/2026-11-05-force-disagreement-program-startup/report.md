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
No calibration-accuracy result is available yet.


## Program startup and accepted brief

The refreshed prompt records earlier findings and four remaining questions. It
removes the false premise that all earlier work used one physical setup. The
ten-world study varied mass and gravity, leaving layout, stiffness and friction
unchanged. The human then selected force/measurement robustness and prioritized
friction, maximum per-anchor error and large-error prevention.

The first bounded investigation is running in its own worktree. Its precise
contract and accepted answers are in the [notebook](research.md). Initial recorder
and inactive-friction pilot problems are preserved by the worker; neither
establishes a calibration-accuracy failure. No scientific admission exists yet.

## References

- [Updated prompt](../../../../research/prompts/autocal_parameter_variation_focus.txt): completed work, unresolved questions and human criteria.
- [Worker notebook](../attempt-939c073f49004eb6949910600200be51/research.md): actual experiment decisions and failures.
- [Worker report](../attempt-939c073f49004eb6949910600200be51/report.md): numerical results when available.
- [Startup notebook](research.md): program identity, authority and budgets.
