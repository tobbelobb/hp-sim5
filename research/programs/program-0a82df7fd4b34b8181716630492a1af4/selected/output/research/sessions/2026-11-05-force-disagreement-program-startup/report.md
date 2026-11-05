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


## Startup audit

The refreshed prompt records the historical findings and four remaining
questions, with links and limitations. It removes the claim that every earlier
run used one physical setup. The ten-world study varied mass and gravity while
leaving layout, stiffness and friction unchanged. Its modest solver difference
does not resolve force robustness or large errors.

The program controller opened successfully in cloud mode with startup grilling
and ongoing consequential questions enabled. Dependency and socket checks passed.
No calibration experiment, implementation change, review or scientific admission
has occurred. See the [notebook](research.md) for the provisional first experiment,
budget and pending interview. The prompt edit passed git diff --check.

## References

See the [abstract](abstract.md) and the evidence links in the
[updated prompt](../../../../research/prompts/autocal_parameter_variation_focus.txt).
