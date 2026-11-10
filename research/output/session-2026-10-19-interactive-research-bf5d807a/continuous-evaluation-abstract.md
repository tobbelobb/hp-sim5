# Bounded continuous pullout: HP3 and HP4 native evaluation

Status: complete — candidate committed for testing; near-edge coverage remains unproven.

## Abstract

We tested which motion primitives can support force tuning without long equilibrium waits. Fresh Rust Native headless HP3 and HP4 experiments requested 25x speedup and measured force/displacement trajectories, encoder settling, body settling and sampled line loads. Useful force-mode motion usually remained unsettled after 12 physical seconds; captured encoder holds settled within the observed 1.75-second measurement floor. Holding encoders did not always settle the body. Cube Corners tests showed that fixed-anchor constraints can turn increased force mainly into elastic displacement. These results motivated replacing repeated pulse braking with a bounded continuous pullout, ramping to useful motion, holding force, then capturing and backing off. Production bootstraps completed 24 points each: HP3 at requested10x used 1023.39 physical seconds and 23.55 wall seconds; HP4 at requested25x used 936.40 physical seconds and 216.39 wall seconds. HP3 uses fewer simulation seconds than the earlier baseline but samples substantially smaller ranges. This is bounded collection evidence, not proof of geometric edge detection, hardware safety or improved calibration accuracy.

## Topics

HP3, HP4, Cube Corners, Hangprinter, autocalibration, force displacement, settling time, Rust Native, continuous pullout, encoder hold, momentum.

## Tested hypotheses

| Hypothesis | Experiments | Outcome | Evidence |
| --- | --- | --- | --- |
| Useful force motion quickly reaches equilibrium | E21 | Rejected within tested 12-second windows | [Curves](force-displacement-settling.png), [measurements](primitive-measurements.json) |
| Capturing encoders reliably stops all body motion | E21 | Mixed: encoder quiet, body sometimes still moving | [HP4 primitives](E21-primitives/summary-hp4.json) |
| Higher force guarantees large travel regardless of constraints | E21 Cube | Rejected: releasing fixed anchors changes travel greatly | [Cube comparison](E21-cube-release/summary-cubecorners.json) |
| Repeated pulse method improves full bootstrap on both machines | E18–E20 | Rejected | [Previous evaluation](pulse-evaluation.md) |
| Continuous bounded method completes both bootstraps | E23, E24 | Supported: 24 points each | [HP4 result](E23-hp4-continuous-25x/result.json), [HP3 result](E24-hp3-continuous-10x/result.json) |
| Continuous method reaches the geometric edge or improves calibration | None | Not tested | No Cartesian boundary or calibration-accuracy validation |

## References

- [Report](report.md): method, comparisons and limits.
- [Notebook](research.md): decisions, budgets and preserved failures.
- [Primitive harness](../../../../../tests/benchmark/force_primitives.mjs): production native force/hold commands and raw measurement collection.
- [HP3 dataset](E24-hp3-continuous-10x/sweeps.json), [HP4 dataset](E23-hp4-continuous-25x/sweeps.json): production collection evidence; sibling world.headless manifests retain configuration.
- [Interrupted HP3 budget trial](E22-hp3-continuous-25x/result.json): preserved physics-budget exhaustion, superseded by larger bounded E24 run.
- Implementation revision `d4a6fd75`: bounded continuous pullout, production integration, primitive harness and regressions.
- Tool revisions `35dd9e79` and `0e044aaa`: current-position capture and correct physical-time waits at accelerated settings.
- [Prior pulse evaluation](pulse-evaluation.md): earlier experiments and references, retained rather than overwritten.
