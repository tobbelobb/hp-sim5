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

## Decision and implementation

Choose a small repair of continuous force pullout rather than repeated pulse braking or fitting an equilibrium knee. Ramp force by 1.6 every two physical seconds until useful drive/free-encoder motion and 8 mm/s drive speed. Then keep force constant. Stop on backtracking, sustained slowdown, or the 60-physical-second travel bound. Capture all positions and return to 90% of the largest observed cable excursion. Record the stop reason and samples; edge_measured is always false. A time bound does not identify the geometric edge. All six production sweep directions on both machines reached this time bound.

Automatic size tuning uses paired drives with the same procedure. Sweep positioning captures motors before ordinary settling checks. Failed tuning throws while holding motors; there is no guessed 1 N fallback. Explicit force caps are respected without silently clamping them to 20 N. The default 20 N ceiling remains an experimental ceiling, not a universal line-load rating. Motor command force is not an upper bound on every supporting line's tension. Generic motor arrays support different axis counts, but hypothetical HP5 is not tested.

## Measured collection results

| Run | Requested speedup | Points | Physical seconds | Wall seconds | Canonical ranges (mm) |
| --- | --- | --- | --- | --- | --- |
| E24 HP3 continuous | 10x | 24 | 1023.39 | 23.55 | 380.59, 523.87, 364.12 |
| E23 HP4 continuous | 25x | 24 | 936.40 | 216.39 | 540.09, 554.52, 491.54 |
| E20 HP3 earlier baseline | 1x | 24 | 2423.04 | 48.73 | 1323.09 each |
| E18 HP3 repeated pulses | 1x | 24 | 2922.93 | 66.77 | See prior evaluation |

The continuous HP3 run uses about 58% fewer physics seconds than E20, but substantially smaller coverage. Different requested speed settings and concurrent workload prevent interpreting observed wall times as a matched speedup benchmark. HP4's requested25x is compute limited: achieved throughput is roughly 4.3 physical seconds per observed wall second. Successful collection is not evidence of better inferred geometry. E22 exhausted its initial 1000-physical-second budget; E24 raised that explicit bound to 1500 seconds and completed.

## Primitive interpretation

HP3 sweep travel after each 12-second force stage at 0.1, 0.5, 1, 2, 5 and 12 N was approximately 0.2, 8, 62, 168, 341 and 407 mm. HP4 was 0.14, 2.45, 15.2, 49.4, 140 and 316 mm. Stages are sequential and history dependent, not independent equilibria. Most useful-motion stages had censored settling time greater than 12 seconds. Thus no universal force/displacement knee or equilibrium curve shape was established.

Captured-position encoder settling was observed at 1.75 seconds, the first eligible quiet-window observation, not an exact physical stopping time. HP4 paired 12 N retained body motion beyond the three-second hold window and reached sampled line tension 24.32 N. Cube Corners with six fixed motors moved only 1.90 mm at 8 N in four seconds; releasing to two fixed motors gave 387.21 mm, still moving, and body settling exceeded the three-second window. More force does not universally yield useful workspace travel; constraint selection matters.

The direct harness isolates motors and physics; the full bootstrap additionally exercises production RRF integration. Numerical JSON recording avoids RRD costs. These are simulated measurements only. Current-position capture and physical-clock fixes are independently committed reusable tooling. The attached supervisor was source-stale and was not used for live movement; isolated fresh native worlds and a successful command-context socket probe were used instead.

## Validation and remaining limits

The selected JavaScript behavior, primitive, hold and RRF compatibility suite passes 100 tests. Regression checks cover force-cap failure, insufficient motion, fixed-anchor slip, travel timeout, endpoint return and an independently bounded wall clock. Prior Python hold regression tested protocol parity, not a Python simulation campaign. No further experiments are pending. Near-edge workspace coverage, hardware strain limits, HP5 behavior and resulting calibration accuracy remain unresolved; this implementation is a bounded candidate for user testing, not a demonstrated solution to all those objectives.
