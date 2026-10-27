# HP3 parameter variation with cheap analytical screens

Status: complete, separate review pending.

Program program-3858dd28150847d492a61ea2345f9d08, assignment variation-first,
attempt attempt-3b2e360ec1834eee9bab4e06f6f880fd. Baseline source revision
67617705b7996ee3a84889e6723cab149b6e5884.

## Objective and constraints

Improve encoder-only HP3 calibration under plausible physical variation, preserving
one-click operation and automatic stopping. User allows two hours total compute.
This worker received 55 minutes from 08:04:40 UTC, 32 experiment-level fresh fits,
four collections and four million physics steps. No runtime ground truth may
choose or initialize a fit. Radii are fixed at 30 mm, buildup is zero, and native
probe scenes have every cable half-width set to zero before baking.

## Initial ideas and prior work

Before experiments I considered repeated force levels to isolate compliance,
reversal pairs to expose hysteresis, solver-disagreement measurements, and varied
worlds as a cheap screen before native collection. Existing reports already show
that blind thinning and unconditional TRF selection can regress. The October 15
TRF study varied mass/gravity but did not establish a force/friction/stiffness
robustness rule. The current code already has warm chaining, recovery, optional
TRF fallback and held-out consensus, so those are not new candidate changes.

The supplied reference-003 directory and raw historical stages are absent. Parent
searched exact reference hash cd938d1f2c6b86c33d49da94dff61151deaf758c16bdf01ec2d68ab6cd805a99
without finding a matching retained dataset. I use its archived numerical summary
only as historical evidence, never as a fresh reproduction or an available replay.

## Decisions and experiments

PV-E01 recomputes RMSE from 17 archived anchor matrices and costs in three old HP3
runs. It verifies the stored RMSE exactly and finds three adjacent cost/geometry
ordering disagreements. This is an arithmetic audit of archived fits, not a new
fit or a reproduction of the complete old workflow.

PV-E02 freezes eight analytical worlds before fitting, four development and four
holdout, with seeds 100-103 and 200-203. The generator varies layout, prescribed
linear material compliance, reversal offsets, encoder noise and arc span. These
are analytical measurement-error models, not force-balanced native dynamics.
Each world has 12 sweeps, 20 points each. Fits train on nine sweeps; the same last
three full-density sweeps test every candidate. Compare lbfgsb/trf and full/half
stored-point density, 32 outer solve_anchors calls. Internal GNC/restarts and
verification-test optimizer calls are distinct from this experimental count.
Acceptance was specified before outcomes in the saved contract and experiment
plan. No threshold was relaxed after fitting. Full coverage means successful
optimization, finite output, all nine training sweeps and every unseen point.

The 32 fits finish in 118.24 seconds elapsed. All inputs and separate evaluation
truth remain under PV-E02. Only six rows meet the full-coverage criteria. TRF
reduces mean holdout error relative to cold lbfgsb, but every broad robustness or
point-thinning proposal fails the declared gates. I retain a reusable bounded
benchmark and change no calibration defaults.

PV-E03 addresses one small analytical/native gap. Copy the source HP3 scene into
this session, set all nine cable half-widths to zero, and compare stiffness
20000 vs 8000 with identical commands. Each fresh native Rust world runs 2000
steps of 2 ms. At step 501, A enters 0.03 Nm torque mode; other motors retain
position mode. The peak paired A encoder difference is 0.5613 mm at 30 mm radius.
This establishes short native parameter sensitivity only, not correct analytical
compliance, collection behavior or calibration accuracy.

## Implementation and checks

New autocal/tools/benchmark_parameter_variation.py retains separate truth/input
files, contracts, hashes, all failed rows, full coverage checks, explicit fit cap
and a soft wall budget checked between fits. The final delivery adds bookkeeping
and guards after the measured matrix. PV-E02/benchmark-measured.py has the exact
SHA recorded at measurement. Generator, world definitions, density selection,
prediction function and solve_anchors invocation are AST-identical between measured
and delivered code. No experimental fits were rerun after the 32-fit matrix.

64 focused checks pass across the new benchmark, core solver and stage replay.
Initial test failures were an overbroad string check that matched num_anchors and
an arbitrary positive-residual threshold. Both are preserved in PV-T01.log and
corrected in the final tests. These were test defects, not environment failures.
No physics or motor code changed, so the physics-delivery gate does not apply.

## Environment and budget

Inherited runtime passed dependency, native artifact and socket checks. The private
supervisor preflight passed. Its untouched continuing world started layered by
default and was not used for collection or movement. PV-E03 explicitly uses copied
unlayered scenes. No actual cloud environment fault needed repair. Missing archived
research data is a provenance gap and was not replaced with invented evidence.

Experimental use is 32 outer core-fit calls, zero production collections, and 4000
native physics steps. The preflight recorder test is a tool check, not calibration.
Verification tests may run their own small optimizer cases and are reported
separately. The entire assignment finished well before its 55-minute deadline.

Next decision: independently review this scoped evaluation capability, then test
ordinary full-loop/native collection on nominal and softer worlds under a separate
bounded assignment. A HeadlessSimulation wrapper can assign a session-local scene
before __enter__; it must also update manifest.scene, which __init__ initializes
from the stock preset. Two bootstrap collections at matched manual forces are
cheaper and more informative than inferring native failure rates from these core
fits. Keep any new code/dataset hashes and actual accepted settings explicit.

Platform handoff note: the actor was interrupted after its source commit with "Selected model is at capacity". Parent resumed this same bounded attempt to finish handoff and archive native telemetry. No new experiments or source changes occurred during that continuation. Deterministic gzip copies preserve both native telemetry streams below the archive size limit; original and compressed hashes are in PV-E03-telemetry-archive.json.
