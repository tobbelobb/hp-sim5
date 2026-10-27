# Cheap analytical HP3 parameter screens expose misleading encoder fit scores

## Abstract

Can varied physical assumptions be screened without paying for many full HP3
collections? Eight deterministic analytical worlds vary geometry, prescribed cable
compliance, reversal offsets, encoder noise and movement span. Thirty-two current
core fits compare lbfgsb/trf and 20/10 stored points per sweep using data-derived
initialization and common unseen sweeps. The matrix takes 118.24 seconds; only six
fits satisfy successful-optimizer and complete-coverage checks. Lower unseen
residual does not reliably imply better anchors. In a clean case, TRF's unseen RMS
falls from 0.02362 to 0.02234 mm after thinning while anchor RMSE rises from 5.89
to 40.20 mm. No default solver or thinning change passes the declared robustness
gates. A retained benchmark separates truth from inputs and preserves failures,
hashes and budgets. Two short native Rust probes confirm encoder sensitivity to
cable stiffness, with peak paired A difference 0.561 mm. These analytical core
fits bypass full-loop recovery, planning, consensus and stopping. They establish
neither faster collection nor hardware accuracy.

## What was tested

PV-E01 recomputes historical RMSE from the archived October 20 E01 arrays, 17
anchor matrices across REF, W00 and W08. Recomputed RMSE matches every stored
value exactly. Three adjacent raw-cost/geometry orderings disagree, with counts
0/1/2. This narrower cost audit differs from the preceding session's seven
history-score disagreements. Its raw source datasets and stage files are absent,
so this is saved-evidence inspection, not a current-code reproduction.

PV-E02 generates exact sphere-intersection circles. Two fixed motors define each
circle; the other two supply drive/sensor lengths relative to the encoder origin.
A linear material-length law l = d * (1 - F/S) adds prescribed tension-dependent
compliance, with fixed reference tensions 4/4/4/12 N. Reversal offsets add opposite
signed drive/sensor errors on the two traversal directions. Independent Gaussian
encoder noise is reproducible from each world's seed. These tensions are imposed;
force equilibrium is not solved. The reversal offset is not native friction.

The fixed radius is 30 mm and buildup is zero. The three lower anchors
share the model's common lower z plane. Nine training sweeps use lower-anchor
deltas 0, +120 and -120 mm, with each lower anchor fixed in turn alongside D.
Three unseen sweeps at +240 mm test every candidate using the same full-density
observations. Point thinning keeps five of ten observations per direction,
including endpoints, without recollecting. Truth is absent from the fitter's
dataset and initialization; it lives in separate evaluation-truth.json files.
The ordinary build_anchor_initial_guess uses the largest observed negative
encoder travel. Here that yields anchor seed radii about 160-274 mm against
actual radii about 1.5-2.3 m. Poor initialization and weak curvature are therefore
competing explanations for large errors, not just the imposed physical changes.

The eight cases and development/holdout split were fixed before fits. D0 is clean,
D1 compliant with stiffness 20000 and 5 N tension span, D2 has 0.6 mm reversal
offsets, and D3 is 1.2x larger with a shifted top anchor. Holdouts H0-H3 are a
0.8x compact layout, a softer 8000-stiffness/8 N-span world, an asymmetric layout,
and a short/noisy/reversal case. Seeds are 100-103 and 200-203.

Each experimental fit calls solve_anchors with four restarts, 400 iterations,
robust loss, ordinary GNC/filtering and sequential execution. These are 32 outer
core-fit calls, with internal restart/stage work. They bypass fit-stage recovery,
warm chaining, active sweep planning, final history consensus and automatic
stopping. Large errors must not be described as demonstrated full-auto failures.

## Measured results

Mean anchor RMSE includes every finite fitted result, including unsuccessful or
incompletely covered fits. This avoids hiding failures through success filtering.
Complete means successful optimizer, finite output, all nine training sweeps and
all 60 unseen points valid. Zero invalid_sweeps alone does not prove coverage,
because trimming can reduce valid training sweeps without incrementing it.

| Split | Points/sweep | Solver | Mean anchor RMSE, mm | Complete fits |
| --- | ---: | --- | ---: | ---: |
| Development | 20 | lbfgsb | 1230.10 | 0/4 |
| Development | 20 | trf | 216.42 | 1/4 |
| Development | 10 | lbfgsb | 1323.97 | 1/4 |
| Development | 10 | trf | 249.25 | 1/4 |
| Holdout | 20 | lbfgsb | 1272.78 | 0/4 |
| Holdout | 20 | trf | 692.75 | 1/4 |
| Holdout | 10 | lbfgsb | 1846.03 | 1/4 |
| Holdout | 10 | trf | 540.06 | 1/4 |

TRF beats the cold lbfgsb fit in all four full-density holdouts, with mean ratio
0.544. It fails the declared full-coverage gate, and absolute errors remain poor.
This is useful evidence for a narrower solver investigation, not a justified
production default switch.

Half density fails its nonworsening criterion. For TRF, compact H0 rises from
35.69 to 64.72 mm and asymmetric H2 rises from 152.08 to 866.64 mm. Its lower
aggregate mean conceals a 714.56 mm individual regression. lbfgsb half density
has a maximum holdout regression of 2096.74 mm. Neither passes.

There are 17 pairwise reversals between common unseen Sampson-derived residual
RMS and geometric RMSE within the eight candidate pools. Two concrete examples:

- Clean D0 TRF full/half yields unseen RMS 0.02362/0.02234 mm, while geometry
  worsens from 5.885 to 40.203 mm. Both rows have complete coverage.
- Clean D0 lbfgsb full predicts unseen points at 0.1474 mm RMS while anchor RMSE
  is 1883.7 mm. Optimization is unsuccessful and only six training sweeps remain.

The first example directly rejects the claim that slightly better unseen encoder
residual always means better geometry, even after basic validity checks. It does
not establish that additional carefully chosen movements are useless.

The complete matrix takes 118.242 s elapsed, with 0.02167 s spent generating
geometry/observations and 118.077 s across fit-plus-prediction/truth-scoring blocks.
Generation timing excludes input serialization; elapsed timing includes input
and result I/O. No native collection was timed against this analytical generator,
so no collection speedup ratio is claimed.

## Native sensitivity probe

PV-E03 uses two session-local copies of the production HP3 USD scene. All nine
cable half-widths are zero before bake, all physical spool radii remain 30 mm,
and only cablePath:stiffness changes between 20000 and 8000. Identical commands
hold position for 500 steps then switch A to 0.03 Nm torque mode. Each native Rust
world runs 2000 fixed 2 ms steps, four simulated seconds. Commands hashes match.
Both trials complete without warnings or missed steps. Their combined physics
wall time is 4.978 s and construction time is 0.383 s.

Peak paired encoder differences, converted with the fixed 30 mm radius, are
0.5613/0.0539/0.0692/0.0315 mm for A/B/C/D. Final A difference is 0.3020 mm.
This proves short native encoder sensitivity to stiffness. It does not validate
the analytical error magnitude, force balance, a completed sweep or calibration.
No production collection occurs in this assignment.

## Retained implementation and verification

The retained change adds a reusable analytical benchmark and documentation. It
writes separate observations and evaluation truth, the exact world contract,
source/input hashes, partial results and failed rows. It enforces an outer-fit
cap and checks a soft wall budget between fits. It reports optimizer status,
finite outputs, training coverage, unseen coverage and aggregate comparisons
separately. It never uses ground truth to select or initialize a runtime fit.
Calibration defaults and the one-click stopping policy do not change.

The exact measured script is PV-E02/benchmark-measured.py, SHA256
bb668edc447eeb82cf479dfce93c62dc9ffaaa1086cd35c8197c36762447c47a.
The final delivery adds validity/failure retention, aggregate summaries, argument
validation and the soft wall guard after measurement. An AST check confirms
unchanged world definitions, generation, thinning, prediction and core solver
invocation. No extra experimental fits were run after the 32-fit screen.

Verification command:

```bash
source scripts/cloud/env.sh
.venv/bin/python -m pytest -q autocal/tests/test_parameter_variation_benchmark.py autocal/tests/test_ellipse_solver.py autocal/tests/test_stage_replay.py
```

All 64 checks pass in PV-T06-final.log. Tests cover exact zero-error circle
geometry, raw-angle consistency, deterministic inputs, split disjointness,
endpoint-preserving thinning, separate truth, fit caps, failed/nonfinite rows,
soft wall budget and complete coverage. Existing solver/stage checks also pass.
Verification may invoke small optimizer cases independently of the 32 experiment
calls. No engine, motor or scene-dynamics implementation changed, so the physics
change gate is not applicable. Native scene copies are experiment inputs only.

Initial PV-T01 failures came from two bad test assertions, a string check matching
num_anchors and an arbitrary positive-residual threshold. Their logs remain;
the final tests check actual truth-field absence and a nonzero model error.

## Environment, limits and next decision

Cloud dependencies, correct Node/Rust/native artifact, independent command sockets
and supervisor preflight passed. No environment failure required repair. The
untouched attached continuing world was layered by default, but it was not used
for movement or collection. PV-E03 uses explicitly unlayered scene copies.
Missing historical raw data remains a provenance limitation, not a fabricated
replacement dataset. No claim of beating reference-003 is possible here.

The analytical worlds are stress cases, not samples from a validated hardware
parameter distribution. They omit effector rotation, force equilibrium, motor
behavior, settling, sag and hardware friction. A low data-derived seed makes this
screen sensitive to cold initialization. Full-loop recovery and warm chains may
avoid many failures. All claims concern simulation or analytical data.

The next useful discriminator is a separately budgeted matched native collection
on nominal and softer scenes with the same forces and commands, followed by the
ordinary fit stage and recovery. A small HeadlessSimulation wrapper can set its
session-local scene before __enter__ and update manifest.scene. Bootstrap-only
collection costs much less than assuming this analytical ranking transfers to
full automatic runs. Keep automatic stopping unchanged until that evidence exists.

## Tested hypotheses

| Hypothesis | IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Raw fit cost always orders archived geometric accuracy | PV-E01 | Rejected in saved evidence | [Audit](PV-E01-historical-audit.json) |
| A cheap varied analytical screen exposes solver/model weaknesses | PV-E02 | Supported in this screen | [Contract](PV-E02/contract.json), [results](PV-E02/results.json) |
| TRF or half density passes the declared replacement gates | PV-E02 | Rejected for these gates | [Summary](PV-E02-delivery-summary.json) |
| Common unseen residual always orders geometry | PV-E02 | Rejected, 17 pairwise reversals | [Pairs](PV-E02-summary.json) |
| Cable stiffness affects native encoder motion under identical commands | PV-E03 | Supported for this short probe | [Native summary](PV-E03-summary.json) |
| These methods reduce fresh collection or improve hardware accuracy | None | Not tested | Limits above |

## References

- [PV-E02 measured source and provenance](PV-E02/provenance.json): exact generating script hash and base revision; [measured script](PV-E02/benchmark-measured.py).
- [PV-E02 results](PV-E02/results.json), [contract](PV-E02/contract.json), [summary](PV-E02-summary.json): complete raw fit outcomes and predefined cases/gates. Per-world observations and evaluation-truth files sit beside them.
- [PV-E03 results](PV-E03-results.json), [summary](PV-E03-summary.json), [plan](PV-E03-plan.json): native encoder sensitivity and experiment provenance. The result manifests link retained telemetry and frozen scenes.
- [Source equality](PV-T05-measured-source-contract.json), [64 checks](PV-T06-final.log): final implementation verification and measured/delivered numerical consistency.
- [October 20 reference-origin study](../2026-10-20-interactive-research-ae0174ce/report.md), [archived source arrays](../2026-10-20-interactive-research-ae0174ce/E01/results.json): source of the arithmetic-only historical audit, not available raw datasets.
- [October 15 solver/varied mass-gravity study](../2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9d784051/abstract.md): prior TRF gains and legacy-fixture regressions motivate avoiding a blanket default switch.
- [October 15 warm-start/thinning study](../2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/report.md): distinguishes offline thinning from fresh low-density collection.
- [Current solver](../../../../autocal/ellipse_solver.py), [initializer](../../../../autocal/calibrate.py), [headless runner](../../../../autocal/headless_sim.py): actual source APIs used and follow-up feasibility.
- [Parent literature note](../2026-10-27-autocal-parameter-variation-manager/literature.md): externally sourced encoder-only calibration context, not empirical support for this screen.

Platform handoff note: the actor was interrupted after its source commit with "Selected model is at capacity". Parent resumed this same bounded attempt to finish handoff and archive native telemetry. No new experiments or source changes occurred during that continuation. Deterministic gzip copies preserve both native telemetry streams below the archive size limit; original and compressed hashes are in PV-E03-telemetry-archive.json.
