# HP3 autocalibration at fresh centered and asymmetric encoder references

Status: completed bounded study through E17; no production solver or stopping change accepted.

## Abstract

Does moving the encoder reference near the anchors' centroid improve HP3 calibration or reduce collection effort? A retrospective fixed-offset rebase worsened 33/36 paired fits, but a causal audit showed changed exact-reference/noise assumptions and origin-dependent seeds/bounds; those failures cannot establish an inherent advantage of the original origin. Removing or falsifying stored M669 anchors gave bit-identical fits, with no direct truth leakage found. We then collected fresh native-Rust HP3 datasets at the original reference, at the exact virtual-anchor centroid (+265 mm vertically), and at an asymmetric reference, using identical manual forces, known 30 mm radii, zero buildup, no layering and normal automatic stopping. Final anchor RMSE was 4.588/4.277/12.602 mm using 140/180/300 points and 1.051/1.131/2.072 million physics steps. Centering slightly improved final accuracy but increased collection effort. A larger asymmetric start failed initial encoder settling. Sampled recordings show small measurement-time rotation for symmetric references but up to 2.11° for the asymmetric run. Limitations: single conditions on one nominal layout, deterministic simulation, changed physical trajectories, and no demonstrated fewer-point or hardware gain.

## Topics

HP3, CDPR, encoder autocalibration, fresh centered reference, origin shift, anchor scale, solver conditioning, symmetric initialization, geometry prerequisites, ground truth leakage audit, warm start, automatic stopping, settling, Rerun, fewer data points

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Low fitting scores reliably identify accurate geometry | E01 | Rejected: seven material adjacent disagreements | [Baseline](E01/results.json), [summary](summary.json) |
| Exact centroid translation uniformly improves local conditioning | E02-bootstrap, E02-corrected | Mixed: modest bootstrap improvement, worse at five sweeps | [Bootstrap](E02-bootstrap/results.json), [five sweeps](E02-corrected/results.json) |
| The implemented nearest-center rebase improves recovery | E03–E06 | Rejected for this workflow; general coordinate-invariant relocation not tested | [Corrected matrix](E05/results.json), [CLI replay](E06/results.json) |
| A feasible asymmetric retrospective reference improves recovery with less data | E05–E06 | Mixed at three sweeps; rejected as a robust policy | [Corrected matrix](E05/results.json), [CLI replay](E06/results.json) |
| Rebased fits are useful seeds after restoring the original objective | E07 | Rejected as a reliable improvement | [Polish](E07/results.json) |
| Reference transformation preserves residuals at one exactly translated geometry and supports ground-truth-free inversion | E02, E08, unit tests | Supported locally; this does not establish equivalent objective families/noise | [CLI checks](E08-cli/checks.log), [tests](tests.log) |
| Shifting reduces catastrophic intermediate failures under default L-BFGS-B | E09 | Rejected for retrospective workflow: chosen fits >100 mm, original 0/17, center 7/17, edge 5/17 | [Reliability audit](E09-reliability.json) |
| Stored M669 ground truth is used by the tested fit path | E10 | Rejected: deletion/falsification produce bit-identical fits | [Controls](E10-causal-audit/results.json) |
| The fixed-offset rebase is a statistically equivalent coordinate change | E11 | Rejected | [Reference residuals](E11-reference-assumptions/results.json) |
| Correctly translating the bootstrap estimate can prevent shifted warm-chain catastrophes | E12 | Supported for one diagnostic chain; no full-auto policy claim | [Warm chain](E12-warm-chain/results.json) |
| Fresh centered-origin collection improves accuracy with less effort | E13–E15 | Mixed: 4.588 -> 4.277 mm, but 140 -> 180 points | [Fresh results](fresh-results.json) |
| TRF resolves the new cold bootstrap failures | E15 | Supported for these two recorded three-sweep prefixes; not a default-policy validation | [Fresh fits](E15-fresh-fits/results.json) |
| A larger asymmetric start yields usable calibration data | E16 | Inconclusive for accuracy: initial settling timed out before measurements | [Failed trial](E16-offset/sweeps.full_auto.log) |
| The smaller asymmetric start improves collection efficiency | E17 | Rejected in this trial: 12.602 mm, 300 points; rotation complicates interpretation | [Fresh results](fresh-results.json), [rotation](sampled-rotation.json) |

## References

- [Full report](report.md): methods, corrected interpretation, measurements and limitations.
- [Research notebook](research.md): decisions, budgets, accepted steering and failed pilots.
- [Fresh dataset/plant/result summary](fresh-results.json): E13–E17 manifests, stage fits and metrics.
- [Fresh source and input audit](fresh-manifest.json): exact scripts, scene/firmware inputs, native artifact and readiness checks.
- [Fresh baseline dataset](E14-baseline/sweeps.json), [centered dataset](E14-center/sweeps.json), [asymmetric dataset](E17-offset-small/sweeps.json): newly collected production encoder/noise records.
- [Causal controls](E10-causal-audit/results.json), [reference-assumption audit](E11-reference-assumptions/results.json) and [warm-chain audit](E12-warm-chain/results.json): qualify earlier retrospective results.
- [Median and reliability audit](E09-reliability.json): process completion, automatic stopping, cold/warm fits and chosen iterations counted separately.
- [Offline revision and source hashes](manifest.json): initial experiment provenance; original inputs verified unchanged.
- [Original transform snapshot](source/autocal/tools/rebase_dataset.py): numerical transform used in the earlier trials; its later documentation explicitly warns about changed reference assumptions.
- [Supplied HP3 reference](../../../extended-autocal/no-buildup-reference-003/sweeps.json): REF source observations.
- [Varied-world study](../2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9d784051/report.md) and [world matrix](../2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9d784051/environment-matrix-20261015-retry1/matrix_summary.json): W00/W08 source simulations and evaluation geometry.
- [Warm-start study](../2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/report.md): prior sparse-data and initialization failures motivating matched seeds.


## Decision

**Updated after E10–E12:** keep production unchanged, and limit the negative conclusion to the implemented fixed-offset workflow. It changed reference assumptions and initialization; it did not fairly test a statistically equivalent coordinate change. The general reference-location idea remains open. The retained source change is an experimental offline tool with tests and documentation, so this idea can be reproduced without silently comparing incompatible coordinate frames.

## Data and evaluation

The initial E01–E09 experiments were offline numerical replays, with no movement or new physics recording; fresh E13–E17 collection is reported separately below. REF is the supplied nine-sweep no-buildup HP3 run. W00 is an independent nominal HP3 collection; W08 uses 1.15× mover mass and gravity 10.8 m/s², with seven available sweeps. These are the same nominal anchor layout under different simulated loading conditions, not three independent robot geometries. Source [W00 world configuration](../2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9d784051/environment-matrix-20261015-retry1/W00-nominal/world.json) and [W08 world configuration](../2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9d784051/environment-matrix-20261015-retry1/W08-heavy-high-g/world.json) establish HP3, native Rust physics, no layering, fixed radii and zero buildup. The fitting schema calls HP3 `hangprinter_4`; that label does not mean HP4 was simulated.

Anchor RMSE is sqrt(mean of squared Euclidean anchor errors), in mm, after restoring the original reference and canonical yaw. Pairwise anchor-distance RMSE is also retained to check that errors are not just frame alignment. Truth enters only these evaluation metrics. Reference choice, optimization, restoration and stopping do not read true anchors. Older checked-in buildup/radius-search fixtures were inspected but excluded rather than interpreted as zero-buildup data.

## Baseline reproduction: E01

Current revision `2c0b86ff189d27859b83543ed3fa837cfa658a3b` was replayed through the complete one-click loop with `--sim --no-collect`, known radius 30 mm and zero buildup. Source copies, commands, every fit/recovery artifact, selected history and logs are retained in [E01](E01/). Replays took 21.1–26.9 seconds wall time each; that measures fitting/planning/replay overhead, not live collection.

| Dataset | Applied RMSE | Best history RMSE | Applied regret | Sweeps used |
| --- | ---: | ---: | ---: | ---: |
| REF | 2.690 | 2.599 | 0.090 | 7 |
| W00 | 2.508 | 2.445 | 0.063 | 9 |
| W08 | 3.689 | 2.808 | 0.880 | 7 |

Seven adjacent score/error direction disagreements survive thresholds of 1e-6 in score and 0.01 mm in RMSE. For example, REF's score rises from 0.183 to 0.225 while geometry improves from 8.001 to 3.248 mm. Its first cold four-sweep solve also reaches the underconstrained score 100 with 1457.827 mm RMSE; the existing warm variant recovers 3.248 mm. Thus both score disagreement and solver fragility were reproduced, while main's existing recovery prevented that failed fit from being applied. See [summary](summary.json) and the preserved [failed cold fit](E01/REF/stages/fit-002-default.json).

## Reference transformation and restoration

For per-cable offsets q, the tool subtracts q from every fixed, drive and sensor length delta, including means and physical-drive setpoints. Absolute encoders and noise metadata remain intact. Point roles are normalized before shifting. Each file records offsets, source hash and reference indices. The solver's current sweep-level fixed-length convention is retained; this experiment does not implement per-point fixed-length modeling.

For a genuine translation p, transformed anchors satisfy `norm(A_i - p) = norm(A_i) + q_i`. Restoration solves the reverse distance problem from the fitted anchors and saved q, then canonicalizes yaw. No guessed Cartesian position is treated as known, and no truth is used. The output reports signed cable closure and RMS closure because four noisy readings need not have an exact three-coordinate solution. Small closure is not a proof of accurate anchors: the full CLI center trials had only 0.054–0.095 mm closure but 4.1–7.0 mm anchor error.

An exact fitted-geometry translation preserves all normalized residuals within 4.4e-11 in E02. Tests also verify noiseless inversion, nonzero inconsistent-reference closure and physical-length preservation. Inference: with noisy q held exact, rebasing can change the treatment of reference error and correlations; the experiments did not implement a statistically equivalent correlated-noise likelihood.

## Conditioning: E02

These are local finite-difference Jacobians of unfiltered normalized point residuals in the solver's nine-parameter gauge. They describe conditioning in that parameterization, not extra physical information or a global convergence guarantee.

| Dataset | 3-sweep original / exact centroid condition number | 5-sweep original / exact centroid |
| --- | ---: | ---: |
| REF | 213.4 / 188.5 | 57.5 / 95.6 |
| W00 | 212.8 / 188.5 | 57.5 / 95.6 |
| W08 | 169.5 / 152.3 | 52.0 / 79.7 |

The centroid modestly helps the bootstrap condition number but worsens all three at five sweeps. A separate asymmetric exact translation worsens all six. This does not support a general removal of scale ill-conditioning by moving the reference. The first sensitivity script selected an invalid cold fit for W00 and failed; its [failure log](E02-runner.log) remains beside corrected results.

## Matched fits: E03–E05

Reference points are located using an estimate fitted to the first three sweeps. “Center” is the observed point nearest its anchor centroid; “edge” is the farthest. Prefix tests use only points in the prefix. All three modes use the same estimated starting geometry, translated/canonicalized where appropriate, four restarts and 400 iterations. This isolates rebasing from acquiring a better initial estimate. Both L-BFGS-B and TRF are tested.

Unrestricted edges in E03/E04 moved anchors outside the production quadrant bounds. Those failed trials are preserved, but are not a fair rejection of feasible references. E05 restricts edge choices to translated seeds within the original bounds. Its [54 fits](E05/results.json) contain 18 originals and 36 shifted fits; **33/36 shifts worsen geometry**.

| Data budget | Solver | Original mean RMSE | Center mean RMSE | Feasible edge mean RMSE |
| --- | --- | ---: | ---: | ---: |
| 3 sweeps | L-BFGS-B | 8.813 | 9.390 | 8.345 |
| 5 sweeps | L-BFGS-B | 3.372 | 5.774 | 17.986 |
| All 7–9 | L-BFGS-B | 3.083 | 7.392 | 5.424 |
| 3 sweeps | TRF | 8.216 | 415.026 | 502.313 |
| 5 sweeps | TRF | 3.372 | 8.281 | 17.986 |
| All 7–9 | TRF | 2.533 | 3.919 | 6.952 |

The small edge benefit at three sweeps under one solver is neither robust to solver choice nor sufficient to match full-data accuracy. These are single fits per cell, not confidence intervals. Candidate scores, future-sweep predictions, per-cable closure and pairwise-distance errors are retained individually.

![Paired reference comparisons](reference-comparison.png)

## Complete autocal.py replay: E06

This implements the requested simple-file experiment: select a point from the original first-three-sweep estimate, shift the complete saved file, then run normal `autocal.py` with its ordinary cold initialization, warm variants, history selection, consensus and stopping. Restore its final M669 summary using the recorded offsets. No solver budget or point-count setting changes.

| Dataset | Original RMSE / points | Center RMSE / points | Feasible edge RMSE / points |
| --- | ---: | ---: | ---: |
| REF | 2.690 / 140 | 5.467 / 140 | 3.797 / 160 |
| W00 | 2.508 / 180 | 4.123 / 180 | 3.049 / 160 |
| W08 | 3.689 / 140 | 7.006 / 140 | 13.323 / 140 |

Original runs and shifted REF/W00 runs stop by the existing consensus-spread rule. **Both shifted W08 runs exhaust the recording and stop at `--no-collect`, rather than autonomously declaring completion.** Their printed summaries are measurable offline outputs, not evidence that a live one-click job would finish with that data. No calibration was sent to firmware. W00 edge uses one fewer sweep but worsens accuracy. No consistent collection saving was demonstrated. See [all terminal logs and artifacts](E06/).

## Restored seeds: E07

A possible salvage is to use rebased solutions only as proposals, then re-optimize on original data and choose using the original objective. In [18 tests](E07/results.json), all three- and five-sweep proposals return essentially the original solution. Full-data proposals are worse or fail into a bad basin. W00's center-derived proposal lowers original cost from 0.265 to 0.213 while raising RMSE from 2.379 to 4.167 mm; original-cost selection therefore does not repair this failure. No such seed policy was integrated.

## Changes, validation and limits

Added [rebase_dataset.py](source/autocal/tools/rebase_dataset.py), its [tests](source/autocal/tests/test_rebase_dataset.py), and usage documentation. It supports both calibration JSON and final autocal console summaries for restoration and refuses to overwrite prior evidence. [E08 CLI checks](E08-cli/) compare file-based shift/restoration with saved numerical results, including rejection of an overwrite. Targeted reference-transform, stage-replay, prediction and history-selection tests passed: **32 tests**, [test log](tests.log). No physics, motor, transport or scene implementation changed.

In that initial offline phase there was no new collection, browser experiment or Rerun recording. About 320 summed seconds were recorded across the replay and fit calls; E07 overlapped part of E06, and timing was not a matched performance experiment. No full-autocal speedup is claimed. Source hashes, installed package versions and unchanged input hashes are in [manifest](manifest.json).

The nearest stored “center” points in the retrospective phase remain 344–549 mm from the estimated centroid. Fresh actual-centroid collection was initially proposed and was subsequently tested in E13–E15 below. Statistically equivalent retrospective reference-uncertainty modeling remains untested. No measurement here establishes hardware behavior or fewer-point accuracy across different anchor layouts.

After the initial phase, proposed follow-ups were explicit reference-error likelihoods, per-point fixed-length modeling and common candidate-validation movements. These remain untested. The proposed new centered physical collection was later completed in E13–E15; it improves final accuracy slightly in this condition but does not save collection effort. The prior solver and warm-start reports linked above explain why blind point thinning or selecting by a new scalar alone is not justified.

## Reproduction

Run from repository root with `.venv/bin/python`. The research scripts create new experiment directories and preserve failures. For a new run, use a new session/output location or change the output label; do not delete this evidence.

```bash
.venv/bin/python -m autocal.tools.rebase_dataset shift INPUT.json \
  --sweep-index 0 --point-index 5 --output shifted.json
.venv/bin/python -m autocal.autocal --sim --no-collect \
  --machine-type hangprinter_3 --dataset shifted.json \
  --base-radii 30 --buildup-factor 0 --find-radii off \
  --find-buildup-factor off > shifted-console.log 2>&1
.venv/bin/python -m autocal.tools.rebase_dataset restore shifted.json \
  shifted-console.log --output restored.json
```

[Experiment orchestrator](tools/experiments.py), [full CLI replays](tools/cli_replays.py), [sensitivity analysis](tools/sensitivity.py), [restored-seed tests](tools/polish.py) and [aggregation](tools/summarize.py) specify the exact options and dataset selection. No external paper or web source was needed for these repository-local tests.


## E09: median scores and reliability audit (follow-up)

The user's follow-up asks whether the mean errors hid a better median or fewer complete failures. This audit re-reads the saved artifacts; no new fitting or collection was performed. [Full counts and individual records](E09-reliability.json) and [audit script](tools/reliability.py) separate cold variants, warm variants, chosen iteration fits, final summaries and automatic stopping. Fit artifact costs were checked against the corresponding iteration logs. “Over 100 mm” is an explicit evaluation threshold, not an existing production failure flag.

All values below use **default L-BFGS-B in the full automatic replays**, not the TRF experiment. Each reference has three datasets and 17 total dataset-growth iterations, though adaptive stopping gives different per-dataset iteration counts.

| Metric | Original reference | Center shift | Feasible edge shift |
| --- | ---: | ---: | ---: |
| Median final displayed fit score (lower better) | 0.1928 | 0.2704 | 0.2411 |
| Median final restored anchor RMSE, mm | 2.690 | 5.467 | 3.797 |
| Median chosen-iteration anchor RMSE, mm | 3.088 | 7.006 | 12.260 |
| Cold fits with underconstrained flag | 4/17 | 2/17 | 8/17 |
| Cold fits over 100 mm RMSE | 5/17 | 11/17 | 11/17 plus 3 unrestorable |
| Chosen iteration fits over 100 mm RMSE | 0/17 | 7/17 | 5/17 |
| Chosen iteration fits with underconstrained flag | 0/17 | 2/17 | 3/17 |
| Replays reaching autonomous consensus stop | 3/3 | 2/3 | 2/3 |
| Process crashes or nonzero exits | 0/3 | 0/3 | 0/3 |

The three unrestorable edge cold fits imply nonpositive original-reference cable lengths, so they cannot receive a meaningful restored-frame RMSE. They are recorded separately, not discarded. Thus 14/17 edge cold fits either exceed 100 mm or cannot be restored. No final printed summary exceeds 100 mm in any mode, because later fits and history selection can still recover. Both shifted W08 replays, however, run out of available measurements instead of establishing autonomous completion.

Default-reference cold fits are already fragile: worst cold RMSE is 2171 mm. Its warm variants avoid this in these runs (0/14 over 100 mm, worst 3.88 mm), leaving 0/17 chosen iterations above the threshold. Shifting does not improve that effective robustness. Center warm variants have 4/12 over 100 mm, edge warm variants 2/12; both modes allow severe failures into chosen iterations. Center's smaller *underconstrained-flag count* is therefore misleading as a reliability proxy.

The conclusion is also not an artifact of averaging. In the matched-seed E05 L-BFGS-B matrix, five-sweep median error is 3.248 mm original, 4.989 center, 15.334 edge; all-data medians are 2.725, 5.651, 5.679 mm. At three sweeps the feasible edge median improves slightly (8.001 -> 7.785 mm), consistent with the narrow benefit already reported. Median scores can still disagree with geometry: at five sweeps edge median cost improves 0.2243 -> 0.1833 while median anchor error worsens 3.248 -> 15.334 mm.

These are descriptive counts from three recorded simulated datasets sharing one anchor layout, not estimates of hardware failure probabilities. A chosen fit is an intermediate candidate, not a calibration applied to a robot. The initial audit encountered the nonpositive-length restoration failures; its traceback is retained in [E09-initial-failure.log](E09-initial-failure.log), and the corrected audit explicitly counts those cases.

## E10–E12: causal audit of the surprising origin dependence

The earlier negative conclusion was too broad. It demonstrates failures of the implemented fixed-offset workflow, not an inherent advantage of a physical origin or a valid rejection of statistically equivalent reference relocation.

The production initializer in [calibrate.py](../../../../autocal/calibrate.py) builds a symmetric lower-anchor triangle around zero, with the top anchor directly above zero and a fixed angular heuristic for lower-anchor height. [theoretical_ellipse.py](../../../../autocal/theoretical_ellipse.py) imposes fixed quadrants, shared lower-anchor height and coordinate bounds. These are origin-dependent geometry assumptions, not true anchor coordinates read from a dataset. [autocal.py](../../../../autocal/autocal.py) warm-starts from the latest history plan; it has no oracle that identifies a geometrically accurate starting history.

E10 controls on the REF five-sweep cold fit deleted `config.m669`, then replaced it with deliberately false anchors. Both returned **bit-identical anchors and cost** to the original. No direct truth leakage was found in this fit path. Known radii are explicitly authorized inputs. No actual anchor constants were found in the production fit modules. This does not rule out historical benchmark over-tuning; the tested datasets all share one symmetric nominal layout.

E11 exposes a change in the model, beyond merely changing coordinates. The selected reference measurement becomes exactly zero in all four cable deltas. Its normalized residual is then approximately zero for every tested anchor scale. In original data it remains an informative observation: at 0.95×, 1× and 1.05× anchor scale its residual is 104.92, 0.177 and -93.54; after rebasing it is below 1.5e-11 in magnitude. Also, the old reference's four-distance consistency is checked only *after* fitting: a tested shifted solution has 0.0835 mm RMS closure. The optimizer does not enforce that constraint. Reference-induced noise correlations were not propagated. Thus the earlier exact-translation check at one frozen geometry was necessary but did not establish equivalence of the full fitting problem.

Matched good-seed controls also separate the effects. REF five-sweep original RMSE is 3.248 mm. With the same physical starting geometry, an observed-q shift yields 4.989 mm and q made exactly consistent with that starting geometry yields 4.748 mm. A cold solve of that consistent-q problem instead lands at 1197 mm. Re-zeroing the raw encoder metadata changes the crude cold seed but does not rescue the tested observed-q cold failure (1187 mm either way). Initialization is therefore significant, but reference-noise treatment alone is not a complete explanation.

The warm-history failure mechanism is visible in the saved traces. Default W00 begins with a 7.38 mm fit that seeds later 3 mm fits. Shifted W00-center begins at 2102 mm, then 4957 mm; it obtains no useful warm history before later entries around 1200 mm. Reusing those entries cannot provide the protection seen with the good original bootstrap. This is a consequence of the seed/history quality, not a separate privileged-original warm-start branch.

E12 supplies a counterfactual diagnostic: start shifted W00-center from the correctly translated estimate fitted to its first three original sweeps, then carry each new shifted fit to the next sweep prefix. No truth is used for the seed. All seven fits remain below 8.04 mm (last 3.237 mm), with **zero >100 mm failures**, versus four such chosen iterations in the original E06 shifted W00 replay. This chain omits the complete CLI's competing cold-candidate pool and stopping logic, so it is not an end-to-end policy validation. It supports the bootstrap/history explanation while leaving residual geometric bias unresolved.

The comparisons are also deterministic center/edge interventions on correlated datasets, not random symmetric perturbations; a 50/50 improvement expectation does not apply. A fairer test is fresh collection at a genuinely new physical reference, plus a coordinate-consistent model if reusing old data.

Evidence: [E10 controls](E10-causal-audit/results.json), [offset/seed inspection](E10-causal-audit/controls.json), [E11 residual audit](E11-reference-assumptions/results.json), [E12 warm chain](E12-warm-chain/results.json). No new physics or collection was used in these causal controls.

## Fresh physical-reference collection: completed E13–E17

The user subsequently authorized fresh centered datasets. Initial E13 jobs collect matched baseline and actual centered HP3 bootstrap data with fixed forces 0.0223/0.4455/6.5739 N, speedup 25, radii 30 mm, zero buildup, layering off and one substep. Scene/firmware inputs and exact commands are frozen per trial. The centered reference corresponds to moving the effector +265 mm vertically relative to unchanged anchor geometry, implemented by translating the static assembly -265 mm. Samples and noise are collected normally, not retrospectively rebased.

Supervised HP3 expected-configuration preflight and separate [command socket probe](command-network-probe.json) passed. These establish tool readiness only. [Physics stability gate](physics-gate.log) passed all 11 tests. Pilot recording was sampled; E14 full-auto follow-ups used events, and E17 used sampled evidence for the asymmetric-motion question. The completed fresh-reference results follow. This section began as a pending plan; they supersede its pending status.


## E13–E17: fresh-reference measurements

Fresh collection changes the earlier conclusion: the centered origin *does* produce a slightly better final result in this trial. It does not reduce collection effort or reliably fix the bootstrap. No production seed, solver, collector or stopping logic was changed.

| Physical reference in original frame, mm | Terminal reason | Sweeps / points | Physics steps | Applied anchor RMSE | Best selected-history RMSE |
| --- | --- | ---: | ---: | ---: | ---: |
| Original (0,0,0) | spread-consensus | 7 / 140 | 1,050,815 | 4.588 mm | 3.557 mm |
| Centroid (0,0,265) | spread-consensus | 9 / 180 | 1,131,349 | 4.277 mm | 4.228 mm |
| Smaller asymmetric (250,100,265) | patience-or-threshold | 15 / 300 | 2,071,672 | 12.602 mm | 12.601 mm |
| Larger asymmetric (500,200,265) | collector initial-settle failure | 0 / 0 | 15,004 | No result | No result |

The completed runs use fresh native Rust worlds, the production collector, known 30 mm radii, Q=0, layering disabled and **one substep**. All use manual force-low/mid/max **0.0223/0.4455/6.5739 N**, speedup 25 and default ten points per sub-sweep (20 per sweep). The actual sensor collection force is 2.2275 N under the existing collector policy. No true anchor seed or fixed true lower-anchor height was passed to fitting. Nominal geometry defines the simulated plant and matching firmware configuration, as for the stock baseline, and is used separately for evaluation. The asymmetric truth is yaw-canonicalized before comparison; pairwise-distance RMSE is also retained.

The centroid is the centroid of the four *virtual anchor* vectors used by the translational calibration model: the initial lower anchors have Z=-280 mm and the top anchor Z=1900 mm, so their mean is +265 mm. The experiment translates the static spool/anchor assembly by -p while leaving the effector assembly at its normal reference. That is mechanically equivalent to moving the starting effector +p relative to the anchors, without inventing a noisy offset from an already collected point. The unlayered baker derives all initial cable geometry from the changed scene. This is a real change in collection location/trajectory, not a pure relabeling of one trajectory.

[Fresh-reference comparison](fresh-reference-comparison.png) shows the selected-history evolution. History excludes entries rejected as underconstrained; their raw and recovery artifacts remain in each trial's stage directory.

![Fresh collection effort and geometry](fresh-reference-comparison.png)

E13 used capped one-iteration pilot jobs with sampled recordings. The existing loop fitted the initial three sweeps and collected one additional adaptive sweep before hitting its cap, producing 80 points each. Both manifests say `stop_reason=max-steps` and `normal_completion=false`. Although they print and list applied parameters, **these are capped diagnostics, not successful automatic calibrations**. Default cold three-sweep fits fail at 2356 and 2610 mm error (original/center). Existing recovery returns 15.168 mm for original but 3062 mm for center. E14 starts new worlds and retains ordinary stopping; both recover at the four-sweep fit (3.579/4.228 mm) and subsequently stop automatically. Thus the centered bootstrap problem is recoverable, but worsens warm-history initialization.

E15 re-fits the newly recorded three/four-sweep prefixes using the two existing solver choices with no true seed. At three sweeps TRF reaches 15.168/13.503 mm original/center, instead of the default cold sentinel failures. At four sweeps both solvers reach 3.579/4.228 mm. This is evidence for a bootstrap-initialization problem in these inputs, not a validation of a global solver/fallback default; earlier legacy regressions still apply.

E16's larger asymmetric reference fails before recording a point: `Timed out waiting for encoder stability after 30000ms`. Its finalized events RRD, partial artifacts, manifest, scene and firmware inputs are preserved in [E16-offset](E16-offset/). It is a collection failure, not a failed anchor-fit accuracy test. The smaller E17 reference succeeds and automatically stops; its physical motion remains a limitation of interpretation.

### Measurement noise, motion and recording evidence

Sampled E13 and E17 recordings retain physical transforms; E14 routine runs use events. Every trial also retains the production command/encoder trace and stage artifacts. RRD sizes are approximately 77/72 MB for the sampled pilots, 100/113 MB for routine full-auto events, 1 MB for the failed large-offset event recording, and 271 MB for the sampled small-offset full run. These are actual full-job artifacts with Python stages/logs and different trajectories; they are **not** the matched replay recording-cost benchmark and imply no recording-mode speed comparison.

Noise was measured through the normal collector. The maximum recorded per-point angular sigma is 0.007413° for the baseline full run, 0.051891° for center, and 0.014826° for the small offset. No synthetic sensor noise was added, and deterministic repeated simulation is not a hardware noise distribution. The E13 and E14 first-three-sweep numeric hashes match exactly for each reference, including raw angles, means, sigmas and sample durations: [bootstrap repeatability](fresh-bootstrap-repeatability.json). Thus sampled versus event recording did not change these repeated bootstrap measurements.

[Sampled rotation audit](sampled-rotation.json) reads production effector quaternions from the RRDs. Maximum rotations near measurement timestamps are 0.0132° original, 0.0288° center, and 2.1086° small-offset; medians are 0.00411°, 0.00431° and 0.01048°. Nearest samples are within 0.1 simulated seconds of point timestamps. Maximum transient rotations over the whole recordings are larger (0.378°, 1.085°, 2.337°). The small offset therefore includes some measurements that challenge the fitter's no-rotation/3DOF assumption; its poorer result cannot cleanly isolate numerical reference conditioning. No physical snapshot was recorded for the larger offset's initial-settling failure, so its rotational cause is not established.

### Readiness, stability, code and bounds

The supervisor initially had the launcher's stock HP4 layered world. It was reconfigured to stock HP3, Q=0, radius 30 mm, no layering, before the complete expected-configuration [preflight](native/preflight/0d5421c013514957b2ea1321720ab9b9/preflight.json). The separate [agent-command socket probe](command-network-probe.json) passed. Those are tool checks, not calibration results. Fresh CLI worlds have their own scene/configuration, verified independently in their manifests. All fresh jobs load the same native artifact SHA-256 `259091f1d385722151a60e050e66f73010ab04de2af9bea235ac0d9fe8b39b50`.

`npm run test:physics` rebuilt native/Wasm and passed all 11 required stability tests: [gate log](physics-gate.log). The isolated [fresh-reference wrapper](tools/fresh_reference.py) selects the trial-authored scene without adding a production machine alias; it creates a separate temporary firmware config and removes that runtime copy at exit, retaining its frozen input in the trial. Stock USD scenes and production collection/physics/solver/stopping source were not edited. Source hashes, trials and input checks are in [fresh-manifest](fresh-manifest.json).

Pilot caps were 2.5 million physics steps and 600 seconds per job; full-auto caps were 5 million and 600 seconds, with ordinary max 20 fitting iterations. Neither successful full run terminated by a cap. The smaller asymmetric run took 176 seconds wall; baseline/center 77/93 seconds. These wall times include different trajectories, point counts, solver work and recording modes and are not a matched speed benchmark. Physical effort is compared by points and physics steps.

The correct outcome is modest: fresh centering slightly improves final accuracy in one paired condition, but costs 40 additional points and 80,534 additional physics steps. An asymmetric reference did not rescue efficiency in the tested cases. The earlier near-universal retrospective degradation had methodological and origin-dependent algorithmic confounds. No hardware validation, generic origin-invariance result, reliable failure probability or safe fewer-point policy has been established.


### Reproduce fresh trials

From the repository root, choose a new prefix to retain existing evidence:

```bash
.venv/bin/python output/research/sessions/2026-10-20-interactive-research-ae0174ce/tools/fresh_pair.py \
  --prefix NEW --max-steps 20 --recording events --variants baseline center
```

The orchestrator sets the per-job native step limit, thread budget and timeout. Its child [fresh_reference.py](tools/fresh_reference.py) freezes scene/firmware inputs, selects the custom scene only in the isolated process, and runs the standard `hangprinter_3` CLI. There is no installed `hangprinter_3_centered` production alias. Use `--variants offset-small` for the smaller asymmetric start, with sampled evidence if inspecting motion. Larger `offset` currently fails its initial quiet check under these unchanged settings; its existing failure is evidence, not a recipe for an accuracy result.
