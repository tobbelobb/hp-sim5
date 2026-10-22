# Stable encoder averaging and numerical fragility in HP3 autocalibration

Status: complete.

## Abstract

This investigation asked how to improve encoder-only HP3 autocalibration with known radii, no line layering and automatic stopping. Profiling the supplied recording found settling dominant, and frozen candidates reproduced score/geometry disagreement. Sparse sphere-closure and initializer trials did not justify fewer-point defaults. A return shortcut saved time in bounded trials but worsened full-auto collection. Original noise samples exposed accumulation error in 413 of 640 encoder mean fields. Generic centered averaging reduced this to two fields and made successful/failed recorded 140-point inputs return the identical constrained 2.4937 mm anchor-RMSE fit. The retained eight-line averaging change finished automatically with 200 points, 5659.096 simulated seconds and 2.3843 mm anchor RMSE. Its unpaired total is 16.24% shorter and its anchor error 15.61% lower than the supplied reference; different adaptive stopping paths prevent attributing this to faster collection. Against the fresh current baseline it uses 25% more points and 12.93% more time for 5.79% less anchor error. The return shortcut was removed. Evidence is simulated or analytical on one stock HP3 geometry; general fewer-point reliability, score-based accuracy and hardware transfer remain unresolved.

## Retained implementation

[encoder_noise.mjs](../../../../autocal/control/primitives/encoder_noise.mjs) now averages deviations from the first finite observation and adds that origin back once:

`mean = origin + sum(sample - origin) / count`

This preserves a constant encoder reading exactly and avoids accumulating small rounding errors while adding large unwrapped angles. It keeps every observation, the existing MAD deviation calculation and the same collector interface. It assumes no particular encoder resolution and adds no tuning parameter. The runtime diff is six additions and two deletions in one function, with two [arithmetic tests](../../../../autocal/control/tests/primitives/encoder_noise.test.mjs) and two README lines. The one-click collector, fitting/planning logic and automatic stopping remain the existing production workflow.

The proposed return-path change was removed after the averaging-only control finished sooner with exactly the same selected anchors. Faster isolated operations were insufficient evidence for retaining it in the full workflow.

![Recorded timings, geometry disagreement, encoder arithmetic and full-auto controls](results.png)

## Ideas and current algorithm

Before searching previous reports, the first-principles list considered redundant sphere closure/latent-pose elimination, actual fixed-encoder geometry, uncertainty from all encoders, comparable future-sweep validation, fewer broad-arc points, task-space precision, continuous trajectories and reuse of measured settling work. Primary papers supported redundant encoder constraints and covariance-guided data selection. Prior local research had already found unreliable thinning and common-validation heuristics, and had already removed a duplicated point-settling wait. Those findings motivated a bounded solver pilot and a separate timing audit.

The current implementation tunes force/span, fixes two HP3 cables and samples both drive/sensor directions. It fits ellipse constraints with graduated robust filtering and deterministic restarts. Known-radii fitting bypasses spool search. A D-optimal planner adds sweeps; history-adjusted scores control patience. Final validation predicts whole later sweeps with frozen models. Candidate-specific evaluation sets, model mismatch and numerical sensitivity can still disagree with geometry. An accepted calibration is not proof of accurate anchors.

## Reproduction and profiling

E01 independently extracts the supplied 1.5 GB RRD in 14.07 seconds and reevaluates its saved history. The recording contains 180 points in nine sweeps and spans 6756.422 simulated seconds. Command-to-next-command intervals attribute 3887.742 seconds to settling polls, 1299.520 to movement waits, 666.540 to force-trial polls and 409.720 to noise sampling. These source-clock intervals include transport/boundaries and are not a disjoint CPU profile.

The best raw score, 0.18349 at iteration 1, accompanies 8.0011 mm anchor RMSE. Final validation returns iteration 4 at 2.8255 mm; iteration 7 reaches 2.5278 mm. The fixed-encoder audit finds less than 1e-6 mm per-point variation, so discarding fixed-reading association cannot explain this reference case.

E12 also reconstructs exact nominal encoder length changes on a 27-point Cartesian grid using equal-weight four-cable forward least squares. Selected/reference-latest models produce 0.12451/0.07981 mm position RMSE. This is a model-derived diagnostic against nominal geometry, not independently measured tool positioning. Anchor error and positioning error are different quantities.

The fresh E02 baseline finishes automatically after six fits and returns its best geometrical candidate, iteration 5 at 2.53079 mm. Its first fit still has the lowest raw score but 7.36953 mm anchor RMSE. An intermediate commentary misread E02's selected iteration; matching saved history to final M669 corrected that reading.

## Negative hypotheses and numerical finding

- E03: 48 sphere-closure fits on four recorded prefixes are fast but geometrically mixed. A 3×3 inverse eliminates each position; sensitivity-normalized residuals use all four cable lengths. Some starts escape toward degenerate large geometries.
- E04: eight initializer trials do not enable three points per direction. All four sparse prefixes retain the cost-100 underconstrained sentinel.
- E05: 36 analytical short-arc fits vary encoder noise, fixed-constraint drift, seed and sampling. Noiseless cases recover truth, but 0.02 mm noise reaches 20.91 mm anchor RMSE with all points; five/three samples per half-arc reach 73.09/86.26 mm. This rejects a general sparse-data default.
- E06/E09: synthetic and fresh bounded physics returns save 1.50–8.064 simulated seconds with equal initial encoders and zero final lengths. Yet E07b return-only full-auto requires 240 points instead of 160, takes 25.78% more simulated time and selects worse anchors. The local shortcut is rejected.
- E07: an incomplete temporary checkout produces a non-UTF8 Git deletion diff and prevents recorder startup before physics. [Failure evidence](E07-candidate-live/sweeps.headless/recorder.log) is preserved; it is a setup failure, not an accuracy result.

In E07b, the first four fitted anchors exactly match baseline; the fifth fails with cost 100 and 7.27280 mm RMSE, versus baseline's constrained 2.53079 mm fit. Matched measurement means differ by only about 1e-13 degrees. E15 reconstructs this failure and tests controlled changes. Canonical RRF means make both inputs return the same constrained fit. Blind nine-decimal length rounding also removes the disagreement but worsens both to 3.32245 mm, so rounding is rejected.

E16 audits the actual RRD noise replies with exact decimal arithmetic. The JavaScript accumulator differs in 413 of 640 baseline mean fields, up to 4.55e-13 degrees. Generic centered averaging leaves only two differing fields, both within 2.84e-14 degrees. E17 applies centered averaging directly to captured samples: both recorded 140-point inputs return identical cost 0.215359 and 2.493734 mm anchor RMSE. Compensated averaging gives the same fit. The initial audit incorrectly interleaved session-local sequence counters; it was corrected to sort by session and sequence. Python 3.12 `sum` is a compensated control, not the naive JavaScript accumulator.

The error is far below physical encoder resolution, but it can change a nonconvex filtered solver's basin and recovery decisions. Correct averaging reduces one source of spurious differences; it does not repair all sensitivity. E20 improves some intermediate geometries and worsens others. E19 still encounters an underconstrained later fit. E22 complete stored-data replay selects the corrected 2.4937 mm result.

## Four fresh full-auto controls

All runs use HP3, 30 mm known radii, Q=0, disabled layering, default force tuning and ten points per direction. Baked scene, firmware configuration and RRF binary hashes match. Each completes automatically, applies M669, finalizes its recording and stops its owned services. Timings below use simulation clocks; overlapping workloads confound wall-time comparisons.

| Run | Points | Simulated seconds | Selected anchor RMSE, mm | Derived position RMSE, mm |
| --- | ---: | ---: | ---: | ---: |
| Current baseline E02 | 160 | 5011.394 | 2.53079 | 0.08342 |
| Return-only E07b, rejected | 240 | 6303.352 | 2.70782 | 0.10140 |
| Combined E19, return change removed | 200 | 5699.560 | 2.38435 | 0.07314 |
| Centered mean only E24, retained | 200 | 5659.096 | 2.38435 | 0.07314 |

Averaging-only and combined select exactly equal unrounded anchors; averaging-only finishes 40.464 simulated seconds earlier. This supported choosing the smaller patch; it does not isolate the return routine's runtime cost because the adaptive paths were not held fixed. The retained run improves anchor RMSE by 5.79% versus fresh baseline and reduces maximum anchor error from 3.5643 to 2.5054 mm, but collects 40 more points and takes 12.93% more simulated time. Relative to the older supplied reference, it is 16.24% shorter and 15.61% better in anchor RMSE, with 200 versus 180 points. Earlier changes already present in the current baseline contribute to that reference comparison; it is not attribution of all time savings to this patch.

The four runs show an observed accuracy/time tradeoff on one geometry. Different adaptive stopping paths prevent treating their totals as controlled collection-speed comparisons, and no universal point-count reduction is established. The stronger causal robustness evidence is the paired recorded fifth-fit failure corrected by accurate averaging. Selection scores, model bias and later underconstrained fits remain unresolved.

## Verification, limits and next experiments

The final controller/CLI suite passes 106 checks. All eleven stored calibration regressions remain exactly equal at every fitted parameter vector. Those fixtures test fitting/selection compatibility; the fresh full-auto controls test the changed collection statistics. Independent RRD inspection verifies every recorded measurement mean, frozen scene/config hashes, stop boundaries, finalized streams and exited services. Preflights and a separate command-context socket probe establish connectivity only.

No hardware trial, repeated statistical comparison across layouts, or independently measured printing-position accuracy is available. Sparse reliable acquisition is still unestablished. Better next experiments would center the squared-length residual calculation itself, assess numerical/gradient stability under tiny perturbations, model pointwise fixed lengths when they actually vary, and compare broad excitation or continuous acquisition at equal collection budgets. Task-space uncertainty should come from the appropriate joint model rather than a transplanted anchor-covariance heuristic. These are proposed experiments, not negative results.

## Tested hypotheses

| Tested hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Fit scores reliably rank known geometry | E01, E02, E12 | Rejected on reference; fresh baseline final validation selects its best fit | [Reference timing/ranking](E01-profile-and-geometry.json), [grid/anchor audit](E12-geometry.json) |
| Lost fixed-encoder variation explains the reference failure | E01 | Rejected for this dataset: variation below 1e-6 mm | [Fixed-encoder audit](E01-fixed-encoder-audit.json) |
| Sparse sphere closure robustly replaces current fitting | E03, E05 | Rejected as a default | [48 recorded fits](E03-closure-pilot.json), [36 synthetic fits](E05-synthetic-closure.json) |
| Closure initialization enables three points per direction | E04 | Rejected: all four sparse prefixes return the sentinel | [Eight initializer trials](E04-seed-pilot.json) |
| Faster individual returns improve full-auto collection | E06, E09, E07b, E19, E24 | Rejected as default; positive local timings do not yield a net gain | [Bounded returns](E09-physical-returns.json), [return-only failure](E13-live-comparison.json), [mean-only control](E26-mean-only-comparison.json) |
| Near-identical encoder means can change recovery and collection | E15 | Supported on paired recorded inputs | [Six numerical controls](E15-numeric-pilot.json) |
| Centered averaging improves mean accuracy and fixes the paired sentinel failure | E16–E18 | Supported on captured samples and paired fits | [Original sample audit](E16-mean-audit.json), [paired generic averages](E17-stable-mean-pilot.json), [arithmetic tests](E18-mean-tests.txt) |
| Centered averaging improves every intermediate geometrical fit | E20 | Rejected; mixed prefix outcomes | [Five additional prefixes](E20-stable-prefixes.json) |
| Averaging alone improves the complete one-click workflow | E22, E24, E26 | Mixed: better geometry, but more points/time than fresh baseline | [Stored replay](E22-replays/stable.log), [fresh comparison](E26-mean-only-comparison.json) |
| Stored-data fitting remains compatible | E11 | Supported: all 11 regressions exactly equal | [Regression log](E11-regressions.txt) |
| Continuous acquisition, task-space precision or pointwise fixed geometry improves calibration | none | Not tested | [Initial hypotheses](research.md) |

## References

- [Notebook](research.md): objective, hypotheses, budgets, steering, experiment decisions and failures.
- [Manifest](manifest.json), [retained diff](final-source.diff), [return-only diff](E07-return-only-source.diff), [combined diff](E19-combined-source.diff): baseline revision `45862559e95af66a5ed749540763d41d859f5b7e`, frozen code/input hashes and which changes survived.
- [Supplied reference](../../../extended-autocal/no-buildup-reference-003/), its [RRD](../../../extended-autocal/no-buildup-reference-003/hangprinter-20261010T182010.757112Z-814ac4d9.rrd), [dataset](../../../extended-autocal/no-buildup-reference-003/sweeps.json) and [stage snapshots](../../../extended-autocal/no-buildup-reference-003/sweeps.stages/): E01/E12 recorded evidence at revision `1e026534483e2175f4c101b645b1f54f65f9faba`.
- [Fresh baseline manifest](native/output/research/sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/E02-baseline-live/sweeps.headless/manifest.json), [return-only manifest](E07b-candidate-live/sweeps.headless/manifest.json), [combined manifest](native/E19-stable-mean-live/sweeps.headless/manifest.json), [retained averaging-only manifest](E24-mean-only-live/sweeps.headless/manifest.json): used HP3 configuration, effective layering, geometry, firmware, timings and applied M669.
- [Independent RRD verification](E14-rrd-verification.json): measurement counts, encoder means, frozen scene/config hashes, physical stop and finalized recordings; tool/recording integrity does not establish calibration accuracy.
- [Prior settling study](../2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/report.md): optimized point waits and endpoint recovery already in the current baseline.
- [Prior thinning/ranking study](../2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-130caf5a/report.md), [covariance study](../2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-76802a07/report.md), [bounded-update study](../b7deb1bcc9cb418388273e2b7b9adc8c/report.md): preceding negative results narrowed this study; their experiments are not counted as new results.
- Yuan, Han; You, Xianghui; Zhang, Yongqing; Zhang, Wenjing; Xu, Wenfu (2019), “A Novel Calibration Algorithm for Cable-Driven Parallel Robots with Application to Rehabilitation,” DOI [10.3390/app9112182](https://www.mdpi.com/2076-3417/9/11/2182): redundant encoder-length least squares motivates the closure pilot; ideal simulated accuracy does not establish noisy HP3 performance.
- Caverly, Ryan J.; Cheah, Sze Kwan; Bunker, Keegan R.; Patel, Samir; Sexton, Niko; Nguyen, Vinh L. (2025), “Online Self-Calibration of Cable-Driven Parallel Robots Using Covariance-Based Data Quality Assessment Metrics,” DOI [10.1115/1.4065236](https://experts.umn.edu/en/publications/online-self-calibration-of-cable-driven-parallel-robots-using-cov/): institutional abstract motivates covariance-guided data retention and stopping; PDOP/ODOP are distinct from the current anchor covariance.
- Alexandre dit Sandretto, Julien; Daney, David; Gouttefarde, Marc (2012), “Calibration of a Fully-Constrained Parallel Cable-Driven Robot,” ROMANSY, [author PDF](https://www-sop.inria.fr/coprin/PDF/a_romansy2012.pdf): redundancy-based joint geometry/pose self-calibration motivates latent-pose elimination; its embodiment differs from HP3.
- [Current algorithm](../../../../autocal/optimization_algorithm_overview), [objectives](../../../../autocal/objective_functions_overview), [ellipse guide](../../../../autocal/README_elliptical_feature_calibration.md), [extended recording guide](../../../../autocal/data/references/extended-reference.md): solver, collector, selection and recording contracts actually inspected.


## Follow-up on speed comparisons and tooling

[Benchmarking and harness review](tooling-review.md) responds to the user's stopping-policy correction. Full-run totals are descriptive policy outcomes, not isolated collection-speed estimates. Proposed fixed-work, common-budget and tool improvements are not tested. No new experiments were run.
