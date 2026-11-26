# Research topic guide

These summaries describe the solver revisions and conditions in the linked reports.
The [local catalog](output/index.md) also lists integration checks, incomplete
sessions, and their status. Use experiment-lookup to find reusable run payloads
in `output/experiments/` and on Drive.

The local catalog is a generated snapshot of the session directories available on
one machine. Its embedded abstracts remain useful when raw files are absent,
but regenerating it in another checkout can produce a different inventory.
Completed program archives under `research/output/` preserve selected reports
and evidence in Git; prefer those links when continuing in another environment.

Four hash-named sessions contain substantive autocal work: three have final reports and one has an unfinished notebook. Missing research.md does not imply that no research happened.

## Full-machine hardware validation

- [COPacabana dynamic PlaneA replay against the FARO tracker and eight cable forces](output/session-copacabana-planea/report.md) (2026-11-26, hardware + simulation, PR #129 & PR #130; see also [autocal/firmware handoff](output/session-copacabana-planea/autocal-handoff.md), [DaRUS data card](../ai_docs/externally_captured_reference_data/DATA_CARD.md), and [constrained ablation script](output/session-copacabana-planea/evidence/calibration_ablation.py)).
  - **Spool & rope physics in physical Newtons:** Confirmed a shared `1/√2` 90° carriage-pulley force-sensor scaling (`F_true = √2 · f_logged`) across DaRUS-3600, 5171, and 4614. Re-identified 6 mm D-Pro XTR (`EA = 118.5 kN`, `k_r = 0.299 µm/N` at `R = 50.18 mm`), Vectran HT (`139.2 kN`, `0.298 µm/N`), and PolyTropFOX (`72.1 kN`, `1.093 µm/N`), updating `DEFAULT_SPOOL_RADIAL_COMPLIANCE` to `1.50e-8 m²/N` (`E_t ≈ 67 MPa`) and preserving all `5171` (`158 / 155 ppm` blind ablation) and `3600` (`−5.96` vs `−6.15 ppm/N`, `13 ppm` unwind/rewind asymmetry) predictive gains.
  - **COPacabana dynamic replay & moving swivel pulleys:** Live `SwivelPulleyAPI` anchors match analytical `pulley_ik` to `1e-15 m`. Kinematic encoder replay (`planeAEncoders`, `3.55e-15 rad` max spool error) settles within `1.6 mm` of commanded targets (`16.0 mm` RMS vs FARO tracker); closed-loop `planeATargets` gives `25.5 mm` RMS. Both replays are guarded in CI (`copacabanaPlaneAReplay.test.js`).
  - **Open hardware/firmware gaps:** (1) Linear `EA = 118.5 kN` under-predicts 13.9→29.1 kg payload sag by 50–58 % (`−1.85 mm` vs `−3.69 / −4.40 mm` FARO); (2) actuator-side force sensors carry `~3.6 %` (lower 1-pulley) vs `~12.8 %` (upper 3-pulley) directional routing friction plus `56–119 N` winch Coulomb friction; (3) PlaneA/CircPath show a model-free `3.5–5.7 %` Y-travel shortfall whose constrained calibration (Stages A0–F2) is strongly collinear across `(sx, sy, sz)`, `s_enc`, `d0_z`, and `EA`, so published `COPacabana.xml` geometry is preserved untouched; (4) RRF/Klipper planners do not yet model swivel pulleys, strained drum storage, radial flattening, or nonlinear low-tension `EA(F)`.

## Anchor layout and research tooling

- [Anchor-layout search within a 2 m radius](output/session-anchor-layouts/report.md)
- [Research Program usefulness audit](output/session-program-usefulness/report.md)
- [Research instruction audit](output/session-research-instruction-audit/report.md)

## Rough-bootstrap robustness: HP5r seed 3 was a one-sample force auto-tune plateau rule

Status: complete, unreviewed, 2026-11-21. Topics: autocalibration, force auto-tune, size tuning, rough bootstrap, imperfect machines, HP3, HP4, HP5, HP5r.

- **Cause.** HP5r's edge ramp creeps about 140° at both 1.44 and 2.02 N, then runs free at 2.83 N. On seed 3 a −0.4° gain fired the single-sample "travel stopped increasing" rule, which set force max at 1.44 N, just above the 1.03 N breakaway. Size tuning then moved 0.25 mm and the rough bootstrap had nothing to fit.
- **Fix** (`9b2941ee`). A flat or reversing sample is one of the two declines that are needed. Seed 3 becomes good-fit at 1.17 mm.
- **No change elsewhere.** 16 of 16 comparable runs are bit-identical. Of 105 replayed archived ramps, only seed 3's and two old HP4 workspace-limit reversals change.
- **Failure rate.** On random machines the bootstrap failed 1/23 before (HP5r 1/10) and 0/23 after.

Simulation only.

[Report and evidence](output/program-2026-11-20-autocal-on-imperfect-machines-pr-121-follow-up-1-a-wrong-configu-4b12cdf0/session-2026-11-21-rough-bootstrap-robustness-on-imperfect-machines-hp5-pulley-redu-20f0ffb5/report.md).

## Force realisation: fit the realised force, then compliance

Status: complete, unreviewed, 2026-11-21. Topics: autocalibration, force realisation, cogging, hysteresis, compliance, imperfect machines, HP3 ellipsoid, HP5r.

- **Cause.** The sim's commanded-vs-realised tension error is pure torque-mode cogging, 0.333 N · sin(50 θ_encoder) per spool. It has no friction, hysteresis or scale error.
- **What the data identify.** Errors that vary with recorded variables (encoder angle, direction of motion) are identifiable from the sweep rms. Once the ripple is modelled, so is a per-motor constant offset/scale (corrected after review). A common scale is degenerate with k.
- **Result.** Fitting per-motor ripple, and a direction offset, jointly with anchors and k from the rms gives the following, with no probe and no configured stiffness:
  - HP3 ellipsoid: 0.16 ± 0.06 mm (11 runs; live 2.22 mm).
  - HP5r: 0.74 ± 0.21 mm (6 runs).
  - The result holds at 2×/4× ripple, including live 2×, and with ±0.2 N hysteresis if a direction offset is fitted (0.32 mm).
- **Blind test** (unknown 137/turn ripple, 0.5 N single-motor offset): the period is found and the offset recovered. HP3e max error is 0.17 mm mean over 7 held-out cases (6/7 < 0.3).
- **Caveats.**
  - The headline numbers use the oracle 50/turn basis.
  - HP5r blind test not done.
  - HP5r collection fails at 2× cogging.
  - The home-stretch emulation was a double count.

Simulation only. Fit-side diagnostic, not integrated code.

[Report and evidence](output/program-2026-11-20-autocal-on-imperfect-machines-pr-121-follow-up-1-a-wrong-configu-4b12cdf0/session-2026-11-21-force-realisation-error-in-autocal-data-replays-t3-test-complian-8252ab16/report.md).

## Compliance probe on HP5r sweeps and the ellipsoid shape

Status: complete, unreviewed, 2026-11-21. Topics: autocalibration, stretch, compliance, imperfect machines, HP3 ellipsoid, HP5r.

- **Result.** Where configured stiffness trades strongly against the top anchor, measuring k matters. Configured S leaves mean max error 2.5–2.6 mm (worst 5.4) at EA ×0.8/×1.2. The force-step probe's k cuts it to 1.0 mm (HP5r) and 1.4 mm (HP3 ellipsoid) for about 3% time.
- **Bias.** The probe's k is biased about ±4%, with the sign depending on the machine. An exact replay shows this is force realisation (commanded ≠ realised tension), plus 2% missing reeving in HP5r's M666 Y. It is not the line model or the sim's compliance. The same realisation error drives most of the fit's remaining top-z error.
- **Joint rms fit.** It is free and as good in mean, but it has occasional 13% misses on HP5r.

Simulation only.

[Report and evidence](output/program-2026-11-20-autocal-on-imperfect-machines-pr-121-follow-up-1-a-wrong-configu-4b12cdf0/session-2026-11-20-compliance-probe-where-stiffness-really-trades-against-the-top-a-62081729/report.md).

## Home-stretch correction as the default autocal fit

Status: complete, unreviewed, 2026-11-21. Topics: autocalibration, stretch, home reference, imperfect machines, HP3, HP4, HP5, HP5r.

- **Change.** The ABC home-strain term `(T/S)(d0+Y/U)` now applies on every equal-strain d+1-line machine (HP3 and HP4), not only HP4.
- **HP3, live and paired on 10 seeds.** Print-frame max error falls from 1.06 ± 0.31 to 0.44 ± 0.10 mm, better in 10/10. The common low-z shift goes from −0.80 to +0.06 mm. Stops are unchanged.
- **Stiffness.** A ±20% EA error costs about ±0.1 mm of common z.
- **HP5/HP5r.** Their per-point flex model already treats encoder zero as unstretched. Forcing the term on doubles the error, so it is excluded there and their results are byte-identical.

Simulation only.

[Report and evidence](output/program-2026-11-20-autocal-on-imperfect-machines-pr-121-follow-up-1-a-wrong-configu-4b12cdf0/session-2026-11-21-make-the-home-stretch-correction-the-default-autocal-fit-the-com-01a484d6/report.md).

## Compliance observability: force-step probe measures line compliance

Status: complete for HP3, 2026-11-21. Topics: autocalibration, stretch, compliance, home reference, imperfect machines, HP3, HP4, HP5.

- **Where it matters.** A wrong configured stiffness trades strongly against the top anchor height only on HP5 pulley-reduced sweeps (8.4 mm per unit compliance) and in the ellipsoid shape (5–10 mm). On HP3 the ~1 mm common z shift is ABC stretch relative to the slack encoder origin.
- **What fixes it.** A fixed-pose force-step probe (~150 s simulated) measures the compliance to ±4%. With it, HP3 D.z lands within 0.04 mm of the true-compliance fit on nine live runs at EA ×0.8–1.2.
- **What remains.** The leftover D.z shape error comes from force realisation.

Simulation only.

[Report and evidence](output/program-2026-11-20-autocal-on-imperfect-machines-pr-121-follow-up-1-a-wrong-configu-4b12cdf0/session-2026-11-20-compliance-observability-for-the-top-anchor-height-with-todays-c-4f2510fd/report.md).

## Uneven low-anchor heights: free low z, gauge at the top anchor

Status: complete, 2026-11-21. Topics: autocalibration, anchor gauge, frame convention, low-anchor heights, HP3, HP4, HP5, HP5r.

The old point fit gave all low anchors one z.
- **HP3/HP4:** one low anchor h mm off became an exact, undetectable tilt, with 0.68·h anchor error.
- **HP5/HP5r:** it distorted the shape (2–2.4·h) and made a live HP5 run fail at 5 mm.

Per the user's convention (origin at home, top anchor straight above it, A.x = 0), every Hangprinter fit now frees each low z, and `--low-anchor-z` is gone.
- **Offline:** error stays flat in h, at HP3 1.0, HP4 0.7, HP5 1.0 and HP5r 1.6 mm, for a cost of ≤ 0.3 mm at equal heights.
- **Live:** HP5 A+5 now completes at 0.78 mm.
- **Unchanged:** the D.z/stiffness coupling.
- **HP5 jackknife:** under the free gauge it printed "poor" in 4 of 9 sub-mm HP5/HP5r runs, because one fold lost a coordinate's only constraint.
  - **Fix:** that fold is now left out of that coordinate alone, found from pose information, and the coordinate is reported as single-sweep supported.
  - **Results:** false "poor" fell from 4/9 to 1/9; all 56 single-sweep corruptions kept their classical label; the 84-run corpus replay changed no decision.
  - **Rejected:** a median spread, which review showed hid 2 of 22 single-sweep failures above 5 mm.
- **Also measured:** HP4 live with A raised 10 mm is 1.04 mm; a top line 50 mm off plumb moves the anchors ≤ 0.09 mm.
- Simulation only.

[Report and evidence](output/program-2026-11-20-autocal-on-imperfect-machines-pr-121-follow-up-1-a-wrong-configu-4b12cdf0/session-2026-11-20-uneven-low-anchor-heights-the-default-point-fit-assumes-all-low-8b2f2abb/report.md).

## Imperfect-machine benchmark: about 1.1 mm on random HP3s; uneven low heights are a gauge on three low anchors

Status: complete, 2026-11-21. Topics: autocalibration, imperfect machines, benchmark harness, anchor heights, top-anchor offset, stiffness, force auto-tune, HP3, HP5.

`python -m autocal.tools.imperfect_machine OUT --seed N` runs the default autocal on a randomized imperfect scene and scores it in the print frame.
- **HP3 spread.** On 10 random HP3 machines the error is 1.11 ± 0.35 mm, mostly the known common z shift.
- **Low heights on HP3.** Uneven low heights are an exact, invisible rigid tilt. Stiffness ±20% moves the anchors by ≤0.25 mm.
- **HP5.** HP5's four low anchors broke the shared-z fit (4–19 mm, flagged). Freeing the low z values fixed it (0.5–1.0 mm).
- **Force auto-tune.** A knife-edge in force auto-tune failed 1/10 machines and was fixed.

Simulation only.

[Report and evidence](output/program-2026-11-20-autocal-on-imperfect-machines-pr-121-follow-up-1-a-wrong-configu-4b12cdf0/session-2026-11-20-imperfect-machine-benchmark-and-baseline-spread-build-a-reproduc-8a1f6003/report.md).

## Ellipsoid autocal: sweeps still win; stretch scale decides accuracy

Status: complete, 2026-11-20. Topics: autocalibration, sphere meshes, ellipsoid fit, point fit, stretch, collector, disturbances, HP3, HP4, HP5.

Sphere-mesh collection was scored by max anchor error × simulated time, with and without offline disturbances, against the default flat sweeps. Six planners were tried.
- Sweeps with the point fit win once disturbances count.
- The earlier fixed schedule's 0.3 mm on HP3/HP4 is a knife-edge: held lengths changed by ≤0.12 mm give 1.1 mm.
- Cause: the held anchor's height is exactly degenerate with the free lines' stretch-correction scale (rms flat, ~8.5 mm per unit). Zero error needs ×0.9 on HP4 and ×1.2 on HP5; HP5 pulley-reduced sweeps also improve from 1.30 to 0.72 mm at ×1.1.

Two collector bugs were fixed:
- relative moves after motor saturation (the HP5 "slip");
- held lengths recorded from a post-sweep encoder read, which double-counted drift in every dataset by up to 0.05 mm, and 0.35 mm in some meshes.

Simulation only. Next: attribute free-line tensions or compliance by exact pose/force replay.

[Report and evidence](output/session-2026-11-20-ellipsoid-autocal/report.md).

## HP4 autocal D.z bias: effector roll and slack parallel lines

Status: complete, 2026-11-18. Topics: autocalibration, HP4, HP5 pulley-reduced, reeving, equal strain, effector rotation.

An exact replay of the benchmark collection recovered true effector poses. Chiral D reeving on one shared spool rolls the HP4 effector about 0.2°/m off-centre. One of each ABC parallel line pair goes slack, and the near-horizontal ABC geometry amplifies the taut-line length error into about 5 mm of D.z. This is not stretch. Doubling the per-strand measurement target on U=2 machines cut single-run error to 1.52 mm (HP4) and 1.34 mm (HP5 pulley-reduced). A home reference under tension remains open. Simulation only.

[Report and evidence](output/session-2026-11-18-hp4-autocal-dz-bias/report.md).

## Separately reviewed audit of saved consensus evidence

Status: complete and separately reviewed, 2026-10-26. Topics: CM-E1, consensus
membership, common unseen sweeps, evidence provenance, research programs.

One local worker audited four saved CM-E1 sources, and a fresh reviewer accepted
the scoped finding. Two per-dataset regressions defeat unconditional nonworsening
despite a small aggregate improvement. The changed selections lack a common
unseen validation sweep, so predictive superiority remains unresolved. Keep
current defaults and investigate compatible unseen sweeps before reopening that
proposal. The audit inspected historical simulation measurements and performed
structural checks; it ran no new numerical experiments and did not independently
reproduce those measurements. The worker, review, admission and result commit
completed through the research program.

[Reviewed report](programs/program-e31e5321be5940baacf677409b2579e0/workers/attempt-8cc3e522df934dafbc74a9639e34721e/report.md)
and [program record and fixed evidence](programs/program-e31e5321be5940baacf677409b2579e0/report.md),
committed in `9f8ff306`.

## Configurable continuing HP3/HP4 worlds through MCP

Status: complete, verified 2026-10-12. Topics: MCP, HP3, HP4, configuration,
layering, reset, firmware, encoder origin, source reload.

Fresh MCP configuration now validates and freezes stock HP3/HP4 settings, bakes
layering-off geometry before construction and restarts firmware/collector
references coherently. A real six-point HP3 collection with 30 mm radii and zero
buildup showed zero baked cable half-width and about 183° encoder travel on each
moving axis. Lifecycle and production-construction checks passed. An initial
zero-step failure is preserved; no calibration accuracy or hardware result was
measured. Custom anchor layouts and arbitrary layered designs remain unsupported.

[Verification, API limits and full evidence](configurable-worlds.md).

## Default-force HP3 settling and no-layering geometry correction

Status: complete, verified 2026-10-10. Topics: autocalibration, HP3, settling, no layering, scene baking, MCP, reset settings.

The reported pullout failure was reproduced exactly through 2,457 encoder readings. Bounding endpoint search, then holding the drive and requiring ordinary measurement settling, recovered collection. Review of commit 383c6d4 found that unset layering enabled layered physics while job status reported it disabled; explicit flags, matching zero-width baking and reset settings were corrected. A production default-force MCP job then accepted automatically after seven iterations / 360 points, with 2.604 mm applied-anchor RMSE. Independent inspection verified every measurement, 300,551 physical samples, the physical stop and a finalized 1.58 GB RRD. Checks and two unchanged HP3 replay regressions passed. This is one full simulated default-force trial, not evidence of fewer points or hardware transfer; candidate ranking remains unchanged.

- [Settling fix, commit review and commands](autocal-settling-review.md).
- [Updated session abstract and tested hypotheses](../research/output/session-2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/abstract.md).
- [Acceptance and geometric accuracy](../research/output/session-2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/E12-acceptance.json).

## Supervisor-owned full-autocal through restricted MCP clients

Status: archived tooling validation; that trial's calibration quality was concerning. See the correction above.

Topics: autocalibration, HP3, HP4, MCP, sandbox, supervisor, cancellation, process groups, Rerun.

A socket-denied SDK MCP client started an isolated HP3 production CLI job in 3 ms. With known 30 mm radii, zero buildup, layering requested disabled and explicit manual forces, it collected eight sweeps / 320 physical measurements and automatically accepted at iteration six through the default patience rule. The finalized RRD contains 209,908 physical samples and exact dataset/log/stage artifacts; the attached HP4 world stayed at step zero. Physical cancellation, HTTP/SIGTERM supervisor exit, owner SIGKILL and physics-budget exhaustion retained readable partial recordings. The preceding default-autotuning trial failed pullout settling and is preserved. The selected score was concerning (6018), so this establishes workflow access and evidence integrity, not calibration accuracy. The follow-up above found that physical layering was actually enabled. Direct Codex chat attachment was unavailable; restricted SDK stdio MCP was exercised.

- [Verification and exact commands](autocal-jobs-verification.md).
- [Session abstract and tested hypotheses](../research/output/session-2026-10-10-supervised-full-autocal/abstract.md).
- [Detailed report and retained artifacts](../research/output/session-2026-10-10-supervised-full-autocal/report.md).

## Faster collection with guarded encoder settling

Status: complete. Topics: autocalibration, encoder-only, HP4, settling, collection time, point thinning.

Fresh replays reproduce 20 ranking/physical-error disagreements. Halving points worsens anchors on three of four fixtures. A small collector change checks the recent quiet window with a tighter range limit, retaining the longer vibration fallback. Matched 36-point production-JS/RRF collection takes 320.364 versus 429.668 simulation seconds (25.4% less); all eleven recorded calibration regressions and stopping decisions remain unchanged. New-data anchor accuracy and hardware transfer remain unverified.

[Report, measurements and limitations](../research/output/session-2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/report.md).

## Autocal history ranking with held-out sweeps

Status: complete.

Topics: autocalibration, HP4, HP3, Slideprinter, candidate ranking, held-out sweeps, D-optimal selection, outliers, poor starts.

This investigation reproduced disagreement between calibration ranking and known physical geometry, then tested synthetic sweep selection and whole-sweep prediction. On seventh HP3, the selected fit had 100.737 mm more summed anchor error than another candidate. A synthetic held-out scorer separated exact and perturbed geometry by 40.55x in robust prediction cost, but poor starts still produced physically bad fits. The archived implementation added future-sweep history selection with the existing rank as fallback. Its static regression replays did not measure an accuracy gain from that gate. Native HP4 collection was cancelled before a complete sweep after tracking losses. The report supports ranking diagnostics and a synthetic discriminator; it does not establish reliable calibration accuracy.

[Abstract and tested hypotheses](../research/output/session-autocal-held-out-history-ranking-7ec4260f/abstract.md).

- [Archived implementation, measurements and limits](../research/output/session-7ec4260f7410499a96f6b6bfe9c3de59/report.md)
- [Hypotheses, E0–E3 decisions and retained collection path](../research/output/session-7ec4260f7410499a96f6b6bfe9c3de59/research.md)

## Autocal ranking, bounded updates and identifiability

Status: complete.

Topics: autocalibration, HP3, Slideprinter, history age bonus, held-out prediction, bounded updates, D-optimal movements, radius uncertainty, negative results.

Seven hypotheses were investigated using recorded histories, controlled replay, synthetic bounded updates and native radius sensitivity. Ranking disagreed with physical accuracy in 22 adjacent transitions across seven of eleven datasets. Whole-sweep prediction selected worse physical results on five datasets. Removing the iteration-age bonus improved one case but increased aggregate selected parameter error by 200.701 mm over the full matrix. Bounded residual updates and informative movement selection worked in some synthetic conditions and failed under others. No tested solver or selection patch met the acceptance criteria. The retained change added ranking/ground-truth diagnostics. This session has a detailed report and experiment registry despite lacking a research.md notebook; online collection accuracy remained unvalidated.

[Abstract and tested hypotheses](../research/output/session-autocal-ranking-bounded-updates-negative-results-b7deb1bc/abstract.md).

- [Seven hypotheses, accuracy criteria and negative findings](../research/output/session-b7deb1bcc9cb418388273e2b7b9adc8c/report.md)
- [Experiment inputs and log paths](../research/output/session-b7deb1bcc9cb418388273e2b7b9adc8c/experiment_registry.json)
- [Frozen source, dataset and evidence hashes](../research/output/session-b7deb1bcc9cb418388273e2b7b9adc8c/artifact_hashes.json)

## Autocal candidate-model validation and collection throughput

Status: complete.

Topics: autocalibration, HP3, HP4, candidate-owned spool models, winding mismatch, elasticity, bounded updates, D-optimal movement, sentinel rejection, headless JS, Python parity, Rerun throughput.

This investigation tested candidate-specific spool validation, local bounded adjustments and informative movements on recorded HP3 and synthetic data. A guarded validation veto and tie-breaker rejected impossible candidates in 15/15 deliberate counterexamples, while all eleven recorded final selections and replay counts remained unchanged. Bounded updates recovered clean synthetic geometry but failed under winding mismatch and poor recorded starts; no HP3 accuracy gain was shown. A real HP4 collector run completed twelve observations in 1693.12 seconds without establishing anchor accuracy. A tooling follow-up measured a 19.03x improvement for matched short recorded HP4 workloads using headless JS. Frozen firmware replay retained encoder parity within 0.002242 degrees. Fresh end-to-end collection throughput and hardware accuracy were not measured.

[Abstract and tested hypotheses](../research/output/session-autocal-candidate-model-validation-and-throughput-fdb616a4/abstract.md).

- [Guarded validation and performance follow-up with limitations](../research/output/session-fdb616a454af4a6d9c1db074fbdc207d/report.md)
- [H1–H4, E01–E07 and P01–P04 budgets and decisions](../research/output/session-fdb616a454af4a6d9c1db074fbdc207d/research.md)
- [Separate physical and prediction errors](../research/output/session-fdb616a454af4a6d9c1db074fbdc207d/prediction-metrics.json)
- [Benchmark method, timings and reproducible replay commands](../research/collection-performance.md)

## Autocal history-ranking baseline, unfinished record

Status: incomplete record; final research outcome not documented.

Topics: autocalibration, Slideprinter, history ranking, ground-truth regret, proposed held-out validation, HP4 collection.

The notebook records a baseline audit of historical Slideprinter histories. Repeated ten_points_bigger_deltas runs selected physically worse candidates with 10.96–14.90 mm parameter regret. It proposes whole-sweep prediction and bounded residual updates, and records a native HP4 collection job plus a running regression baseline. The last saved decision still awaits those results. There is no final report or documented completion of the proposed selector experiments. Although exit.json records a successful process exit, that does not resolve the unfinished research record. Treat this as useful baseline evidence and a record of intended experiments, rather than a completed assessment of a new algorithm.

[Abstract and tested hypotheses](../research/output/session-autocal-history-ranking-baseline-incomplete-a71aec03/abstract.md).

- [Saved hypotheses, baseline measurements and outstanding decisions](../research/output/session-a71aec03021f470796a789ba7382e28a/research.md)

## Why the fifth HP3 reference trajectory differs today

Status: complete.

Topics: autocalibration, HP3, historical reference, numerical environment, sentinel fits, regression metrics.

Isolated March and current source replays used the same present-day numerical environment. They produced identical intermediate fits, both differing from the historical trajectory by 390.788 mm in the diagnostic mean. March code still returned the saved 70.314 mm calibration; current history selection returned 70.787 mm. The large trajectory penalty is dominated by underconstrained attempts that are excluded from final selection. Subsequent tracked algorithm changes are therefore unnecessary to reproduce the trajectory difference. Numerical environment or execution differences remain a leading explanation, but the original runtime lacked enough provenance to establish the cause. Source rollback alone did not restore the historical trajectory.

[Abstract and tested hypotheses](../research/output/session-fifth-hp3-reference-audit/abstract.md).

- [Metric decomposition, source history and replay controls](../research/output/session-fifth-hp3-reference-audit/report.md)
- [Controlled comparison measurements](../research/output/session-fifth-hp3-reference-audit/results.json)
- [Preserved historical settings](../research/output/session-fifth-hp3-reference-audit/reference-settings.json)

## Future-sweep selection versus refreshed autocal references

Status: complete.

Topics: autocalibration, HP3, Slideprinter, whole-sweep prediction, history selection, regression references, concurrency.

Eleven new main-branch reference logs were generated with source and environment provenance, then reproduced exactly in a bounded two-worker regression run. Against that baseline, research_findings_1 changed final history selection while preserving every intermediate fit. Summed returned parameter error fell from 885.158 to 816.936 mm, a 7.71% reduction. Three datasets improved, five worsened and three were unchanged. This is a net gain on this battery with explicit regressions, not a uniform accuracy improvement. Candidates were evaluated on different future-sweep sets. Earlier full-concurrency runs suffered native crashes; the successful bounded runs do not establish a root-cause fix or guaranteed crash prevention.

[Abstract and tested hypotheses](../research/output/session-reference-refresh-october-7-2026/abstract.md).

- [Per-dataset errors, selections and validation-set limitations](../research/output/session-reference-refresh-october-7-2026/report.md)
- [Machine-readable branch comparison](../research/output/session-reference-refresh-october-7-2026/comparison.json)
- [Source, runtime and scheduling controls](../research/output/session-reference-refresh-october-7-2026/provenance.json)

## Autocal native crash isolation to the compiler path

Status: investigation incomplete; no verified production fix.

Topics: autocalibration, JAX, jaxlib, XLA, protobuf, SIGSEGV, SIGABRT, compiler lifetime, Valgrind.

Native crashes were reproduced in objective replay outside the sandbox and in compile-only runs without objective execution. A direct jaxlib replay of saved compiler modules reproduced two segfaults among eleven children without importing JAX or autocal. GDB observed a crash during protobuf object destruction inside the XLA compilation path, but did not locate the first invalid write or free. Small reduced controls passed; they are finite controls rather than fixes. Valgrind timed out with uninitialized-value reports, and the sanitizer build failed to produce a usable wheel. No production fix was established. These observations narrow the necessary trigger set toward native compilation and object lifetime.

[Abstract and tested hypotheses](../research/output/session-autocal-crash-fix-20261007/abstract.md).

- [Isolation decisions and budgets](../research/output/session-autocal-crash-fix-20261007/research.md)
- [Tracked findings and compiler reduction results](../research/output/session-autocal-native-crashes-october-7-2026/autocal-crash-compiler-update.md)

## HP4 Print Squares substeps and PR 77

Status: partial physics correction validated, 2026-10-21. Topics: HP4, Print Squares,
missed steps, open-loop torque balance, rotor damping, physics substeps.

Removing cable-only electromagnetic stiffness corrects artificial holding power
and passes the default-one-substep stability gate. The PR's damping correction
fails loaded-state settling and is deferred. Matched native Squares prefixes
still pull B out at finer steps; coarse zero-loss behavior is unreliable evidence
of accuracy. [Review, measurements and integration decision](hp4-squares-substeps.md).

## Experimental HP4 pinhole-gate topology

Status: removed, 2026-11-12; was implemented with structural and short-run
checks passing. Topics: HP4, machine model, pinhole gates, CableJoints, Native
Rust, Wasm, JavaScript.

The experimental HP4 scene replaced the ABC effector's long
pinhole-wheel-wheel-pinhole chain with six stateful two-port gates, reducing the
topology from 51 to 33 cable joints. Native Rust, Wasm and JavaScript implemented
the same gate-flow law; the independent Python backend rejected the scene. Gate
parameters and visual bottom-wheel scaling remained provisional synthetic values,
not hardware calibration. The machine broke during the CableJoints overhaul in
`a92b6c45`, and the overhauled solver was fast enough that condensed cable gates
were no longer needed, so the feature was removed rather than repaired.

[Mechanical model, checks, evidence and removal note](pinhole-gate.md).

## Autocal flag audit and experiment handoff

Status: static investigation complete, 2026-10-22; numerical experiments not run.
Topics: autocalibration, CLI flags, robust filtering, scale polish, solver budgets,
parameter interactions, ranking, experiment design.

[Assessment and bounded experiments](output/session-autocal-flag-audit/report.md),
[92-option inventory](output/session-autocal-flag-audit/inventory.md),
[abstract](output/session-autocal-flag-audit/abstract.md), and
[notebook](output/session-autocal-flag-audit/research.md). Preserves the user's KEEP annotations;
no numerical default was established as optimal and no runtime flags were removed.

The [six-lane launch index](output/session-autocal-flag-audit/handoffs/README.md) provides separate
parallel handoffs with ownership, budgets, output isolation and integration rules.

## HP3 nominal truth and lane 01 solver integration

Status: complete, 2026-10-22. Topics: effective anchors, simulation truth, solver
tracing, iteration budgets, TRF fallback. [Integration report](autocal-solver-lane01-integration.md).
Historical/current HP3 geometry agrees with the existing nominal truth; poor fit
agreement with an older fitted reference does not establish a scene mismatch.
Compact solver diagnostics and an invalid-primary retry fix preserve default
numerics on five frozen fits and three full replay regressions.

## Autocal Lane 02 integration: unused fit controls

Status: complete, 2026-10-22. Topics: autocalibration, filtering, noise normalization,
CLI cleanup, old stage artifacts, reference provenance and consensus.

[Integration report](autocal-filtering-lane02-integration.md): removes unused
anchor-fit threshold and misleading sigma-used override; retains numerical
defaults and old artifact replay. Four bounded fits and three full fixture
regressions preserve default results; 267 targeted tests pass. The alleged layered
fallback regression compared an individual fit with a selected final consensus.
[Session abstract](../research/output/session-2026-10-22-autocal-filtering-integration/abstract.md)
and [notebook](../research/output/session-2026-10-22-autocal-filtering-integration/research.md)
retain raw evidence and limitations.

## Spool lane 03 integration

[Integration report](autocal-spool-lane03-integration.md): removes scale-fix and
inner-spool-budget controls while preserving the previous default. Twelve fits
on three additional simulated prefixes support fixed per-pass scale polish and
a cap of 30. Full validation details and limitations are linked in the report.

## Autocal lane 04 cleanup integration

[Integration report](autocal-ranking-lane04-integration.md): removed the unused
histogram flag, extracted final history selection for shared offline replay,
and verified identical results on three saved histories. Noise metrics were
present in all 14 saved candidates. Numerical defaults remain unchanged.

## Lane 05 physical-model integration

Status: complete, 2026-10-22. Topics: autocalibration, flex removal, zero-tension
projection, invalid geometry, CLI cleanup. [Integration report](output/session-autocal-physical-model-cleanup/report.md),
[abstract](output/session-autocal-physical-model-cleanup/abstract.md), [notebook](output/session-autocal-physical-model-cleanup/research.md).
Removed inert ellipse flex settings and fixed silent projection fallback. Seventy-two
fit-free comparisons match exactly, and two complete replays match references.
Saved sentinel geometries remain invalid without sweep rejection; trajectory-level
solver diagnosis is still needed. No hardware or new collection evidence.

## Lane 06 integration: planning and stopping cleanup

Status: complete, 2026-10-22. Topics: autocalibration, CLI flags, candidate ranges,
duplicate exclusion, spread stopping and held-out final selection.

[Implementation and validation](autocal-planning-lane06-integration.md): seven
CLI controls removed, range/exclusion bugs fixed, selected final model identified.
Six paired replays did not justify changing stopping defaults; 533 tests and four
selected calibration regressions passed. [Session evidence](../research/output/session-2026-10-22-autocal-lane06-integration/report.md).

## HP3 stretch bias and measurement design (2026-11-14)

[Canonical report](output/autocal_research_with_Fable5p2/report.md), [abstract](output/autocal_research_with_Fable5p2/abstract.md), and [topic synthesis](topics/autocal-stretch.md). Original simulation diagnosis and exploratory direct-collector design; production follow-up is linked below, with no hardware validation. Excellent fit scores can hide millimetre anchor bias. [Recovery guide](output/autocal_research_with_Fable5p2/recovery.md) locates the unfinished implementation snapshot, additional small evidence and Git-ignored local copies of raw recordings; no off-host backup is established.

## Autocal production hardening and robustness (2026-11-15)

[Report](output/program-2026-11-15-review-and-harden-stretch-insensitive-autocal-robust-bootstrap-h-3fa9855a/report.md), [abstract](output/program-2026-11-15-review-and-harden-stretch-insensitive-autocal-robust-bootstrap-h-3fa9855a/abstract.md), and [topic synthesis](topics/autocal-stretch.md). Implements checked rough bootstrap, default offsets, consistent measurement models, independent recorded-data fitting and full HP3/HP4 sanity tools. [Progressive workspace collection](output/program-2026-11-15-review-and-harden-stretch-insensitive-autocal-robust-bootstrap-h-3fa9855a/validation/workspace-expansion/report.md) removes the permanent ~390 mm budget, preserves initial half-turn reserve and verifies later coupling/recovery. One full nominal HP3 trial passes the coordinate gate at 0.91 mm; nominal HP4 remains 4.65 mm. Independent recorded-force review identifies a strong motor-cogging discrepancy, without proving its causal share of anchor error. Includes negative results, a guarded but not ordinarily operational home correction, four-high/cube pilots and a [controlled next-step plan](output/program-2026-11-15-review-and-harden-stretch-insensitive-autocal-robust-bootstrap-h-3fa9855a/next_steps.md). No robust sub-millimetre or hardware guarantee is established. Large recordings remain local only.

## HP5 spare-line sensing (2026-11-17)

[Reviewed findings and production validation](output/program-2026-11-17-compare-single-sensor-versus-all-non-fixed-non-driving-sensor-po-796a850f/report.md)
implement measurement-only spare-line sensing with local coupling checks and
force-aware range fitting. Both simulated continuations apply near-1 mm anchors;
matched final-data comparisons improve direct HP5 and worsen reduced HP5. See
the report's negative result and limitations before extending the policy.

## Autocal on imperfect machines (2026-11-21/23)

[Program agenda](output/program-2026-11-20-autocal-on-imperfect-machines-pr-121-follow-up-1-a-wrong-configu-4b12cdf0/agenda.md).
The paired studies cover uneven anchor heights, wrong line stiffness and elevated
motor cogging. See the [HP5 stopping and load-cap results](output/program-2026-11-20-autocal-on-imperfect-machines-pr-121-follow-up-1-a-wrong-configu-4b12cdf0/validation/high-cogging/report.md),
the [guarded force fit](output/program-2026-11-20-autocal-on-imperfect-machines-pr-121-follow-up-1-a-wrong-configu-4b12cdf0/session-2026-11-21-production-force-realisation-fitting-reuse-blind-period-scan-and-b5e5ea67/report.md),
and the [HP5r rotation replay and collection comparison](output/program-2026-11-20-autocal-on-imperfect-machines-pr-121-follow-up-1-a-wrong-configu-4b12cdf0/session-2026-11-21-isolate-the-remaining-hp5r-top-anchor-height-bias-with-controlle-76b38f06/report.md).
