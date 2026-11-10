# Faster HP3 encoder autocalibration by reusing settled measurements

Status: complete.

## Abstract

We profiled a supplied HP3 encoder-autocalibration recording, audited candidate anchors against simulated truth, tested sparse recorded datasets, and compared matched physical collectors with known radii and no line buildup. Settling dominated collection time. Raw score disagreed with geometry, and blind thinning often failed. Reusing a completed quiet check reduced 120-point collection from 2,733.214 to 2,571.350 simulated seconds (5.92%), while pooled anchor RMSE stayed 2.11845 mm. A follow-up reproduced the default-force pullout failure exactly through 2,457 encoder readings. Bounding endpoint search, then holding and strictly settling before measurement, recovered collection without relaxing guards. Review of supervised-job commit 383c6d4 exposed unset layering misreported as disabled, plus scene baking and reset settings; both were corrected. A full default-force HP3 job through a socket-denied MCP client then accepted automatically after seven iterations, with 360 points and 2.604 mm anchor RMSE. Its finalized 1.58 GB recording independently contains every measurement and the physical stop boundary. This is simulation evidence from one full default-force trial. Hardware transfer, reliable point-count reduction and score-based geometry selection remain unresolved; the full job is not a paired speed benchmark.

## Implemented result

The collector now passes the just-completed stable encoder reading into point collection after a drive move if the measurement preload is still active. A drive move does not change the sensor force. Previously, the collector marked the preload inactive, reapplied identical modes and waited for a second quiet window before recording the point.

The first point of a direction, force changes and zero-tension projection retain their settling behavior. Fixed-angle validation, all noise samples, direction recollection, point count, sweep planning, solver, history selection, patience and CLI remain in place. The user gets the same one-click workflow and automatic acceptance logic, with no new setting. This is a modest collection optimization; score-versus-geometry disagreement remains unresolved.

The runtime change is approximately 17 lines across [sweep collection](../../../../autocal/control/behaviors/sweep_data_collection.mjs) and [point collection](../../../../autocal/control/primitives/uncalibrated_actions.mjs). Three [controller boundary tests](../../../../autocal/control/tests/primitives/uncalibrated_actions.test.mjs) and a [README explanation](../../../../autocal/README.md) accompany it. [final-source.diff](final-source.diff) freezes the reviewed change.

![Timing diagnosis, reference geometry and matched collection comparison](results.png)

## Hypotheses and outcomes

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Settling is a larger collection cost than encoder averaging or motion waits in the supplied run | E01 | Supported | [Timing and geometry audit](E01-profile-and-geometry.json) |
| Lower raw fit score reliably identifies more accurate anchors | E01, E02, E07–E08 | Rejected | [Original candidates](E01-profile-and-geometry.json), [thinning fits](E02-thinning.json), [matched fits](E08-paired-comparison.json) |
| Uniformly keeping five or six points per direction, or one direction, is a safe default | E02 | Rejected | [Sixteen recorded-data trials](E02-thinning.json) |
| Reusing the quiet check after an unchanged-force move saves time without materially degrading geometry | E03, E05–E08 | Supported in simulation | [Duplicate readings](E03-duplicate-checks.json), [synthetic guards](E05-synthetic-controller-tests.txt), [matched comparison](E08-paired-comparison.json) |
| A single three-sweep span set preserves pooled-fit geometry | E07–E08 | Rejected for these datasets | [Independent and pooled fits](E08-paired-comparison.json) |
| Shorter pullouts can preserve observability | E03 diagnostic only | Not tested | [Travel trace](E03-long-pullouts.json); no altered collection |
| Pointwise fixed-encoder geometry, pose elimination or information-per-time acquisition improves calibration | none | Not tested | Proposed below; no new accuracy result |

## Encoder-only ideas, then prior evidence

With known radii, encoders provide metric length changes but do not directly observe tool position. Constant encoder offsets, anchor geometry and latent positions must satisfy all cable constraints together. Coordinate gauge freedoms must be fixed. HP3 has useful redundancy because the fourth cable and fixed-length constraints restrict the possible positions. Quiet encoders establish repeatability, not an absence of systematic bias.

The initial ideas were formed from first principles and primary literature before searching old reports and reading the implementation:

1. Fit unknown point positions and anchor geometry jointly, or eliminate the point positions algebraically. Use redundant length closure rather than an ellipse-only approximation.
2. Preserve each point's measured fixed-motor lengths. Treat a measured fixed-encoder deviation as changed geometry; separately propagate uncertainty instead of absorbing every discrepancy into free nuisance corrections.
3. Retain complete informative sweep directions and broad spans, choosing samples by information about weak parameter directions. Rank acquisition by information gained per collection time.
4. Compare frozen candidates on whole held-out sweeps with a common evaluation policy; keep uncertainty and model bias visible. Geometry accuracy requires simulated truth or external hardware validation.
5. Profile the actual collection and reuse already finished settling/noise work where force and mode are unchanged.
6. Investigate continuous trajectories with a dynamics-aware measurement model, rather than forcing every observation to be static. This would require a different collector and independent validation.

Alexandre dit Sandretto, Daney and Gouttefarde (2012) support joint calibration from redundant cable lengths with pose/gauge constraints; their eight-cable robot is not HP3. Maye et al. (2016) motivate information selection and handling unobservable parameters. Wang et al. (2017) motivate choosing informative calibration configurations. These are conceptual sources, not evidence that any proposed HP3 change works.

Prior sessions narrowed the experiments. The [October 8 guarded-settling report](../2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/report.md) found throughput gains but thinning failures. The [spool-cache and validation report](../2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-130caf5a/report.md) found that apparently fair ranking and informative thinning were unreliable. The [covariance report](../2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-76802a07/report.md) found failed covariance heuristics and cheap planning. The [earlier no-buildup reference investigation](../2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-1687b8ed/report.md) supplied RRD profiling precedent. The [bounded-update/ranking negative-results report](../b7deb1bcc9cb418388273e2b7b9adc8c/report.md) records unsuccessful latent-fit and ranking variants. These findings discourage replacing the production selector with a heuristic based on one favorable run; they do not disprove every joint-fit or acquisition formulation.

## Current algorithm

The implementation and [optimization overview](../../../../autocal/optimization_algorithm_overview), [objective overview](../../../../autocal/objective_functions_overview) and [ellipse guide](../../../../autocal/README_elliptical_feature_calibration.md) were read after the initial idea/prior-report search.

The collector bootstraps three role configurations, tuning forces/travel when not overridden. It fixes two cables, drives one and measures the other in both directions, recording encoder means, noise and fixed-motor drift. Large drift can trigger direction recollection at lower forces. The geometrical fit uses the ellipse relationship generated by the fixed lengths. The current modeled-length path reduces fixed readings to a sweep median, so per-point fixed deviations are not modeled individually.

With radius search off, fitting goes directly through the anchor calibration; spool optimization is bypassed. Filtering and recovery help rescue underconstrained fits. Planning ranks additional sweeps by D-optimal information gain. History-adjusted fit ranking controls patience (default three non-improving iterations). Prediction-based final selection freezes candidates and scores later whole sweeps; candidates can have different held-out sweep sets. A latest candidate with no held-out data is ineligible when older candidates have finite prediction scores. The displayed fit quality is not anchor error.

## E01: reproduce the supplied reference

[profile_reference.py](profile_reference.py) filtered relevant RRD event streams using RrdReader and extracted them in 13.88 wall seconds. The original 1.5 GB recording spans 6,756.422 simulated seconds and contains 180 measurements in nine sweeps, with seven fitting iterations. The collection context confirms HP3, closed-loop motors and Line Layering disabled; a paused default HP4 context at the start was excluded.

The seven Python-reported collection durations sum to 1,076.452 wall seconds. The 530.122-second bootstrap includes tuning and the first three sweeps; it is not all force tuning.

Command-to-next-command intervals, grouped by originating call site, identify the dominant waits:

| Operation | Observed simulated seconds | Observed wall seconds |
| --- | ---: | ---: |
| Settling polls | 3,887.742 | 651.335 |
| Commanded movement waits | 1,299.520 | 213.795 |
| Force-trial observation polls | 666.540 | 109.800 |
| Encoder noise samples | 409.720 | 76.582 |

These intervals include transport/boundary time, use observed clocks that can be stale, and omit cross-session intervals. This is a recording-based collection profile, not CPU attribution. It nevertheless identifies settling as the first optimization target. Evidence: [events](E01-events.jsonl), [summary](E01-event-summary.json), [audit](E01-profile-and-geometry.json), [audit script](audit_reference.py).

Anchor RMSE here means the square root of the mean squared Euclidean error across the four anchors, against the nominal simulated point-anchor geometry in the HP3 scene. It is not hardware accuracy or an independently measured tool-position error.

| Iteration | Points | Raw score | Anchor RMSE, mm |
| --- | ---: | ---: | ---: |
| 1 | 60 | 0.18349 | 8.0011 |
| 2 | 80 | 0.22469 | 3.2480 |
| 3 | 100 | 0.22469 | 3.2478 |
| 4, returned | 120 | 0.19282 | 2.8255 |
| 5 | 140 | 0.36287 | 3.8808 |
| 6 | 160 | 0.20784 | 2.7324 |
| 7 | 180 | 0.23588 | 2.5278 |

The returned result has 0.29770 mm more anchor RMSE than iteration 7. Iteration 1 has the best raw score despite substantially worse geometry. Training noise-normalized residuals and candidate-specific prediction sets explain why neither ordering is an oracle for anchor accuracy; this audit does not isolate a single causal error in the selector.

## E02–E03: failed shortcuts and observed opportunities

Sixteen direct fits used four recorded prefixes and four data-retention variants: all points, six per direction, five per direction and only the first direction. Known radii and original saved fit settings were retained. These are direct fitting trials, not full-auto recovery runs.

Brief negative findings:

- At iteration 4, six points per direction worsened anchor RMSE from 2.8255 to 4.6367 mm.
- At iteration 7, six per direction was underconstrained; five per direction had 23.65 mm RMSE with the underconstrained sentinel.
- Iteration 7 with only one direction had a lower score (0.22265 versus 0.23588) but worse RMSE (10.5172 versus 2.5278 mm).
- The full iteration-2 direct fit itself returned the underconstrained sentinel and roughly 1.46 m error. Production recovery had obtained 3.2480 mm; removing its recovery context is not a fair production-quality comparison.
- Five-point and one-direction controls also produced severe failures in earlier prefixes.

Blind reduction is rejected as a default, not all possible informative sparse collection. [E02-thinning.json](E02-thinning.json) preserves every result and links to per-trial logs/datasets; [thin_reference.py](thin_reference.py) reproduces the trials.

E03 examined 18 pullouts taking about 75–100 simulated seconds. At 30 seconds, they had only reached about 59–69% of their eventual encoder travel. A 30-second cap was not tested: a trace prediction is not a failed altered-calibration result. See [pullout trace](E03-long-pullouts.json).

All 162 points after the first point in each direction had the same earlier settled angles as their eventual recorded raw angles at the reference's 0.01-degree resolution. Their repeated settling windows total 305.580 simulated seconds, or a hypothetical 4.52% of the original recording duration. This is a retrospective savings estimate, not a measured modified full run. See [duplicates](E03-duplicate-checks.json) and [saving estimate](E03-reference-saving-estimate.json).

## E04–E08: matched physical simulation

The normal headless HTTP run was blocked because this execution environment cannot bind AF_INET sockets. An initial malformed --stage-artifacts invocation and the corrected socket failure are preserved in [E04-baseline.stdout.log](E04-baseline.stdout.log), [E04-baseline.run.log](E04-baseline.run.log) and [failed manifest](E04-baseline/sweeps.headless/manifest.json). Neither is a calibration accuracy result. The attached native world was HP4 and remained idle.

A session-local [batch harness](collect_hp3_batch.mjs) therefore uses the production HP3 physics, production collector and production encoder endpoint. Actual RRF file-mode CAN captures generate the relative H2 movement profiles. A checked Y188.496 mm / F1400 move exactly matched the reference's step count and acceleration/steady/deceleration timing. The harness is limited: firmware restarts for each relative motion, encoder requests go directly to the production endpoint, and known R30/unit gearing converts requested force to torque. It does not establish persistent HTTP firmware behavior.

Failed trials were preserved. The first batch attempt had a context error before motion. The first completed baseline/candidate pair disabled layering flags only after constructing the world, leaving nonzero cable half-width baked into geometry. Nominal geometry residuals and metre-scale anchor errors exposed this setup error. [Invalid baseline](E04-batch-baseline-2/manifest.json) and [invalid candidate](E05-batch-reuse/manifest.json) are excluded from the valid no-buildup comparison.

For E07/E08, the harness explicitly baked cablePathHalfWidthOverride=0 before world construction, then disabled all layering flags. Both manifests record identical baked scene hashes. The frozen baseline modules come from revision 359a55fcd1220d8165af2d84ad11b7e32bd33a46 with imports adjusted only to resolve locally. Candidate source remains in the workspace. The E07 manifest's working-source hashes describe that workspace; its collectorModule points to the actual frozen baseline, whose hashes are in the session manifest.

Two span sets use all three role configurations and both directions, ten points per direction, sixteen noise samples per point, the same reused reference forces, return to origin, and known R30. The first span set is 516.922/0 mm; the second is 387.692/-100 mm. Each fresh collector world produces 120 points at dt=0.002 s. Force autotuning is excluded from this matched test.

| Measurement | Frozen baseline E07 | Reuse E08 |
| --- | ---: | ---: |
| Simulation seconds | 2,733.214 | 2,571.350 |
| Points / movement commands | 120 / 273 | 120 / 273 |
| Encoder requests | 6,949 | 6,519 |
| Torque-mode commands | 242 | 134 |
| Noise samples per point | 16 | 16 |
| Maximum recollection attempt | 0 | 0 |
| First 60-point anchor RMSE, mm | 8.053646 | 8.053773 |
| Second 60-point anchor RMSE, mm | 16.036100 | 16.036100 |
| Pooled 120-point anchor RMSE, mm | 2.118451 | 2.118447 |
| Pooled maximum anchor error, mm | 2.518345 | 2.518326 |

Time falls by 161.864 simulated seconds (5.9221%). Maximum RMSE deterioration across the three fits is 0.000127 mm, well within the predeclared 0.5 mm criterion. Every point produces a valid nominal-geometry residual. Fixed drift and noise maxima are equal; noise mean lengths agree within 5.7e-14 mm. Raw single encoder readings can differ by one quantization step, so the mean/sampling checks matter. This supports keeping the change without claiming faster collection from fewer points.

First-set fits predict the held-out second set with scores 7.98801/7.98819. Second-set fits predict the held-out first set with identical score 17.94128. Pooled second-set scores are in-sample diagnostics, explicitly labeled accordingly. The second set alone has a better training cost than the first or pooled fit, yet worse anchor accuracy: a fresh reproduction of score disagreement.

Wall time was 121.056 versus 114.347 seconds; this is secondary because numerical checks ran in the background. RRF motion planning itself totaled only about one wall second per run. Do not compare the 120-point matched collection directly with the 180-point, autotuned full supplied reference. Pooled 2.11845 mm error is not a controlled accuracy improvement over its returned 2.82549 mm because data and selection differ.

Evidence: [baseline manifest](E07-no-layering-baseline/manifest.json), [candidate manifest](E08-no-layering-reuse/manifest.json), [comparison](E08-paired-comparison.json), baseline [first dataset](E07-no-layering-baseline/sweeps-0.json) and [second dataset](E07-no-layering-baseline/sweeps-1.json), candidate [first dataset](E08-no-layering-reuse/sweeps-0.json) and [second dataset](E08-no-layering-reuse/sweeps-1.json), [fit driver](fit_batch.py), [comparison driver](summarize_pair.py). Both variant directories retain merged datasets, detailed fit logs, scene source and motion captures.

## Verification and limits

The [synthetic controller checks](E05-synthetic-controller-tests.txt) confirm that reuse keeps sixteen samples, invokes the fixed-angle guard and cannot bypass zero-tension trials. The broader [collector suite](E06-collector-tests.txt) passed 73 tests across 17 suites; the HTTP socket-binding file was excluded after the environment restriction was established.

[Eleven full-auto recorded regressions](E06-regressions.txt) match their saved summaries and all iteration parameters exactly, including selected results and stopping behavior. Their successful tool checks are compatibility evidence, not new geometric accuracy. Both fresh datasets also complete offline full-auto replays ([E07](E07-replay.log), [E08](E08-replay.log)); these stop at --no-collect after stored sweeps are exhausted and do not exercise a fresh live patience stop.

The [Python suite](E06-python-tests.txt) had 350 passing tests, two deselected and five failures exclusively at forbidden socket binding in test_headless_sim. Those checks remain environment-blocked. git diff --check passed. No full normal HTTP run, new complete RRD collection or hardware validation is claimed.

## Next experiments, not tested

The best next model experiment is candidate-dependent per-point fixed-encoder geometry, with bounded synthetic perturbations, NumPy/JAX residual and gradient parity, and frozen whole-sweep predictions. This targets a documented lost measurement association rather than changing score weights arbitrarily.

For fewer points, compare broad role-balanced acquisition against the current collector at equal movement/force budgets, using information gain per predicted collection time and the weakest observable directions. Existing D-optimal planning alone does not prove safety of thinning. Keep automatic stopping but assess it against repeated simulated truths, biased/noisy data and missed-excitation cases before changing patience or selection.

A persistent-firmware HP3 run and hardware comparison should validate the settling reuse and any larger acquisition/model change when those environments are available.

## References and provenance

- [Session manifest](manifest.json): baseline revision 359a55fcd1220d8165af2d84ad11b7e32bd33a46, original reference revision 1e026534483e2175f4c101b645b1f54f65f9faba, reference/scene/RRF hashes, frozen baseline and final artifacts. [Source diff](final-source.diff) identifies the implementation relative to baseline; it is now committed as 536a2e3660fa25cd241fb0a14ae8e5c2dcdcc88e.
- [Supplied reference directory](../../../extended-autocal/no-buildup-reference-003/), its [RRD](../../../extended-autocal/no-buildup-reference-003/hangprinter-20261010T182010.757112Z-814ac4d9.rrd), [dataset](../../../extended-autocal/no-buildup-reference-003/sweeps.json), [full-auto log](../../../extended-autocal/no-buildup-reference-003/sweeps.full_auto_log.jsonl) and [fit artifacts](../../../extended-autocal/no-buildup-reference-003/sweeps.stages/): E01–E03 timing, candidate and thinning evidence.
- [Extended reference instructions](../../../../autocal/data/references/extended-reference.md): normal full recording workflow; blocked in this environment.
- Alexandre dit Sandretto, Julien; Daney, David; Gouttefarde, Marc (2012). “Calibration of a Fully-Constrained Parallel Cable-Driven Robot,” ROMANSY. [Inria author PDF](https://www-sop.inria.fr/coprin/PDF/a_romansy2012.pdf). Supports joint redundant-length geometry/pose calibration and gauge reasoning.
- Maye, Jérôme; Sommer, Hannes; Agamennoni, Gabriel; Siegwart, Roland; Furgale, Paul (2016). “Online self-calibration for robotic systems.” DOI [10.1177/0278364915596232](https://journals.sagepub.com/doi/10.1177/0278364915596232). Abstract supports information-based data retention and treatment of unobservable directions.
- Wang, Hongbo; Gao, Tianqi; Kinugawa, Jun; Kosuge, Kazuhiro (2017). “Finding Measurement Configurations for Accurate Robot Calibration: Validation with a Cable-Driven Robot,” IEEE TRO 33(5), 1156–1169. DOI 10.1109/TRO.2017.2707562; [institutional abstract](https://hub.hku.hk/handle/10722/302973). Supports observability-oriented configuration selection.
- The five preceding-session reports linked above supply negative-results context and profiling precedent; their results are not counted as experiments performed in this session.

## E09: sandbox diagnosis and tooling mitigation

The user asked why the HTTP workflow was blocked and how future agents can avoid this limitation. A fresh same-context localhost probe returned EPERM again. The Python test trace locates the failure inside the AF_INET socket constructor, before binding; the earlier wording about binding was broader than the actual failure point. The launcher explicitly chooses workspace-write and approval_policy="never", and the effective session disallows network access. There was no available approval escalation.

The existing MCP runtime remains connected, idle at step zero, with live RRF/collector/physics services. Those are launcher-owned outside the agent command sandbox. Thus the limitation is specific to services or clients launched through sandboxed commands, not a missing simulation capability in the host. Evidence: [diagnostic](E09-sandbox-and-tools.json), [Python traceback](E06-python-tests.txt), [launcher](../../../../scripts/research_agent.py), [standalone socket allocation](../../../../autocal/headless_sim.py).

Two additional integration gaps matter: this session was launched as HP4, reset retains that launch selection, and start_collection exposes sweep jobs rather than the existing full calibration/selection/stopping loop. Launching with --machine hp3 fixes the first gap for a new session but does not by itself provide full-auto execution.

Recommended tool work, **not implemented or tested here**:

1. Add supervisor-owned asynchronous full-autocal jobs running the existing CLI, with status/cancel operations, explicit HP3/HP4 and layering settings, fixed-radius options, frozen inputs, source hashes, logs, stage artifacts and finalized RRD. Preserve the algorithm's own automatic acceptance; do not recreate it as a sequence of agent prompts.
2. Expose configuration of fresh experiment worlds so a robot/design mismatch can be resolved without constructing a new collector adapter. Validate scene/RRF compatibility and bake no-layering geometry before construction.
3. Add a lightweight connectivity/recorder preflight and a scoped supervisor job for socket-dependent integration tests. Report effective networking and available job types before a long collection.

For operator-configured local CLI sessions, the documented sandbox_workspace_write.network_access=true option offers an immediate route while retaining the filesystem sandbox. It grants command networking generally, not just localhost; managed requirements and proxy policies can still constrain local sockets. This session's policy cannot be changed by the agent, and the override was **not validated** here. Verify socket creation, binding and a local HTTP round trip in the new session before collection. [research/README.md](../../../../research/README.md) now explains these distinctions and the launch command; [documentation diff](E09-tooling-docs.diff) preserves that update.

Official sources consulted: [OpenAI sandbox documentation](https://learn.chatgpt.com/docs/sandboxing) explains that spawned commands inherit sandbox restrictions and that approvals are a separate control; [configuration reference](https://learn.chatgpt.com/docs/config-file/config-reference) documents workspace networking and local/proxy policy settings. These support configuration guidance, not a claim that the proposed job API or override has been tested.

## E10: implementation prompts and networking default

The user requested efficient prompts for the three tooling improvements and a default networking setting. Created [full-autocal jobs](../../../../research/prompts/tooling_improvement_1.txt), [configurable fresh worlds](../../../../research/prompts/tooling_improvement_2.txt) and [preflight/test jobs](../../../../research/prompts/tooling_improvement_3.txt), roughly 320 words each. They have separate scopes, existing-code entry points, lifecycle boundaries and concrete acceptance checks; run 1 then 2 then 3 or integrate shared runtime edits explicitly. The proposed APIs are still not implemented or tested.

The launcher now passes sandbox_workspace_write.network_access=true for interactive and batch Codex sessions and includes it in desktop attachment.toml. Explicit forwarded false overrides remain last and take precedence. Networking stays useful for ad hoc local HTTP experiments and tests; the prompts require supervisor tools to work with restricted command networking too. This default applies to new sessions and does not change this running chat's managed policy.

Validation: .venv/bin/python -m pytest tests/python/test_research_agent.py -q -k launcher passed 12 tests (27 deselected). The checks exercise generated commands, default/false override ordering, interactive/batch configuration and generated desktop attachment settings. git diff --check passed. No new network-enabled live simulation was run, so this establishes configuration behavior only. Evidence: [launcher tests](E10-launcher-tests.txt), [tooling source diff](E10-tooling-source.diff), [manifest](manifest.json).

## E11–E12: default settling fix and supervised-job review

The [tracked review report](../../../../research/autocal-settling-review.md) explains the reproduction, corrections and validation. E11 isolated endpoint holding under the original flawed scene; [2,457 encoder readings and clocks matched exactly](E11-baseline-prefix-comparison.json) through the old failure boundary. [Collection recovered](E11-settling-result.json), but that scene yielded a 558.69 mm first-fit anchor RMSE and is not valid no-layering accuracy evidence.

E12 fixes the no-layering bake, preserves reset settings, and verifies the default-force production CLI through a seccomp socket-denied SDK client. [Automatic acceptance](E12-acceptance.json) followed seven iterations and 360 measurements, giving 2.604 mm applied-anchor RMSE. [Independent recording inspection](E12-rrd-verification.json) confirms all measurements, the physical stop, twelve stage artifacts, frozen hashes and exact final dataset/log content. [Candidate geometry](E12-fit-geometry.json) retains underconstrained attempts and shows that the accepted iteration 5 is slightly worse geometrically than the first recovered fit (2.602 versus 2.384 mm before firmware rounding). Ranking remains unchanged.

The complete job took 6,011.018 simulated seconds and 1,100.446 wall seconds. It used 360 points versus the supplied reference's 180, so this follow-up establishes default workflow recovery and correct scene construction, not fewer points or a matched speed improvement. Hardware transfer and direct launcher-attached Codex/desktop verification remain untested.

Evidence: [collector checks](E11-collector-tests.txt), [HTTP retry](E11-http-tests.network-enabled.txt), [HP3 regressions](E11-hp3-regressions.txt), [geometry checks](E12-geometry-tests.txt), [headless/job checks](E12-headless-and-job-tests.network-enabled.txt), [live lifecycle checks](E12-live-lifecycle-tests.network-enabled.txt), [CLI checks](E12-cli-tests.txt), [MCP retry](E12-mcp-schema-tests.network-enabled.txt), [restricted-client script](E12-supervised-verification.py), [recording inspector](E12-inspect-recording.py), and [final source diff](E12-source.diff). The [preceding implementer's report](../2026-10-10-supervised-full-autocal/report.md) preserves the default-force failure and manual-force acceptance against base 8cdfdac4; the reviewed implementation is revision 383c6d453a2027fb99ebc7936862053878661e57 plus the retained E11/E12 patch.
