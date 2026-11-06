# Research session catalog

One result sentence per session; follow the report for evidence.

- [Faster encoder autocalibration through exact spool-model dataset reuse](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-130caf5a/report.md) — No result summary available.

- [Faster repeated encoder autocalibration with persistent JAX compilation](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/report.md) — Browser fixture provenance clarified by the user: reference data came from browser physics collection; much configuration is already in source and JSON.

- [Improve encoder-based CDPR autocalibration in hp-sim5. The current algorithm can](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b4c1444b/research.md) — The session has started.

- [Faster encoder-only CDPR calibration collection with guarded settling windows](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/report.md) — This study asks whether Hangprinter calibration can collect encoder data faster while preserving its one-click workflow and automatic stopping.

- [Independently testable encoder calibration stages](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-df7f5fd7/report.md) — Encoder autocalibration was split into modeled-length transformation, fitting and assessment, frozen sweep planning, prediction evaluation and pure history ranking.

- [Improve encoder-based CDPR autocalibration in hp-sim5. The current algorithm can](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-f5e17805/research.md) — The session has started.

- [Critical review of headless autocal recording finalization](2026-10-09-headless-recording-review/report.md) — Review of commit 8ded7420 examined headless recording startup, clock labels, acknowledgement backpressure and shutdown.

- [Recovering autocal slips and retaining fixed encoder errors](2026-10-09-slip-recovery-and-encoder-model/report.md) — This investigation implements bounded reduced-force recovery after a fixed encoder slips and automatic closed-loop/layering/recording startup.

- [HP3 trial-002 return settling and recording performance](2026-10-09-trial002-return-settling/report.md) — Two stopped manual browser recordings show that sampled extended recording increased achieved playback from 0.570x to 5.980x while reducing file bytes per simulation second by 11.18x.

- [HP3 trial-003 workspace measurement exceeds the settling deadline](2026-10-09-trial003-size-measurement/report.md) — Trial-003 completed force tuning but failed during workspace-size measurement, before the first sweep.

- [Fixed-motor slipping corrupts upper autocal sweeps](2026-10-09-trial004-upper-sweeps/report.md) — Trial-004 contained two upper sweeps with visibly irregular paths.

- [Autocal return waits fixed by lossless recorder batching](2026-10-10-autocal-recorder-waits/report.md) — The recorded reference run showed a 16,162-command return burst delaying event logging by 21.88 seconds and blocking physics after about 0.64 simulation seconds.

- [HP3 encoder autocalibration: reference failure and runtime bottlenecks](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-1687b8ed/report.md) — This investigation asks why zero-buildup HP3 encoder autocalibration is slow and can return poor anchors, and whether fewer points are safe.

- [Faster HP3 encoder autocalibration by reusing settled measurements](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/report.md) — We profiled a supplied HP3 encoder-autocalibration recording, audited candidate anchors against simulated truth, tested sparse recorded datasets, and compared matched physical collectors with known radii and no line buildup.

- [Covariance metrics for encoder based CDPR autocalibration](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-76802a07/report.md) — This study asks whether covariance metrics from Caverly et al.'s online CDPR self-calibration paper can improve hp-sim5's encoder-based autocalibration ranking.

- [Supervisor-owned HP3 full-autocal through restricted MCP clients](sessions/2026-10-10-supervised-full-autocal/report.md) — This session validates launcher-owned asynchronous full-autocal jobs through MCP with client socket/connect syscalls denied by Linux seccomp.

- [Configurable HP3/HP4 research worlds through MCP](sessions/2026-10-12-configurable-research-worlds-fee56c1a/report.md) — MCP world configuration was tested against production scene construction and the existing RRF collector.

- [Stable encoder averaging and numerical fragility in HP3 autocalibration](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/report.md) — This investigation asked how to improve encoder-only HP3 autocalibration with known radii, no line layering and automatic stopping.

- [Network-enabled preflight revalidation and lifecycle fixes](sessions/2026-10-12-network-enabled-preflight-revalidation-31a423c7/report.md) — After command networking was enabled, socket creation, loopback binding and HTTP all passed in the actual command context in 0.013 seconds.

- [Preflight and supervised integration tests with command sockets denied](sessions/2026-10-12-preflight-and-supervised-integration-tests-718724fb/report.md) — Can agents check recording/connectivity and run socket-dependent integration tests without command networking?

- [Headless JavaScript physics profiling](sessions/2026-10-13-headless-js-profiling/report.md) — The production headless JavaScript HP3/HP4 physics was measured on fixed firmware logo trajectories and complete recorded autocal movement/settling schedules, with layering enabled and disabled.

- [Improve encoder-based CDPR autocalibration in hp-sim5. The current algorithm can](sessions/2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-76fad208/research.md) — The session has started.

- [Improve encoder-based CDPR autocalibration in hp-sim5. The current algorithm can](sessions/2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-881b7eb9/research.md) — The session has started.

- [Faster, more robust encoder-only autocalibration for a Hangprinter 3: a shorter encoder-quiet window plus a bracketed geometric consensus of fitted anchors](sessions/2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b65aa69c/report.md) — Encoder-only autocalibration infers anchor geometry from cable-length measurements, so its two user-facing costs are collection time and the risk of applying a fit that scattered.

- [PBD constraint hotspot optimization](sessions/2026-10-13-pbd-constraint-hotspot/report.md) — We tested whether settled-constraint skipping and per-update invariant caching can speed up the JavaScript 3D PBD solver without changing physics.

- [Cumulative validation of three headless physics optimizations](sessions/2026-10-13-three-optimizations/report.md) — This experiment checked the cumulative effect of three JavaScript simulation optimization commits on exact trajectory behavior and unprofiled physics timing.

- [Native Rust headless throughput versus Wasm and Python](sessions/2026-10-14-native-throughput/report.md) — Native Rust commit ab833759 was benchmarked against the exact preceding Rust/Wasm headless default at 7905e64f, and against the former Python default for fresh research trials.

- [Fuller Rust simulation engine for hp-sim-3d](sessions/2026-10-14-rust-engine/report.md) — The full Rust/Wasm engine is 2.37–2.86 times faster than the previous Wasm default on eight frozen HP3/HP4 logo and autocal workloads.

- [Autocal recording-mode cost on a frozen production trace](sessions/2026-10-15-autocal-recording-modes/report.md) — We measured the time and disk cost of four full-autocal recording modes using one frozen production HP3 command/encoder schedule.

- [History-seeded fit variants for robust encoder autocalibration (HP3, no buildup)](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-267d719f/report.md) — Encoder autocalibration repeatedly fits the growing sweep dataset, and a cold multi-start fit sometimes lands on the cost-cap sentinel or stalls in a bad local optimum.

- [Faster HP3 encoder-only autocalibration: a consensus spread gate and a compliance-aware fixed-anchor tolerance](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092/report.md) — Question: can HP3 encoder-only autocalibration collect the same data faster, and can it stop choosing worse anchors because its selection score disagrees with geometry?

- [Warm-starting the HP3 encoder autocalibration anchor solve (and a negative result on point thinning)](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/report.md) — Encoder-only HP3 autocalibration is slow, and its candidate score disagrees with true geometry (selection regret 5.47 mm on the supplied reference).

- [Trust-region anchor solving makes HP3 encoder autocalibration more accurate, cheaper to compute and less selection-sensitive](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9d784051/report.md) — Encoder-based CDPR autocalibration in hp-sim5 searches for anchor geometry with a multi-start local solver, and earlier sessions showed its selection score often disagrees with true geometric accuracy.

- [Consensus-spread early stopping: fewer HP3 autocalibration sweeps without losing geometry](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b75b3bdf/report.md) — hp-sim5's full-auto encoder autocalibration keeps collecting sweeps after the applied geometry has converged, because it stops on patience or an absolute cost/std threshold whose ranking scalar disagrees with geometrical accuracy.

- [Native PGO and columnar Rerun performance](sessions/2026-10-15-native-optimization/report.md) — This study compares native headless simulation with the fuller Rust/Wasm runtime, using the same Rust physics code, commands, fixed timesteps and output cadence.

- [HP4 effector rocking: separate the firmware tick from physics integration](sessions/2026-10-16-hp4-effector-instability/report.md) — The production pipeline used the 2 ms firmware tick as one mechanical integration step.

- [HP4 motor load bookkeeping: one contribution per physical constraint](sessions/2026-10-16-hp4-load-bookkeeping/report.md) — Recording motor cable load torque, stiffness and damping once per constraint, rather than per solver iteration, improves a matched one-step 30 s pull from56.8to84.8mm without extra integration work.

- [Restore HP4 single-step performance; autocal remains unresolved](sessions/2026-10-16-hp4-one-step-autocal/report.md) — Default physics integration is restored to one 2 ms step in JavaScript and Python.

- [HP4 sluggish torque response: implicit motor stiffness dominates a roller redistribution counterfactual](sessions/2026-10-16-hp4-roller-propagation/report.md) — A controlled 30 s HP4 pull at fixed torques tested slow local rest-length propagation against motor-load numerical damping.

- [HP4 substep options and bounded size-tuning travel](sessions/2026-10-16-hp4-substep-options/report.md) — Substeps are now selectable in the browser and through autocal's --physics-substeps setting for browser/headless collection, defaulting to one.

- [interactive research](sessions/2026-10-16-interactive-research-1c2eae68/research.md) — The session has started.

- [interactive research](sessions/2026-10-16-interactive-research-2ecf1b18/research.md) — The session has started.

- [interactive research](sessions/2026-10-19-interactive-research-050ce13b/research.md) — The session has started.

- [HP4 one-substep autocal wobble: XPBD damping feedback and acceptance status](sessions/2026-10-19-interactive-research-41c8bab5/report.md) — Loaded HP4 autocal wobbled with the default one 2 ms physics step, disturbing settling and force selection.

- [HP4 autocal failures: numerical slowing, missing travel budget and a repeatable four-substep workflow](sessions/2026-10-19-interactive-research-8d7a82a2/report.md) — We investigated recurring HP4 autocal failures using October 16–19 logs and Rerun recordings, native Rust force comparisons, an exact replay and bounded headless calibration trials.

- [Physical validation of HP4 local endpoint discovery and backoff](sessions/2026-10-19-interactive-research-bf5d807a/report.md) — Following a failed manual HP4 test, we inspected actual effector translation, rotation and cable slack in sampled production Rust Native recordings, rather than accepting point counts as evidence of sound motion.

- [HP4 autocal wobble: one XPBD damping displacement per step (per-visit re-evaluation is an energy source)](sessions/2026-10-19-interactive-research-f3c115bf/report.md) — HP4 autocal at the default `--physics-substeps 1` did not settle: the effector limit-cycled, settling checks timed out and force selection moved.

- [Stateful pinhole gates for the experimental HP4](sessions/2026-10-20-interactive-research-38da70f4/report.md) — This study replaced the ideal, massless ABC reeving abstraction in the experimental HP4 scene with six stateful two-port pinhole gates.

- [HP4 bearing spin, tension reporting and a two-port pinhole gate](sessions/2026-10-20-interactive-research-62300644/report.md) — Implemented an opt-in HP4 two-port pinhole_gate attachment with a condensed unilateral length solve, common tension and summed carrier force/torque Jacobians.

- [HP3 autocalibration at fresh centered and asymmetric encoder references](sessions/2026-10-20-interactive-research-ae0174ce/report.md) — Does moving the encoder reference near the anchors' centroid improve HP3 calibration or reduce collection effort?

- [interactive research](sessions/2026-10-20-interactive-research-edf2eb7d/research.md) — The session has started.

- [HP4 Print Squares substep bias and PR 77 review](sessions/2026-10-21-interactive-research-cdfb5278/report.md) — PR #77 proposes two corrections for HP4’s greater step loss at finer physics substeps.

- [Everyday autocal help and extracted recovery/stopping](sessions/2026-10-22-autocal-cli-orchestration/report.md) — Ordinary autocal help now shows routine settings and a short usage line; extended `--help-research` lists every control by topic.

- [Autocal Lane 02 cleanup and reference-comparison audit](sessions/2026-10-22-autocal-filtering-integration/report.md) — Lane 02 found an unused anchor-fit threshold and a sigma override that did not control per-point normalization.

- [Autocal planning, recovery and stopping flags: candidate grid, ridge, stop rules and sparse recovery](sessions/2026-10-22-autocal-flags-06-planning-stopping/report.md) — Question: which of the 16 planning, recovery, stopping and candidate-count options in `autocal.py` are used, and which could be removed, hard-coded, redesigned or retained?

- [Autocal lane 04 integration and history selection replay](sessions/2026-10-22-autocal-lane04-cleanup/report.md) — This integration removes an unused histogram CLI option and extracts final history selection into a function shared by full-auto and offline replay.

- [Lane 06 integration: planning CLI cleanup and stop attribution](sessions/2026-10-22-autocal-lane06-integration/report.md) — This investigation asks which lane 06 findings justify changes to autocal.

- [Autocal ellipse flex removal and zero-tension validation](sessions/2026-10-22-autocal-physical-model-cleanup/report.md) — Lane 05 found that ellipse flex correction never applied on the reference corpus because the collector writes no per-point tensions.

- [HP3 nominal truth verification and lane 01 solver integration](sessions/2026-10-22-autocal-solver-integration/report.md) — The lane 01 report inferred a seventh-fixture scene mismatch from poor fits and agreement with an older fitted reference.

- [Integrating spool scale policy and removing two autocal controls](sessions/2026-10-22-autocal-spool-simplification/report.md) — Lane 03 identified per-pass scale polish and the default inner spool cap as simplification candidates.

- [Faster failure handling for encoder-only CDPR autocalibration](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/report.md) — Can encoder-only CDPR autocalibration become faster without degrading geometry or changing one-click stopping?

- [Can cable-flex compensation improve HP3 autocalibration?](sessions/2026-10-22-interactive-research-49c8bfde/report.md) — I tested why the removed autocal `--flex` flag was inactive and whether a repaired flex correction can improve Hangprinter 3 calibration.

- [interactive research](sessions/2026-10-22-interactive-research-7345527c/research.md) — The session has started.

- [HP3 encoder autocal: where simulated collection time goes, and feed rate as a speed lever](sessions/2026-10-22-interactive-research-a430536f/report.md) — Can encoder-only autocal collect data faster without losing robustness?

- [interactive research](sessions/2026-10-22-interactive-research-d1d37c7f/research.md) — The session has started.

- [Autocal research allocation after mixed consensus evidence](sessions/2026-10-23-autocal-agenda/report.md) — What should receive the next research allocation after broader replay rejects blanket finite-prediction consensus membership?

- [Finite-prediction consensus: broader replay rejects a blanket restriction](sessions/2026-10-23-consensus-membership/report.md) — Should final calibration consensus exclude fits without a finite held-out prediction?

- [Research handoffs retain scoped evidence after worktree turnover](sessions/2026-10-23-durable-handoff/report.md) — Can another researcher inspect a useful result after its original worktree or supervisor is gone?

- [Queryable autocal evidence with scoped withdrawal consequences](sessions/2026-10-23-program-evidence-graph/report.md) — Can program memory reconstruct a research decision and the consequences of withdrawing a necessary premise?

- [Research-program sanity check: a useful consensus lead and larger task outcomes](sessions/2026-10-23-program-task-sanity/report.md) — Reviewed the three committed workflow changes and independently recomputed the finite-held-out-only consensus comparison proposed by task 003.

- [Existing research worker dispatch with observed ownership and cleanup](sessions/2026-10-23-program-worker-dispatch/report.md) — Can the manager dispatch a bounded assignment through the existing research launcher while retaining ownership and evidence?

- [Research program smoke checks and a question-only starting prompt](sessions/2026-10-23-research-program-smoke/report.md) — Can a user start this research workflow with only a question?

- [Fixed submissions and controller-owned evidence admission](sessions/2026-10-23-reviewed-claim-admission/report.md) — This session tested whether a controller can retain exact claim/evidence/candidate versions, commission a separate saved-evidence reviewer and admit only that assessment.

- [Finite research allocations and recovery across manager interruptions](sessions/2026-10-24-bounded-controller/report.md) — Can a manager continue distinct investigations within finite allocations and recover without duplicate dispatch or false physical-stop claims?

- [Chat-driven manager live gate blocked by runtime prerequisites](sessions/2026-10-24-live-manager-release/report.md) — Can the implemented manager carry a question through real workers, production tools, separate review and replanning?

- [Program-bound manager tools for an existing research chat](sessions/2026-10-24-manager-chat-entry/report.md) — This session tested whether a repository-connected coding chat can create/open a research program and operate its durable graph, workers and separate reviews through structured tools without manual service setup or a manager physics world.

- [Evidence-based route decisions and human steering in the research manager](sessions/2026-10-24-manager-strategy/report.md) — Can the existing manager compare routes, fund a useful rival discriminator and change direction without confusing execution failure with scientific evidence?

- [Reviewed findings survive combined patch failures](sessions/2026-10-24-tested-integration/report.md) — This engineering task asks whether reviewed scientific findings can remain useful when candidate patches conflict, become stale, change after review or fail only in combination.

- [Bounded research workers across competing approaches](sessions/2026-10-24-worker-pool/report.md) — The manager now coordinates a small local worker pool through the existing launcher.

- [Saved consensus evidence rejects blanket nonworsening adoption](sessions/2026-10-26-cm-saved-evidence-review/report.md) — This iteration asks whether saved CM-E1 evidence justifies unconditional finite-prediction-only final-consensus adoption.

- [Research Program usefulness versus the direct Research Agent](sessions/2026-10-27-program-usefulness-audit/report.md) — Does the saved record support lower usefulness from Research Programs?

- [HP3 force and friction robustness program](sessions/2026-11-05-force-disagreement-program-startup/report.md) — The program refreshed the parameter-variation agenda against earlier research, then tested encoder-only HP3 force handling under guide-friction and sensor-noise variation.

- [Retained batch research entry route](architecture-batch/report.md) — Retrospective summary of saved evidence, written 2026-10-08.

- [Architecture interactive](architecture-interactive/abstract.md) — Retrospective summary of saved evidence, written 2026-10-08.

- [Architecture interactive 2](architecture-interactive-2/research.md) — Retrospective summary of saved evidence, written 2026-10-08.

- [Interactive research tools and accepted steering](architecture-interactive-3/report.md) — Retrospective summary of saved evidence, written 2026-10-08.

- [Architecture resume](architecture-resume/research.md) — Retrospective summary of saved evidence, written 2026-10-08.

- [Architecture validation](architecture-validation/abstract.md) — Retrospective summary of saved evidence, written 2026-10-08.

- [Architecture viewer final](architecture-viewer-final/research.md) — Retrospective summary of saved evidence, written 2026-10-08.

- [Architecture viewer validation](architecture-viewer-validation/research.md) — Retrospective summary of saved evidence, written 2026-10-08.

- [CM-E1 saved-source audit: aggregate improvement fails per-dataset nonworsening](sessions/attempt-8cc3e522df934dafbc74a9639e34721e/report.md) — Do four archived CM-E1 sources support unconditional finite-prediction-only final-consensus nonworsening?

- [Autocal validation, collection throughput and compiled Python physics](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/report.md) — Saved autocal and throughput evidence plus compiled-solver follow-up, updated 2026-10-08.

- [HP3 comfortable force tuning and PJRT cache warnings](autocal-comfortable-force-20261010/report.md) — This investigation tested why a no-buildup HP3 reference run selected 12.8849 N and why autocal printed native PJRT compatibility warnings.

- [Autocal native crash isolation to the compiler path](autocal-crash-fix-20261007/research.md) — Retrospective summary of saved evidence, written 2026-10-08.

- [Autocal history ranking with held-out sweeps](sessions/autocal-held-out-history-ranking-7ec4260f/report.md) — Retrospective summary of saved evidence, written 2026-10-08.

- [Autocal history-ranking baseline, unfinished record](sessions/autocal-history-ranking-baseline-incomplete-a71aec03/research.md) — Retrospective summary of saved evidence, written 2026-10-08.

- [Autocal persistent-cache performance and quiet native logging](autocal-jax-cache-20261010/report.md) — Five complete 11-dataset calibration-log regression batteries compared a fresh persistent cache, cache disabled, and warm persistent caching using JAX/jaxlib 0.9.2.

- [JAX cache reuse on the recorded HP3 extended reference](autocal-jax-reference-cache-20261010/report.md) — Does the autocal JAX cache help the recorded HP3 no-buildup reference, and do executables warmed by the regression battery transfer?

- [Autocal ranking, bounded updates and identifiability](sessions/autocal-ranking-bounded-updates-negative-results-b7deb1bc/report.md) — Retrospective summary of saved evidence, written 2026-10-08.

- [Why the fifth HP3 reference trajectory differs today](fifth-hp3-reference-audit/report.md) — Retrospective summary of saved evidence, written 2026-10-08.

- [Production JS headless full-auto autocalibration](headless-autocal-validation/report.md) — The integration runs the existing full-auto autocal loop against production JavaScript physics in Node.

- [Headless extended autocal recording validation](headless-extended-autocal-validation/report.md) — The production headless JavaScript world completed default full-auto calibration with one finalized extended-reference RRD: eight sweeps, 160 points, 267,711 physics observations and 616,025 events in 1525.41 wall seconds.

- [HP4 baseline and A-axis motion tool check](sessions/hp4-baseline-a-axis-tool-check-93c49db4/report.md) — Retrospective summary of saved evidence, written 2026-10-08.

- [Planned HP4 motion check with no recorded results](sessions/hp4-motion-check-no-results-3c944b0c/abstract.md) — Retrospective summary of saved evidence, written 2026-10-08.

- [Saved HP4 recording Viewer inspection](sessions/hp4-saved-recording-viewer-check-338606b1/report.md) — Retrospective summary of saved evidence, written 2026-10-08.

- [Native and Viewer MCP round-trip check](sessions/native-and-viewer-mcp-check-3725e1e9/report.md) — Retrospective summary of saved evidence, written 2026-10-08.

- [Future-sweep selection versus refreshed autocal references](reference-refresh-october-7-2026/report.md) — Retrospective summary of saved evidence, written 2026-10-08.

- [Unassigned research session with no summarized result](sessions/unassigned-research-no-results-1cb5b420/research.md) — Retrospective summary of saved evidence, written 2026-10-08.

- [Unassigned research session with unsuccessful exit](sessions/unassigned-research-unsuccessful-exit-f9120c9b/research.md) — Retrospective summary of saved evidence, written 2026-10-08.

- [Unsuccessful launch with no research record](sessions/unsuccessful-launch-no-record-32819264/abstract.md) — Retrospective summary of saved evidence, written 2026-10-08.

- [Unsuccessful launch with no research record](sessions/unsuccessful-launch-no-record-4d08c2cf/abstract.md) — Retrospective summary of saved evidence, written 2026-10-08.

- [Unsuccessful launch with no research record](sessions/unsuccessful-launch-no-record-8225a40f/abstract.md) — Retrospective summary of saved evidence, written 2026-10-08.
