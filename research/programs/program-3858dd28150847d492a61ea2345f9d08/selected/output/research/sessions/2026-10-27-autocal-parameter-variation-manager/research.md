# HP3 autocal parameter variation program

Program: program-3858dd28150847d492a61ea2345f9d08

Status: three bounded cycles complete. Latest reviewed changes are published; chronology below preserves earlier decisions.

Objective: improve encoder-only HP3 calibration speed and robustness across plausible physical setups, preserving one-click operation and automatic stopping. The user allows two hours of simulation/compute and requests cloud environment repairs without permission questions.

Budget started approximately 2026-10-27 08:01 UTC. Stop numerical work by 10:01 UTC. The finite controller allocation runs 105 minutes from 08:04 UTC with 190 seconds cleanup, two tasks, one worker, and separate 10-minute review reservations. First worker contract is 55 minutes, at most 32 fresh fits, four collections and four million physics steps. These numerical limits are worker contracts, not aggregate command meters.

Constraints: HP3 only, fixed known radii, buildup zero, simulator layering disabled. Ground truth may evaluate simulated results offline but may never select deployed candidates. Use headless native production physics, no recording for large numerical sweeps and events for routine collection. No physics changes planned.

Competing explanations:

- Nominal-world overfitting hides useful improvements that a cheap varied-world test can identify.
- Force-dependent model inadequacy requires staged inference or new measurements.
- Existing candidates contain the answer but encoder-only ranking selects poorly.

The first discriminator will reproduce ranking mismatch on available recorded HP3 data, inspect relevant earlier negative results, then test a small candidate on explicit parameter variation. A useful evaluation tool is an acceptable retained change if calibration rules regress. The supplied reference path output/extended-autocal/no-buildup-reference-003 is absent in this checkout. Its provenance or a preserved equivalent must be investigated; no direct victory over that run may be claimed without its data.

Environment observations:

- Managed cloud environment became connected/running. Network policy is enforced and unrestricted, with no configured secrets, runtime variables, outbound identities or VPN.
- Controller runtime_check passed Python dependencies, Node dependencies, Rust/Cargo/LLVM tools, native physics, firmware and launcher socket checks. No installation repair is indicated by these checks.
- Independent agent-command socket creation, loopback binding and HTTP round-trip passed in 0.015 seconds. This establishes tool health, not calibration accuracy.
- Old cloud bridge requests from October 24 are completed/cancelled historical records. They were not redispatched.

Worker attempt: attempt-3b2e360ec1834eee9bab4e06f6f880fd, assignment variation-first. Review is pending. Next decision: inspect the worker's concrete experiment plan and evidence, then submit fixed evidence to a fresh reviewer.

## 08:14 UTC update

The investigator predeclared eight analytical worlds, four development and four holdout, with fixed world specs before fitting. PV-E02 completed 32 individual core fits. Its measured loop took 118.24 seconds and analytical generation took 0.022 seconds. These times exclude full calibration, collection and physical dynamics. A plot of saved rows is [analytical-screen.png](analytical-screen.png); [plot_results.py](plot_results.py) runs no fits.

The matrix does not support a default solver or thinning change. In one clean world, full-data L-BFGS-B predicts unseen measurements at 0.147 mm RMS with 1883.69 mm anchor RMSE. Full-data TRF gives 5.89 mm anchor RMSE, while half-data TRF gives 40.20 mm and slightly lower unseen residual. Other varied cases also regress. These are core-fit outcomes from a data-derived cold seed, not final full-auto consensus errors. The complete coverage and validity criteria must be checked before admitting a result.

PV-E03 completed two 2,000-step, 4-second native Rust stiffness probes, with identical commands and 20,000 vs 8,000 N axial stiffness. They are sensitivity probes, not complete calibration or a numerical validation of the analytical error model. Exact command and scene hashes are retained by the worker.

The cloud resource snapshot shows a two-CPU quota and 8 GiB memory limit. BLAS and OMP threads are both one. No dependency, socket or process environment failure has appeared. Direct publisher web requests returned HTTP 429; the literature note uses available indexed text and an institutional abstract and records this retrieval limit.

No exact reference-003 dataset hash match was found in 218 candidate JSON files, totalling 9.7 MB. This is a bounded archive search, not proof that no equivalent observations exist anywhere. Historical reports preserve the reference's 2.828 mm result but cannot replace its missing raw data.

Next decision: freeze the worker's evidence and candidate, obtain a separate review, and decide whether a targeted native collection followup would resolve the remaining analytical-to-production gap.

## Cloud actor interruption

After creating candidate cc52307f and reports, the actual cloud worker ended with the platform error "Selected model is at capacity" before writing handoff.json/final-message.md. The parent did not fabricate a child response or acknowledge successful bridge completion. It resumed the same actor and original bounded attempt solely to finish the handoff. No model, authentication, dependency or controller mode was changed, and no new numerical experiment was commissioned by this recovery. The original measurements and commit remain intact.

## Cloud review path repair

The first review guard exited with code 2 before creating a cloud request. Its stderr was "Final message must be inside repository output". The controller's normal default store places reviewer responses under .research-programs/reviews, outside output. This was a real path-validation mismatch in scripts/cloud/cloud_client_bridge.py.

Commit ae0b20c4 adds a narrow allowance for final-message.md inside repository-contained .research-programs*/reviews/review-run-<32 hex digits> paths, after symlink resolution. Existing output paths remain accepted. Tests reject control files, malformed run names, paths outside the checkout and review-store symlink escapes. The exact default-store completion fixture fails before the patch; 51 transport/review-controller checks pass after it. Logs are cloud-review-path-before.log and cloud-review-path-after.log. These fixture checks are distinct from real model dispatch.

The failed review consumed its reserved allowance. The manager stopped the completed worker allocation and explicitly funded a new 10-minute review allowance. The retry created bridge request 697c5e9996044784bbecc1eb24088011 in the default store and was bound to actual fresh reviewer /root/autocal_screen_reviewer. This verifies the corrected launch boundary. Its evidence assessment remains pending. No store, authentication, model or historical evidence was replaced to bypass the error.

## Second discriminator, 08:31 UTC

The first cycle was saved with no skipped files in archive commit 2a5ef25b. The cloud bridge source repair is ae0b20c4. Actual separate review-f674059afe874bdca72e7f4f5f8366ee found supported/keep, ready code, no discrepancy, and independently recomputed saved data. Admission-d4a35af838c44f1f9b7c10f88df01741 applies it. The manager did not force this narrower supported claim into the predeclared mixed program-outcome field.

A new native-transfer route funds assignment native-pair, attempt-982dc44e4c9b4e5da9b3232c26d54bf3, with 30 minutes from 08:31 UTC, two production collections, eight high-level fit-stage calls and two million physics steps. A separate 10-minute review remains reserved in a 45-minute allocation. Numerical work should finish by 08:56 UTC, final worker handoff by 09:01 UTC, well inside the global 10:01 UTC limit.

The exact discriminator uses session-local copied scenes differing only stiffness 20000 versus 8000 N, no layering, fixed 30 mm radii and buildup zero. Match collector settings and target three sweeps/60 points in each world. Evaluate ordinary fitting and recovery where supported, truth-free, with geometry scored offline. Manual matched forces isolate a physical variable but do not test automatic tuning. Bootstrap-only work does not establish automatic full-loop stopping or fewer-point savings. Preserve partial failures and actual baked scene/engine identities. Parent will obtain a fresh review before interpreting results.

Before native outcomes, source inspection corrected sweepPoints from 20 to 10 per direction, giving 20 points per combined sweep and 60 over three sweeps. The worker initially proposed a 0.01 N sensor preload. The manager cited research/native-collection.md, which documents a stationary sensor at lower preload and recommends 1 N for this scene. The worker confirmed no collection had started and amended both worlds to 1 N before outcomes. No trial was discarded or tuned after observing stiffness results. Other matched proposed forces are low/mid/max 0.01/0.1/1 N, feed 1400 and production noise defaults. The attached stock layered world passed preflight but remains unused for movement; the standalone copied worlds must verify unlayered baked inputs separately.

## Standalone Node mismatch, 08:35 UTC

Both initial native startup launches aborted in napi_detach_arraybuffer before completing physics step 1, with zero points and zero fitting. The standalone driver shell had inherited system Node 24.19.0, while the supervised runtime had correctly loaded pinned Node 23.11.1. The exact crash is documented in scripts/cloud/README.md. Command sockets and supervisor preflight do not establish the Node version of an independent agent shell.

The worker acknowledged that it had omitted source scripts/cloud/env.sh for the standalone driver. Parent independently verified /opt/codex/.../node v24.19.0 before sourcing and /workspace/.tools/node/node-v23.11.1-linux-x64/bin/node v23.11.1 afterward. The worker now checks the exact subprocess version before startup and writes to a new pinned/ artifact directory, leaving both originals intact.

The manager recorded a finite correction: exactly two replacement startup launches, four startup attempts total including the zero-step crashes, at most two completed physical collections, unchanged two-million total steps, eight high-level fit stages and original 09:01 UTC worker deadline. No force retuning, native engine changes or shared rebuild. This distinguishes setup failures from a negative scientific result and does not assume that a successful launch establishes accuracy.

## Native point and span results

Pinned nominal and soft runs each collected three complete 20-point sweeps. Nominal took 26.324 world seconds and 192909 physics steps. Soft took 48.699 world seconds. No fits or points were produced by the earlier Node24 launches.

Correction to the earlier worker message and parent commentary: the nominal 1.269 mm size-tuned value is a FIXED-anchor target delta, not the drive arc. Measured first-sweep drive span is about 94.106 mm. The incorrect narrow-drive inference must not be used in conclusions. Actual per-sweep drive/sensor spans will support the report.

Four ordinary fit stages returned valid=true and underconstrained=false, so the configured standard recovery hook did not run. The worker reports boundary-limited/invalid conventional geometry on the soft data. Raw collection metadata contains M669 firmware guesses coinciding with conventional scene-home truth. Although the inspected initializer uses raw encoder extents rather than M669, the worker is spending the remaining four fit-stage calls on copies with stored geometry metadata removed. The final native claim will use those stripped inputs and preserve raw calls as a sensitivity audit, within the eight-stage budget. Known radii and measured observations remain unchanged.

## Continuation after platform usage interruption

The user said continue after a platform pause. Current host time had advanced to11:54UTC. The worker actor reported a model usage-limit error. Its launcher had already enforced the original09:01UTC deadline, exited130 and reported launcher_finally_completed cleanup. The eight fit-stage outputs and both native datasets were present; no worker report/final response or handoff existed. The parent verified the actor's stopped state and acknowledged cancellation to the cloud bridge. No historical PID was signalled and no source model/authentication was changed.

The numerical budget stayed closed. The manager wrote explicitly attributed closure reports and arithmetic summaries from saved data, completed the worker's unfinished collection helper, and ran19 focused mocked/data-contract checks without new simulation or fitting. Candidate a4da0d73 adds the bounded collection helper and its tests/documentation. Partial worker artifacts and both Node24 startup failures remain intact. A new seven-minute saved-evidence-only review was explicitly funded for submission-999974fdb09c44b4ae8f09131caa9ef6. No new numerical experiment was funded by the continuation.

Manager arithmetic confirms602454 native steps,120 finite points and75.023 total measured world seconds. Removing only config.m669 yields identical returned anchors, metrics and optimizer histories for all four pairs. Conventional scene-home RMSE is2023.36/2003.45mm nominal lbfgsb/trf and3648.26/2676.18mm soft. Reference pose/rotation were not observed. These are bootstrap estimates, not demonstrated full-loop accepted calibrations. The saved driver valid flag is explicitly a finite-score/covariance flag rather than complete automatic acceptance.


## Final decision

Native review-5f62ee3093f94f7a9af98157ecb24066 supports/keeps the scoped claim and marks a4da0d73 ready. Admission-7d84479eb45c4688a07fd8c62f9f3cf0 applies it. No further checks are required. The numerical budget is closed and program paused. Keep both tools and runtime remedies; retain calibration defaults. All fixed evidence and failed/interrupted trials remain preserved.

## User-directed improvement cycle

The user requested pushing the reviewed changes and continuing with explicit approach pruning. Published HEAD703e1a68 to origin/codex/autocal-parameter-variation-guards, verified by ls-remote. Reopened the same program and resumed its memory rather than creating a new program.

Current allocation runtime_allocation-acfdca734b854d67b48228eb35ed978c funds assignment coverage-vs-force, attempt-902d9713dcbe436d8ddc1ba23fd82baa. Worker deadline is approximately13:07UTC, with45 worker minutes from12:22UTC,4 collections,24 fit stages and4M steps. Separate review reserves8minutes within60 total manager minutes. This is the bounded continuation authorized by the new user message. It does not silently restart a cancelled old attempt.

The [approach ledger](approaches.md) retires blanket point thinning and unconditional TRF replacement in their tested scope, parks residual-only ranking, and prioritizes redistributing60 points across more independent sweep geometries. Force adequacy is the funded rival. The worker must cut the main route if it has little progress at the10-15minute checkpoint and preserve the reason. No new generic tooling project is the main objective.

A proposed initialization explanation was checked before funding: ellipse_solver.py already uses uniform full-bound random starts after the supplied data-derived guess. It is inaccurate to assume all starts are tiny perturbations. This untested broader idea is not being funded without a narrower discriminator.


Before current outcomes, the worker inspected the actual baseline fixed_lengths: each pair has one nonzero tuner-derived fixed level, not literal origin. The test is now described precisely as one versus two levels at60points. Proposed per-world offset delta is min40mm,one-quarter minimum observed drive span, centered on the baseline tuner output. The20percent error gate is exploratory only; metre-scale errors are not an acceptable product result. Parent has independently confirmed nonzero baseline curvature, so matching drive/sensor spans must not be described as zero-curvature data.


## Early route cut and force continuation

The6x10,two-level coverage variant completed both60point datasets but failed the paired20percent exploratory gate. Nominal lbfgsb/trf conventional RMSE3214/1930mm versus2023/2003mm controls; soft3096/3972mm versus3648/2676mm. Two comparisons worsen; all four fail the20percent target. Stop refining this variant and park its broader route pending new force-adequate evidence. These producer outcomes await separate review.

The next two automatic-force runs learned materially different forces, then both hit the preserved1M per-world step limit before completing60points. This does not reject force accuracy. Four collection invocations used2,883,371steps in total. Manager explicitly permits two fresh-world data collections with each worlds already learned encoder-derived forces, avoiding repeat search and avoiding outcome-based retuning. Cumulative limits become six invocations and6Msteps,1.55M per new world, with24fit stages and original13:07UTC worker deadline unchanged. No further expansion if they fail. Active-time budget remains bounded by the same60minute cycle authorized by the user.


## Productive force route

Two fresh force-replay datasets completed3x20points. Nominal741617steps/76.87s and soft541269steps/about55s. NominalTRF gives about3.29mm conventional anchor RMSE; softLBFGSB/TRF about10.43/11.64mm. Nominal ordinaryLBFGSB reaches sentinel100, but existing restore_sweeps then restore_points recovers the same3.29mm in0.46s. Thus no solver modification is justified by this case. Ten ordinary/recovery fit stages used.

The supported code direction is early atomic force-selection persistence, addressing the observed interrupted searches that completed551484/569073steps of tuning before data collection failed. It preserves tuned/manual/fallback settings with context and timestamp, without automatically applying an old cache. A final dataset refresh keeps any force changes made during sweep recovery. Optional per-config targets remain separately scoped experiment-enabling capability, not an accuracy improvement. No new numerical experiment is planned before fixed evidence review.


## Final review after continuation

A second platform usage-limit interruption arrived after the worker wrote its genuine report.md and final-message.md and completed source commits39510991/25822cbb. It did not finish handoff.json. The original launcher deadline still cancelled the supervisor with confirmed cleanup. Parent preserved the genuine output, acknowledged actual cancellation, and prepared an explicitly manager-selected evidence packet rather than inventing a worker handoff. No new numerical experiment was run after the user continued.

The fixed final submission is submission-72b96ed174494e6286eba68667db11cf, with23selected assets including explicit compressed measurement/source sets. An8minute saved-evidence review is funded separately. Candidate25822cbb retains optional per-config targets separately from early atomic force-selection persistence; production force/solver/stopping defaults are unchanged. Producer checks show69JS cases and17Python cases passing, with627 verification physics steps already included in the4,166,884step total. The final reviewer is prohibited from new physics/fitting.


## Final publication

Review-8b7d4a844d274f31855086c3c4175d5d supports/keeps checkpoint25822cbb and optional per-config targets39510991, code ready, no required checks. Admission-44964abf6b884000a838a6d017c7ae69 applies it. Source pushed to origin/codex/autocal-parameter-variation-guards. Numerical work is closed. Keep force adequacy and existing recovery, preserve force knowledge, cut the tested coverage variant and leave faster force search as an unfunded question.
