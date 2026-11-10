# Cloud relay enables production trials but exposes two remaining tool defects

Status: complete bounded validation, independent review pending.

## Abstract

This actual cloud actor tested source 379318b2 through the launcher-owned MCP file relay while its own socket creation remained denied. All 23 advertised tools were invoked. Three fresh Rust-native experiments completed in the shared-output worktree; the recorded and unrecorded 500-step pair had byte-identical 501-row telemetry. Research-services passed 176 tests. HP3 configuration, preflight, six-point collection, collection cancellation, real Rerun inspection and browser pause/50-step/context capture worked. The one-iteration full-autocal CLI produced 24 measurements and stopped at its budget after 1,305,459 physics steps. Its finalized RRD was readable. Two defects remain. Browser interventions returns a list rejected by the MCP dictionary schema. Infinite autocal progress makes strict status serialization fail and leaves a stale active job, blocking recovery reset and a second isolated cancellation trial. These are simulated tool-health results, not calibration accuracy or hardware evidence.

## Execution and budgets

Program `program-7aa80aa18f10499e85cb24ffeae712cb`, assignment `cloud-production-after-viewer-fix`, attempt `attempt-e13302e362f545e39608027dcb85f963`. The allocation was 25 minutes starting 22:43 UTC, two direct collections, two autocal jobs, eight fresh fits and 20,000 direct experiment/step steps. No source code, shared build or dependency was changed by this actor. All real hp_sim5 and Rerun calls used `scripts/cloud/cloud_tool_bridge.py` through the owned file relay. No actor network permission was requested and no nested coding CLI or agent was launched.

The default actor socket probe returned EPERM. This remains separate from the supervised preflights and relay success. The cloud environment reported current enforced unrestricted HTTP policy, which did not remove the observed command socket restriction.

Direct numerical use was 1,500 fresh native experiment steps, 100 explicitly requested attached-world steps, 52 steps draining a real G-code move, and 50 browser steps. That is 1,702 observed direct steps. Collection jobs used 20,243 plus 504 steps. The single launched full-autocal job used 1,305,459 of its 2,000,000-step limit. Total observed production physics was 1,327,908 steps, excluding isolated named-suite fixture worlds whose aggregate steps were not measured. The two direct collections exhausted that allocation. One real autocal job launched; the second start was rejected before creating a job. Three fit stage artifacts, including two recovery fits, were produced. No extra direct fits were run. The full-autocal contract used events recording, one thread, max_steps 1, sweep_points 3, wall_timeout_s 600, solve_iterations 4, solve_restarts 1 and spool_outer_iters 1.

## Numerical and recording evidence

| Observation | Actual result | Evidence |
| --- | --- | --- |
| Initial HP4 preflight af107ddfdeaa4182abda0f934d66b2c3 | Passed, 0.863947 s; attached world at step zero | [Raw result](research_preflight.json) |
| Recorded HP4 run 60d2127db7d443b7863d0add0272c51a | 500 steps, 2 ms, native Rust; complete; no warnings | [Run manifest response](experiment-recorded.json) |
| Unrecorded paired run a974fbcebaa645bdaf47abaf53f4a053 | Same commands and scene; all 501 telemetry rows byte-identical | [Verification](numerical-recording-verification.json) |
| Changed run db54f06bbb60465180e590ffbf49fb6c | Final effector distance 9.4932929749e-6 m; changed command hash disclosed | [Comparison](experiment-difference.json) |
| Numeric page and native capture | Steps 245 through 248 read; exact step 250 captured at 0.5 s, capture 00941372f44445648351f88ff4eaad41 | [Page](experiment-numeric-page.json), [capture](native-context.json) |
| Recorded scalar and blueprint | Actual RRD scalar at step 250 read; one blueprint with a 3D view and five time-series views inspected | [Scalar](recorded-sample-verification.json), [blueprint](native-blueprint-summary.txt) |
| Paired trajectory hash | f9ca994cb413bf4a08398ff5a05ee61712e555c83be99a3439b0b58f2ffeaa4e | [Hashes](numerical-recording-verification.json) |

The recorded run's 14,776,566-byte RRD is readable. The record on/off comparison has zero metric deltas and zero final effector distance, while input comparison correctly reports the recording option difference. This establishes trajectory equality only for this specific fixed schedule. Wall times are not a throughput comparison because suite and collection activity overlapped.

Rerun Viewer MCP connected to the actual headless Viewer, opened the recorded RRD, inspected timelines and reports, and selected `sim_step=250`. The selected state identifies the correct native run and lists six blueprint views without view reports. Viewer logs returned no buffered entries. Screenshots were decoded to disk and inspected through view_image. [The native screenshot](viewer-recorded-native.png) shows the selected native run and geometry. It was captured while asynchronous chart data was loading. Later incoming browser streams changed the active Viewer recording, which is retained in [the later screenshot](viewer-recorded-ready.png). Numeric conclusions use recorded data, not pixels. The Viewer visibly uses software rasterization on this CPU platform.

## Configuration, test and collection lifecycles

`send_gcode` planned `G1 H2 X1 F600` through real RRF and the encoder read drained the motion. Explicit stepping advanced the continuing HP4 world to step 152. `configure_session` then created HP3 generation 2, unlayered, four 30 mm physical radii, zero buildup and matched `sys/config_hp3.g`. Preflight bd077a8ec3b2403dbff1aa2e59a22489 passed against the complete expected configuration. Reset created generation 3 with a new world ID and retained accepted settings. Geometry has zero cable half-width and all three layering flags disabled. [Configuration](configure-hp3.json), [preflight](preflight-hp3.json) and [reset](reset-hp3.json) preserve the exact observations.

Named test job ba1cd62109d84f98a66387f9861dfce9 was observed running, cancelled and polled terminal. It exited -15 after 0.137214 s with cancellation preserved. Full research-services job 419ce7d0faf74c2abf76be2a9fb3018f passed 176 tests, 14 deselected, in 118.85 s pytest time and 121.242 s job wall time. Its artifacts include a compiled Warp solver in the job-owned cache. The unchanged 49-case headless suite and the manager's physics gate were not duplicated. [Terminal test result](test-services-poll3.json), [JUnit](native/tests/419ce7d0faf74c2abf76be2a9fb3018f/junit.xml) and [stdout](native/tests/419ce7d0faf74c2abf76be2a9fb3018f/stdout.log) support these counts.

Tiny collection d86b1e5ea7a84165bc21aadabba53470 used the documented/tested HP3 defaults with sweepPoints 3 and noiseSamples 4. It completed six real measurements over steps 0 to 20,243. Independent production validation found schema 2.0, six residuals, drive span 95.441585 mm and physical sensor spans 48.558550 and 48.610910 mm. The firmware-anchor residual RMS 0.257168 mm is an ingestion check, not a fitted calibration-accuracy estimate. Its 52,292,812-byte RRD and six-point partial journal were readable.

Second collection ea2e05e46bd14417866db1f849c43ebd was cancelled after observable startup. It stopped at step 20,747, exactly 504 steps after its start, before the next fixed physics step and collector command. Queues were empty and subsequent status retained that boundary. Its 2,152,231-byte partial RRD was finalized and readable. It completed zero measured points, so no partial-points file exists. This is disclosed rather than claimed as lost measurement evidence. `step_physics(1)` correctly rejected the reset-required world. Recovery reset was attempted later but blocked by the unrelated stale autocal active job. [Collection verification](collection-evidence-verification.json), [independent validation](collection-independent-validation.json), [cancel result](collection-cancel-terminal.json) and [fixed boundary](runtime-cancel-still-fixed.json) preserve these paths.

## Browser checks and remaining defect

The manager owned and connected headless Chromium in its permitted context. The first URL retired during the required native configure/reset sequence; its refused connection was retained by the manager. The replacement browser service remained stable for all actor checks. The browser world is separate from the attached native HP3 world and used the browser's HP4 scene.

The actual page ID was `2b7e9890-6ee2-4494-94a9-cc73c5b235dd`. Through the actor relay, capabilities, pause, observe, 50 fixed steps, a second pause and context capture all passed. Context `46177dae-2002-446b-8326-ff2c58e2fd37` retained page, scene generation 3, step 50, time 0.1 s and actual numerical observation. Disabling recording finalized a 1,636,739-byte RRD with 51 physics samples and the exact 0-to-50 boundary. The RRD and blueprint were read back. See [context](browser-context.json), [recording verification](browser-recording-verification.json) and [manager connection](../2026-10-26-cloud-platform-audit/browser-after-reset/result.json).

An extra read-only `browser_action` request with action `interventions` failed. The application returns an array of intervention records, but `scripts/hp_sim5_mcp.py:297` declares `dict[str, Any]`. MCP result conversion raises Pydantic `dict_type` validation. The smallest reproduction is this single action on a connected page after any intervention. [Raw error](browser-interventions.json) and [traceback](browser-interventions-failure.txt) retain it. The manager was informed promptly; any later repair is outside this source boundary.

## Full-autocal result and remaining defect

Job d5f69c419e474b54b098a2077715349d ran the production CLI with the declared bounds. Initial MCP status polls worked and reported genuine physical progress. It collected 24 measurement records and produced one planned fit plus two recovery fit artifacts. The tiny dataset was underconstrained, with sentinel score 100 and no valid best score. The CLI reached max-steps, finalized recording and stopped all five owned service PIDs. The headless manifest says complete, normal_completion false, stop_reason max-steps, 1,305,459 steps, 2,610.918 simulated seconds and 157.785449 s headless wall time. This is budget-stop evidence, never acceptance.

The events RRD is 96,931,452 bytes and readable. Readback counted 350,789 recorded events, including 24 collector measurement events, one headless service_stopped event and one Python run_complete event. [Artifact verification](autocal-artifact-verification.json) and [event readback](autocal-recorded-events.json) support those figures.

Terminal MCP status failed. The final full-auto log contains `best_cost=Infinity` and `best_score_ui=Infinity`. `AutocalJobs.status` copies best_score_ui into progress; strict runtime HTTP encoding uses allow_nan false and closes the connection with a ValueError. The terminal persistence path also calls strict write_json on this result before clearing `self.active`, leaving the old job registered as active even though physics and child processes stopped. Repeated status and cancel calls fail. The second autocal start and collection recovery reset reject the stale active job. The final job.json remains the original running record, so no successful terminal MCP classification is claimed. [Runtime traceback](autocal-status-runtime-failure.txt), [failed status](autocal-terminal-probe.json), [failed cancel](autocal-nonfinite-cancel-probe.json), [blocked second start](autocal-cancel-start-attempt.json), [blocked reset](collection-recovery-reset-attempt.json) and [final runtime state](runtime-after-recovery-attempt.json) preserve the failure.

This blocks the requested second isolated cancellation trial and the post-job usability check. Attached world identity and its exact collection stop boundary remained intact, but the active-job error prevents resetting it for more movement. The second trial is not a negative physical result because it never launched.

## Coverage and limits

[coverage.json](coverage.json) records each of the 23 advertised tools, successful calls, mixed paths and blocked checks. All 23 were invoked; browser_action, start_autocal, autocal_status and reset_session have both useful successful paths and failed or blocked paths. cancel_autocal did not complete a usable lifecycle. One expected step rejection after collection cancellation is not counted as a defect. The result is mixed, not blanket production readiness.

GPU, native-warp-cuda, hardware, large held-out calibration, calibration accuracy and global optimality were not tested. The Viewer software renderer works for this inspection but is not a graphics performance finding. Fontconfig emitted writable-cache warnings during isolated autocal and local dataset validation; those did not prevent the measured CLI workload or readback. No private attachment credentials are included in the report. File-relay calls remain distinct from command-network authorization.

## Tested hypotheses

| Hypothesis | Experiment or job IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| The owned file relay permits actual actor tools while actor sockets remain denied | Attempt e13302e362f545e39608027dcb85f963 | Supported | [Socket probe](command-sockets-default.json), [relay calls](relay-calls.jsonl) |
| Shared-output and isolated-cache repairs enable the formerly failing production paths | Runs 60d2127d, a974fbce, db54f06b; test 419ce7d0 | Supported within this scope | [Numerical verification](numerical-recording-verification.json), [test terminal result](test-services-poll3.json) |
| Recording preserves this fixed native trajectory | Runs 60d2127d and a974fbce | Supported | [Exact telemetry hashes](numerical-recording-verification.json), [comparison](experiment-equality.json) |
| Current tools complete every requested production lifecycle | Collection d86b1e5e and ea2e05e4, autocal d5f69c41, browser page 2b7e9890 | Mixed | [Coverage](coverage.json), [autocal failure](autocal-status-runtime-failure.txt), [browser failure](browser-interventions-failure.txt) |
| A second isolated autocal can be cancelled and leave the attached world usable | No second job launched | Not tested, blocked by stale active job | [Rejected start](autocal-cancel-start-attempt.json), [rejected reset](collection-recovery-reset-attempt.json) |

## Decision handoff

- Scope: source 379318b26773b8dc37c191b506da1c57b5886f6f, managed CPU cloud, headless-js hosting Rust native, stock HP4 and unlayered HP3 with physical 30 mm radii and zero buildup.
- Review: this actor made the relay calls, inspected raw numerical pages, read finalized RRDs and viewed saved screenshots. The manager connected and owned headless Chromium. Independent review remains pending.
- Decision: retain the relay/output/cache/Viewer fixes and the successful production evidence. Full workflow readiness remains blocked by nonfinite autocal status and the browser interventions return schema.
- Next: normalize unavailable autocal scores for strict JSON, clear active ownership even when final status persistence fails, and align browser interventions with its MCP result contract. Then repeat terminal status, recovery reset and one controlled isolated cancellation on a new source boundary.

## References

- [Full report](report.md), [notebook](research.md) and [coverage](coverage.json) preserve this attempt's decisions and measurements.
- [Baseline audit](../attempt-ca404fcaf323469eb35cc37b515ca3a7/report.md) identifies the original shared-output and Warp-cache defects. Historical test results are not counted as this attempt's measurements.
- [Research tool guide](../../../../research/README.md), [integration rationale](../../../../research/investigation.md), [native collection guide](../../../../research/native-collection.md), [autocal job verification](../../../../research/autocal-jobs-verification.md) and [recording cost study](../../../../research/autocal-recording-performance.md) define the production settings and evidence contracts used here.
- [Manager browser connection](../2026-10-26-cloud-platform-audit/browser-after-reset/result.json) records the separate permitted Chromium owner. Actor browser actions remain in this session.

## Final tool matrix

| Tool | Outcome | Tested path |
| --- | --- | --- |
| capabilities | successful | 23 tools and production limits discovered; initial raw result retained |
| runtime_status | successful | Initial HP4, configured/reset HP3, cancellation boundary and stale autocal ownership observed |
| research_preflight | successful | HP4 and complete expected HP3 configuration passed |
| start_test | successful | Named cancellation job and complete176-case research-services suite |
| test_status | successful | Running cancellation job, terminal cancel and176passed suite observed |
| cancel_test | successful | Running job cancelled and terminal cleanup polled |
| send_gcode | successful | Real RRF movement plus encoder drain |
| start_collection | successful | Two actual direct collections; tiny completion and controlled cancellation |
| collection_status | successful | Progress, six-point completion and terminal cancellation observed |
| cancel_collection | successful | Fixed20747boundary after504steps; finalized readable partial recording |
| start_autocal | mixed | First production CLI launched and ran bounded workload; second start blocked by stale active state |
| autocal_status | mixed | Initial physical progress worked; terminal Infinity progress fails strict JSON |
| cancel_autocal | failed | Existing nonfinite progress prevents usable cancellation/status result; second cancellation job never launched |
| configure_session | successful | Complete HP3 unlayered30mm/zero-buildup world and matched firmware |
| reset_session | mixed | Normal reset retained settings; later recovery blocked by stale autocal ownership |
| step_physics | successful | 100steps advanced; expected reset_required rejection after cancellation |
| start_browser_service | successful | Initial and replacement service URLs returned; native configure/reset retires prior service |
| capture_native_context | successful | Exact native step250/0.5s captured |
| browser_status | successful | Actual connected page and finalized51sample recording/context observed |
| browser_action | mixed | Requested capabilities/pause/observe/50step/context/record paths passed; extra interventions list rejected by dictionary schema |
| run_experiment | successful | Three fresh native500-step runs in approved shared-output worktree |
| read_experiment | successful | Actual numeric samples245to248 read |
| compare_experiments | successful | Record on/off equality and changed commands disclosed |
| Additional paths | Untested or blocked | Second isolated cancellation, recovery reset usability, GPU, hardware and calibration accuracy |

Observed physics totals: 1,702 direct steps, 20,747 collection steps and 1,305,459 isolated full-autocal steps. Total 1,327,908. Named-suite fixture totals were not measured. Two collections and one launched autocal job were used; three fit stage artifacts were produced.

Current platform limits are actor command socket EPERM, CPU software rendering, browser interventions typing and nonfinite autocal status with stale job ownership. Repair the last two, then run a distinct short terminal/reset/cancellation trial at its new source revision.

[Source excerpts](source-evidence.txt) identify the tested return/serialization contracts. [Raw Infinity row](autocal-infinity-row.jsonl) preserves the actual nonstandard score record without normalization.
