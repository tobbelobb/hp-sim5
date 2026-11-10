# Critical review of headless autocal recording finalization

Status: complete.

## Abstract

Review of commit 8ded7420 examined headless recording startup, clock labels, acknowledgement backpressure and shutdown. Nine fast Python headless tests and thirteen JavaScript recorder tests passed. A short fault injection killed the real owned Node physics process after successful requests, using synthetic optimization-completion markers to isolate lifecycle validation. The process exited -9, yet context shutdown succeeded and the manifest reported complete status and normal_completion=true. The recorder finalized its RRD and recorded autocal_complete=true, with no physics service_stopped event. Recorder finalization does not establish graceful physics-source draining. The test establishes a failure-reporting gap, not a measured amount of lost telemetry or any calibration-accuracy result. No production code was changed; the slow full-auto/parity suite was not rerun.

## Finding

P2: autocal/headless_sim.py:164-173 ignores the return code of an already-dead physics process. It then reads a stale clock.json whose error field can remain null. Lines 195-197 validate only the recorder process and finalized recording statuses. If physics dies after the last successful RPC, including during the finalization window, the capture can be certified complete without the physics source completing its drain. The return code and graceful shutdown completion should be checked independently of recorder file finalization, while allowing expected shutdown semantics for the separate RRF process.

E1 used the actual Node/RRF/recorder processes and .05 seconds of fixed-step physics. Completion markers stand in for the final optimizer state; no numerical fit was run. SIGKILL yielded physics_returncode=-9, context_exit=success, status=complete, normal_completion=true, recording_error=null, recorder_status=finalized, autocal_complete=true and zero service_stopped events. All owned services were stopped by the probe lifecycle. The sampled tail difference is expected from stride ten and is not evidence of a sampling defect. Amounts of dropped telemetry were not measured.

## Checks

- T1: `.venv/bin/python -m pytest autocal/tests/test_headless_sim.py -m 'not slow' -q`: 9 passed, 2 deselected, 5.02 seconds. Covers real owned/external recorder startup, resetting source clocks, disconnect failure, configuration validation and finalization.
- T2: `npm test -- --runInBand tests/js/hp-sim-3d/flightRecorder.test.js`: 13 passed. Includes sampled delivery, source labels, event acknowledgements and capacity-based resumption.
- E0: sandbox denied the probe's socket creation. After approval E1 ran successfully; E0 does not test recording behavior.
- Code inspection: separate source clocks are explicit, physics capacity waits do not require draining the entire event stream, and metadata/scene/configuration artifacts are embedded.
- The commit's verification document reports a red slow browser/Node parity check, reproduced against the earlier service. It was not independently rerun or attributed to this commit in this review.

## Evidence and limits

- [Result](result.json), [manifest](sweeps.headless_1/manifest.json) and [probe](probe.py) establish E1.
- [Abstract](abstract.md) lists tested hypotheses and references.
- Reviewed revision: 8ded742059bbb0061ce6cafb7895bb120ab65bb7.
- This review establishes one reproducible failure-reporting defect. It does not establish hardware validity, fit accuracy, universal parity or performance bounds.
