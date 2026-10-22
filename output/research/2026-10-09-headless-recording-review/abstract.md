# Critical review of headless autocal recording finalization

Status: complete.

## Abstract

Review of commit 8ded7420 examined headless recording startup, clock labels, acknowledgement backpressure and shutdown. Nine fast Python headless tests and thirteen JavaScript recorder tests passed. A short fault injection killed the real owned Node physics process after successful requests, using synthetic optimization-completion markers to isolate lifecycle validation. The process exited -9, yet context shutdown succeeded and the manifest reported complete status and normal_completion=true. The recorder finalized its RRD and recorded autocal_complete=true, with no physics service_stopped event. Recorder finalization does not establish graceful physics-source draining. The test establishes a failure-reporting gap, not a measured amount of lost telemetry or any calibration-accuracy result. No production code was changed; the slow full-auto/parity suite was not rerun.

## Topics

autocalibration, headless simulation, Rerun RRD, extended reference, finalization, crash detection, wall clock, acknowledgement backpressure

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Unexpected physics death prevents successful recording completion | E1 | Rejected | [Result](result.json), [manifest](sweeps.headless_1/manifest.json), [probe](probe.py) |
| Existing focused lifecycle and delivery checks pass | T1, T2 | Supported within their tested scope | [Report](report.md) |

## References

- [Full report](report.md): finding, checks and limitations.
- [Research notebook](research.md): constraints, permission limitation and outcome.
- [Reviewed lifecycle](../../../autocal/headless_sim.py): commit 8ded7420 shutdown logic.
- [Implementation verification](../../../research/headless-autocal-verification.md): full-run evidence and documented parity failure; not independently rerun here.
