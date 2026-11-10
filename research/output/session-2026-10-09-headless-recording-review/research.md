# Headless extended recording commit review

Objective: critically review commit 8ded7420 without changing production code.
Steering: inspect lifecycle, clocks, failure reporting and completeness; isolate any probes from other running services.
Budget: fast focused tests and short fault injections, no full autocal run or hardware movements.

H1: unexpected physics-process death is rejected by recording finalization. E1 rejected: a real owned physics process killed with SIGKILL exited -9, but lifecycle exit succeeded, normal_completion was true and the recorder finalized. The RRD contains zero service_stopped events. Evidence: result.json and sweeps.headless_1/manifest.json.

Fast checks: 9 Python headless tests passed (2 slow deselected); 13 JavaScript recorder tests passed.
E0: first E1 invocation could not open local sockets under the sandbox. This is a tool permission limitation, not a negative recording result. The approved retry ran successfully in an isolated service directory.
Next decision: report the exit-validation defect; recommend checking physics return status and graceful drain completion, with a crash-after-last-RPC regression. No production edits. Review complete; no full optimization or slow parity suite rerun.
