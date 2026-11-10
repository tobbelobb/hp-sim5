# HP3 cloud tools reveal shared-output and isolated-cache defects

Status: complete baseline audit, independent review pending.

This audit asked whether current cloud research tools operate smoothly after runtime setup passes. The real worker discovered 23 advertised tools through its owned HP3 Rust-native supervisor. Worker-authored MCP probes ran in the parent's permitted command context after the worker's permission-bearing call stalled and its default socket probe returned EPERM. Non-moving preflight passed in 0.818 seconds and its finalized 67,028-byte RRD was read back. The headless-autocal suite passed 49 tests. Research-services passed 172 tests and failed three, exposing two defects. Shared worktree output paths are rejected by fresh-experiment containment checks, and isolated test jobs discard the writable Warp cache setting. Both direct fresh-experiment requests failed before physics began. The attached world retained identity and step zero. No collection, fit, calibration-accuracy or hardware result was produced. Passing setup is insufficient to claim all tools work optimally.

## Scope and method

Program `program-7aa80aa18f10499e85cb24ffeae712cb`, assignment `cloud-tool-inventory`, attempt `attempt-ca404fcaf323469eb35cc37b515ca3a7`. Source was `67617705b7996ee3a84889e6723cab149b6e5884` in launcher worktree `bounded-research-assignment-0889b918`. The supervisor used `headless-js`, native artifact SHA-256 `66a8451bc1a42310e4dfae166288737e595f75bc1173c8f88e92163afb94a5d8`, HP3, layering enabled, four physical 30 mm base spools and 2/pi buildup.

The allocation was 22 minutes, no collections or fresh fits, and at most 20,000 direct physics steps, plus named isolated suites with their own timeouts. No source edits or shared builds were made. Direct physics steps were zero because both requested 500-step trials failed before run creation. Suite fixture worlds are separate tool-health evidence.

The worker authored [audit_tools.py](audit_tools.py) and [audit_suites.py](audit_suites.py). Its first additional-network execution request yielded no result for 341.6 seconds and was interrupted. The actor could not determine whether approval was pending. The same conversation continued, and its default socket probe returned EPERM. The parent then executed the authored scripts in its already-permitted context against this attempt's private supervisor. This establishes actual MCP/supervisor behavior in that context, not worker command networking. See [boundary observations](worker-observations.md) and [default probe](command-sockets-default.json).

## Measured results

| Check | Actual outcome | Evidence |
| --- | --- | --- |
| Tool discovery | 23 schemas; six distinct tools invoked, five without MCP errors and run_experiment with errors | [Schemas](tool-schemas.json), [calls](tool-calls.jsonl) |
| HP3 preflight f2b3dcc8b52644d1a9cbd9bf3746df8d | Passed in 0.818409 s supervisor time; firmware, live anchors, collector, recorder and services passed | [Preflight](research_preflight.json) |
| Preflight RRD readback | 67,028 bytes, six data chunks, six rows readable | [Independent readback](preflight-rrd-readback.json) |
| research-services ce13fb80cdbd4bbcac28f0fbd234768f | 172 passed, three failed, 14 deselected in 68.70 s pytest time; job wall 70.486 s; exit 1 | [JUnit](native/tests/ce13fb80cdbd4bbcac28f0fbd234768f/junit.xml), [stdout](native/tests/ce13fb80cdbd4bbcac28f0fbd234768f/stdout.log) |
| headless-autocal 46d4d37e13c541a9bf8e8d403c121f66 | 49 passed, two deselected, eight warnings in 32.83 s pytest time; job wall 33.554 s; exit 0 | [JUnit](native/tests/46d4d37e13c541a9bf8e8d403c121f66/junit.xml), [stdout](native/tests/46d4d37e13c541a9bf8e8d403c121f66/stdout.log) |
| Fresh HP3 baseline and motion requests | Both failed before physics, no run IDs or production recordings created | [Baseline](experiment-baseline.json), [motion](experiment-motion.json), [tracebacks](audit-suites.log) |
| Attached-world preservation | World 9fe8decd0a60434ca1e62a5da4971a73 retained identity and step zero | [Before](runtime-initial.json), [after](runtime-after-suites.json) |

## Reproduced defects and proposed fixes

Fresh experiments fail in shared-output worktrees. `experiments.run_directory` delegates to `repo_path`, which resolves `output/research/experiments/<UUID>` through the launcher's approved shared-output symlink. The resolved path is outside the worktree root and is rejected as an input path. The standalone MCP integration test and both direct requests reproduce the failure. No native run starts. Validate generated experiment output under the approved resolved output root while preserving repository input containment. This proposal was not implemented or validated by this assignment.

Warp CPU tests fail because `test_jobs.job_environment` scrubs the environment and does not set a writable Warp cache. Cloud `WARP_CACHE_PATH` is discarded, so `warp.init()` tries read-only `/home/agent/.cache/warp`. Both `test_compiled_fresh_experiment_records_backend` and `test_compiled_session_encoder_barrier_reset_and_cancel` fail there. Warp itself honors `WARP_CACHE_PATH`. Set job-owned writable Warp and XDG caches without inheriting supervisor credentials or arbitrary caller overrides. No physics code change is indicated by these tracebacks.

The worker command permission request caused material workflow friction. Default command EPERM is observed; the stalled grant request's cause remains unknown. Parent-assisted execution completed this audit but is not proof of smooth independent worker attachment. A later trial needs a permitted worker context or a documented attachment transport that uses the already-permitted supervisor without worker command sockets.

## Hypotheses and limits

Setup readiness alone is insufficient in this baseline. Non-moving preflight and the headless suite demonstrate working components, while output and cache defects reject blanket readiness. The unchanged attached world supports isolation for these particular checks. [The abstract](abstract.md) lists each tested hypothesis and outcome.

[Coverage](coverage.json) names every advertised tool. No successful fresh experiment was available for read_experiment, compare_experiments or saved capture_native_context. Collection, full-autocal, configuration, G-code, stepping and cancellation have fixture coverage within named suites but were not independently exercised against this attached world. Browser tools and Viewer MCP were not used because viewer was deliberately none. GPU and calibration accuracy were not tested. No hardware finding is available.

Preserve these baseline failures. Repair the two narrow defects, repeat the exact failing checks, then run a successful production trial with numerical and RRD readback. Worker attachment, Viewer and production collection/autocal lifecycle remain distinct follow-ups. Separate review should assess these scoped tool-health findings.

## References

- [Coverage](coverage.json), [test summary](test-summary.json) and [call log](tool-calls.jsonl) support the measurements above.
- [Notebook](research.md) and [boundary record](worker-observations.md) preserve allocation and interruption details.
- [Research guide](../../../../research/README.md), [architecture](../../../../research/architecture.md) and [cloud workflow](../../../../scripts/cloud/RESEARCH.md) define tested contracts.
- [Prior cloud pilot](../../../../research/cloud-client-pilot/2026-10-24/README.md) used an earlier transport and does not substitute for this baseline.
- [Preflight verification](../../../../research/preflight-tests-verification.md) and [full-autocal verification](../../../../research/autocal-jobs-verification.md) informed test selection; historical results are not new measurements here.
