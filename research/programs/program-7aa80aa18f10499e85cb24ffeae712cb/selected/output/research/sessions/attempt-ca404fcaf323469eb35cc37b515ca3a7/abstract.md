# HP3 cloud tools reveal shared-output and isolated-cache defects

Status: complete baseline audit. Independent review pending.

## Abstract

This audit asked whether current cloud research tools operate smoothly after runtime setup passes. The real worker discovered 23 advertised tools through its owned HP3 Rust-native supervisor. Worker-authored MCP probes ran in the parent's permitted command context after the worker's permission-bearing call stalled and its default socket probe returned EPERM. Non-moving preflight passed in 0.818 seconds and its finalized 67,028-byte RRD was read back. The headless-autocal suite passed 49 tests. Research-services passed 172 tests and failed three, exposing two defects. Shared worktree output paths are rejected by fresh-experiment containment checks, and isolated test jobs discard the writable Warp cache setting. Both direct fresh-experiment requests failed before physics began. The attached world retained identity and step zero. No collection, fit, calibration-accuracy or hardware result was produced. Passing setup is insufficient to claim all tools work optimally.

## Topics

Cloud research, HP3, Research Program, worktree, shared output, Warp cache, local sockets, MCP, preflight, Rerun, integration tests.

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Setup readiness suffices for smooth cloud worker tools | This attempt | Rejected in this baseline | [Coverage](coverage.json), [context boundary](worker-observations.md) |
| Fresh native experiments work from the shared-output worktree | Two failed requests, no run IDs created | Rejected | [Baseline request](experiment-baseline.json), [motion request](experiment-motion.json), [traceback](audit-suites.log) |
| Isolated research-services jobs support installed Warp CPU | ce13fb80cdbd4bbcac28f0fbd234768f | Rejected in this environment | [Failure summary](test-summary.json) |
| Preflight and isolated suites preserve the attached world | f2b3dcc8b52644d1a9cbd9bf3746df8d and both suite jobs | Supported | [Initial](runtime-initial.json), [final](runtime-after-suites.json) |
| Headless-autocal suite completes in this runtime | 46d4d37e13c541a9bf8e8d403c121f66 | Supported as tool health | [JUnit](native/tests/46d4d37e13c541a9bf8e8d403c121f66/junit.xml) |
| All tools are optimal or calibration is accurate | None | Not tested | [Coverage limits](coverage.json) |

## Decision handoff

Scope is source 67617705b7996ee3a84889e6723cab149b6e5884 on this cloud instance, HP3 layered Rust-native, plus isolated test worlds. The worker inspected raw evidence and independently read the preflight RRD; separate review is pending. Repair generated-output validation and job-local caches, then repeat failures and a successful fresh trial. Worker attachment, Viewer and production collection/autocal still need live trials.

## References

- [Report](report.md), [notebook](research.md) and [original task](prompt.txt).
- [Coverage matrix](coverage.json) lists all 23 tools and actual call coverage.
- [Test summary](test-summary.json) preserves JUnit identities and failure details.
- [Prior cloud pilot](../../../../research/cloud-client-pilot/2026-10-24/README.md) explains why current-transport verification was needed.
