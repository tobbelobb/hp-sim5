# Cloud relay enables production trials but exposes two remaining tool defects

Status: complete bounded validation, independent review pending.

## Abstract

This actual cloud actor tested source 379318b2 through the launcher-owned MCP file relay while its own socket creation remained denied. All 23 advertised tools were invoked. Three fresh Rust-native experiments completed in the shared-output worktree; the recorded and unrecorded 500-step pair had byte-identical 501-row telemetry. Research-services passed 176 tests. HP3 configuration, preflight, six-point collection, collection cancellation, real Rerun inspection and browser pause/50-step/context capture worked. The one-iteration full-autocal CLI produced 24 measurements and stopped at its budget after 1,305,459 physics steps. Its finalized RRD was readable. Two defects remain. Browser interventions returns a list rejected by the MCP dictionary schema. Infinite autocal progress makes strict status serialization fail and leaves a stale active job, blocking recovery reset and a second isolated cancellation trial. These are simulated tool-health results, not calibration accuracy or hardware evidence.

## Topics

Cloud research, MCP file relay, denied command sockets, HP3, HP4, Rust-native physics, Rerun Viewer, shared-output worktrees, Warp cache, full-autocal budgets, cancellation, browser context, nonfinite status serialization.

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

[Source excerpts](source-evidence.txt) identify the tested return/serialization contracts. [Raw Infinity row](autocal-infinity-row.jsonl) preserves the actual nonstandard score record without normalization.
