# Preflight and supervised integration tests with command sockets denied

Status: complete.

## Abstract

Can agents check recording/connectivity and run socket-dependent integration tests without command networking? Implemented bounded non-moving preflight and allowlisted supervisor test jobs using the existing runtime. A real SDK stdio MCP client denied socket/connect syscalls by Linux seccomp completed preflight in 0.737 seconds, including an isolated recorder acknowledgment, finalization and readable RRD. The final headless/recorder suite passed 26 tests and research-services passed 130 tests. Invalid suites, mismatched settings, disconnected services, cancellation, one-second timeout and detached-child retirement were checked; three failed development trials remain available. The attached HP4 world stayed at step zero with its recording unchanged. The command sandbox itself denied socket creation with EPERM; its HTTP capability remains unknown. CLI preflight passed in 0.736 seconds. This validates tooling on Linux, not calibration accuracy or hardware behavior. A live Codex chat attachment with network_access=false was not available; the socket-denied SDK path and launcher override tests were exercised.

## Topics

preflight, supervisor, socket policy, integration tests, recorder finalization, autocal, cancellation, process groups

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Agent command context permits localhost sockets | command-network | Rejected | [Actual probe](command-network.json) |
| Supervisor preflight works with client socket calls denied | b042c5e15bed4c5a8a4506afde7e76a2 | Supported | [Preflight](mcp-preflight.json), [denied client](restricted-client-network.json) |
| Fixed suites pass through supervised MCP jobs | 6d55ab3885944680b1450857a0e33ca2, 6b1a474b446547839d1f457f997eeb2a | Supported | [Headless suite](headless-autocal-result.json), [research suite](research-services-result.json) |
| Cancel/timeout/disconnection report clear outcomes and preserve the attached world | b89593a989274378ba10f5f5445d9d38, 5c810c59adb24cccb521e90541cac51b | Supported | [Cancellation](mcp-cancelled.json), [timeout](mcp-timeout.json), [disconnection](disconnected-supervisor.json), [summary](acceptance.json) |
| CLI preflight leaves step zero without Viewer or Codex startup | cli-preflight | Supported | [CLI result](cli-preflight.stdout.json) |
| Tool success establishes calibration accuracy | none | Not tested | No accuracy experiment requested |

## References

- [Report](report.md): exact jobs, test results, failure history and limits.
- [Notebook](research.md): objective, budgets and decisions.
- [Acceptance harness](verify.py): uses real supervisor and MCP, not a replay or synthetic collector.
- [Implementation prompt](../../../../research/prompts/tooling_improvement_3.txt): requirements.
- [Previous socket-denied client validation](../2026-10-10-supervised-full-autocal/verify.py): reused Linux seccomp restriction helper.
- Base repository revision `06f190dde4626998f21e75c7634797641fa48fc2` plus this working tree: code under test; [working diff](working.diff) records tracked changes; [source hashes and snapshots](source-hashes.json) preserve every changed/new source file.
