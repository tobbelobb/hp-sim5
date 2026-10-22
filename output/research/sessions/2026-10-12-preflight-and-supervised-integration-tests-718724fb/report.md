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

## Measured tool checks

Preflight: 0.737 s excluding supervisor startup (0.755 s) and shutdown. CLI preflight: 0.736 s. Every recorder probe has a unique isolated directory and a step-zero empty sample labeled tool-check-preflight; no attached-world commands are sent.

| Suite | Job ID | Exact pytest result | Supervisor wall time | Exit |
| --- | --- | --- | --- | --- |
| headless-autocal | `6d55ab3885944680b1450857a0e33ca2` | `26 passed, 2 deselected in 20.17s` | 20.694 s | 0 |
| research-services | `6b1a474b446547839d1f457f997eeb2a` | `130 passed, 14 deselected in 37.42s` | 38.263 s | 0 |

Both suites ran `.venv/bin/python -m pytest -q -m 'not slow'` with the fixed paths listed in `capabilities()`, repository cwd, per-job `--basetemp` and `--junitxml`. Complete argv and stdout/stderr are in each linked result. They include the real live MCP preflight/cancel/timeout/disconnected acceptance test and detached-service cleanup regressions.

Reproduction command from repository root:

```bash
.venv/bin/python output/research/sessions/2026-10-12-preflight-and-supervised-integration-tests-718724fb/verify.py
.venv/bin/python output/research/sessions/2026-10-12-preflight-and-supervised-integration-tests-718724fb/verify.py --cli-only
.venv/bin/python scripts/research_socket_probe.py
```

The last command was run in the actual command sandbox and returned exit 1 with EPERM on socket creation. The supervisor validation required the approved supervisor execution context. It launches one normal runtime; named tests run over its MCP tools. Its SDK client is restricted with seccomp after MCP startup, so command sockets are denied independently of supervisor/MCP sockets. There is no general shell-execution MCP tool.

## Failed development trials

- [Initial research suite](initial-research-services-result.json): 4 failed, 123 passed, 14 deselected. Long TMPDIR caused RRF firmware path truncation; catalog tests assumed outside-artifact placement. Use default short temporary VSD paths and isolated pytest artifact directories.
- [Second research suite](second-research-services-result.json): 2 failed, 125 passed, 15 deselected. The catalog excludes nested test directories by design; test now checks the correct catalog root and only expects directly cataloged sessions.
- [Third research suite](third-research-services-result.json): 2 failed, 128 passed, 14 deselected. Nested jobs inherited their parent's process registry and could not immediately retire detached children. Each job now owns its registry; parent cleanup also visits nested registries.

All three failed jobs, stdout/stderr, JUnit, inputs, partial recordings and manifests remain under `native/tests/`. The final job passed all 130 research-service tests. No live jobs or evidence were deleted to retry.

## Boundaries

Supervisor checks report command networking unknown even when all supervised checks pass. Firmware health reads a bounded object-model endpoint and the collector's initial firmware identity; preflight checks the accepted world configuration and source freshness, without physical accuracy/settling collection. Process ownership uses Linux `/proc` birth identities and POSIX process groups; other OSes were not validated. Shutdown/cancel unit tests and real MCP lifecycle checks cover cleanup; partial cancelled test recordings can remain incomplete and are not accepted calibration evidence.

No doctor, slow physical/full-auto smoke, hardware measurement or calibration-accuracy experiment was run. The tested SDK restriction is not a live Codex chat with a managed networking override; this chat has no research MCP connector. Launcher dry-run tests retained enabled networking defaults and the explicit false override. Required supervised checks passed. Command-context socket creation was blocked (EPERM), binding/HTTP remain unknown, and a live Codex managed-policy attachment was not available.
