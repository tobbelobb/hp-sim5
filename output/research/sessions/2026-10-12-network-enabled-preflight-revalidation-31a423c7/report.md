# Network-enabled preflight revalidation and lifecycle fixes

Status: complete.

## Abstract

After command networking was enabled, socket creation, loopback binding and HTTP all passed in the actual command context in 0.013 seconds. Further testing reproduced two bugs: preflight accepted Cartesian firmware using cached startup identity, and detached-service cleanup missed a grandchild after its group leader was reaped. The fixes check live read-only firmware kinematics/anchors and verify surviving process ownership with private job markers. Final direct regressions passed 165 tests (16 deselected), plus three live-launcher tests. Named supervisor suites passed 26 and 138 tests through a deliberately socket-denied SDK stdio client. Healthy MCP preflight took 0.738 seconds, CLI preflight 0.840 seconds. Real graceful and signal shutdown retired services from running test jobs. The attached HP4 world stayed at step zero and retained its recording. This is Linux tooling evidence, not hardware or calibration-accuracy validation; a live Codex research MCP attachment remains unavailable.

## Topics

preflight, networking, firmware compatibility, anchors, process groups, reaped leader, test jobs, recorder, shutdown

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Actual command socket/bind/HTTP now work | command-network | Supported | [Probe](command-network.json) |
| Cached startup identity detects live incompatible firmware | firmware-reproduction | Rejected before fix | [Cartesian while preflight passed](firmware-mismatch-reproduction.json) |
| Cleanup retires grandchildren after detached leader is reaped | reaped-leader | Rejected before fix; supported after fix | [Failed trial](failed-cleanup-reproduction.txt), [fixed regression JUnit](direct-regressions.xml) |
| Live kinematics/anchor drift is detected without world movement | live-mcp-preflight | Supported after fix | [Regression JUnit](direct-regressions.xml), [supervised research job](research-services-result.json) |
| Graceful and signal shutdown stop test-owned socket services | live-supervisor-shutdown | Supported | [Regression JUnit](direct-regressions.xml) |
| A stale registry can retire an unrelated live group | stale-registry | Rejected | [Regression JUnit](stale-registry.xml) |
| Updated suites remain available from a socket-denied SDK client | 6c2742ced4524fc3b19ed1e47053fd2a, 3afc3c2920024414ba40b3c9bc4fbf29 | Supported | [Headless job](headless-autocal-result.json), [research job](research-services-result.json) |

## References

- [Notebook](research.md): objectives, constraints and decisions.
- [Full report](report.md): exact commands, outcomes and limitations.
- [Previous validation](../2026-10-12-preflight-and-supervised-integration-tests-718724fb/report.md): baseline evidence and archived pre-fix source.
- [Acceptance harness](verify.py): real supervisor/MCP, reuses the existing seccomp client restriction.
- [Source hashes](source-hashes.json): snapshots of every changed/new file under test.
- [Feature prompt](../../../../research/prompts/tooling_improvement_3.txt): original requirements.

## Results and reproduction

Command-context probe: all three checks passed, 0.012779 seconds; no permission escalation was needed. This supersedes the previous conversation's measured EPERM in its earlier policy. The supervisor still correctly reports command networking unknown because it does not run commands in the agent context; the separate probe establishes the current capability.

Direct regression command:

```bash
.venv/bin/python -m pytest -q tests/python/test_research_agent.py tests/python/test_research_preflight.py tests/python/test_research_test_jobs.py tests/python/test_native_collector.py tests/python/test_world_configuration.py tests/python/test_autocal_jobs.py autocal/tests/test_headless_sim.py tests/python/test_hangprinter_flight_recorder.py --junitxml=output/research/sessions/2026-10-12-network-enabled-preflight-revalidation-31a423c7/direct-regressions.xml
```

Result: 165 passed, 16 deselected in 74.29 seconds. The initial unchanged baseline passed 156 tests (16 deselected) in 50.68 seconds; nine new meaningful regressions cover firmware drift/malformed values, reaped leaders, stale ownership and real graceful/signal shutdown.

Live launcher tests:

```bash
.venv/bin/python -m pytest -q -m slow tests/python/test_research_agent.py --junitxml=output/research/sessions/2026-10-12-network-enabled-preflight-revalidation-31a423c7/launcher-live.xml
```

Result: 3 passed, 36 deselected in 3.91 seconds. These are real launcher/runtime lifecycle checks, not physical full-auto smoke tests.

Supervised MCP validation:

```bash
.venv/bin/python output/research/sessions/2026-10-12-network-enabled-preflight-revalidation-31a423c7/verify.py
.venv/bin/python output/research/sessions/2026-10-12-network-enabled-preflight-revalidation-31a423c7/verify.py --cli-only
```

The first runs both fixed suites through MCP after deliberately denying socket/connect syscalls in the SDK client. Headless job: 26 passed, 2 deselected in 20.74 seconds; supervisor wall time 21.378 seconds, exit 0. Research job: 138 passed, 14 deselected in 63.95 seconds; wall time 64.873 seconds, exit 0. The last stale-registry test was added afterward and included in the final direct 165-test run. The MCP harness also exercised invalid suites, expected-settings mismatch, real cancellation/timeout and disconnected supervisor outcomes, preserving artifacts. The second validates the CLI's non-moving, no-Viewer/no-Codex route. Its default Viewer setting was left unchanged to verify preflight skips Viewer startup explicitly.

## Reproduced bugs and fixes

Before the fix, a fresh isolated HP4 supervisor passed preflight. Sending only `M669 K0` changed firmware to Cartesian; a read-only object-model query confirmed it, yet preflight still passed. The world never advanced beyond zero. Evidence remains in firmware-mismatch-reproduction.json. Live preflight now reads state plus move.kinematics and compares its anchor coordinates with the collector's startup configuration, allowing 0.1 mm rounding. Regression coverage uses actual MCP calls for both changed anchors and wrong kinematics, and synthetic malformed object-model responses for validation boundaries. It never resets or moves the tested attached world. The initial implementation of this fix compared a lowercase name literally; real RRF returns capitalized Hangprinter for the full kinematics object. The focused live test caught that and name matching now accepts case differences.

The detached-child regression started an isolated detached group, registered its leader, reaped that leader, and left a grandchild running. The old cleanup marked the parent job passed but could not find ownership through the missing leader PID. The test failed (1 failed, 20 deselected in 1.60 seconds), and its finally block retired the known isolated group. Cleanup now finds live members whose environment contains the private matching registry marker. It checks original PID birth identity when available, revalidates before each signal, and preserves markers in test-owned autocal children. The fixed test, stubborn-child tests and actual supervisor shutdown tests pass. A deliberately stale registry pointing at an unrelated process group leaves that process alive.

## Limits

The main validation attachment stayed at step zero with world ID and RRD path unchanged. Test fixtures can move their own isolated worlds. No doctor, physical full-auto/calibration smoke, hardware measurement or accuracy test was run. Firmware spool settings remain represented by accepted generation metadata and cached startup identity; the new live object-model check verifies kinematics and anchors, not every possible mutable RRF parameter. Process ownership uses Linux /proc and POSIX groups; other platforms were not tested. Current command networking is verified; a native research connector in this Codex conversation remains unavailable, so SDK stdio access should not be described as a live Codex attachment. No required checks are blocked.
