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
