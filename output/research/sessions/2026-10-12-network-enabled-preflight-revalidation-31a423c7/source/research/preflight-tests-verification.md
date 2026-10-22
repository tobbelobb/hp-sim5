# Preflight and supervised integration-test verification

Revalidated after command networking was enabled on 2026-10-12. Socket creation,
loopback binding and HTTP round trip now all pass in the actual command context
(0.013 seconds). The [follow-up report](../output/research/sessions/2026-10-12-network-enabled-preflight-revalidation-31a423c7/report.md)
preserves the reproductions, fixes, JUnit results and current evidence.

Two bugs were reproduced and fixed:

- Cached startup firmware identity let preflight pass after firmware changed to
  Cartesian kinematics. Preflight now reads live kinematics/anchors through the
  read-only object model and blocks incompatible modes or changed anchors. Anchor
  comparison allows 0.1 mm for firmware object-model rounding.
- Cleanup skipped a detached group after its leader was reaped, leaving a
  grandchild alive. Cleanup now verifies surviving members using private job
  markers, rechecks ownership before each signal, and propagates the ownership
  marker through test-owned autocal processes. A stale registry does not stop an
  unrelated live group.

Final direct regressions: **165 passed, 16 deselected in 74.29 seconds**, plus
**3 live launcher tests passed, 36 deselected in 3.91 seconds**. Real graceful and
signal supervisor shutdown tests retire services created by running socket tests
and preserve the attached world's step zero. Both named suites also passed
through a deliberately socket-denied SDK MCP client: 26 headless/recorder tests
and 138 research-service tests. The additional stale-registry regression passed
directly after that supervised run. MCP preflight took 0.738 seconds and CLI
preflight took 0.840 seconds; the attached world and recording remained unchanged.
No required checks remain blocked by command networking. This chat still has no
research MCP connector; SDK stdio tests do not establish a live Codex attachment.

## Initial verification before command networking was enabled

Validated on Linux, 2026-10-12. Implementation request:
[tooling improvement 3](prompts/tooling_improvement_3.txt).
The [session report](../output/research/sessions/2026-10-12-preflight-and-supervised-integration-tests-718724fb/report.md)
contains the exact commands, job results, failed trials and limitations.

The final non-moving MCP preflight took **0.737 seconds**; supervisor startup took
0.755 seconds separately. Recorder configuration/acknowledgment, finalization and
RRD readback passed using an isolated step-zero `tool-check-preflight-*` sample.
CLI `--preflight` passed in 0.736 seconds, with no Viewer/Codex startup and world
step zero. Normal launcher startup still performs no preflight or doctor.

| Supervisor suite | Exact pytest result | Job wall time | Exit |
| --- | --- | --- | --- |
| `headless-autocal` | 26 passed, 2 deselected in 20.17s | 20.694s | 0 |
| `research-services` | 130 passed, 14 deselected in 37.42s | 38.263s | 0 |

Both named jobs returned over MCP in about 6 ms. They used fixed argv,
repository cwd, `.venv/bin/python`, isolated pytest/JUnit/log artifacts, and
excluded slow physical/full-auto smoke tests. Research-service coverage includes
real MCP preflight, cancellation, timeout and disconnection, plus unit tests for
unhealthy services, incompatible settings, recorder failure, invalid suites,
shutdown, owner exit and stubborn/detached child cleanup.

The actual command sandbox denied socket creation with EPERM. A separate real
SDK stdio client was restricted by Linux seccomp after its MCP child started;
socket/connect syscalls remained denied while preflight and both supervised
test suites succeeded. The supervisor correctly reported command networking
`unknown`: its successful requests do not prove command socket access. Launcher
tests retained the enabled default and explicit `network_access=false` override.
No research MCP connector was attached to this Codex chat, so an actual Codex
conversation under that override was not exercised.

MCP rejected an invalid suite, blocked mismatched HP3 settings on the HP4
attachment, and returned an actionable blocked result after supervisor shutdown.
An explicit cancel returned `cancelled` with exit -15 and preserved logs. A
one-second timeout returned `failed`, stop reason `timeout`, exit -15, after
1.259 seconds including cleanup. The attached HP4 world ID, step zero and live
recording path were unchanged after these checks.

Three development failures are preserved in the session: long `TMPDIR` paths
exceeded RRF's VSD filename limit, catalog assertions assumed directly indexed
sessions, and nested test jobs initially inherited their parent's detached-service
registry. The final implementation uses short temporary VSD paths, keeps pytest
artifacts inside each job, and gives every job its own registry. Parent cleanup
also visits nested registries; `/proc` birth identities guard against stale PIDs.
Normal services retain their existing process groups and behavior.

All required supervised checks passed. Command-context binding and HTTP remained
untested after socket creation was blocked. Process ownership was validated on
Linux with POSIX groups and `/proc`; other platforms were not tested. No doctor,
slow movement/full-auto smoke, calibration-accuracy or hardware experiment was
run. These are tool-health results.
