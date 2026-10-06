# Research architecture verification

Verified on 6 October 2026 with the repository's `.venv/bin/python`, Codex CLI
0.160.1 and Rerun 0.38.1. The implementation follows
[the architecture investigation](../research-agent-architecture.md).

The default launcher now gives the terminal to interactive Codex. `--batch`
retains the JSONL `codex exec` route; `--serve` supplies private, explicit MCP
attachment for a desktop/project session. Both MCP entry routes load the research
instructions. Long collection uses start/status/cancel jobs. The existing browser
application exposes its controllers through local MCP and optional WebMCP, and
its requested flight recorder belongs to the supervisor.

## Live conversation and resume

Codex conversation `01a111ca-bbc6-7540-a55b-4d59a951a40f` ran a baseline and then
accepted steering for the next trial in the same conversation:

| Trial | Run ID | Inputs |
| --- | --- | --- |
| Baseline | `aeb62bbdbdda4527978599f294bb852d` | Two steps, hold targets, no recording |
| Steered | `165d697d3d5f4e18b62065fa65ec5703` | Three steps, absolute A target 0.001 rad, no recording |

The [session research record](../output/research/architecture-interactive-3/research.md)
retains both IDs and their numeric telemetry. This is an interface verification,
not an optimization comparison: duration, command schedule and source hashes
differ. Resuming that conversation through the launcher retained both IDs and
returned a fresh native world at step zero with an empty queue. The resumed
session and batch session both exited with status zero. The
[batch report](../output/research/architecture-batch/report.md) records a live
capabilities/runtime-status round trip.

Attachment tests exercised the exact wrapper named in generated project MCP
configuration: reconnect retains a live world, another connection is refused,
coordinated reset changes the world identity, and an ended supervisor fails
attachment. The project configuration snippet is generated without modifying
user configuration. Desktop window interaction itself was not automated.

## Cancellation and complete collection

The final cancellation test stopped run
`4a06d737b90a42728b29449c4246c232` before physics step 136, at step 135 / 0.27 s.
Its [manifest](../output/research/architecture-cancellation-tests/test_live_collection_job_cance0/8124a29de2e0436d887645c8b38f21e7/4a06d737b90a42728b29449c4246c232/manifest.json)
records both the native timestep boundary and collector command boundary.
The test verified unchanged subsequent step count, empty native queue, retired
RRF/bridge processes, finalized RRD/event evidence, and an explicit reset
requirement. Reset then produced a usable world at step zero.

The full production collection passed twice. The final check used the job API
and took 840.72 seconds. It verified terminal `complete` status, six validated
version-2 measurements, six journaled raw-angle points with their physical roles,
finite autocal residual evaluation, continued session use, coordinated reset and
process cleanup. Pytest's temporary collection artifacts were automatically
cleaned; the retained cancellation and interactive/browser artifacts above are
separate durable evidence. These checks establish execution and lifecycle,
not calibration parameter accuracy.

## Shared browser and Rerun

The [browser evidence](../output/research/architecture-validation/browser-latest-evidence.json)
records motor commands, exact bounded stepping, an immutable context at step 20,
subsequent motion to step 30, loading HP3 through the existing scene controller,
and rejecting an unknown scene without clearing the world. Native physics
remained at step zero. The user-facing “Capture for research” action also stored
the submitted words and entity with the page, generation and time. Direct
commands reject invalid batches before enqueueing any member.

The final browser recording ID was `260a12ac86e247a0ade8d62ad9ba8b9e`, for browser
session `84066f74-17f2-4b17-af84-a9dfdb4b6b70`, scene generation 3. Its finalized
RRD spans steps 0–30. The supervisor's assigned Viewer received that same ID,
selected the captured `sim_step=20`, and saved a
[screenshot](../output/research/architecture-viewer-final/selected-step-20.png).
The [context/recording mapping](../output/research/architecture-viewer-final/context-evidence.json)
and [Viewer state](../output/research/architecture-viewer-final/viewer-selected.json)
prove the recording and selected time match. Native selections are captured
with an explicit run/session and observed step; an unavailable historical step
is rejected rather than approximated.

This Chromium instance did not expose WebMCP. Feature detection left the normal
page working, and local MCP controlled the same page. The optional registration
was exercised against its documented API in a focused test. Availability in a
particular desktop client remains dependent on that client's rollout.

Vite initially exhausted file watchers on generated autocal data. The watch
configuration now excludes generated output, virtual environments and firmware
trees. The final browser remained live. A regression test deliberately terminated
its owned Vite process: native stepping remained usable and restarting Vite
retained native session identity.

## Checks

All checks passed for the changed paths:

- Research, native session and browser receiver Python checks: 25 fast tests.
- Launcher lifecycle: three slow tests, including real service startup and a
  clean service-only shutdown.
- Native integration: real firmware/MCP/browser startup, live Viewer delivery
  and seeking, exclusive attachment/reconnect, cooperative cancellation, and
  full job completion.
- Native recording lifecycle and JS/Python machine pipeline/lifecycle parity:
  28 checks in the selected group (including receiver checks).
- Browser 3D and production sweep collector JavaScript suites: 137 tests.
- `git diff --check` and Python compilation.

All services and browser sessions created for live verification were stopped.
No custom chat client, outer Codex continuation loop or second simulator was
introduced. Research setup and migration commands are in
[the research README](README.md).
