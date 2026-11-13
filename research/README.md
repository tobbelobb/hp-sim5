# Hangprinter research tools

Use hp-sim5's native physics, production firmware/collector, autocalibration, and
Rerun evidence from an ordinary coding conversation. The runtime starts with no
physics world; configure_session creates one when the researcher needs it.

The [research-program guide](../research-program/README.md) describes programs,
parallel researchers, private runtime state, explicit resume, and worktrees.
The [architecture map](architecture.md) explains physics and recording ownership.

## Setup and interactive use

```bash
.venv/bin/python -m pip install -r requirements.txt
npm run build:native
codex login                            # or: claude auth login
./hp-sim5-research-agent --codex --prompt 'Investigate calibration reliability'
./hp-sim5-research-agent --claude --worktree --prompt-file research/prompts/autocal.txt
./hp-sim5-research-agent --resume INVESTIGATION_ID
```

The launcher opens the selected ordinary client and owns its private tools until
exit. Ctrl-C interrupts an interactive client turn; explicitly cancel experiment
jobs to stop them. Goal mode is enabled by default; use --no-goal-mode to disable it.
Use --dont-ask to disable structured clarification questions. Forward additional
client flags after --. --model works with either backend; --model-provider is
Codex-only. Authentication uses existing client credentials. OpenAI API keys are
removed for the default ChatGPT sign-in route. Claude never loads an API key from
the repository .env; explicitly exported keys retain normal CLI billing behaviour.

--resume reuses the investigation's artifact directory. With no value it reopens
the last investigation on this host. Backend conversation IDs are separate: Claude
and batch Codex retain them; interactive Codex offers its native picker when no ID
is bound. Use --resume DIRECTORY --conversation CLIENT_ID to bind an old conversation.
The previous live physics state is not restored after shutdown. Existing evidence
is retained and a new world generation records its own provenance.

Opening and closing a client writes only ignored .research-state metadata and logs.
No placeholder abstract, empty evidence directory, or catalog entry is published.
Reports and original experiment evidence go into the assigned stable artifact path.
For corpus discovery and publishing, use the
[research-corpus skill](../.agents/skills/research-corpus/SKILL.md).

--worktree creates a checkout from the current commit, links ignored dependencies,
and keeps tracked research as ordinary files. New evidence belongs to that worktree.
Resume returns to it. For a T3-managed child, let T3 create its workspace and use
--resume INVESTIGATION_ID --workspace to bind the prepared handoff. Do not create
an additional worktree inside that child.

## Tools in an existing T3 or hosted conversation

No nested client is needed. Prepare an investigation and start tools when useful:

```bash
./hp-sim5-research-agent --prompt 'Investigate the observed failure'
./hp-sim5-research-agent --resume INVESTIGATION_ID --start-runtime
.venv/bin/python scripts/research_tools.py list TOOL_CONNECTION
.venv/bin/python scripts/research_tools.py call TOOL_CONNECTION configure_session --params '{"configuration":{"machine":"hp3"}}'
.venv/bin/python scripts/research_tools.py call TOOL_CONNECTION runtime_status
./hp-sim5-research-agent --resume INVESTIGATION_ID --stop-runtime
```

TOOL_CONNECTION is the returned tool_connection path. File calls expose the real
MCP tools without requiring command sockets. Each researcher owns its own runtime.
Add --viewer headless or --viewer window when recording inspection is useful;
--no-record skips Rerun logging. --serve keeps tools in the foreground instead of
starting a detached supervisor. Stop detached tools explicitly before finishing;
platform cancellation alone does not confirm service shutdown.

See [platforms.md](../research-program/platforms.md) for cloud setup and socket
constraints. Large recordings must reach durable storage before a cloud workspace
is disposed. The plain program and corpus remain portable without live credentials.

## Batch mode and diagnostics

```bash
./hp-sim5-research-agent --codex --batch --prompt-file research/prompts/autocal.txt
./hp-sim5-research-agent --codex --dry-run --prompt 'Inspect the tools'
./hp-sim5-research-agent --codex --doctor
./hp-sim5-research-agent --preflight
```

Batch client events, prompts, final messages, and exit observations stay in private
launcher state. --dry-run creates no files. --doctor explicitly checks dependencies,
client authentication, and real physical collection; --preflight performs bounded
non-moving tool checks. Neither establishes scientific success. Read the returned
tool schema for the precise units, configuration, and supported operations.

## Experiment tools

| MCP tool | Result |
| --- | --- |
| `capabilities()` | Scenes, command units, limits and known gaps |
| `run_experiment(scene, steps, dt, commands, commands_file, label, record)` | A fresh physics run, metrics, warnings and absolute artifact paths |
| `read_experiment(run_id)` | Provenance and summary |
| `read_experiment(run_id, start_step, limit)` | A page of numeric samples, up to 100 |
| `compare_experiments(baseline_id, candidate_id)` | Changed inputs, metric differences and final effector distances |
| `runtime_status()` | Persistent world, clock, queues, service PIDs/logs and stale source detection |
| `research_preflight(expected_configuration=None)` | Bounded non-moving service/settings/firmware/collector/recorder check; separate command-network result remains unknown |
| `start_test(suite, timeout_s=300)` | Immediate job ID for a fixed integration suite on the supervisor |
| `test_status(job_id)` | Test progress, pass/fail/blocked outcome, exit code, log tails and isolated artifacts |
| `cancel_test(job_id)` | Retire test runner and descendants, preserving partial evidence |
| `send_gcode(line)` | One line through real RRF planning into the continuing native world |
| `step_physics(steps)` | Advance the continuing world by fixed steps |
| `start_collection(configs, options, settling_timeout_s)` | Start a collection job and return its job ID immediately |
| `collection_status(job_id)` | Running/cancelling/complete/cancelled/failed status and evidence |
| `cancel_collection(job_id)` | Known physics stop boundary, retired firmware/bridge queues, partial evidence and reset requirement |
| `start_autocal(machine, base_radii_mm, buildup_factor, find_radii, find_buildup_factor, optimizer, budgets, collection, artifact_dir, recording_mode)` | Immediate job ID for an isolated world running the production full-auto CLI |
| `autocal_status(job_id)` | Physical progress, iteration/decision, stop reason, CLI exit status and complete/partial artifacts |
| `cancel_autocal(job_id)` | Stop the job-owned world, drain recording and retire all CLI children; attached world remains usable |
| `configure_session(configuration)` | Validate settings, archive the old world and create a fresh HP3/HP4 world with matched firmware and encoder origin |
| `reset_session()` | Archive state and restart world, firmware and bridge references with the accepted configuration |
| `start_browser_service(record=False)` | Exact shared browser-js URL; optionally owns its flight recorder |
| `browser_status()` | Connected page identity and immutable user-submitted context |
| `browser_action(page_id, action, args)` | Controls and numerical observations from that exact existing page |
| `capture_native_context(message, step, selected_entity, run_id)` | Selected native run/session and time with actual numerical observation |

A launcher or desktop attachment starts with no world. `configure_session`
builds the first one, and later calls replace it without restarting Codex or
writing a simulator adapter. Native physics checks and any required rebuild are
deferred until a world is requested. For example:

```json
{"configuration": {"machine": "hp3", "base_radii_mm": [30], "buildup_factor": 0}}
```

`machine` is a canonical name or nickname from
[`machines/catalog.json`](../machines/catalog.json). The catalogue entry selects
the scene, RRF configuration and line layering together (for example `hp3` and
`hp3_w_line_layers`). It is required for the first world; later calls without it
keep the current machine. Other settings are a complete fresh-world request:
stock 30 mm base radii, and stock buildup (2/pi) on line-layering machines. One
radius applies to all four spools; four values select A/B/C/D individually
(1–200 mm). Custom radii require a machine without line layering and zero
buildup. Machines whose scene or firmware is not a four-anchor Hangprinter are
rejected explicitly. The backend remains the launcher's selected backend.

Layering affects the bake, physical cable half-width and all three runtime
layering flags. The selected firmware is frozen in each generation and booted
from a short private VSD (RRF limits host filename length). `runtime_status`
and world/collection manifests report requested/effective settings, authored,
design and baked scene hashes, firmware configuration/replies and generation
identity. The collector and residual validation use the selected scene's catalogued
machine type, including `hangprinter_3` for HP3.

The MCP experiment tool requires an explicit scene. The standalone Python
experiment API defaults to HP3; this does not choose a machine for the attached world.

An ordinary `reset_session()` retains accepted settings and reloads scene, JS
and firmware sources. Python edits require a launcher restart, as status reports.
Cancel an active collection or full-autocal job explicitly before reset or
configuration. Validation and backend-preparation failures keep the live world
and its evidence intact; failed attempts have separate manifests. If firmware
startup fails after retirement, status remains available and reports
`reset_required`; reset retries the last accepted configuration. Old recordings,
events, scenes and collection results remain in their generation directory. The
assigned session path, desktop attachment and conversation remain stable.

The API uses the attached MCP server's supervised service connection. It also
works when command sandbox networking is disabled; launcher networking defaults
remain enabled. Isolated `start_autocal` settings still select its own CLI world;
its radii/buildup arguments are fitting seeds, while `configure_session` radii
specify physical spool geometry. Shared validation checks their common bounds
and line-layering constraints.

Runs accept 1–10,000 steps. Commands are a JSON array with at most one record per
step. Omitted records keep the previous targets for the remaining steps. `Move`
sets absolute motor angles in radians; `{}` consumes a timestep while holding
targets. Example:

```json
[{"type": "Move", "A": 0.0003}, {}, {"type": "Move", "A": 0.0006}]
```

`SetTorqueMode` takes `axis` and `torqueNm`; `SetPositionMode` takes `axis`.
`Add to reference` changes reference angles. `E` is deposited length in metres.
Unknown axes/types and non-finite commands are rejected. `commands_file` reads a
JSON array inside the repository; it cannot be combined with inline commands.
See [native recording recipes](../hp-sim-3d/FLIGHT_RECORDER.md) for motion and
[firmware command scheduling](../tests/benchmark/README.md) for real print logs.

Every run freezes the composed, baked USD scene and command array. Its manifest
records their hashes, the Python source hash, loaded native artifact/build metadata,
Git revision, interpreter,
package versions, timestep, warnings and timing. `telemetry.jsonl` includes
step zero and every physics update: world-space effectors, motor targets and
encoder errors, missed-step peaks, cable lengths and segment forces. `final.json`
contains the final full frame/cable observation. `recording.rrd` contains the
existing every-step native flight recording and blueprint. `record=false`
retains numerical evidence while avoiding Rerun logging work.

`encoder_angle_rad` is the raw unwrapped encoder component, matching the browser
endpoint. `diagnostic_encoder_angle_rad` subtracts the missed-step diagnostic
offset and is used for tracking error; that offset is also reported explicitly.
Older experiment manifests with `schema_version=1` used the diagnostic value for
`encoder_angle_rad`; new experiment manifests use version 2.

Compare identical timelines. Input differences are disclosed, rather than
silently treated as optimization improvements. Metrics describe behavior;
force/error reduction alone is not a calibration success criterion. Timings
include numeric sampling and, when enabled, Rerun logging. Use the existing
physics-only benchmark for throughput claims. Snapshots cannot resume a run.

Rerun inspection uses the [official Viewer MCP](https://rerun.io/docs/reference/viewer/mcp).
The agent opens the saved RRD, reads Viewer warnings, selects simulation time
and captures useful screenshots. Numeric values come from the experiment tools;
Viewer MCP manipulates the UI. For desktop attachment use the supervised configuration above. Fresh numerical
trials also work with the standalone MCP script. MCP uses the official Python
SDK's current 2.x API; see [SDK documentation](https://py.sdk.modelcontextprotocol.io/).

## Jobs, shared browser state and steering

Long collection is asynchronous. Start a job, poll status between conversational
turns, and explicitly cancel it when steering requires movement to stop. A chat
interrupt or Rerun playback pause is not a physics stop. Cancellation cooperates
at the next native fixed-step boundary and before the next collector command,
flushes the completed-point journal, retires RRF/bridge queues, and finalizes
partial RRD/events/manifest evidence. It reports `end_step`, `steps_executed`,
the actual cancellation boundaries and `reset_required=true`. Reset world,
firmware and bridge together before more movement. `partial-points.jsonl`
preserves measured points with their actual drive/sensor configuration; it is
partial evidence, not a complete autocal dataset. Completed jobs retain the
production version-2 dataset and validation path.

Full-autocal jobs use the production one-click CLI once per job. They own a fresh
headless JS scene, RRF, collector and optional recorder independently of the
attached session. A chat can request another machine without changing its current world.
Only one full-autocal job runs per supervisor; collection jobs use the attached
world independently. For known HP3 radii and no layering, call:

```json
{"machine":"hp3", "base_radii_mm":[30], "buildup_factor":0}
```

`machine` is a catalogue name or nickname; omitted, it is the configured world's
machine, and with no world it is required. Radii contain one or four finite
values in [1, 200] mm and default to [30]. `find_radii` and `find_buildup_factor`
accept `off`, `global`, `per-anchor`. Nonzero buildup or buildup fitting requires
a line-layering machine. Buildup is bounded to [0, 2]. The machine's catalogued
firmware config is used, and line layering enables the existing collector's
simulator setting.
Headless construction also uses the browser's matching cable-length bake:
zero cable width when layering is off. Layering changes rebake/reset the world;
resets retain closed-loop motor and layering settings. Each generation keeps
its baked scene and hash in the recording and job artifacts.

`recording_mode` controls evidence cost independently of the attached session's
recording setting. It defaults to `events`:

| Mode | Recorded evidence | Use |
| --- | --- | --- |
| `none` | Numerical datasets, manifests and command/encoder traces; no RRD recorder | Large automated sweeps |
| `events` | G-code, responses, encoder reads, calibration stages and logs in an RRD; no physics snapshots | Most research trials |
| `sampled` | Events plus physics every 100 steps with compact geometry | Investigating behavior |
| `full` | Existing detailed extended RRD: events plus physics every 10 steps with compact geometry | Debugging and validating discoveries |

For example, `start_autocal(recording_mode="none")` avoids physics snapshot,
transport and RRD costs while preserving numerical evidence. All recorded modes
retain lossless batched events, explicit clocks and finalization on cancellation.
Physics sampling never samples control events or changes physics integration.
The selected mode is stored in job settings and the headless manifest.

For budgeting, the [matched recording benchmark](autocal-recording-performance.md)
measured roughly 4.8/5.0/5.5/15.2 seconds and 0/3.4/11/54 MB RRD for
`none`/`events`/`sampled`/`full` on the same 200,000-step HP3 workload. All cases
retained a shared 6.58 MB command/encoder journal; recorded cases retained the
same 34,017 events. The report explains exact equivalence checks and scope:
these are frozen recording-workload costs, not complete autocal job timings.

`optimizer` accepts `mode` (`fast`, `fast-fd`, `legacy`) and `solver` (`lbfgsb`,
`trf`). Defaults are `fast`/`lbfgsb`, 400 `solve_iterations` (1–2000), four
`solve_restarts` (1–16), and three `spool_outer_iters` (1–20). Inner spool
refinement uses a fixed cap of 30 with tolerance/stall stopping. Optional `r0_bounds_mm` and `b_bounds` are increasing
two-element lists within the radius/buildup bounds above. `budgets` defaults to
20 `max_steps` (1–100), 20 `sweep_points` per sub-sweep (3–100), 3600
`wall_timeout_s` (1–14400), 5,000,000 `max_physics_steps` (1–10,000,000), and one
BLAS/OpenMP `threads` (1–8). Wall and physics limits also bound force tuning and
bootstrap collection. No arbitrary CLI arguments, shell commands or external
firmware/recorder URLs are accepted.

Optional `collection` settings `force_low_n`, `force_mid_n`, `force_max_n` must
be supplied together, within [0.001, 10] N and satisfying low < mid <= max.
They select the existing CLI's manual-force path; omitted forces preserve
autotuning. `sensor_collection_force_n` independently overrides measurement
preload within the same bounds. These are experiment inputs, not acceptance
thresholds. Some force/geometry combinations fail physical settling; status
reports that failure and preserves its partial recording.

`artifact_dir` optionally names a new directory within launcher session artifacts;
the default is `native/autocal/<job_id>/`. Existing directories and escaping
paths/symlinks are rejected. Status lists stdout/stderr, complete or partial
measurement journals, iteration logs, stage artifacts, original/baked scenes,
firmware settings, source hashes, service logs and finalized recordings as they
become available. A finalized partial RRD is evidence of the interrupted run,
not an accepted calibration.

Terminal states are `accepted`, `budget_exhausted`, `cancelled`, `failed`.
`accepted` requires the CLI's genuine automatic patience/threshold stop,
successful parameter application and, when recording, recorder finalization. `--max-steps`
remains a budget stop. Poll status between conversational turns; chat interruption
does not cancel movement. Cancellation and supervisor exit signal the CLI owner,
stop physical activity, drain physics before the recorder, and retire the job's
process group. Unlike interactive CLI Ctrl-C, supervisor cancellation never
accepts best-so-far. Inspect the final headless clock for the actual stopped
step and queue boundary. A crashed owner triggers supervisor cleanup and retains
partial evidence; recording/cleanup errors are disclosed.

Open the exact URL from `start_browser_service(record=True)` in the browser.
`browser_status` names the connected page; every MCP action requires its
`page_id`, so a reload cannot silently substitute a different world. The page
shows its backend identity. Native Python and browser JS are separate worlds.
The production JS parity harness remains available for standalone parity work;
use the actual browser backend for UI, workers and shared human interaction.

The browser actions are `capabilities`, `observe`, `pause`, `resume`, `step`,
`reset`, `load_scene`, `commands`, `record`, `capture_context` and `interventions`.
They reuse the existing application controllers and runner. Pause and finish
active workers before direct commands or bounded stepping. Recording uses the
existing acknowledged, every-step flight recorder, supervised receiver and
assigned Viewer. The receiver preserves a separate recording per browser
session/scene generation. `browser_status` returns their exact RRD paths,
recording IDs and finalized step/time bounds for precise Viewer inspection.

On compatible desktop clients, feature-detected WebMCP tools expose the same
application API from the open page. Local MCP works when site tools are absent;
see [site tools availability](https://learn.chatgpt.com/docs/webmcp).
When saying “investigate this”, use the page's “Capture for research” action or
`capture_context` immediately: it freezes the words, entity/measurement, backend,
page/run, scene generation, selected simulation step/time and numerical snapshot.
Later navigation or motion cannot change it. Native selections use
`capture_native_context` with the selected run and `sim_step`. Continuing-world
history requires an actually recorded observation at that step; it never silently
substitutes a nearby time. Passive camera/timeline navigation does not steer the
objective. Use captured evidence when selecting the next trial.

## Autocal scope

The supplied [autocal prompt](prompts/autocal.txt) asks the research agent to
investigate structured trial-and-error, test scoring against physical accuracy,
and preserve the one-click workflow. This integration supplies the execution
and evidence tools; it does not itself introduce a new calibration solver.

Offline datasets and fitting/regression tools remain available. Native HP4/RRF
collection now uses collection jobs, which reuses the production collector and
bridge with a continuing headless world. It preserves version-2 millimetre records,
configuration, canonical roles and noise statistics, and rejects stationary
physical sensors. The [native collection guide](native-collection.md) explains
the clock, artifacts and calibration limits. Browser full-auto and Klipper
collection retain their existing [autocal setup](../autocal/README.md).

For a standalone full-auto run with one extended reference RRD, use
`autocal/autocal.py --headless-sim --extended-reference` with the intended
machine, dataset and optimizer flags. It owns the Node/RRF/recorder services and
finalizes physics, collection events, Python logs and stage artifacts together
without a browser. See the [headless extended reference guide](../autocal/data/references/extended-reference.md#headless-full-auto-collection).

Headless physics runs as fast as possible. `--speedup` retains the requested
playback setting, but collector waits and deadlines always use actual simulation
seconds, matching the browser clock. Increasing playback speed does not lengthen
force trials or settling windows.

### Local networking for full-auto runs

Standalone `--headless-sim` needs localhost sockets for RRF, physics and
recording. The launcher enables command networking by default while retaining
the `workspace-write` filesystem sandbox, and includes that setting in desktop
`attachment.toml`. Launcher-owned MCP services run outside the command
sandbox. Use `start_autocal` for the complete physical collection, fitting,
planning, selection and automatic-acceptance workflow when command sockets are
denied. `start_collection` runs only sweeps in the attached world, whose machine
`configure_session` chose; `reset_session` retains that machine.

Command networking remains useful for ad hoc HTTP experiments and integration
tests after supervised job tools are available. To disable it explicitly:

```bash
./hp-sim5-research-agent --codex --prompt-file research/prompts/autocal.txt -- \
  -c sandbox_workspace_write.network_access=false
```

The enabled default grants command network access, not just localhost access.
Managed policies and network-proxy rules may still restrict local sockets; verify socket creation,
loopback binding and an HTTP round trip before a long experiment. See the
[official configuration reference](https://learn.chatgpt.com/docs/config-file/config-reference).
Changing the launcher affects new sessions; it does not change a running chat's
effective policy. This option has not been validated under every client or
managed policy.

Before long collection call `research_preflight()`, optionally supplying the
complete `configure_session` request as `expected_configuration`. It checks the
attached supervisor's service connectivity, accepted machine/settings and source
freshness, firmware HTTP health, live Hangprinter kinematics/anchors, collector
bridge and initial firmware identity, and availability of full-auto/configuration/test
jobs. Live anchors must match the startup configuration within 0.1 mm (object-model
rounding); cached startup identity alone cannot establish current compatibility.
A separate recorder process
receives one empty, step-zero sample labeled `tool-check-preflight-*`, acknowledges
it, finalizes its isolated RRD and reads it back. The check has a 15-second budget
(plus process cleanup), never sends movement commands, resets the world or invokes
doctor, and preserves each trial under `native/preflight/`. Live jobs continue.
Recorder checks also run for `--no-record` sessions so recorder availability is
known before choosing a later recorded experiment. No Viewer is required.

`--preflight` starts a supervisor, performs that check and exits without launching
Codex or a Viewer. Service startup and shutdown are additional to the reported
preflight duration. Normal launcher startup still runs no diagnostic. A missing
or disconnected MCP attachment returns an actionable `blocked` result.

The supervisor reports command networking as `unknown`. Measure it from the
agent's own command context with:

```bash
.venv/bin/python scripts/research_socket_probe.py
```

This short probe tests socket creation, ephemeral loopback binding and a local
HTTP round trip independently. It prints actual `passed`/`blocked` checks, marks
unreached checks `unknown` and returns exit code 1 when blocked. Supervisor success
cannot replace this measurement; managed networking policy can still deny commands
even when the launcher's default is enabled.

Use `start_test(suite="headless-autocal", timeout_s=300)` or
`start_test(suite="research-services", timeout_s=300)` over MCP for socket-dependent
integration tests. `capabilities()` lists the exact fixed suite files. The first
covers headless autocal socket/recorder lifecycle and flight recording; the second
covers research MCP, native collection, world configuration, autocal jobs and the
preflight/test-job lifecycle. Both exclude `slow` tests. Their tests may advance
their own isolated worlds; they do not move the attached world. No arbitrary argv,
shell command, environment or external artifact path is accepted.

Each supervisor permits one active test job, independently of collection/autocal
jobs. Timeout is 1–900 seconds; status and cancellation stay available throughout.
Terminal states are `passed`, `failed`, `blocked` and `cancelled`, with a separate
`outcome` (`passed`/`failed`/`blocked`), stop reason and pytest exit code. Missing
executables or pytest usage/no-test exits are blocked, assertions/collection errors
and timeouts fail, cancellation is a blocked unfinished check. Logs, JUnit XML,
pytest temporary artifacts and manifests stay under `native/tests/<job_id>/`,
including failures. Cancellation, timeout, owner exit and supervisor shutdown retire
the test process group, including descendants that ignore SIGTERM. Test jobs strip
supervisor credentials and caller-controlled autocal/pytest overrides.
Detached services are tracked by PID birth identity and private job marker.
Cleanup also finds surviving members when their detached group leader has been
reaped, and rechecks ownership before each signal.

See [preflight and test-job verification](preflight-tests-verification.md) for
measured results and the socket-denied MCP client check. These checks establish
tool health, not calibration accuracy. Physical full-auto/movement smoke tests
and the expensive `--doctor` remain separate explicit diagnostics.

Full-autocal MCP jobs require a live launcher supervisor and MCP attachment.
They do not require agent command networking or broader sandbox permissions.
A bare standalone MCP registration cannot start these services. Report a blocked
standalone run honestly; offline replay does not validate the live workflow.
The [supervised job verification](autocal-jobs-verification.md) records real HP3
automatic stopping through a socket-denied MCP client, lifecycle failures and
finalized recording checks. The [settling follow-up](autocal-settling-review.md)
records its reproduction, endpoint recovery, corrected no-layering bake and a
complete default-force acceptance run.

Implementation prompts are [full-autocal jobs](prompts/tooling_improvement_1.txt),
[configurable fresh worlds](prompts/tooling_improvement_2.txt) and
[preflight/test jobs](prompts/tooling_improvement_3.txt). Run them in that order
in the same branch, or integrate their shared runtime changes explicitly.

The [investigation](investigation.md) maps Robium skills and OmniSim's architecture
to this integration, and separates the working features from future work.

The [verified live check](verification.md) records actual Codex tool use,
measurements and Rerun inspection from this checkout.

## Verification

```bash
.venv/bin/python -m pytest -q tests/python/test_research_agent.py
.venv/bin/python -m pytest -q tests/python/test_native_collector.py
.venv/bin/python -m pytest -q tests/python/test_native_collector.py -m slow
.venv/bin/python -m pytest -q tests/python/test_research_preflight.py
```

The first exercises real native experiments, deterministic replay, validation,
numeric comparisons and the MCP stdio protocol. The second verifies native RRD
telemetry and lifecycle behavior. A live Codex task additionally verifies model
access and agent tool use; these checks do not simulate a live Codex session.
