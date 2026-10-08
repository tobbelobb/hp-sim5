# Hangprinter research with Codex

`hp-sim5-research-agent` opens an ordinary interactive Codex conversation with
native experiment tools and an assigned Rerun Viewer. Conversation, steering,
interrupts, resume and Goal mode belong to Codex. The launcher owns the continuing
physics world, real RRF planner, production collector bridge, optional browser
service and requested browser recorder for the lifetime of the session.

## Setup and interactive use

From the repository root:

```bash
.venv/bin/python -m pip install -r requirements.txt
codex login                       # choose ChatGPT
./hp-sim5-research-agent
./hp-sim5-research-agent --prompt "Compare HP4 settling with a small A-axis movement. Run both experiments, inspect their recordings and explain the measured difference."
./hp-sim5-research-agent --prompt-file research/prompts/autocal.txt
./hp-sim5-research-agent --name "HP3 held-out sweep ranking" --prompt-file research/prompts/autocal.txt
./hp-sim5-research-agent --viewer none --prompt "Investigate autocal candidate ranking on recorded datasets."
./hp-sim5-research-agent --machine hp3 --viewer none --no-record --prompt "Collect independent HP3 calibration sweeps."
```

Continuing collection defaults to the production JavaScript physics pipeline in
headless Node, with the same fixed steps, forces, motors, encoders and cable
solver as the browser. `--machine hp3` selects the HP3 scene and matching RRF
configuration; HP4 remains the default. Use `--physics-backend native-python`
for the independent Python engine, or `--viewer none --no-record` for numerical
batches without Rerun logging. Fresh `run_experiment` trials still use Python.
See [measured collection performance](collection-performance.md).

The terminal goes directly to Codex. Continue talking and steering in that same
conversation; services remain alive between turns. Background services have
closed input and separate process groups, so collection cancellation or session
reset cannot restore Codex's terminal modes or consume its keys. Ctrl-C interrupts a Codex
turn without shutting down the supervisor. It does **not** cancel an experiment
job: call `cancel_collection` and inspect its stopped boundary. Exit Codex to
stop owned services. Use `/goal` for autonomous continuation with a measurable
outcome and explicit experiment, movement and compute limits. See the official
[interactive CLI reference](https://learn.chatgpt.com/docs/developer-commands)
and [Goal mode documentation](https://learn.chatgpt.com/docs/long-running-work).

Put normal Codex arguments after `--`; the launcher forwards them unchanged:

```bash
./hp-sim5-research-agent -- --model MODEL --no-alt-screen
./hp-sim5-research-agent -- --image output/example.png
./hp-sim5-research-agent -- resume --last
```

Structured clarification questions are enabled by default during interactive
research, including Default mode. Codex can present short questions with answer
choices and free-text input when your answer would materially improve the work.
Use `--dont-ask` before the argument separator to disable these Default-mode
question prompts:

```bash
./hp-sim5-research-agent --dont-ask --prompt-file research/prompts/autocal.txt
```

The underlying Codex feature is `default_mode_request_user_input`. You can also
enable it explicitly with a forwarded Codex option:

```bash
./hp-sim5-research-agent -- --enable default_mode_request_user_input
```

Forwarded Codex options can override launcher defaults. Codex 0.160.1 labels
this feature “under development”. Batch mode disables it because `codex exec`
does not support interactive answers. The desktop `attachment.toml` includes
the selected feature setting too; merge its `[features]` section as well as the
MCP sections when attaching a desktop chat.

Codex already supplies generic instructions for `request_user_input`: use it
when available for optional questions whose answers materially improve the work,
and continue with best judgment if no answer is returned. Comparing
`codex debug prompt-input` with the feature off and on in 0.160.1 showed identical
Default-mode instructions. The feature enables the tool; it does not add a
research-specific policy for when to ask. No extra questioning instructions in
`AGENTS.md` are needed for the basic behavior. Disabling the feature does not
prohibit questions in ordinary conversation or change action-approval settings.

Resume preserves the Codex conversation and experiment history, but this CLI
invocation starts a **fresh native world**. It states this in the session
instructions. Final snapshots are observations, not resumable checkpoints.
To retain a world across clients, use the service-only route below while its
supervisor remains alive. Use a separate worktree and supervisor for each
independent research chat; do not forward `--cd` or `--worktree` to move the
conversation away from its attached tools.

### Recover an interrupted conversation

If `codex resume` says “This conversation is open in another app”, the transcript
has loaded but another client owns the conversation. Close that conversation in
the other Codex client and press `r` to retry. If the desktop app retains it,
quit the app before retrying. An idle conversation can still be open in a client.
Use the exact conversation ID rather than `--last`, which can select an unrelated
chat. After releasing the other client, resume with fresh supervised tools:

```bash
./hp-sim5-research-agent -- resume CONVERSATION_ID
```

Read the previous session's `research.md`, experiment manifests and current Git
diff before continuing. The new session has a new artifact directory and world;
link the previous evidence in its research record. A cancelled collection cannot
continue from its final snapshot or partial-point journal. Keep that evidence
and start a new collection when needed. A live service-only supervisor can be
reattached through its existing `attachment.toml` instead.

The terminal's `f` shortcut offers a fork if the other client cannot be closed.
That creates a separate conversation with copied history; it does not restore
physics or release the original client's tool attachment. See the official
[resume and fork commands](https://learn.chatgpt.com/docs/developer-commands).

### Keep research conversations together

Use an **Autonomous research** custom sidebar section in the desktop app for
research runs and validation conversations. A section organizes existing chats
without changing their working directories or tool attachments. It is distinct
from ChatGPT project membership: the CLI uses its working directory as its local
project and does not expose the ChatGPT Projects view. See
[Projects and chats](https://learn.chatgpt.com/docs/projects).

Categorization uses a `UserPromptSubmit` MCP lifecycle hook, outside the model's
instructions. It passes the exact `${session_id}` from the lifecycle event to
`codex_app.move_thread_to_sidebar_section`. Select an existing section with
`--sidebar-section SECTION_UUID`, or save `{"section_id":"SECTION_UUID"}` in
the ignored local file `output/research/sidebar.json`. Use `--sidebar-section none`
to disable it. The launcher includes the hook in its Codex configuration and the
desktop `attachment.toml`; merge that hook only into a dedicated research
worktree's configuration when using desktop attachment.

For CLI sessions, the launcher checks `codex mcp list --json` and skips its
sidebar hook with a startup notice when `codex_app` is absent or disabled.
The bundled app-tools server is disabled by default and requires a
desktop-provided `CODEX_APP_TOOLS_PIPE_PATH`; enabling it alone in a terminal
does not establish that connection. Use `--sidebar-section none` to suppress
the startup notice when you do not need categorization. Service-only mode keeps
the hook in `attachment.toml` for the desktop client's configuration.

Codex requires reviewing and trusting the hook with `/hooks` before it runs.
The connected `codex_app` MCP server must expose the sidebar tool. An enabled
server can still fail to connect or lack the tool; those errors and
disabled/untrusted hooks leave categorization unavailable without blocking
research. This does not launch the desktop app or assign ChatGPT project
membership. See the official
[MCP lifecycle hooks](https://learn.chatgpt.com/docs/hooks).

The launcher uses the existing Codex ChatGPT login, makes no OpenAI API calls,
selects the OpenAI provider, and removes API-key variables from its Codex child.
ChatGPT plan limits apply. Its defaults are `workspace-write` and
`approval_policy="never"`; normal Codex options can override them. Service
startup leaves physics at step zero and never selects experiments or runs doctor.
Default Viewer mode is `--viewer headless`; use `--viewer window` for a desktop
window or `--viewer none` for numerical research. Headless Rerun still requires
a graphics backend. Failed Viewer startup names its log and suggests numeric mode.

The CLI warning “Running without the shared background server … requires
embedded mode” is expected. Per-session `-c` overrides attach the experiment
tools and instructions, so Codex runs its backend inside the CLI process instead
of using the shared daemon. This still uses the normal Codex conversation and
ChatGPT login. It does not refer to embedding the Rerun Viewer.

## Desktop attachment

Start a supervisor in a dedicated worktree and keep its terminal open:

```bash
./hp-sim5-research-agent --serve --session-dir output/research/my-chat
```

The directory must be new. The launcher prints the path to `attachment.toml`.
Copy its feature and MCP sections into that trusted worktree's `.codex/config.toml`,
merging with existing settings, then open or restart the Codex desktop chat in that
worktree. This is project configuration shared by local Codex clients; see the
[official MCP documentation](https://learn.chatgpt.com/docs/extend/mcp).
The launcher never changes global or project configuration automatically.

`research_attach.py` resolves the selected private `connection.json`, verifies
that its supervisor is live and belongs to this worktree, and supplies credentials
to the MCP child. Credentials are excluded from launch metadata, tool results
and the configuration snippet. One native MCP attachment holds an exclusive
lease: a second chat gets an explicit error instead of sharing mutable physics.
The same chat can reconnect after its MCP connection closes. A coordinated
reset reports a fresh world identity; an ended supervisor fails attachment
and requires a new service session. Bare registration of `hp_sim5_mcp.py` only
supports fresh numerical trials, not continuing-world tools.

Research instructions are supplied through MCP initialization for every entry
route and through Codex developer instructions for launcher sessions. Maintain
`research.md` in the session artifact directory with the objective, hypothesis,
constraints/budgets, experiment IDs, accepted steering and next decision.

## Finding previous research

Open the [research topic guide](catalog.md) for the main investigations and
the local [session catalog](../output/research/index.md) for short abstracts,
tested hypotheses, outcomes and evidence links. The local catalog includes
sessions under `output/research/sessions/` and explicitly named sessions directly
under `output/research/`. It reads `abstract.md`, so searching it does not require
reading long reports or raw telemetry:

```bash
.venv/bin/python scripts/research_index.py
.venv/bin/python scripts/research_index.py --search "winding"
```

New default directories use `YYYY-MM-DD-topic-unique-suffix`, with a UTC date. The topic comes
from `--name` or the first prompt line. Use `--name` for a short, specific topic;
interactive sessions without an initial prompt start as `interactive-research`.
`--session-dir` still selects an exact new directory. Runtime world IDs and
experiment IDs remain independent of these names.

Each agent maintains `abstract.md` using the [abstract template](abstract-template.md),
alongside its notebook and report. It records pending, completed, interrupted or
blocked work explicitly and links the measurements behind each tested claim.
The launcher creates a pending abstract and refreshes the local catalog at
startup and exit; the agent refreshes it during research turns, including desktop
attachment. Legacy sessions without an abstract are labeled as missing a summary.

Older hash paths contain absolute references in reports, manifests, attachment
configuration and conversation history. Descriptive directory aliases let readers
browse those sessions without moving the evidence. The catalog lists each
physical session once and prefers its descriptive alias. Keep both paths stable
while a supervisor is running. Session artifacts and aliases are ignored by Git;
the topic guide preserves a short account of the existing findings in the repo.

## Batch mode and diagnostics

```bash
./hp-sim5-research-agent --batch --prompt-file research/prompts/autocal.txt
./hp-sim5-research-agent --dry-run --prompt "Optimize the HP4 anchor layout"
./hp-sim5-research-agent --doctor
```

Batch mode retains `codex exec`, prompt input, JSONL events and the separate
`final-message.md`. Interactive transcripts stay in Codex's own conversation
store. Both routes keep `prompt.txt`, `launch.json`, the research record/report,
service logs and `exit.json` in their session directory. Private attachment
credentials live in mode-600 `connection.json`. Default directories are under
`output/research/sessions/`; fresh trials are under `output/research/experiments/`,
and continuing collection evidence is under each session's `native/` directory.
All are ignored by Git. The explicit doctor checks dependencies/login/Viewer
MCP and performs real RRF/native HP4 collection with autocal validation; it takes
about fourteen minutes on the historical Python backend; the default backend
has since changed. It does not establish calibration accuracy or model
access. See [collection evidence](native-collection-verification.md).

## Experiment tools

| MCP tool | Result |
| --- | --- |
| `capabilities()` | Scenes, command units, limits and known gaps |
| `run_experiment(scene, steps, dt, commands, commands_file, label, record)` | A fresh physics run, metrics, warnings and absolute artifact paths |
| `read_experiment(run_id)` | Provenance and summary |
| `read_experiment(run_id, start_step, limit)` | A page of numeric samples, up to 100 |
| `compare_experiments(baseline_id, candidate_id)` | Changed inputs, metric differences and final effector distances |
| `runtime_status()` | Persistent world, clock, queues, service PIDs/logs and stale source detection |
| `send_gcode(line)` | One line through real RRF planning into the continuing native world |
| `step_physics(steps)` | Advance the continuing world by fixed steps |
| `start_collection(configs, options, settling_timeout_s)` | Start a collection job and return its job ID immediately |
| `collection_status(job_id)` | Running/cancelling/complete/cancelled/failed status and evidence |
| `cancel_collection(job_id)` | Known physics stop boundary, retired firmware/bridge queues, partial evidence and reset requirement |
| `reset_session()` | Archive state and restart world, firmware and bridge references |
| `start_browser_service(record=False)` | Exact shared browser-js URL; optionally owns its flight recorder |
| `browser_status()` | Connected page identity and immutable user-submitted context |
| `browser_action(page_id, action, args)` | Controls and numerical observations from that exact existing page |
| `capture_native_context(message, step, selected_entity, run_id)` | Selected native run/session and time with actual numerical observation |

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
records their hashes, the native Python source hash, Git revision, interpreter,
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

The [investigation](investigation.md) maps Robium skills and OmniSim's architecture
to this integration, and separates the working features from future work.

The [verified live check](verification.md) records actual Codex tool use,
measurements and Rerun inspection from this checkout.

## Verification

```bash
.venv/bin/python -m pytest -q tests/python/test_research_agent.py
.venv/bin/python -m pytest -q tests/python/test_native_collector.py
.venv/bin/python -m pytest -q tests/python/test_native_collector.py -m slow
.venv/bin/python -m pytest -q tests/python/cable_joints_3d/test_machine_recording.py
```

The first exercises real native experiments, deterministic replay, validation,
numeric comparisons and the MCP stdio protocol. The second verifies native RRD
telemetry and lifecycle behavior. A live Codex task additionally verifies model
access and agent tool use; these checks do not simulate a live Codex session.
