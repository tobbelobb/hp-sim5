# Hangprinter research with Codex

`hp-sim5-research-agent` opens an ordinary interactive Codex conversation with
native experiment tools and an assigned Rerun Viewer. Conversation, steering,
interrupts, resume and Goal mode belong to Codex. The launcher owns the continuing
physics world, real RRF planner, production collector bridge, optional browser
service and requested browser recorder for the lifetime of the session.

Start with the [execution architecture map](architecture.md) for ownership,
configuration flow and recording scope across the five research workflows.
This README contains the operating instructions and tool arguments.

## Setup and interactive use

From the repository root:

```bash
.venv/bin/python -m pip install -r requirements.txt
npm run build:native               # Rust, C compiler and Node headers
codex login                       # choose ChatGPT
./hp-sim5-research-agent
./hp-sim5-research-agent --prompt "Compare HP4 settling with a small A-axis movement. Run both experiments, inspect their recordings and explain the measured difference."
./hp-sim5-research-agent --prompt-file research/prompts/autocal.txt
./hp-sim5-research-agent --name "HP3 held-out sweep ranking" --prompt-file research/prompts/autocal.txt
./hp-sim5-research-agent --viewer none --prompt "Investigate autocal candidate ranking on recorded datasets."
./hp-sim5-research-agent --machine hp3 --viewer none --no-record --prompt "Collect independent HP3 calibration sweeps."
./hp-sim5-research-agent --model-provider opencode_go --model deepseek-v4.1-flash --prompt "Say hello"
```

The launcher checks the native Rust physics build before starting services and
runs `npm run build:native` automatically when it is stale or missing, printing
`Native Rust physics build is stale, recompiling`. A failed build stops startup.

Continuing collection defaults to the production native Rust physics engine in
headless Node, with the same fixed steps, forces, motors, encoders and cables
as the browser. JavaScript handles scene preparation, commands and observations.
See the [native host implementation](rust-native-engine.md) and the earlier
[Wasm engine measurements](rust-wasm-engine.md). `--machine hp3` selects the HP3 scene and matching RRF
configuration; HP4 remains the default. Use `--physics-backend native-python`
for the independent Python engine, or `--viewer none --no-record` for numerical
batches without Rerun logging. Fresh `run_experiment` trials also default to native
Rust; `physics_backend='native-python'` selects the independent reference.
Fresh trials can select `cable_solver_device='cpu'` for the optional compiled
Warp cable solver. Continuing sessions use `--physics-backend native-warp`;
`native-warp-cuda` is experimental and requires a CUDA device. Other physics
systems still execute in Python. See the [compiled solver measurements and
limits](compiled-python-solver.md) and [measured collection performance](collection-performance.md).

The terminal goes directly to Codex. Continue talking and steering in that same
conversation; services remain alive between turns. Background services have
closed input and separate process groups, so collection cancellation or session
reset cannot restore Codex's terminal modes or consume its keys. Ctrl-C interrupts a Codex
turn without shutting down the supervisor. It does **not** cancel an experiment
job: call `cancel_collection` or `cancel_autocal` and inspect its stopped boundary. Exit Codex to
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
the selected feature setting too; merge its networking and `[features]` sections
as well as the MCP sections when attaching a desktop chat.

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

### Research in a Git worktree

`--worktree` creates a fresh worktree and runs the whole session from it, so
`ROOT`, `--cd`, the MCP `cwd`, the sandbox and the session artifacts all follow
the worktree instead of the original checkout:

```bash
./hp-sim5-research-agent --worktree --prompt-file research/prompts/autocal.txt
./hp-sim5-research-agent --worktree --name "deferred bake" --worktree-dir /tmp/hp-sim5-worktrees
```

It runs `git worktree add -b research/<name>-<id> <dir>` from the current `HEAD`,
links the Git-ignored pieces a bare checkout lacks (`.venv`, `.env`,
`node_modules`, `RRF/build` and the compiled native physics addon), and copies
the repository paths listed in `.worktreeinclude` (when present) into the new
worktree so prior local research travels with it. It then re-executes the
worktree's own launcher. The default
parent directory is a sibling named `<repo>-worktrees`; override it with
`--worktree-dir`. Each worktree keeps its own `output/research/` evidence. The
shared addon is only a starting build: each worktree has local build metadata
and build output, so rebuilding its native physics does not change another
worktree. Each launched supervisor also owns its own physics process and session
artifacts. The branch name derives from `--name`, otherwise from the prompt's first line,
otherwise `research`. `.worktreeinclude` accepts comments, plain relative paths
and simple globs; missing entries are ignored.

The worktree checks out committed `HEAD`, so commit launcher or tool changes
before using `--worktree`; uncommitted edits in the original checkout are not
carried over. `--worktree` is a launcher option (before `--`); forwarding
`--worktree` to Codex after `--` is not the same thing and is still unsupported.

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

The launcher defaults to the existing Codex ChatGPT login: it selects the OpenAI
provider, makes no OpenAI API calls, and removes `OPENAI_API_KEY` and
`CODEX_API_KEY` from its Codex child. ChatGPT plan limits apply. `--model-provider`
and `--model` select another configured provider and model, and forwarded Codex
`-c model_provider=... -m ...` arguments are detected too:

```bash
./hp-sim5-research-agent --model-provider opencode_go --model deepseek-v4.1-flash
./hp-sim5-research-agent -- -c 'model_provider="opencode_go"' -m deepseek-v4.1-flash
```

When the selected provider declares an `env_key` in the Codex configuration, the
launcher fills a missing variable from the Git-ignored repository `.env` before
starting Codex. It reads only that provider's variable, leaves the surrounding
environment unchanged, and never writes the value to prompts, launch metadata or
logs. Such providers do not need the `codex login` step above. The launcher's
defaults are `workspace-write`, `approval_policy="never"` and
`sandbox_workspace_write.network_access=true`; normal Codex options can override
them. Service
startup leaves physics at step zero and never selects experiments or runs doctor.
Default Viewer mode is `--viewer headless`; use `--viewer window` for a desktop
window or `--viewer none` for numerical research. Headless Rerun still requires
a graphics backend. Failed Viewer startup names its log and suggests numeric mode.

The CLI warning “Running without the shared background server … requires
embedded mode” is expected. Per-session `-c` overrides attach the experiment
tools and instructions, so Codex runs its backend inside the CLI process instead
of using the shared daemon. This still uses the normal Codex conversation and the
configured login. It does not refer to embedding the Rerun Viewer. Provider and
model selection apply to launcher sessions; a desktop client selects its own
provider in its client configuration.

## Desktop attachment

Start a supervisor in a dedicated worktree and keep its terminal open:

```bash
./hp-sim5-research-agent --serve --session-dir output/research/my-chat
```

The directory must be new. The launcher prints the path to `attachment.toml`.
Copy its sandbox networking, feature and MCP sections into that trusted
worktree's `.codex/config.toml`, merging with existing settings, then open or
restart the Codex desktop chat in that
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
./hp-sim5-research-agent --preflight
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
| `research_preflight(expected_configuration=None)` | Bounded non-moving service/settings/firmware/collector/recorder check; separate command-network result remains unknown |
| `start_test(suite, timeout_s=300)` | Immediate job ID for a fixed integration suite on the supervisor |
| `test_status(job_id)` | Test progress, pass/fail/blocked outcome, exit code, log tails and isolated artifacts |
| `cancel_test(job_id)` | Retire test runner and descendants, preserving partial evidence |
| `send_gcode(line)` | One line through real RRF planning into the continuing native world |
| `step_physics(steps)` | Advance the continuing world by fixed steps |
| `start_collection(configs, options, settling_timeout_s)` | Start a collection job and return its job ID immediately |
| `collection_status(job_id)` | Running/cancelling/complete/cancelled/failed status and evidence |
| `cancel_collection(job_id)` | Known physics stop boundary, retired firmware/bridge queues, partial evidence and reset requirement |
| `start_autocal(machine, layering, base_radii_mm, buildup_factor, find_radii, find_buildup_factor, optimizer, budgets, collection, artifact_dir, recording_mode)` | Immediate job ID for an isolated world running the production full-auto CLI |
| `autocal_status(job_id)` | Physical progress, iteration/decision, stop reason, CLI exit status and complete/partial artifacts |
| `cancel_autocal(job_id)` | Stop the job-owned world, drain recording and retire all CLI children; attached world remains usable |
| `configure_session(configuration)` | Validate settings, archive the old world and create a fresh HP3/HP4 world with matched firmware and encoder origin |
| `reset_session()` | Archive state and restart world, firmware and bridge references with the accepted configuration |
| `start_browser_service(record=False)` | Exact shared browser-js URL; optionally owns its flight recorder |
| `browser_status()` | Connected page identity and immutable user-submitted context |
| `browser_action(page_id, action, args)` | Controls and numerical observations from that exact existing page |
| `capture_native_context(message, step, selected_entity, run_id)` | Selected native run/session and time with actual numerical observation |

A desktop or launcher attachment can change its continuing world without restarting
Codex or writing a simulator adapter. For example, from an HP4 attachment call:

```json
{"configuration": {"machine": "hp3", "layering": false,
                   "base_radii_mm": [30], "buildup_factor": 0}}
```

Pass this object to `configure_session`. Configuration is a complete fresh-world
request: defaults are HP4, layering enabled, stock 30 mm base radii and stock
buildup (2/pi). It is not a patch to the current world. One radius applies to all
four spools; four values select A/B/C/D individually (1–200 mm). Custom radii
require layering disabled and zero buildup. Layered worlds currently support
only stock radii/buildup. Optional `scene` must select the matching repository
`public/usd_scenes/hp3_rigid_body.usda` or HP4 file; optional `firmware_config`
is relative to `RRF/run/vsd` and must match machine/layering. Custom anchor
layouts and other scene/backend combinations are rejected explicitly. The
backend remains the launcher's selected backend.

Layering affects the bake, physical cable half-width and all three runtime
layering flags. The selected firmware is frozen in each generation and booted
from a short private VSD (RRF limits host filename length). `runtime_status`
and world/collection manifests report requested/effective settings, authored,
design and baked scene hashes, firmware configuration/replies and generation
identity. HP3 and HP4 share the collector's four-anchor Hangprinter profile.

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
and machine/layering constraints.

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
attached session. An HP4 chat can request HP3 without changing its current world.
Only one full-autocal job runs per supervisor; collection jobs use the attached
world independently. For known HP3 radii and no layering, call:

```json
{"machine":"hp3", "layering":false, "base_radii_mm":[30], "buildup_factor":0}
```

Those are also the defaults. `machine` accepts `hp3`/`hp4`; radii contain one or
four finite values in [1, 200] mm. `find_radii` and `find_buildup_factor` accept
`off`, `global`, `per-anchor`. Nonzero buildup or buildup fitting requires
`layering=true`. Buildup is bounded to [0, 2]. Layering selects the matching
firmware preset and enables the existing collector's simulator setting.
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
`solve_restarts` (1–16), three `spool_outer_iters` (1–20), and 30
`spool_inner_iters` (1–2000). Optional `r0_bounds_mm` and `b_bounds` are increasing
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

### Local networking for full-auto runs

Standalone `--headless-sim` needs localhost sockets for RRF, physics and
recording. The launcher enables command networking by default while retaining
the `workspace-write` filesystem sandbox, and includes that setting in desktop
`attachment.toml`. Launcher-owned MCP services run outside the command
sandbox. Use `start_autocal` for the complete physical collection, fitting,
planning, selection and automatic-acceptance workflow when command sockets are
denied. `start_collection` runs only sweeps in the attached world, whose machine
is chosen at launch; `reset_session` retains that machine.

Command networking remains useful for ad hoc HTTP experiments and integration
tests after supervised job tools are available. To disable it explicitly:

```bash
./hp-sim5-research-agent --machine hp3 --prompt-file research/prompts/autocal.txt -- \
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
.venv/bin/python -m pytest -q tests/python/cable_joints_3d/test_machine_recording.py
```

The first exercises real native experiments, deterministic replay, validation,
numeric comparisons and the MCP stdio protocol. The second verifies native RRD
telemetry and lifecycle behavior. A live Codex task additionally verifies model
access and agent tool use; these checks do not simulate a live Codex session.
