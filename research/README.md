# Hangprinter research with Codex

`hp-sim5-research-agent` runs Codex with a native simulation MCP and Rerun's
Viewer MCP. Give it a robot design, experiment or code improvement task. Codex
can edit the repository, run physics trials, inspect measurements and recordings,
and write an evidence-backed report. It uses the existing Codex ChatGPT login;
the launcher makes no OpenAI API calls, selects Codex's OpenAI provider and
removes API-key environment variables from its Codex child. ChatGPT plan limits
still apply. See the official
[authentication guide](https://learn.chatgpt.com/docs/auth) and
[non-interactive Codex guide](https://learn.chatgpt.com/docs/non-interactive-mode).

## Setup and first run

From the repository root:

```bash
.venv/bin/python -m pip install -r requirements.txt
codex login                       # choose ChatGPT
./hp-sim5-research-agent --prompt "Compare HP4 settling with a small A-axis movement. Run both experiments, inspect their recordings and explain the measured difference."
```

`--doctor` is an optional, explicit diagnostic command. Ordinary prompt runs
start the supervised tools and let the agent decide which experiments to run;
they do not invoke doctor or collect autocal data automatically.

```bash
./hp-sim5-research-agent --doctor  # optional collection diagnostic (~14 minutes)
```

The doctor imports dependencies, checks Codex login and Rerun MCP, then runs
real RRF/native HP4 collection and loads six measurements through autocal's
schema, role and residual pipeline. It checks physical sensor response, drive
travel, units and noise statistics. The verified run took about fourteen minutes;
see [collection evidence](native-collection-verification.md). Python physics
runs independently of wall time. It does not establish calibration accuracy,
graphics or model access. The launcher starts its own Rerun Viewer on a
free loopback port, passes both MCP configurations to `codex exec` for this
invocation, and stops its Viewer when Codex ends. It does not write global Codex
configuration. Default viewing is `--viewer headless`; use `--viewer window`
for a desktop window or `--viewer none` for numerical research without graphics.
Headless Rerun still needs a working graphics backend. Viewer failures include
the log path and suggest numerical mode.

The parent launcher also owns a persistent native runtime, RRF and the collector
bridge. MCP proxies to these services; the agent's shell does not need permission
to bind their ports. Readiness requires a firmware identification reply and a
connected native encoder client. Normal startup leaves the world at step zero,
records service status in `launch.json`, and starts the agent. Explicit doctor
runs archive collection evidence and stop their services. Vite starts on demand
through `start_browser_service`. All owned services stop with the launcher.

```bash
./hp-sim5-research-agent --prompt-file research/prompts/autocal.txt
./hp-sim5-research-agent --viewer none --prompt "Investigate autocal candidate ranking on recorded datasets. Reproduce a mismatch before changing code."
./hp-sim5-research-agent --dry-run --prompt "Optimize the HP4 anchor layout"
```

`--model MODEL` overrides the existing Codex model setting. The launcher uses
`workspace-write` and `approval_policy="never"` so it can run authorized local
experiments unattended. Commands outside that sandbox fail; managed policies
still apply. This is a local code editing agent: use a separate checkout for
independent tasks. Interrupt with Ctrl-C. Session paths are printed at launch.

Each session in `output/research/sessions/` contains the full prompt, launch
configuration, Codex JSONL events, research `report.md`, Codex's separate
`final-message.md`, Viewer log and exit status.
Experiment artifacts live separately under `output/research/experiments/`.
Native collection artifacts live under each session's `native/` directory;
standalone doctor artifacts live under `output/research/preflight/`. All are
excluded by the repository's `/output` ignore rule.

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
| `collect_sweeps(configs, options, settling_timeout_s)` | Production collector JSON, finalized RRD, command/sensor trace and provenance |
| `reset_session()` | Archive state and restart world, firmware and bridge references |
| `start_browser_service()` | Ready launcher-owned Vite URL for the independent 3D web app |

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
Viewer MCP manipulates the UI. For standalone use in another Codex session:

```bash
codex mcp add hp_sim5 -- "$PWD/.venv/bin/python" "$PWD/scripts/hp_sim5_mcp.py"
codex mcp add rerun -- "$PWD/.venv/bin/rerun" viewer-mcp
.venv/bin/rerun --headless --bind 127.0.0.1
```

These optional registration commands change your Codex configuration; the
launcher itself needs neither. MCP uses the official Python SDK's current 2.x
API. See [SDK documentation](https://py.sdk.modelcontextprotocol.io/).

## Autocal scope

The supplied [autocal prompt](prompts/autocal.txt) asks the research agent to
investigate structured trial-and-error, test scoring against physical accuracy,
and preserve the one-click workflow. This integration supplies the execution
and evidence tools; it does not itself introduce a new calibration solver.

Offline datasets and fitting/regression tools remain available. Native HP4/RRF
collection now uses `collect_sweeps`, which reuses the production collector and
bridge with a continuing Python world. It preserves version-2 millimetre records,
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
