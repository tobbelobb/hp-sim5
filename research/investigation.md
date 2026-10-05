# Robotics agent integration investigation

Investigated 2026-10-05. Robium source inspected at
[`de46ef6`](https://github.com/robium-ai/robium/tree/de46ef6df3286c24ea1e1c7eaec1af56bce8d248).
OmniSim HEAD was
[`2fa99ea`](https://github.com/omnilink-tech/omnisim/tree/2fa99ea59ee4c799919a83cd1dcac223fe88172a).
The findings below come from upstream instructions and source inspection, not
from running OmniSim benchmarks or demonstrating physical robot transfer.

## Robium skills that transfer

Robium currently contains 27 skills and a Codex plugin. Its setup creates editable
Robium and reference-app checkouts. Installation is optional here; no upstream
skill code or hooks were installed in hp-sim5. If desired, upstream documents
`npx robium-ai setup --agent codex` and `npx robium-ai doctor`.
[Source](https://github.com/robium-ai/robium/tree/de46ef6df3286c24ea1e1c7eaec1af56bce8d248).

These are my transfer recommendations, rather than claims that Robium already
supports Hangprinter mechanics or the hp-sim5 autocal schema:

| Skills | Transfer to hp-sim5 |
| --- | --- |
| [architect](https://github.com/robium-ai/robium/blob/de46ef6df3286c24ea1e1c7eaec1af56bce8d248/skills/architect/SKILL.md) | Start with a robot behavior and an experiment that can disprove the main assumption; reuse the existing cable simulator. |
| [testing](https://github.com/robium-ai/robium/blob/de46ef6df3286c24ea1e1c7eaec1af56bce8d248/skills/testing/SKILL.md), [test-assets](https://github.com/robium-ai/robium/blob/de46ef6df3286c24ea1e1c7eaec1af56bce8d248/skills/test-assets/SKILL.md) | Match evidence to the physical claim, record fixture provenance, bound runs and test repeatability. Use existing USD presets, synthetic ground truth and held-out sweeps. |
| [rerun](https://github.com/robium-ai/robium/blob/de46ef6df3286c24ea1e1c7eaec1af56bce8d248/skills/rerun/SKILL.md), [visualization](https://github.com/robium-ai/robium/blob/de46ef6df3286c24ea1e1c7eaec1af56bce8d248/skills/visualization/SKILL.md) | Save recordings, choose the sink deliberately and align commands, observations and predictions on simulation time. Rerun is already the native recorder. |
| [simulation](https://github.com/robium-ai/robium/blob/de46ef6df3286c24ea1e1c7eaec1af56bce8d248/skills/simulation/SKILL.md) | Name the fidelity contract: cable elasticity/friction, winding, encoder errors, force response and frame conventions. Its default simulator choices need adaptation to the specialized XPBD model. |
| [data](https://github.com/robium-ai/robium/blob/de46ef6df3286c24ea1e1c7eaec1af56bce8d248/skills/data/SKILL.md) | Track embodiment, action space, sensor rates, episode boundaries, collection conditions and splits. Apply this to calibration datasets without converting them into imitation-learning datasets. |
| [environments](https://github.com/robium-ai/robium/blob/de46ef6df3286c24ea1e1c7eaec1af56bce8d248/skills/environments/SKILL.md), [integration](https://github.com/robium-ai/robium/blob/de46ef6df3286c24ea1e1c7eaec1af56bce8d248/skills/integration/SKILL.md) | Make interpreter, dependencies, clocks, units, transport failures and process ownership explicit. Preserve this repo's `.venv` convention. |
| [learning-loop](https://github.com/robium-ai/robium/blob/de46ef6df3286c24ea1e1c7eaec1af56bce8d248/skills/learning-loop/SKILL.md), [skill-author](https://github.com/robium-ai/robium/blob/de46ef6df3286c24ea1e1c7eaec1af56bce8d248/skills/skill-author/SKILL.md), [mining](https://github.com/robium-ai/robium/blob/de46ef6df3286c24ea1e1c7eaec1af56bce8d248/skills/mining/SKILL.md) | Useful for maintaining evidence-backed experiment recipes. Keep tentative results separate from validated guidance; avoid importing background hooks just to record a trial. |

The other skills have conditional value: `ros2`, `foxglove`, `rviz2` when adding
a ROS bridge; `mujoco`, `gazebo`, `isaac-sim`, `isaac-lab` for an explicit second
simulator or training task; `lerobot`, `huggingface`, `gemini-robotics` for learned
policies or dataset/model distribution; `cloud-run`, `runpod`, `live-demo`,
`app-publishing` for requested hosting or compute. `navigation` targets mobile
robot navigation and does not directly solve cable workspace planning.
[Catalog](https://github.com/robium-ai/robium/tree/de46ef6df3286c24ea1e1c7eaec1af56bce8d248/skills).

## Why OmniSim agents can do experiments

OmniSim's agent workflow gives the agent a running system to control. Its
`AGENTS.md` begins with a doctor that checks the binary, physics runtime and
controller compatibility. It directs the agent to drive load, sync, reset,
step, scene inspection and screenshots itself. The instructions contain
specific failure knowledge: a successful load is insufficient evidence of
working physics, unsupported fields can be ignored, and scene appearance can
disagree with collision registration. That operating knowledge is more useful
than a large generic coding prompt.
[AGENTS.md](https://github.com/omnilink-tech/omnisim/blob/2fa99ea59ee4c799919a83cd1dcac223fe88172a/AGENTS.md).

The main separation is engine/controllers, an HTTP harness with an injected
supervisor, and MCP tools that proxy that harness. The agent can discover
capabilities and diagnostics, author a world/controller, load it, inspect scene
fields, advance the experiment and measure the result. The documented sync
path distinguishes pose updates from full reloads. HTTP health and physics
readiness have different meanings.
[Harness](https://github.com/omnilink-tech/omnisim/tree/2fa99ea59ee4c799919a83cd1dcac223fe88172a/scripts/harness),
[protocol](https://github.com/omnilink-tech/omnisim/blob/2fa99ea59ee4c799919a83cd1dcac223fe88172a/PROTOCOL.md).

The inspected MCP implementation is a stateless adapter: tool handlers make
typed HTTP requests over a pooled connection; the running harness owns state.
It implements a tools-only stdio protocol with Python's standard library.
Its `harness_status` checks reachability before use, and its contact/capability
responses describe measurement gaps so an empty result cannot silently imply
that no contact occurred. These are concrete interface choices worth adopting.
[MCP source](https://github.com/omnilink-tech/omnisim/blob/2fa99ea59ee4c799919a83cd1dcac223fe88172a/packages/omnisim-mcp/src/omnisim_mcp/server.py).

Its newer controls include leased pause and event breakpoints. Their timing
contract matters: supervisor time and engine time differ, and requested steps
need not equal executed steps when a breakpoint fires. Named pose snapshots
are not complete checkpoints. Upstream also explicitly limits sim-to-real
claims and reports no policy validation on hardware. I would borrow the control
and evidence discipline, while retaining hp-sim5's cable physics and existing
step clock. The current OmniSim README says cables/rods lack an OmniSim node,
which makes replacing hp-sim5 premature.
[Agent timing and limitations](https://github.com/omnilink-tech/omnisim/blob/2fa99ea59ee4c799919a83cd1dcac223fe88172a/AGENTS.md),
[README](https://github.com/omnilink-tech/omnisim/blob/2fa99ea59ee4c799919a83cd1dcac223fe88172a/README.md).

## Applied architecture and next research steps

Rerun's 0.38 release adds an experimental native agent panel supporting installed
coding agents. The documented stdio Viewer MCP is also available to an external
Codex process, including with a headless Viewer. This integration uses that
interface alongside hp-sim5's experiment tools; the installed Viewer/SDK is
0.38.1.
[Release notes](https://rerun.io/docs/changelog/changeset-0-38),
[Viewer MCP](https://rerun.io/docs/reference/viewer/mcp).

The working integration keeps three responsibilities explicit:

1. Codex handles the user task, source edits, hypotheses and experiment choices,
   using its normal ChatGPT login and sandbox.
2. hp-sim5 MCP constructs fresh native worlds, validates motor commands, advances
   real physics and produces numerical observations and provenance.
3. Rerun MCP inspects saved recordings, Viewer state, warnings and screenshots.
   Viewer control is separate from numerical data access, as
   [Rerun documents](https://rerun.io/docs/reference/viewer/mcp).

Compared with a persistent HTTP harness, fresh trials simplify repeatability
and keep failed experiments from contaminating subsequent initial state.
`research/AGENTS.md` supplies the experiment loop and domain-specific traps.
The integration adds no dependency on ROS, OmniSim, GPU training, or an API key.

The largest remaining bridge is native online autocal collection: translate
firmware movement/force control into native commands, reproduce encoder queries
and sweep records, then compare collector output against the browser path.
That needs explicit parity evidence before replacing the one-click collector.

For the proposed algorithm, the strongest starting experiment is a ranking
audit: find candidates whose fitted score improves while known anchor error or
held-out sensor prediction worsens. Then compare bounded residual-driven updates
and informative movement selection against the current active-learning loop.
Assess uncertainty and parameter identifiability, particularly anchor/radius/
buildup ambiguity. Accept updates on unseen movement predictions and rollback
on deterioration. These are proposed experiments, not measured algorithm gains.
