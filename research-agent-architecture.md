# Interactive research agent architecture

Investigated 6 October 2026 against `/home/torbjorn/repos/hp-sim5`, Codex CLI 0.160.0 and Rerun SDK 0.38.1. This is an architecture recommendation based on source inspection, official documentation and locally generated Codex protocol schemas. No implementation changes or new live agent experiments were performed.

**Recommendation.** Make the research agent an ordinary Codex session equipped with hp-sim experiment tools and research instructions. Use the full Codex interface for conversation and steering. Retain hp-sim's existing supervisor, physics, firmware, collector, telemetry and Rerun integration. Add a small interface to the existing browser application when shared browser experimentation is needed. Build a custom chat client only if a distinct product experience becomes a requirement.

This separates two decisions: where the conversation lives, and where experiments run. Choosing Codex for the conversation does not require moving simulation execution into Codex or replacing the existing playgrounds.

**What the checkout already supplies.**

| Existing component | Reuse | Remaining limitation |
| --- | --- | --- |
| `scripts/research_agent.py` | Service startup, per-session directories, temporary MCP configuration, ChatGPT authentication and cleanup | Runs `codex exec`, closes its input and prints completed agent messages; exposes no interactive conversation interface |
| `scripts/research_runtime.py` and `src/python/hp_sim5_research/services.py` | Persistent native world, real RRF planning, production collector bridge, on-demand Vite and service readiness | Runtime endpoint/token are supplied through the launcher; collection has no explicit cancel/pause operation |
| `scripts/hp_sim5_mcp.py` | Capability discovery, fresh experiments, numerical reads/comparisons, continuing-world operations and sweep collection | Current experiment wrapper is native 3D; opening a web page does not connect its state to the native world |
| `src/python/hp_sim5_research/experiments.py` | Frozen USD inputs, fixed-step execution, source/input hashes, metrics, telemetry and optional RRD | Synchronous run; final observations are not resumable checkpoints |
| `hp-sim-3d/app/flightRecorder.js`, `flightRecorderSnapshot.js` and `scripts/hangprinter_flight_recorder.py` | Existing browser snapshots, acknowledged delivery and Rerun recordings | Browser recorder is a separate workflow, not automatically started by the research supervisor |
| 3D application controllers and JS physics modules | Existing scene, command, feature, timing and inspection logic | `createHpSimApp` currently returns only load/bind/start lifecycle methods |
| `tests/parity3d/oracle.mjs` and parity fixtures | Established way to exercise production JS physics and compare numerical snapshots | A test harness, not a general experiment service; reuse its production dependencies and contracts rather than adopting the entire oracle as the product API |
| `research/AGENTS.md` | Hypothesis/baseline/trial/comparison loop and domain-specific measurement rules | Ensure every entry route loads these instructions; instructions scoped to a subdirectory are not a universal research mode |

Source locations: [launcher](/home/torbjorn/repos/hp-sim5/scripts/research_agent.py:77), [MCP](/home/torbjorn/repos/hp-sim5/scripts/hp_sim5_mcp.py:1), [runtime dispatch](/home/torbjorn/repos/hp-sim5/scripts/research_runtime.py:170), [experiments](/home/torbjorn/repos/hp-sim5/src/python/hp_sim5_research/experiments.py:118), [browser bootstrap](/home/torbjorn/repos/hp-sim5/hp-sim-3d/app/appBootstrap.js:104), [browser recorder](/home/torbjorn/repos/hp-sim5/hp-sim-3d/app/flightRecorder.js:61).

**Architecture choices.**

| Choice | Assessment |
| --- | --- |
| Full Codex CLI with attached simulation services | Smallest immediate change. Preserve service setup; give the interactive CLI the terminal and forward normal Codex arguments. Keep batch mode for automation. |
| Codex desktop project with the same tools | Preferred visual workflow. Reuse its conversation UI and show the playground in its browser; keep native Rerun available. Requires durable MCP attachment to the supervisor. |
| Custom hp-sim chat over Codex app-server | Appropriate if hp-sim must be a standalone product. Reuses the Codex runtime, but requires maintaining conversation, event, approval, question and reconnect interfaces. |
| Rerun's built-in agent panel | Useful inspection-first alternative. Experimental; verify Codex behavior and hp-sim tool attachment before making it the primary research interface. |
| New outer agent that repeatedly invokes Codex | Adds another conversation and control loop. No demonstrated requirement here justifies that duplication. |

Codex's documented app-server provides bidirectional thread/turn control, streamed events, approval requests and user questions. Active steering uses `turn/steer` with an `expectedTurnId`; interruption uses `turn/interrupt`. A completed turn requires a new turn rather than steering. The documented WebSocket transport remains experimental. If a custom local client is needed, prefer a backend using stdio, with browser connections terminating at that backend. [Official app-server documentation](https://learn.chatgpt.com/docs/app-server).

The current SDK documentation distinguishes automation from rich clients and documents a Python SDK over app-server. Therefore an SDK is not inherently unsuitable: verify its exposed control methods. There is simply no need to add one when using Codex's own interface. [Codex SDK](https://learn.chatgpt.com/docs/codex-sdk).

**The smallest first implementation.** Preserve `RuntimeService`, Viewer startup and all existing MCP handlers. Add an interactive launcher path that runs the normal Codex CLI with inherited terminal input/output, a positional initial task and the existing per-session MCP settings. The current pipe-and-JSON event reader belongs in batch mode. Let Codex maintain the conversation transcript; retain hp-sim's experiment manifests and research report. Keep the supervisor alive across multiple conversational turns, and shut it down when the session ends. Forward Codex options rather than maintaining a separate list of model/session/research controls.

Use Codex Goal mode for autonomous continuation with a measurable outcome, constraints and a definition of done. It already accepts steering in the same session. This avoids implementing another continuation loop. [Long-running work](https://learn.chatgpt.com/docs/long-running-work).

For the desktop route, add a service-only supervisor mode and an MCP attachment wrapper that resolves its session descriptor. The current bare MCP registration is insufficient for continuing-world operations: `supervised_request` explicitly requires the launcher endpoint and token. Scope attachment to the research session/worktree, avoid silently sharing one mutable world across independent chats, and retain credentials outside model-visible results. Project-scoped MCP configuration is supported for trusted projects and shared across local Codex clients. [MCP configuration](https://learn.chatgpt.com/docs/extend/mcp?surface=cli).

A tools-and-skills plugin is a useful eventual packaging format, not a prerequisite for the first implementation. Keep the existing research instructions as the source of truth and have a small research skill load them. [Plugin architecture](https://developers.openai.com/plugins/concepts/plugins).

**Access to every playground.**

Use native Python for repeatable numerical trials and online native HP4/RRF collection. Use the JS engine to investigate JS behavior and parity. Use the actual browser application for UI, worker, firmware/browser integration and human interaction. Label every run with its backend; matching scenes do not make separate running worlds identical.

For the 3D browser, expose a narrow experiment API around existing controllers: capabilities, current scene/state, reset/load, command submission, pause, bounded fixed-step advance, numerical observation and recording. Add the equivalent adapter to the 2D `hp-sim` application only where its experiments require it. Retain different capabilities where the engines differ rather than pretending all backends support the same operations.

In Codex desktop, WebMCP site tools can expose that API from the exact page the user has open. Feature-detect support and verify the installed client/model rollout. For CLI/headless access, use a local MCP adapter to the same application API. Ordinary browser automation remains useful for UI verification. [Site tools documentation](https://learn.chatgpt.com/docs/webmcp).

The existing 3D flight recorder already converts browser snapshots into Rerun recordings. Add its receiver to supervisor ownership when requested, route it to the assigned Viewer, and reuse the snapshot format. Opening Vite alone currently provides neither that receiver nor a shared native world. A 2D recorder would be additional work, not an existing feature.

Use the native Rerun Viewer MCP for inspection. Its tools control the Viewer, not physics and not raw numeric queries. Read numbers through hp-sim telemetry, the Viewer catalog, or `rerun.chunk.RrdReader` as appropriate. Prefer high-level `rerun_*` tools. [Rerun Viewer MCP](https://rerun.io/docs/reference/viewer/mcp).

For an embedded display, Rerun's JS viewer package supplies programmable control; an iframe is simpler but lacks that control. Do not assume a browser viewer has the native Viewer's MCP service or automatically shares its cursor. [Embedding Rerun](https://rerun.io/docs/howto/integrations/embed-web). The native agent panel is explicitly experimental in [Rerun 0.38](https://rerun.io/docs/changelog/changeset-0-38).

**Steering needs an experiment contract as well as a chat contract.**

The current supervisor rejects competing mutations while busy. `collect_sweeps` can run for minutes, and cancellation of the Codex turn does not prove that firmware queues, collector threads and native physics have stopped. Make long collections jobs with start/status/cancel and cooperative cancellation at documented safe boundaries. Retain a simple synchronous path for short trials. Chunked stepping is useful only if it preserves the same continuing world and exact timestep schedule.

Separate three user actions: steer the research objective, pause/cancel execution, and scrub a recording. Rerun playback pause does not pause simulation. An accepted steering message is not evidence that an experiment was cancelled. Report the actual stop boundary and steps executed, flush partial evidence, and record cancelled runs as cancelled. If recovery cannot preserve valid world/firmware references, require the existing coordinated reset and say so.

When the user says “investigate this”, capture the relevant backend/session/run, recording, scene generation, timeline/time and selected entity or measurement alongside their words. Capture at message submission so subsequent navigation cannot change its meaning. Passive camera or timeline navigation should not itself change the research objective. Log human interventions so baseline/trial comparisons remain interpretable.

**Research and efficiency.** Keep the existing hypothesis → baseline → informative trial → measurements → revision loop. Maintain a small persistent research record with objective, current hypothesis, constraints, experiment IDs, accepted steering and next decision. For autocal, judge held-out sweep prediction and physical parameter accuracy separately from the fitting score. Bound experiment count, movement and compute separately from the agent's token budget.

Let bulk computation stay in the existing simulation/collector code. Return compact metrics, warnings and artifact paths; query selected time windows for detail. Keep numerical-only trials available and enable full recording for failures and selected candidates. Preserve the existing every-step recording contract when recording is requested. Apply the repository Rerun skills' schema inspection, scoped queries, reader/lens processing and screenshot verification when relevant; importing a cloud catalog or dataset platform is unnecessary for the current local workflow.

**Acceptance checks for implementation.**

1. During a multi-trial task, steering changes subsequent experiment choice while preserving the same Codex conversation and experiment history.
2. During a long collection, cancel returns a known execution boundary, stops further movement, preserves partial evidence and leaves an explicit usable-or-reset-required state.
3. A user-selected browser/native run and simulation time are the same ones inspected by the agent; backend identity is visible.
4. Existing numerical replay, collector, RRD lifecycle and JS/Python parity checks still pass for changed paths.
5. Resume reconnects to a valid service session or explicitly starts a fresh world; it never presents a final snapshot as a restored checkpoint.

This investigation confirmed the steering/interrupt fields in schemas generated from installed Codex 0.160.0. It inspected source and existing verification records, but did not establish live desktop attachment, WebMCP availability, Rerun panel feature parity or cancellation correctness. Those are concrete implementation acceptance checks, not reasons to replace the existing architecture.
