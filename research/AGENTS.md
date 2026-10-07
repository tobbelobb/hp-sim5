Use `.venv/bin/python`. Read `research/README.md` for tools and
`research/native-collection.md` before online HP4/RRF collection.

## Research

- Choose the backend and experiment from the user's question. `run_experiment`
  creates a fresh native Python world; online collection uses the continuing
  launcher-owned Python/RRF session; browser work uses a separate JS world. Call
  MCP `capabilities` first.
- State a measurable success criterion. Compare a baseline and candidate with the
  same scene, timestep and command schedule, changing only what the hypothesis
  requires. Read numeric telemetry and provenance; a successful run, plausible
  recording or lower fitting score alone is not evidence of better behavior.
  Retain failed trials and report what the measurements support.
- Native commands use metres, newtons, Nm and absolute motor angles in radians.
  Each command consumes one fixed timestep; `{}` holds the previous targets.
  Snapshots are not resumable checkpoints, and MCP timings include telemetry and
  optional Rerun overhead. Use `tests/benchmark/hp4_logo.py` for throughput claims.
- Inspect Rerun recordings at the relevant `sim_step`/`sim_time` when useful, but
  use experiment telemetry for numeric claims. If no Viewer is available, continue
  numerically and say visual inspection was skipped. The launcher owns its Viewer
  and supervised services.
- Keep session `research.md` current with the objective, hypothesis, constraints
  and budgets, experiment IDs, accepted steering and next decision. Finish with a
  report linking evidence and conclusions. Use Goal mode only when the user asks
  for autonomous continuation.

## Continuing collection and browser work

- Collection advances the continuing native world while it waits. A G-code reply
  means RRF accepted/planned a command, not that motion finished; use `M569.3` to
  drain queued motion and check settling before interpreting measurements. Sim
  time differs from wall time. On failure or cancellation, inspect the stopped
  step and partial evidence, then call `reset_session` before more movement.
  Neither interrupting Codex nor pausing Rerun playback stops a job: cancel it
  explicitly. Hardware movement requires an explicit user request.
- Use `start_browser_service` for the shared page; do not start Vite yourself.
  Browser JS and native Python are separate simulations. Get the page's `page_id`
  from `browser_status` and use it for every action. Before bounded stepping or
  direct motor commands, pause the simulation and finish active workers. Start the
  service with `record=True` when recording a browser run; inspect native RRDs in
  Rerun.
- When asked to investigate a selected live state, capture it immediately with
  `capture_native_context` or browser `capture_context`, including the selected
  entity/measurement and simulation step. Base follow-up on that frozen
  observation; later navigation or movement does not change it.

## Autocal

Read `autocal/README.md` and relevant fitting, active-learning and dataset-role
code before changing calibration. Hold out whole sweeps/configurations. Judge
candidates by physical anchor/radius error and held-out sensor prediction, as well
as score, failures, movement count and runtime. Keep autocal millimetres separate
from native physics metres. Follow the task prompt and
`research/native-collection.md` for workflow and collection limits.
