Use `.venv/bin/python` and keep source changes small. Read `research/README.md`
for the tools and `research/investigation.md` for the integration rationale.

Choose experiments and collections according to the user's prompt and your
current hypothesis. The launcher prepares tools; it does not prescribe an
autocal collection or run experiments before you start.

You can perform experiments yourself. For design or optimization tasks, define a
measurable hypothesis, run a baseline, change one relevant variable, run a trial,
and compare the observations. Keep failed trials and revise the hypothesis when
the evidence contradicts it. Finish with the best supported implementation and
a report linking the actual experiment IDs, inputs, measurements and checks.

Call hp-sim5 MCP `capabilities` first. `run_experiment` constructs a fresh native
world, advances fixed physics steps and saves per-step numeric telemetry and an
optional Rerun recording. Each JSON command consumes one timestep; `{}` holds
the previous motor targets. Move targets are absolute motor angles in radians,
not XYZ positions. Geometry is metres, forces newtons, torque Nm. Compare the
same initial scene, timestep and command schedule unless changing one of those
is the hypothesis. `compare_experiments` lists changed inputs and physical
differences; you must supply the task's success criterion.

Use `read_experiment` for actual numbers. A successful process, a plausible
screenshot or a smaller fitting score alone does not establish better robot
behavior. Native snapshots are observations, not resumable checkpoints. Timings
from the MCP include numeric sampling and optional Rerun overhead; use
`tests/benchmark/hp4_logo.py` for physics performance claims.

When Rerun MCP is available, open the returned RRD in its assigned Viewer, inspect
the blueprint and warnings, move to the relevant `sim_step`/`sim_time`, and save
a screenshot inside the session directory when it adds evidence. Prefer
`rerun_*` tools. Close stale recordings before repeatedly opening replacements.
The launcher owns its Viewer process; leave other Viewers alone. In numeric mode
continue with telemetry and state explicitly that visual inspection was skipped.

For autocal research, inspect `autocal/README.md`, `autocal/active_learning.py`,
`autocal/ellipse_active.py`, `autocal/dataset_roles.py` and
`autocal/tools/regress_calibration_logs.py`. Use `start_collection`, `collection_status` and `cancel_collection` for online
native HP4/RRF collection: it runs the existing collector against a continuing
Python world. `runtime_status`, `send_gcode`, `step_physics` and `reset_session`
operate that same session. A G-code reply acknowledges firmware planning; an
M569.3 read drains queued motor commands, and collector settling has a
simulation-time bound. Collector waits advance fixed physics steps, so simulation
time differs from wall time. Failed or cancelled collection preserves evidence and requires
`reset_session`. Poll jobs while continuing the conversation. An interrupted Codex
turn does not stop movement: explicitly cancel the active job and inspect its
actual step boundary, partial artifacts and reset-required state before changing
the experiment. Cancel freezes native physics before its next fixed step and
retires RRF/bridge queues. Rerun playback pause only affects inspection. The launcher owns RRF, its bridge and optional Vite outside your
command sandbox; use `start_browser_service` instead of starting Vite yourself.
That web scene is independent of native physics. Inspect native live/saved
recordings in Rerun. Read `research/native-collection.md` for units, references
and validation limits. Keep autocal millimetres separate from native physics
metres. Hardware movement requires an explicit user request.

For the structured trial-and-error calibration task, investigate a bounded
predict → choose informative movement → observe encoder residual → adjust →
validate loop. Hold out entire sweeps or configurations to avoid point leakage.
Measure anchor/radius error on synthetic ground truth, held-out prediction error,
failure rate, movement count and runtime separately from the selection score.
Test noise, outliers, cable elasticity, winding buildup and bad initial guesses.
Preserve the one-click UX and the existing automatic termination path. Accept a
candidate only when its selected result improves those measurements; detect and
report cases where score ranking disagrees with physical accuracy. Do not claim
a new calibration algorithm works until actual collector and regression
evidence supports that claim.


Research runs in an ordinary Codex conversation. Use Codex Goal mode when the
user asks for autonomous continuation; do not build an outer prompt loop. Keep
`research.md` in the supplied session directory current with objective,
hypothesis, constraints, movement/experiment/compute budgets, experiment IDs,
accepted steering and the next decision. Update it when steering changes the
experiment choice. Preserve the conversation and prior evidence across turns.

For research-run conversations, when the Codex app sidebar tools are available,
use `list_threads` to find the custom section named `Autonomous research`, create
it only if missing, and move your own conversation into it with
`move_thread_to_sidebar_section`. Identify your exact current thread from the
client context or `CODEX_THREAD_ID`/`CODEX_SESSION_ID`; never guess from the most
recent chat. Reuse the section on resume. Do not move other conversations or
change the working directory to achieve categorization. When these tools or the
current thread identity are unavailable, state that sidebar organization needs
the desktop app; do not claim it happened.

For shared browser work call `start_browser_service(record=True)` when recording
is needed, open its exact URL, inspect `browser_status`, and pass that page's
`page_id` to `browser_action`. Browser JS, standalone JS parity fixtures, and
native Python are distinct backends. The browser API uses the existing scene,
worker, motor, timing and inspection controllers. Pause before bounded stepping;
finish active workers before direct motor commands. The optional WebMCP site
tools operate the same API in the open desktop page; use local MCP when those
site tools are unavailable. The supervisor owns the requested browser flight
recorder and sends recordings to its assigned Viewer.

When accepting “investigate this”, immediately capture the selected native run
or continuing session with `capture_native_context`, or the selected browser
page with `browser_action(action="capture_context")`. Include the user's words,
selected entity/measurement and selected `sim_step`. Browser users can also use
“Capture for research”; `browser_status` exposes those immutable submissions.
Inspect the captured observation/time, not whichever state is current later.
Navigation alone does not change the objective. Browser mutations and human
control interventions are retained with their state/time for comparisons.

Resume restores the Codex conversation, not a physics checkpoint. The CLI
launcher explicitly starts a fresh world. Desktop attachment reconnects only
to an explicitly selected live supervisor; an ended supervisor requires a new
one. Never represent a final snapshot as restored simulation state. Use a
separate supervisor and worktree for an independent chat.
