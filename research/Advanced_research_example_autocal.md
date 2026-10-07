Autocal research is an example of what you can do.
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
