Use `.venv/bin/python` and keep source changes small. Read `research/README.md`
for the tools and `research/investigation.md` for the integration rationale.

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
`autocal/tools/regress_calibration_logs.py`. The native motor experiment API does
not replace the one-click browser/firmware sweep collector. Use recorded datasets
for offline fitting experiments; use the existing `--sim` collector workflow
when validating online collection. Keep autocal millimetres separate from native
physics metres. Hardware movement requires an explicit user request.

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
