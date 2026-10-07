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
