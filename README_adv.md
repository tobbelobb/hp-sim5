# hp-sim5 Advanced Guide

This guide expands on the main `README.md`. It covers the shared Rust physics
engine, the 3D application layer and the headless research workflows.

## Physics Engine and Cable Joints Library

The simulator uses one Rust implementation of the cable, rigid-body and motor
pipeline. The same crate compiles to:

- a WebAssembly module loaded by the browser and headless Wasm benchmarks;
- a native Node addon used by research, autocal, profiling and regression tests.

JavaScript owns USD scene construction, the ECS application layer, command
dispatch, rendering and interactive state. Python is used for research tooling,
autocal, tests and Rerun recording, but it does not contain a second physics
engine.

The engine implements (extended) Position-Based Dynamics. Cable segments slide
over wheels, wrap, and maintain tension through constraints solved with XPBD.
Cable paths, rigid assemblies, spools and motors are described in
[`hp-sim-3d/README.md`](hp-sim-3d/README.md).

### Repository Layout

- `src/wasm/cable_solver/` - shared Rust physics crate and Node-API bridge.
- `src/js/cable_joints_3d/` - ECS, scene-facing components, Rust host adapters,
  rendering and geometry helpers.
- `hp-sim-3d/` - browser application, machine catalog, command players and
  headless world composition root.
- `src/python/hp_sim5_research/` - Python research tools, observation mirror and
  Rerun recording. Physics runs in the Node worker.

## Headless Experiments

Use `hp_sim5_research.experiments.run_experiment` for bounded, repeatable trials.
It writes a manifest, the frozen scene, commands, per-step telemetry, a final
snapshot and an optional Rerun recording:

```bash
PYTHONPATH=src/python .venv/bin/python - <<'PY'
from hp_sim5_research.experiments import read_run, run_experiment

manifest = run_experiment('.', steps=200)
rows = read_run('.', manifest['run_id'], 0, 5)['samples']
print(manifest['physics_engine'], manifest['metrics'])
print(rows[-1]['effectors'])
PY
```

The MCP tools described in [`research/README.md`](research/README.md) expose the
same workflow to agents. Continuing collection, full autocal and test jobs run
through the launcher-owned supervisor.

Native commands use motor radians and metres of extrusion. The headless worker
does not consume G-code directly; use the firmware command players or the
browser uploader.

### World Configuration

`configure_session` selects the stock HP3/HP4 scene, layering, firmware
configuration and physical spool radii. The configuration is validated before a
new world is prepared. Layered worlds use the stock radii and buildup; custom
radii require layering off.

### Recording

Set `record=True` in a fresh experiment or start a browser recording from the
3D app. RRD files can be opened with `.venv/bin/rerun PATH.rrd`. See
[`hp-sim-3d/FLIGHT_RECORDER.md`](hp-sim-3d/FLIGHT_RECORDER.md) for the browser
recorder and manual checklist.

## 3D Application Implementation Notes

The 3D Hangprinter app lives in `hp-sim-3d/`. Scene interpretation is split
between the coordinator in `app/setupScene.js`, the builders in `app/scene/`,
and system registration in `app/sceneSystems.js`. The ECS uses full 3x3 inertia
tensors in constraint and motor reaction paths. Spool rotors use a specialized
one-axis projection rather than independent rotor bodies and generic hinge
constraints.

The Rust engine owns the numeric solve. Feature flags such as line layering and
closed-loop motors are validated resources that the host adapter forwards to
Rust. Changing a flag invalidates the prepared world so the next step rebuilds
its bindings.

See [`hp-sim-3d/README.md`](hp-sim-3d/README.md) for the current system order,
cable behavior, rigid-body model and remaining hinge-joint gap.
