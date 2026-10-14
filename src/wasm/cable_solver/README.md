# Rust physics core

The browser, headless collection workers and profiling CLI default to a continuing
Rust/Wasm world. One Rust step owns body integration, position and torque motors,
spool projection and reaction torque, cable attachment geometry, winding and layer
ramps, split/merge topology, hybrid transitions, friction, XPBD constraints,
over-correction, velocity reconstruction, encoders and missed-step counters.
The same dependency-free crate builds as a native Rust library (`rlib`).

JavaScript parses and bakes scenes, prepares the world, dispatches commands and
handles transport, interactive edits, rendering, extrusion observations and
recorders. Numeric ECS vectors and scalars view Rust memory directly. Physics
steps pass resource settings and call Rust once; they do not repack entities or
copy poses back. Structural ECS edits rebuild the prepared world. Numeric edits
write the shared records, and topology changes reconcile cable entities and views.
Each world has its own instance and state. Views refresh when Wasm memory grows.

```bash
npm run build:wasm
cargo build --manifest-path src/wasm/cable_solver/Cargo.toml --release
cargo test --manifest-path src/wasm/cable_solver/Cargo.toml
node --test tests/benchmark/wasm_cable_solver.test.mjs
```

The checked-in `public/wasm/cable_solver.wasm` runs without a Rust toolchain.
Rebuild it after changing Rust. `wasm32-unknown-unknown` is the Wasm target.
Arithmetic remains f64 with strict floating-point operations. Trigonometry and
exponentials execute in Rust on both targets; the Wasm module has no JavaScript
imports. Small rounding differences are expected. No timestep, solver iteration,
compliance, layering or settling simplification is made.

In Node, `createHeadlessWorld(sceneText)` synchronously loads the checked-in
module once, then creates a full engine per world. In a browser, await
`loadDefaultCableSolverBackend()` before scene registration, as appBootstrap does.
For an explicit JavaScript reference, use `createHeadlessWorld(sceneText,
{ cableSolverBackend: null })`, `?solver=js` in the app, or `--solver js` in the
profiler. `WasmCableSolver` remains a standalone constraint/geometry adapter for
isolated differential fixtures; production defaults use `WasmPhysicsEngine`.

## Native boundary and prepared records

Native callers use `hp_sim5_cable_solver::engine::PhysicsWorld::new(entities,
paths)`, set `options`, then call `step(dt)`. The core retains state and exposes
entity records and `path_data(index)` for observations or edits. There are no
browser callbacks, JS transcendental imports, scene parsers, sockets or logging
dependencies in this API. The C ABI in `abi.rs` only allocates/accesses that world.
A production native collection adapter can supply the same prepared scene and
motor records later. The three native tests exercise continuing integration,
cables/forces and both motor modes.

Records use f64 slots, including flags and indices. Entity references in prepared
paths and parents are indices into the entity array; public entity IDs remain in
slot 128. The JavaScript preparation/binding code in `wasm_physics_engine.js` is the
complete schema implementation. The solver's original 0–68 slots are documented
in `wasm_cable_solver.js`; additional entity fields are:

| Slots | Fields |
| --- | --- |
| 69–74 | Linear velocity, angular velocity |
| 75 | Flags: velocity 1, angular velocity 2, gravity 4, spool 8, motor 16, body 32, inertia 64, mass 128, cable link 256, local cable axis 512, radius 1024 |
| 76–89 | Spool local axis/reference quaternion, body synced position/quaternion |
| 90–103 | Attachment history position/world quaternion/local quaternion, plane normal |
| 104–106 | Radius, friction coefficient, member physical mass (NaN means absent) |
| 107–120 | Motor command, reference delta, holding torque, pole pairs, damping, speed limit, closed loop, torque target, windage, Coulomb friction, stiction torque/speed, cogging torque/frequency |
| 121–127 | Encoder axis, mass, missed-step maximum/current/reference offset |
| 128–135 | Public ID, machine index, reserved, member order, reserved, knot presence, reserved, count of path-specific knots |

Optional motor friction parameters and absent missed-step reference offsets use
NaN sentinels internally; these are not non-finite physical observations.

A path has a nine-slot header (joint count, public path ID, spring constant,
compliance, damping, iterations, half width, total rest length, machine index),
then `count + 1` five-slot links (kind, clockwise, stored length, knot angle,
knot initialization marker), then `count` sixteen-slot joints (public ID,
endpoint indices, rest length, two attachment vectors, lambda, force magnitude,
transferred magnitude, force vector). Kinds are attachment 0, rolling 1, pinhole 2,
hybrid 3, hybrid-attachment 4. Missing knot angles use NaN. Joint/link views must
refresh after a topology version change. Native records are valid prepared data;
constructors do not parse or validate arbitrary scene documents.

Options 0–2 hold gravity; 3 enables layering; 4 attachment updates; 5 rest-length
clamping; 6 merging; 7 splitting; 8 hybrid transitions; 9 grabbed public entity ID
(-1 for none); 10 next public ID; 11 closed-loop constraint stiffness; 12 open-loop
constraint stiffness scale; 13 holding torque scale; 14 global closed-loop mode;
15 constraint timestep (NaN selects the `step` argument); 16–18 default encoder
axis. Merge/split remain disabled by default, matching the shared JS pipeline.

## Measurements

Use the [profiling recipe](../../../tests/benchmark/HEADLESS_JS_PROFILING.md) and
[full engine report](../../../research/rust-wasm-engine.md). Repeats must have
identical checkpoint hashes within a backend. Cross-backend comparisons check
finite observations, matching structure and discrete metadata, and declared
absolute/relative tolerances. The profiler records both hashes and numerical
error statistics; it does not claim exact rounding parity. Long feedback replays
can exceed those tolerances. `--allow-numerical-drift` records the failed check
explicitly for a reviewed performance experiment. This does not establish
calibration accuracy or acceptance of the changed trajectory.
