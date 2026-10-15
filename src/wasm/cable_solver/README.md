# Rust physics core

The browser uses a continuing Rust/Wasm world. Local headless worlds, collection
workers, fresh research trials and the profiling CLI default to native Rust. One Rust step owns body integration, position and torque motors,
spool projection and reaction torque, cable attachment geometry, winding and layer
ramps, split/merge topology, hybrid transitions, friction, XPBD constraints,
over-correction, velocity reconstruction, encoders and missed-step counters.
The same dependency-free crate builds as native Rust libraries (`rlib`, `staticlib`
and `cdylib`); neither host implements separate physics.

JavaScript parses and bakes scenes, prepares the world, dispatches commands and
handles transport, interactive edits, rendering, extrusion observations and
recorders. Numeric ECS vectors and scalars view Rust memory directly. Physics
steps pass resource settings and call Rust once; they do not repack entities or
copy poses back. Structural ECS edits rebuild the prepared world. Numeric edits
write the shared records, and topology changes reconcile cable entities and views.
Each world has its own instance and state. Views refresh when Wasm memory grows;
native path buffers detach before topology changes return to JavaScript. Native
allocations remain alive until their views are collected, including escaped views.

```bash
npm run build:native
npm run build:wasm
cargo build --manifest-path src/wasm/cable_solver/Cargo.toml --release
cargo test --manifest-path src/wasm/cable_solver/Cargo.toml
node --test tests/benchmark/native_physics_engine.test.mjs tests/benchmark/wasm_cable_solver.test.mjs
```

The checked-in `public/wasm/cable_solver.wasm` runs without a Rust toolchain.
Rebuild it after changing Rust. `wasm32-unknown-unknown` is the Wasm target.
Arithmetic remains f64 with strict floating-point operations. Trigonometry and
exponentials execute in Rust on both targets; the Wasm module has no JavaScript
imports. Small rounding differences are expected. No timestep, solver iteration,
compliance, layering or settling simplification is made.

Build native execution once with `npm run build:native` (Rust, a C compiler and
Node development headers are required; Linux/macOS build recipe). Set
`NODE_INCLUDE_DIR` if `node_api.h` is outside the usual include directories.
Release and profiling builds use PGO by default: install the matching LLVM tools
with `rustup component add llvm-tools-preview`. The build trains four HP3/HP4
worlds for 40,000 fixed updates, then recompiles using their profile. It targets
the host CPU unless `RUSTFLAGS` already specifies `target-cpu`.
Use `npm run build:native -- --no-pgo` for an untrained portable build, or set
`RUSTFLAGS='-C target-cpu=x86-64'` for PGO with a portable x86-64 target.
Dev builds default to untrained. Explicit `profile-generate`/`profile-use` flags
require `--no-pgo`; instrumentation links Rust's profiling runtime into the addon.
The generated `target/native/physics.node` uses Node-API 8 and statically links
this crate. Its adjacent JSON records the artifact hash, source hashes, compiler
and build profile, plus the PGO profile/trainer hashes and training settings.
Generated host artifacts are ignored by Git. Missing or stale
default builds fail with a rebuild instruction; there is no automatic Wasm fallback.

In Node, `createHeadlessWorld(sceneText)` loads that native addon synchronously,
then creates a full engine per world. The common `rust_physics_engine.js` prepares
and binds the same records for both `NativePhysicsEngine` and `WasmPhysicsEngine`.
Native views use external Float64Array buffers; steps do not copy entity state.
In a browser, await `loadDefaultCableSolverBackend()` before scene registration,
as appBootstrap does. Browser rendering, edits and connections remain JavaScript.

For an explicit JavaScript reference, use `createHeadlessWorld(sceneText,
{ cableSolverBackend: null })`, `?solver=js` in the app, or `--solver js` in the
profiler. Pass `new WasmPhysicsEngine(module)` for a Wasm world in Node.
`WasmCableSolver` remains a standalone kernel adapter for differential fixtures.
The profiler offers `--solver native|wasm|js`, plus `--native-addon PATH` to select
an exact native build. `HP_SIM5_NATIVE_ENGINE=PATH` selects a custom native artifact
for local services. These explicit artifacts may come from other revisions; their
actual artifact hashes and build metadata are recorded.

For native symbol profiling, build a separate optimized artifact with DWARF:

```bash
npm run build:native -- --profile profiling --output output/native-profile/physics.node
perf record -g --call-graph dwarf -- node scripts/profile_headless.mjs \
  --solver native --native-addon output/native-profile/physics.node \
  --commands INPUT.json --output output/native-profile/run.json
```

`--profile release` is the default; `--profile dev` builds without optimization.
The profiler never builds an artifact. V8 CPU/heap profiles describe the JS host;
use a native profiler to inspect Rust frames. Record plain timings separately.

## Native boundary and prepared records

Native callers use `hp_sim5_cable_solver::engine::PhysicsWorld::new(entities,
paths)`, set `options`, then call `step(dt)`. The core retains state and exposes
entity records and `path_data(index)` for observations or edits. There are no
browser callbacks, JS transcendental imports, scene parsers, sockets or logging
dependencies in this API. The C ABI in `abi.rs` only allocates/accesses that world.
The Node-API adapter in `native/node_bridge.c` supplies the same prepared scene
and motor records through this ABI, and owns only validation, views and lifetime.
Native Rust tests and native/Wasm fixture tests exercise continuing integration,
cables/forces, both motor modes, scene edits and topology changes.

Records use f64 slots, including flags and indices. Entity references in prepared
paths and parents are indices into the entity array; public entity IDs remain in
slot 128. The JavaScript preparation/binding code in `rust_physics_engine.js` is the
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
