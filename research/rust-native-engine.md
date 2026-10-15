# Native Rust engine

[Measured performance against Wasm](native-performance.md) covers physics,
recording and real firmware collections. Columnar logging removes much of the
recording overhead; profile-guided compilation further improves native physics.

Local headless worlds, continuing collection, full-autocal, fresh research
trials and the Python recording CLI now default to the native build of the
same Rust `PhysicsWorld` that runs in the browser as Wasm. No physics equations
were duplicated. JavaScript still prepares scenes, dispatches commands, exposes
observations and owns browser rendering, interactive edits and connections.

Build with `npm run build:native`. The Linux/macOS recipe needs Rust, a C compiler,
Node development headers (`NODE_INCLUDE_DIR` overrides their location) and
`rustup component add llvm-tools-preview` for the default PGO build.
Optimized builds train for 40,000 fixed HP3/HP4 updates with layering on/off,
position/torque motors and closed-loop transitions, then compile with that profile.
They target the local CPU unless `RUSTFLAGS` specifies `target-cpu`.
`npm run build:native -- --no-pgo` retains the portable untrained build; `--profile dev`
also defaults to untrained. Training runs only during an explicit build.
The generated Node-API addon statically links the dependency-free Rust crate.
Generated binaries remain under the ignored Cargo target directory. A missing
or stale default build fails with an explicit rebuild instruction.

The [common adapter](../src/js/cable_joints_3d/rust_physics_engine.js) supplies
identical prepared records to [native](../src/js/cable_joints_3d/native_physics_engine.js)
and [Wasm](../src/js/cable_joints_3d/wasm_physics_engine.js) hosts. The small
[Node-API bridge](../src/wasm/cable_solver/native/node_bridge.c) only owns host
validation, lifetime and external array views. Entities and options remain
views of Rust allocations; steps do not serialize/repack entities or copy poses
back. Split/merge invalidates old path buffers before JavaScript can observe
them, and the adapter rebuilds path bindings. Disposal detaches views; GC keeps
unreleased native storage alive until the last view goes away.

The collection transport name stays `headless-js` for compatibility. Status,
recordings, contexts and manifests identify `physics_engine: rust-native` and
the actual native artifact/build hashes. Source freshness checks cover Rust,
the host adapter, build recipe and binary. Continuing-world command ordering,
2 ms buckets, encoder observations, 50-step cancellation boundaries, coordinated
resets and recording contracts remain in the [collection service](native-collection.md).
Python supplies a detached telemetry mirror and never advances its physics in
the Rust path. Explicit Python and Warp backends remain available.

The profiler exposes `--solver native|wasm|js` and `--native-addon PATH`. A
separate `--profile profiling` build contains optimized code and debug symbols
for native profilers. It records the exact artifact, profile and source hashes;
it never rebuilds or silently substitutes engines. Build metadata includes the
PGO profile hash, training settings and trainer hash. Add `--no-pgo` to the build
command to profile an untrained artifact. See the
[profiling recipe](../tests/benchmark/HEADLESS_JS_PROFILING.md) and
[crate setup and record schema](../src/wasm/cable_solver/README.md).

CI builds native release without PGO and rebuilds Wasm before running the Rust
tests and Node backend suites. A source contract compares Cargo's dependency
files for the two compiled libraries and rejects conditional compilation in
the shared Rust sources. Extensive numerical native/Wasm comparisons are not
required: native tests cover integration, cables/forces, motors, command/encoder
smoke runs and host lifetime, including topology-invalidated views and forced GC.
JS/Wasm differential tests cover production-pipeline fixtures and eight authored
machines with layering on/off. The profiler and UI integration suites also run
in CI. `scripts/run_ci_tests.sh` runs these checks before Jest and Python tests.

The differential fixture selection includes friction, integration, standalone
motors and rigid members. Collision, optional distance-constraint and slack
systems remain in their separate JS/Python tests because hp-sim-3d's production
pipeline and Rust core do not register them. Motor-member fixtures also guard
the body-inertia fallback: parent-linked members absent from the body's member
list must not contribute to its aggregate inertia or body synchronization.
Long-run numerical drift and equivalent full-autocal outcomes are not CI
acceptance criteria; native and Wasm math can round differently. Earlier drift
against JS remains documented in the [Wasm report](rust-wasm-engine.md).

```bash
npm run build:native
npm run build:wasm
cargo test --manifest-path src/wasm/cable_solver/Cargo.toml
node --expose-gc --test tests/benchmark/native_physics_engine.test.mjs tests/benchmark/wasm_cable_solver.test.mjs tests/benchmark/profile_headless.test.mjs
.venv/bin/python -m pytest -q tests/python/test_research_agent.py tests/python/test_native_collector.py tests/python/test_world_configuration.py autocal/tests/test_headless_sim.py -m 'not slow'
```
