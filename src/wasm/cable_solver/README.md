# Rust/WASM cable physics

This dependency-free Rust crate implements the full cable constraint iteration
loop and the attachment tangent geometry. JavaScript retains scene construction,
ECS storage, winding/topology/history updates and the other simulation systems.
Each constraint update packs live state once, calls WASM once for every solver
iteration, then writes back the changed poses, encoders, forces and cable loads.
Attachment geometry uses a separate small reusable buffer and one call per joint.
No worker, transport, JSON serialization or ECS callback runs inside the solver.

Build from the repository root:

```bash
rustup target add wasm32-unknown-unknown
npm run build:wasm
```

The build copies the release artifact into `public/wasm/cable_solver.wasm`. This
artifact is checked in so running the simulator and tests does not require Rust.
Rebuild it whenever the Rust source changes. Cargo has no external dependencies.
The current build uses Rust 1.98.1, strict f64 arithmetic, release optimization
and LTO; it does not enable fast math or approximate physics.

Use the optional backend in Node:

```js
import fs from 'node:fs';
import { compileCableSolverWasm, WasmCableSolver } from './src/js/cable_joints_3d/wasm_cable_solver.js';
import { createHeadlessWorld } from './hp-sim-3d/app/headlessWorld.js';

const module = await compileCableSolverWasm(fs.readFileSync('public/wasm/cable_solver.wasm'));
const sceneText = fs.readFileSync('public/usd_scenes/hp4_rigid_body.usda', 'utf8');
const instance = createHeadlessWorld(sceneText, {
  cableSolverBackend: new WasmCableSolver(module),
});
instance.world.update(instance.dt);
```

Compile a module once and create a backend for each world. A backend refreshes
all component and topology reads each update, including after clear/replacement.
Its buffers grow with topology and otherwise reuse their capacity. The browser
API uses the same module compiled from `(await fetch(url)).arrayBuffer()` and
passes the backend to `registerSimulationSystems`. Browser startup remains JS
by default; no asynchronous initialization was added to the app.

`sin`, `cos`, `asin`, `acos` and `atan2` use the host's Math functions to retain
the existing physics rounding. Three-component vector lengths run in WASM using
a scaled compensated sum matching V8's three-argument `Math.hypot`; two-component
lengths use its scaled two-term sum. Exact parity is measured on Node 24/V8 13.6.
Other JavaScript engines require their own numerical and performance checks.

Run the every-timestep physics and lifecycle checks:

```bash
node --test tests/benchmark/wasm_cable_solver.test.mjs
```

The [profiling recipe](../../../tests/benchmark/HEADLESS_JS_PROFILING.md) explains
how to freeze workloads. Use `--solver wasm` with `profile_headless.mjs` and
`--compare` a JS baseline. The [matrix runner](../../../scripts/benchmark_cable_wasm.mjs)
runs three sequential repeats per backend for the eight original workloads:

```bash
node scripts/benchmark_cable_wasm.mjs --inputs DIR --output NEW_DIR
```

`DIR` must contain `hp3-logo.json`, `hp4-logo.json`, `hp3-autocal.jsonl` and
`hp4-autocal.jsonl`. Comparisons require exact checkpoint hashes; backend choice
is recorded separately from the shared workload contract. Outputs record the
Rust/JS source hash and WASM artifact hash. Plain physics timers include packing,
unpacking and GC, and exclude construction, observations and firmware I/O.
