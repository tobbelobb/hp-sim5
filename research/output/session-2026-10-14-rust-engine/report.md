# Fuller Rust/Wasm engine, 2026-10-14

The full Rust/Wasm engine is 2.37–2.86 times faster than the previous Wasm default
on eight frozen HP3/HP4 logo and autocal workloads. Three repeats per backend
cover 7,544,232 measured updates. Rust owns persistent physics state, winding,
topology, constraints, friction, integration, motors and encoders. JavaScript
retains scene preparation, commands, edits and observations. The same source
builds and steps natively. Short differential checks and bounded one-step forks
agree tightly, but five long feedback cases fail the initial pointwise tolerance.
The layered HP3 autocal replay reaches 2.85 degrees of encoder-read difference;
this is recorded as drift. Repeats remain deterministic within each backend.
These are simulated results from one host, with no hardware or fresh adaptive
calibration accuracy claim.

## Implementation

[PhysicsWorld](../../../../src/wasm/cable_solver/src/engine.rs) implements body integration,
position/torque motors and reactions, spool projection, attachment geometry,
winding and layer ramps, merge/split topology, hybrid transitions, attachment
history, friction, XPBD constraints, over-correction, velocity reconstruction,
encoders and missed-step counters. Its prepared records have no browser or ECS
callback dependency. The [ABI](../../../../src/wasm/cable_solver/src/abi.rs) wraps the same
native world. `Cargo.toml` builds `rlib` and `cdylib` targets.

The [bridge](../../../../src/js/cable_joints_3d/wasm_physics_engine.js) binds numeric ECS
vectors/scalars directly to Rust memory. A step updates resource settings and
calls Rust once. Entity state is not repacked or copied back each step. Structural
ECS edits rebuild the prepared state; topology changes reconcile cable entities
and views. Memory growth refreshes views. Load-map observations are lazy and
retain their identity until the next step, including paused scene appends.

The Wasm module has no imports. Rust evaluates f64 trig and exponentials locally.
No timestep, iteration, compliance, layering or settling shortcut was introduced.
The final artifact is about 180 KB. Matrix/friction operation order follows the
JS reference, and rigid cables retain its finite torque-load coefficient policy.

Production registration is now RemoteSpoolSystem → RustPhysicsSystem →
ExtruderSystem → optional recorder. Rendering, extrusion observations and
connection management remain in JavaScript. Explicit JS reference selection is
`?solver=js`, `--solver js`, or `cableSolverBackend: null`. Collection transport
keeps its `headless-js` contract; `physics_engine: rust-wasm` names the core.
Manifests and stale-source checks include Rust and the Wasm artifact.

## Plain measurements

Three fresh-world repeats per backend, with a separate 5,000-step warmup.
**48 trajectories, 7,544,232 measured updates.** Baseline Wasm/bridge came from
`8e0f788d`, using the current unchanged JS physics reference systems. Inputs are
from the original [profiling session](../../../../output/research/sessions/2026-10-13-headless-js-profiling/inputs/).
Timers include JS command dispatch, Rust stepping, view maintenance, extrusion
observations and GC. Construction, observations, firmware, transport, rendering
and recording are excluded. Node 24.14.1/V8 13.6, i7-12700K, sequential runs.

| Workload | Previous seconds | Full seconds | Speedup | Pointwise check |
| --- | ---: | ---: | ---: | --- |
| hp3-logo-on | 2.204 | 0.931 | 2.37× | Pass |
| hp3-logo-off | 1.952 | 0.795 | 2.45× | Drift |
| hp3-autocal-on | 35.080 | 12.869 | 2.73× | Drift |
| hp3-autocal-off | 35.016 | 12.940 | 2.71× | Drift |
| hp4-logo-on | 9.994 | 3.600 | 2.78× | Pass |
| hp4-logo-off | 8.401 | 2.935 | 2.86× | Pass |
| hp4-autocal-on | 25.584 | 9.420 | 2.72× | Drift |
| hp4-autocal-off | 25.631 | 9.334 | 2.75× | Drift |

The final 24 full-engine repeats reuse the preceding 24 baseline measurements;
they ran later on the same host. Their checkpoint hashes match the preceding
full-engine matrix exactly. Native tests, profiles and other tests did not
interleave plain timing. Other desktop/research processes were not stopped;
there is no CPU-affinity or cross-host result. Throughput ratios do not establish
matching trajectories in cases marked Drift.

## Numerical results

Every repeat within a backend has identical checkpoint hashes. Comparisons
require finite values, matching structures, clocks, IDs and command metadata.
The declared mixed-unit tolerance is `atol=1e-6, rtol=1e-6`. Report-only drift was
selected after reviewing the failed trial; failed checks remain false in results.

The three passing logo cases have length differences below 2.3e-13 m and motor
encoder differences below 5.2e-13 rad. The unlayered HP3 logo reaches 3.99 mm in
sampled geometry, 0.00311 rad in sampled encoders and 1.55 N in cable forces.
The layered HP3 autocal replay reaches 0.577 mm in geometry, 0.0131 N in forces
and **2.85 degrees at its 1,956 recorded encoder reads**. Its unlayered counterpart
is a stress replay of a trace recorded with layering, and differs physically even
in the baseline; its original-read error is not an engine-parity measure.
HP4 autocal length differences stay below 0.88 micrometres, sampled motor encoder
differences below 3.5e-5 rad, and force differences below 0.00061 N. Free-wheel
rotations and angular velocities also differ; raw quaternion component errors
include equivalent q/−q representations. The [rotation analysis](../../../../output/research/sessions/2026-10-14-rust-engine/rotation-errors.json)
measures angular differences independently of quaternion sign.

Two bounded fork checks isolate one-step equations from feedback amplification.
For 3,000 unlayered HP3 logo steps, synchronize from the exact JS state before
each step: maximum component error is 2.86e-12 N. For 1,000 steps beginning at
step 260,000 of the layered autocal trace, maximum error is 4.59e-15 rad/s.
These support close one-step physics, while the long-run pointwise result is
mixed. They do not prove calibration accuracy or bound all possible trajectories.

The profiler formerly stored references to live velocity components in earlier
checkpoints. Both comparison harnesses now detach velocities when observed.
This corrects checkpoint timing and changes hashes without changing stepping.

## Verification and evidence

- 80 Node checks cover standalone kernels, the full pipeline, all eight scene
  designs with layering on/off, scene replacement/append, shared joints, topology,
  motor modes, rigid-cable coefficients, memory growth, clear/reused IDs and edits.
- Three native Rust tests exercise continuing integration, cables/forces and
  position/torque motors. Native release build and Clippy with warnings denied pass.
- Chromium HP3/HP4 checks cover 200 commanded steps each; maximum position error
  is 1.06e-13 m and encoder error 1.21e-11 rad against JS.
- Python checks: 401 passed, one skipped and eight deselected in the broader
  parity/recording/autocal-job suite; 72 research/collection/configuration checks
  passed across the initial run and targeted corrections. Slow live movement
  tests were excluded. The initial missing-artifact test fixture was updated.
- Jest's full run had 762 passes and two obsolete wiring expectations; their
  affected suites now pass (29 checks). One existing test remains skipped.

[Final matrix](../../../../output/research/sessions/2026-10-14-rust-engine/matrix-precision/matrix.json),
[baseline/raw checkpoints](../../../../output/research/sessions/2026-10-14-rust-engine/matrix-final/),
[early fork](../../../../output/research/sessions/2026-10-14-rust-engine/one-step-forks.json),
[late fork](../../../../output/research/sessions/2026-10-14-rust-engine/late-forks.json),
[browser check](../../../../output/research/sessions/2026-10-14-rust-engine/browser.json),
[source manifest](../../../../output/research/sessions/2026-10-14-rust-engine/final-source-manifest.json)
and [notebook](../../../../output/research/sessions/2026-10-14-rust-engine/research.md) retain
measurements, failed trials, settings and revisions. The source and binary are
archived alongside the report. The [previous solver report](../../../../research/rust-wasm-solver.md)
explains the old boundary; the [native collection contract](../../../../research/native-collection.md)
explains the retained command, clock and cancellation behavior.
