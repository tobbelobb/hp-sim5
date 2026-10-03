# Python headless 3D Hangprinter parity

Behavioral reference: JavaScript on main after [PR #61](https://github.com/tobbelobb/hp-sim5/pull/61)
(`7d47fedc`). Same authored machine data, ECS state and timestep sequence;
Rerun is the Python visualization/recording path. This is a working checklist,
not a claim of complete engine parity.

| Simulation functionality (JS source) | Python location / status | Dependencies / evidence needed |
| --- | --- | --- |
| World scheduling, pause/error, ECS query ordering (`cable_joints/ecs.js`) | `cable_joints/ecs.py`: partial | Differential query-order and paused-step fixtures. |
| Vectors, quaternions, geometry (`cable_joints_3d/`) | `vector3.py`, `quaternion.py`, `geometry3.py`: partial | NumPy vectors are intentional divergence in API; missing plane-projected tangents/arcs. Test arbitrary axes and noncommuting rotations. |
| Full inertia tensors (`inertia_tensor.js`) | `inertia_tensor.py`: partial | PR #61 fixed Python small/singular PSD inversion; JS still has an absolute determinant cutoff. Demonstrate and resolve separately, preserving Python corrections. |
| Prediction, previous poses, PBD velocities (`commonSystems.js`) | `common_systems.py`: partial | Differential initial/one/many-step fixtures; remaining member exclusions. |
| Rigid-member frames, endpoint reaction mapping (`rigid_bodies.js`) | missing | Foundation for cable, distance and motor reactions. |
| Rigid-body synchronization (`commonSystems.js`) | missing | Member frames + spool state/projection; verify body deltas, member velocities, references and repeated sync. |
| One-axis spool state/helpers (`hangprinter_spools.js`) | missing | Quaternion/frame semantics; preserve specialized twist projection, not generic hinges. |
| Cable components/path construction (`cable_joints_core.js`, `createCablePaths.js`) | missing | Rigid frames + plane geometry; inspect/reuse Python 2D algorithms without their scalar-orientation assumptions. |
| Attachments/cache, hybrid transitions, split/merge, layering (`cable_joints_core.js`, `cable_attachment_cache_system.js`) | missing | Cable components + spool/member frames. Test stored/rest/geometric length conservation and cache timing. |
| Friction redistribution (`cable_friction_system.js`) | missing | Cable components + attachments; equal extension, capstan bounds, free rolling guides, dt-scaled iterations. |
| XPBD cable solve (`cable_joints_core.js`) | missing | Attachments/cache + friction + tensor reactions + motor state; per-path iterations and force/load telemetry. |
| Cable over-correction (`pbdResolveCableOverCorrections.js`) | missing | Cable solve + member reaction mapping. |
| Distance XPBD (`commonSystems.js`) | missing | Rigid endpoints + tensor angular corrections. Used by fixtures; not currently registered by the Hangprinter app. |
| Ball/obstacle collisions, bump and slack systems (`cable_joints_3d/`) | missing | Relevant fixture coverage after cable core; not currently registered by the Hangprinter app. |
| Position motors (`hangprinter_stepper_motor.js`) | missing | Spool state + rigid sync/reactions; open/closed-loop and member-local integration. |
| Torque motors (`torqueModeSystem.js`) | missing | Position-motor reactions + cable load torque/stiffness/damping; update after PBD velocities. |
| Encoder unwrapping (`commonSystems.js`) | missing | Spool/member frames; parent motion must not count as rotor turns. |
| Missed-step state (`motor-diagnostics.js`) | missing | Encoder + motor state, reference changes and torque/position transitions. |
| Effector frames/extrusion (`hangprinter_extruder.js`) | missing | Rigid/member state + authored center/tip offsets + commands. |
| USDA machine builders (`app/scene/`) | missing | Components and scene semantics above. Reuse `pxr.Usd`, `UsdGeom`, `UsdShade` patterns from Python demo loaders; keep web server imports out of simulation. |
| Commands (`remoteSpoolSystem.js`, `hangprinter_runtime.js`) | missing | Motor/extruder state; one queued command per step and mode/reference transitions. Worker/backpressure transport is browser-only/not required. |
| Composition root (`sceneSystems.js`) | missing | Register meaningful systems in JS order, no global substep loop; run full authored machines, especially `hp4_rigid_body.usda`. |
| Snapshot / Rerun (`flightRecorderSnapshot.js`, `FLIGHT_RECORDER.md`) | `rerun_system.py`: partial | Preserve PR #61 color, identity, static clearing and pause fixes. Add authoritative time/step, member hierarchy, cables, forces and lengths; reuse recorder contract where practical. |
| Three.js renderer, DOM/pointer/UI, upload controllers, workers | browser-only/not required | Do not port. Render-only slack/wrap geometry may be reused for Rerun presentation. |

## Differential harness

Shared JSON fixtures under `tests/fixtures/python_3d_parity/` specify named
entities, components, ordered systems, timestep sequences and state mutations.
`tests/parity3d/oracle.mjs` imports and executes the actual JS engine in Node;
the Python adapter uses the native engine. No reimplemented oracle formulas.
Snapshots include initial state and every completed step. Structural fields and
entity relationships compare exactly. Numeric comparisons use explicit fixture
absolute/relative tolerances and reject nonfinite values; quaternions compare
up to sign. Failures report the fixture, step and state path.

Grow coverage in dependency order: foundations → rigid frames/distance → cable
construction/attachments/friction → cable solve/over-correction → motors and
encoders → authored USDA machines and commands → rich Rerun recording. Synthetic
ECS construction does not establish USDA loading parity. Passing a subsystem
fixture does not establish full-machine parity.

## Architectural checks

- Preserve world vs member-local rotation and attachment frames, copies and
  mutable component identity; compare stale ECS poses against live parent poses.
- Query insertion order and system registration order affect coupled solvers.
- Zero mass does not prevent kinematic prediction; rigid members are explicit.
- Spools retain one-axis dynamics and custom parent reaction, including current
  discarded off-axis bearing energy. Do not substitute a generic body engine.
- Numerical tolerances must follow measured error, not hide differing state
  transitions. Known JS defects need regression evidence and an isolated fix
  (or an explicit intentional divergence).
- Rerun static archetypes require static clearing; preserve namespace/id paths
  and record actual simulation time through pause/reset.

## Completion gate

Headless Python loads the same relevant USDA scenes and runs the meaningful JS
pipeline. Differential tests prove initial construction, one/multiple steps,
rigid poses/velocities, cable lengths/forces, friction, spool/motor/encoder and
effector/extruder state, including representative complete machines. Remaining
differences are browser-only or documented intentional divergences. Existing
JS/Python tests pass. Deliver reviewable PRs and keep this checklist open until
that whole gate is met.
