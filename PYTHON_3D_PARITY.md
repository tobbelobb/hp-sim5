# Python headless 3D Hangprinter parity

Behavioral reference: JavaScript on main after [PR #61](https://github.com/tobbelobb/hp-sim5/pull/61)
(`7d47fedc`). Same authored machine data, ECS state and timestep sequence;
Rerun is the Python visualization/recording path. This is a working checklist,
not a claim of complete engine parity.

| Simulation functionality (JS source) | Python location / status | Dependencies / evidence needed |
| --- | --- | --- |
| World scheduling, pause/error, ECS query ordering (`cable_joints/ecs.js`) | `cable_joints/ecs.py`: equivalent | `query_order.json` and paused/error steps in `motion.json`; preserve smallest-store insertion order. Python's single-class query shorthand is an intentional API divergence. |
| Vectors, quaternions, geometry (`cable_joints_3d/`) | `vector3.py`, `quaternion.py`, `geometry3.py`: equivalent for covered simulation operations | NumPy vectors are intentional divergence in API. `geometry.json` and `geometry_degenerate.json` cover projected tangents/arcs, all winding choices, arbitrary/zero axes, axial offsets and intersections; reuse Python 2D geometry. |
| Full inertia tensors (`inertia_tensor.js`) | `inertia_tensor.py`: equivalent for covered PSD tensors | `inertia.json` differentially covers rotated small SPD, rank-2/rank-1 and zero tensors. Isolated JS fix preserves Python's PR #61 scale-aware pseudoinverse. |
| Prediction, previous poses, PBD velocities (`commonSystems.js`) | `common_systems.py`: equivalent | `motion.json`, `rigid_members.json`; explicit member exclusions preserve kinematic zero-mass motion and world angular frames. |
| Rigid-member frames, endpoint reaction mapping (`rigid_bodies.js`) | `rigid_bodies.py`: equivalent | `rigid_members.json`, `distance_members.json` probe live attachments, world/local inversion and internal/external reactions. Foundation for cable/motor reactions. |
| Rigid-body synchronization (`commonSystems.js`) | `common_systems.py`: equivalent | `rigid_members.json`: body deltas, member offset velocities, spool references, repeated/paused sync; no hidden post-constraint resync. |
| One-axis spool state/helpers (`hangprinter_spools.js`) | `spools.py`: equivalent | `spool_projection.json`, `rigid_members.json`: tilted axes, swing/velocity projection and reference transport. Motor-driven free-twist integration still missing. |
| Cable components/path construction (`cable_joints_core.js`, `createCablePaths.js`) | `cable_joints_components.py`, `create_cable_paths.py`, `cable_frames.py`: equivalent for valid authored paths | Construction fixtures cover local/world joints, live tilted member planes, intermediate wraps, hybrid knots, stored overrides, endpoint cuts, empty paths, parameter clamps and zero/infinite stiffness. Python factories intentionally keep the World out of data components. |
| Attachment cache (`cable_attachment_cache_system.js`) | `cable_attachment_cache_system.py`: equivalent | `cable_cache_members.json` covers member-local vs world orientation and moving parents for 200 steps; ownership tests check copies and mutable cache identity. Register after attachment rebuilding, before friction. |
| Dynamic attachments, hybrid transitions, split/merge (`cable_joints_core.js`) | missing | Components + frames/cache are present. Next: rebuild attachments, rotation/stored changes and topology updates, preserving length and cache timing. |
| Layer/ramp winding (`cable_joints_core.js`) | `cable_layering.py`: partial | Inverse stored-to-radius/angle mapping supports hybrid initialization; forward winding and dynamic transitions still missing. |
| Friction redistribution (`cable_friction_system.js`) | missing | Cable components + attachments; equal extension, capstan bounds, free rolling guides, dt-scaled iterations. |
| XPBD cable solve (`cable_joints_core.js`) | missing | Attachments/cache + friction + tensor reactions + motor state; per-path iterations and force/load telemetry. |
| Cable over-correction (`pbdResolveCableOverCorrections.js`) | missing | Cable solve + member reaction mapping. |
| Distance XPBD (`commonSystems.js`) | `common_systems.py`: equivalent | `distance_members.json`: off-center tensor corrections, accumulated multipliers and internal endpoints. Used by fixtures; not currently registered by the Hangprinter app. |
| Ball/obstacle collisions, bump and slack systems (`cable_joints_3d/`) | missing | Relevant fixture coverage after cable core; not currently registered by the Hangprinter app. |
| Position motors (`hangprinter_stepper_motor.js`) | missing | Spool state + rigid sync/reactions; open/closed-loop and member-local integration. |
| Torque motors (`torqueModeSystem.js`) | missing | Position-motor reactions + cable load torque/stiffness/damping; update after PBD velocities. |
| Encoder unwrapping (`commonSystems.js`) | `common_systems.py`: equivalent | `spool_projection.json`, `rigid_members.json`: several turns, fallback axes and parent/reference motion. Motor-integrated encoder updates remain part of the motor slice. |
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

The new inertia fixture demonstrated JS returning zero inverse inertia where
Python returned `1640000` for a rotated small SPD tensor. JS now uses a scaled
symmetric eigendecomposition with the same supported-eigenvalue cutoff as
Python. This is a deliberate correction of the reference, in a separate commit,
not a relaxation of parity tolerances. The inertia fixture uses absolute
`1e-8` and relative `1e-12` tolerance on inverse moments (order `1e6`);
motion fixtures use absolute `1e-10` and relative `1e-9`.

The geometry probes demonstrated NaN tangents in JS for coincident equal-radius
guides. A separate reference correction now uses the existing Python 2D radial
fallback for projected center separation below `1e-9`. Both engines use that
deterministic convention for the geometrically underdetermined case; JS unit
tests and `geometry_degenerate.json` cover coincident/near-coincident guides.

## First PR boundary

The first PR adds the executable differential harness and closes the covered
World/order, prediction/PBD velocity, rigid frames/sync, spool projection,
distance-constraint and encoder layers above. The inertia correction is its
own commit with JS unit and cross-language regressions. All PR #61 corrections
remain, including its Rerun lifecycle tests.

Six shared fixtures in the first PR prove the covered synthetic ECS pipeline;
none claims full authored-machine parity.

## Cable construction follow-up

The follow-up adds shared plane geometry, native cable state/path construction,
hybrid knot initialization and member-aware attachment caching. It retains the
specialized spool model. Dynamic attachment rebuilding, forward winding,
split/merge, friction and cable XPBD still follow; USDA loading,
motors/commands/extrusion, the complete Hangprinter composition root and rich
Rerun snapshots remain open.

## Completion gate

Headless Python loads the same relevant USDA scenes and runs the meaningful JS
pipeline. Differential tests prove initial construction, one/multiple steps,
rigid poses/velocities, cable lengths/forces, friction, spool/motor/encoder and
effector/extruder state, including representative complete machines. Remaining
differences are browser-only or documented intentional divergences. Existing
JS/Python tests pass. Deliver reviewable PRs and keep this checklist open until
that whole gate is met.
