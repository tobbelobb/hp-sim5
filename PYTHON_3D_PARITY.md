# Python headless 3D Hangprinter parity

Behavioral reference: JS main after [PR #61](https://github.com/tobbelobb/hp-sim5/pull/61)
(`7d47fedc`), plus the isolated reference corrections below. Goal: same relevant
USDA data, ECS state and timestep pipeline, with materially equivalent physics.
Python uses Rerun. This working checklist does **not** claim full-machine parity.

Statuses describe covered behavior; synthetic ECS fixtures do not establish USD
loading or complete-machine equivalence. NumPy vectors, Python component factories
and optional configuration represented by `None` are intentional API divergences.

| Simulation functionality (JS source) | Python location / status | Dependencies / evidence needed |
| --- | --- | --- |
| World scheduling, pause/error, ECS query ordering (`cable_joints/ecs.js`) | `cable_joints/ecs.py`: equivalent | `query_order.json` and paused/error steps in `motion.json`; preserve smallest-store insertion order. Python's single-class query shorthand is an intentional API divergence. |
| Vectors, quaternions, geometry (`cable_joints_3d/`) | `vector3.py`, `quaternion.py`, `geometry3.py`: equivalent for covered simulation operations | NumPy vectors are intentional divergence in API. `geometry.json` and `geometry_degenerate.json` cover projected tangents/arcs, all winding choices, arbitrary/zero axes, axial offsets and intersections; reuse Python 2D geometry. |
| Full inertia tensors (`inertia_tensor.js`) | `inertia_tensor.py`: equivalent for covered PSD tensors | `inertia.json` differentially covers rotated small SPD, rank-2/rank-1 and zero tensors. Isolated JS fix preserves Python's PR #61 scale-aware pseudoinverse. |
| Prediction, previous poses, PBD velocities (`commonSystems.js`) | `common_systems.py`: equivalent | `motion.json`, `rigid_members.json`; explicit member exclusions preserve kinematic zero-mass motion and world angular frames. |
| Rigid-member frames, endpoint reaction mapping (`rigid_bodies.js`) | `rigid_bodies.py`: equivalent | `rigid_members.json`, `distance_members.json` probe live attachments, world/local inversion and internal/external reactions. Foundation for cable/motor reactions. |
| Rigid-body synchronization (`commonSystems.js`) | `common_systems.py`: equivalent | `rigid_members.json`: body deltas, member offset velocities, spool references, repeated/paused sync; no hidden post-constraint resync. |
| One-axis spool state/helpers (`hangprinter_spools.js`) | `spools.py`: equivalent | `spool_projection.json`, `rigid_members.json`: tilted axes, swing/velocity projection and reference transport. Position/torque motor free-twist integration is covered. |
| Cable components/path construction (`cable_joints_core.js`, `createCablePaths.js`) | `cable_joints_components.py`, `create_cable_paths.py`, `cable_frames.py`: equivalent for valid authored paths | Construction fixtures cover local/world joints, live tilted member planes, intermediate wraps, hybrid knots, stored overrides, endpoint cuts, empty paths, parameter clamps and zero/infinite stiffness. Python factories intentionally keep the World out of data components. |
| Attachment cache (`cable_attachment_cache_system.js`) | `cable_attachment_cache_system.py`: equivalent | `cable_cache_members.json` covers member-local vs world orientation and moving parents for 200 steps; ownership tests check copies and mutable cache identity. Register after attachment rebuilding, before friction. |
| Dynamic attachments and hybrid transitions (`cable_joints_core.js`) | `cable_attachment_update_system.py`: equivalent for covered Hangprinter configuration | Motion/member/clamp/transition fixtures cover world vs onboard frames, rolling non-slip payout, skew planes, center placeholders, layered hybrid winding, clamp reactions before phase projection, hysteresis, degeneracy, feature flags and pause/step counter. Seven scenarios now run 200 steps. |
| Dynamic split/merge (`cable_joints_core.js`) | missing | Attachments + plane geometry are ready. The app registers `CableAttachmentUpdateSystem(false)`; Python currently rejects enabled split/merge before mutating state. Port separately for topology fixtures. Construction-time splitting at fixed attachments is already covered. |
| Layer/ramp winding (`cable_joints_core.js`) | `cable_layering.py`: equivalent for covered mappings | Signed forward/inverse mappings, radius/ramp transitions, rotation prediction and clamp inversion. `cable_winding.json` covers both endpoint signs, negative stored length, zero-radius/linear limits, wrap boundaries and the 2048-layer cap; motion/clamp fixtures cover integration. |
| Friction redistribution (`cable_friction_system.js`) | `cable_friction_system.py`: equivalent | Friction fixtures cover equal extension, capstan bounds, free rolling spools, fixed attachments, slack, zero-rest spans, arbitrary 3D pinholes and dt-scaled ordered chain iterations, including 200 steps. Moving attachment/cache/friction integration is covered. |
| XPBD cable solve (`cable_joints_core.js`) | `pbd_cable_constraint_solver.py`: equivalent for covered specialized dynamics | Body/spool/pinhole fixtures exercise tensor reactions, direct and indirect one-axis dynamics, holding release, closed-loop stiffness, torque-load maps, damping, alternating per-path iterations and force transfers. Four solver integration scenarios run 200 steps. |
| Cable over-correction (`pbdResolveCableOverCorrections.js`) | `pbd_resolve_cable_over_corrections.py`: equivalent for covered reactions | Shared-correction averaging/gates, tensor host/member reactions, hybrid-only pinhole coupling, duplicate joint membership and last-path metadata. Runs immediately after the cable solver in integration fixtures. |
| Distance XPBD (`commonSystems.js`) | `common_systems.py`: equivalent | `distance_members.json`: off-center tensor corrections, accumulated multipliers and internal endpoints. Used by fixtures; not currently registered by the Hangprinter app. |
| Ball/obstacle collisions, bump and slack systems (`cable_joints_3d/`) | missing | Relevant fixture coverage after cable core; not currently registered by the Hangprinter app. |
| Position motors (`hangprinter_stepper_motor.js`) | `stepper_motor.py`: equivalent for covered integration | Open/closed-loop torque and pose updates, live member aggregate inertia with physical-mass fallback, host reaction, member-local vs standalone integration and cable/PBD/encoder ordering. Standalone/member/cable scenarios run 200 steps. |
| Torque motors (`torqueModeSystem.js`) | `torque_mode_system.py`: equivalent for covered integration | Droop, windage/friction/cogging defaults and overrides, signed/implicit cable loads, drive-only host reaction, mode transitions and member-local integration. Update after PBD velocities; five scenarios run 200 steps. |
| Encoder unwrapping (`commonSystems.js`) | `common_systems.py`: equivalent | `spool_projection.json`, `rigid_members.json`: several turns, fallback axes and parent/reference motion. Position/torque motor and constraint encoder integration is covered. |
| Missed-step state (`motor-diagnostics.js`) | `motor_diagnostics.py`: equivalent for covered state | Persistent full-turn encoder baselines, current/peak counts, half-step rounding, machine resets, torque transitions and member-local fallback. Diagnostic reads preserve their state updates. |
| Effector frames/extrusion (`hangprinter_extruder.js`) | `extruder.py`: equivalent for covered state | Authored triangle frames, center/root/tip/cold offsets, numeric machine-key order, degenerate/missing source fallback and live constrained members. Authored USD bindings and full-machine command deposition are covered. |
| USD cable initialization (`usd/cable_scene_baker.js`) | `usd/cable_scene_loader.py`, `usd/value_readers.py`: equivalent for covered baking | Native pxr.Usd stage, reuse tangent/arc/layer helpers; authored/manual/automatic/derive-all policies, layered radii, parent frames and width overrides. Same hp4/hp3/rigid-pinhole files and dedicated policy fixtures compare before ECS construction. |
| USDA machine builders (`app/scene/`) | `machine_scene.py`: equivalent for covered construction | Native pxr.Usd stage and shared value readers; body/gravity/material/axis state, rigid mass/tensor aggregation, member conversion, distance/cable joints, path initialization, extruder bindings and append/namespaces. Eight authored scenes plus strict double-precision and append fixtures compare initial ECS. |
| Commands (`remoteSpoolSystem.js`, `hangprinter_runtime.js`) | `remote_spool_system.py`, `machine_runtime.py`: equivalent for covered headless records | One queued record per step, pause/zero-dt, mode/reference updates, machine targeting, playback history/reset, callbacks and extrusion colors. Python deque ownership/API is intentional divergence; worker/backpressure transport is browser-only/not required. |
| Composition root (`sceneSystems.js`, `simulationSystems.js`) | `machine_simulation.py`: equivalent for the registered headless pipeline | Production JS and Python registration, exact 19-system order; initial extruder update, no global substep loop. HP3, HP4, rigid pinhole and double-authored minimal machines run 200 repeatable steps; HP4 commands exercise modes, extrusion, diagnostics, pause and distinct update/resource dt. |
| Snapshot / Rerun (`flightRecorderSnapshot.js`, `FLIGHT_RECORDER.md`) | `rerun_system.py`: partial | Preserve PR #61 color, identity, static clearing and pause fixes. Add authoritative time/step, member hierarchy, cables, forces and lengths; reuse recorder contract where practical. |
| Three.js renderer, DOM/pointer/UI, upload controllers, workers | browser-only/not required | Do not port. Render-only slack/wrap geometry may be reused for Rerun presentation. |

## Differential evidence

Seventy-four shared JSON fixtures under `tests/fixtures/python_3d_parity/` execute
production JS in Node (`tests/parity3d/oracle.mjs`) and the native Python engine.
Adapters construct/serialize state; they contain no physics oracle formulas.
Snapshots compare initial state and every timestep, including named relationships,
query order, poses/velocities, attachments, cable lengths/forces, spool/motor/encoder
state, effector frames, command playback/callbacks and entity-keyed torque loads.
Structural fields compare exactly; quaternions
compare up to sign. Nonfinite physics state fails. Guard checks ensure targeted
constraints, transitions and reactions activate. Thirty-two scenarios run 200 steps
and require exact repeatability within each engine; a further stiff-motor probe
runs 200 steps at 20 microseconds. See `tests/parity3d/README.md` for fixture coverage.

Motion/solver tolerances remain absolute `1e-10`, relative `1e-9`. Inverse moments
around `1e6` use absolute `1e-8`, relative `1e-12`. The commanded cable case uses
the authored 0.5 Nm motor scale. Synthetic 100 Nm stiffness at millisecond steps
produces violent motion and amplifies roundoff; the stress case uses 20 microsecond
steps. Neither engine receives hidden substeps, speed clamps or relaxed tolerances.
Native USD honors authored float32 types while JS's parser retains numeric literals
as doubles. Initial authored-scene comparisons use absolute `5e-10`, relative `6e-8`
to cover float32 input quantization and aggregate-center subtraction. A guard
demonstrates the source rounding and rejects a changed member frame. The authored
double-precision scene and native baking retain `1e-10`/`1e-9`.

Full authored-machine runs use field-specific absolute bounds, with relative
`1e-9` on those fields. Unlisted parameters retain the initial float32 input bound;
categorical fields, relationships, query order and system order remain exact.
Only extrusion positions receive the length bound; deposited lengths and command
records retain the default comparison.

| Dynamic field | Absolute bound |
| --- | --- |
| Positions, attachments, geometric/rest/stored lengths, effector points | `1e-8` m |
| Quaternions and transported axes | `3e-7` per component |
| Linear velocity | `1e-6` m/s |
| Angular velocity | `2e-5` rad/s |
| Encoder angle | `5e-7` rad |
| Cable force vectors/magnitudes | `2e-5` N |

The 200-step original-file comparisons cover HP3, HP4 and rigid pinhole machines.
Existing numerical cutoffs amplify small roundoff differences near rest, even
with identical double-authored inputs. A separate 20-step comparison promotes
the shared fixture's USD types to doubles and retains `1e-10`/`1e-9` for every
engine field. Production loaders never rewrite authored numeric types. Guards
reject changed frames, velocities, forces and deposited lengths rather than
widening a global tolerance.

## Architectural checks

- Preserve PR #61 world angular/PBD, scale-aware inertia, zero-mass kinematic,
  member exclusion/copy and Rerun identity/color/static-clear/pause corrections.
- Preserve live parent/member frames, cached copies and mutable component identity.
  Constraint systems leave member ECS poses stale until the next registered sync.
- Query insertion order and system registration order affect coupled dynamics.
  The cable solver reads the World `dt` resource; motors use the update argument.
- Cable solving captures local attachments once, alternates path/joint iterations,
  records force telemetry on iteration zero and accumulates torque loads on every
  iteration, including virtual loads on immovable bodies.
- Spools retain one-axis dynamics and bearing projection. Over-correction retains
  full-tensor member corrections and opposite host reaction signs; it does not
  introduce a hidden sync. Do not substitute a generic body engine.
- Motor reactions prefer the live aggregate of member tensors plus parallel-axis
  physical-mass contributions. Closed-loop reactions mutate the parent's live
  quaternion before member recomposition. Member rotors integrate inside motors;
  standalone rotors wait for angular prediction. Torque-mode housing reactions
  use drive torque only; external cable loads already react through constraints.
- Rerun must preserve actual time through pause/reset and clear static archetypes.
- Effector systems read live parent/member transforms after constraints without
  adding a sync. Command deposition precedes current-step prediction. Numeric
  machine and axis keys retain JS array-index ordering. Python queues copy their
  containers and use a deque; playback snapshots copy records as the reference does.

## Demonstrated reference corrections

Each is isolated in its own commit, with JS and differential regression evidence:

- Small PSD inertia: JS previously discarded a valid rotated tensor for which
  Python returned inverse inertia `1640000`. Scaled symmetric eigendecomposition
  now uses the PR #61 supported-eigenvalue cutoff.
- Coincident guides: JS tangent geometry returned NaN. Both engines now use the
  existing Python 2D deterministic radial fallback below `1e-9` center separation.
- Zero cable stiffness: infinite-compliance products yielded NaN. The solver now
  evaluates the finite limit of its existing damping equation; undamped zero
  stiffness leaves state unchanged, and damping agrees with near-zero stiffness.
- Over-correction tangents: omitted radii silently caused center comparisons and
  missed slack spans. Attachment rebuilding now defaults to the existing layered
  radii; rolling and hybrid regressions demonstrate the correction.
- Effector pose: constraints changed live body transforms while cached member
  positions remained stale. Extruder source/average/frame reads now use the existing
  live-world helper. Authored and fallback regressions require the final body center
  and preserve the distinct rotated/unrotated offset rules; no extra sync is added.
- Authored cube USD: a three-value Euler rotation was declared `quatf`, which USD
  rejected. Its declaration is now `double3`, preserving values and the complete JS
  construction snapshot. Native USD parsing is covered by a regression.
- Authored zero stiffness: the JS scene builder's truthy fallback replaced zero
  with infinity. Both builders now retain zero. A production-loader regression
  failed before the isolated JS correction and passes after it.
- Rigid-group relationship fallback: an empty JS array hid the supported legacy
  member spelling and produced no assembly. The fallback now checks array length;
  a loader regression and a strict cross-language fixture require three members.
- Small PBD rotations: `acos(w)` returned zero when the quaternion scalar rounded
  to one, discarding a representable `1e-8` rad rotation. Both engines now recover
  the angle with `atan2(norm(vector), w)`. JS and differential regressions require
  the expected angular velocity while retaining world frames, quaternion sign
  handling and the existing small-angle/time cutoffs. Python follows the JS scalar
  operation order before vector multiplication. The deterministic flipper demo's
  contact trajectory changes its stable score from 10 to 18; its regression records
  the corrected result, with the pre-correction checkout confirming the old score.

## Reviewable slices and next dependencies

| PR | Covered slice | Depends on |
| --- | --- | --- |
| [#62](https://github.com/tobbelobb/hp-sim5/pull/62) | Differential oracle, rigid frames/sync, spool projection, distance/encoders | main after #61 |
| [#63](https://github.com/tobbelobb/hp-sim5/pull/63) | Geometry, cable construction/cache/friction | #62 |
| [#64](https://github.com/tobbelobb/hp-sim5/pull/64) | Signed winding, attachments/clamps, hybrid transitions | #63 |
| [#65](https://github.com/tobbelobb/hp-sim5/pull/65) | Cable solving, load telemetry and shared over-correction | #64 |
| [#66](https://github.com/tobbelobb/hp-sim5/pull/66) | Position/torque integration and body reactions | #65 |
| [#67](https://github.com/tobbelobb/hp-sim5/pull/67) | Commands, effector/extrusion state and missed-step diagnostics | #66 |
| Authored-scene follow-up | Native USD baking, construction, semantic composition and full-machine differentials | #67 |

Next: richer native Rerun recording and a headless recording entry point. Dynamic
topology and relevant collision fixtures remain open checklist items. Full-machine
tests cover the app's current registered pipeline, not every optional engine system.

## Completion gate

Python loads the same relevant USDA machines headlessly and runs the meaningful
JS pipeline. Differential checks prove initial construction, one/multiple steps,
rigid poses/velocities, cable lengths/forces/friction, spool/motor/encoder and
effector/extruder state, including representative complete machines. Remaining
differences are browser-only or documented intentional divergences. Existing JS
and Python tests pass. Keep this checklist open until the whole gate is met.
