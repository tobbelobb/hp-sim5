# Python headless 3D Hangprinter parity

Behavioral reference: JS main after [PR #61](https://github.com/tobbelobb/hp-sim5/pull/61)
(`7d47fedc`), plus the isolated reference corrections below. Goal: same relevant
USDA data, ECS state and timestep pipeline, with materially equivalent physics.
Python uses Rerun. This working checklist does **not** claim the goal is complete.

Statuses describe covered behavior; synthetic ECS fixtures do not establish USD
loading or complete-machine equivalence. NumPy vectors, Python component factories
and optional configuration represented by `None` are intentional API divergences.

| Simulation functionality (JS source) | Python location / status | Dependencies / evidence needed |
| --- | --- | --- |
| World scheduling, pause/error, ECS query ordering (`cable_joints/ecs.js`) | `cable_joints/ecs.py`: equivalent | `query_order.json` and paused/error steps in `motion.json`; preserve smallest-store insertion order. Python's single-class query shorthand is an intentional API divergence. |
| Vectors, quaternions, geometry (`cable_joints_3d/`) | `vector3.py`, `quaternion.py`, `geometry3.py`: equivalent for covered simulation operations | NumPy vectors are intentional divergence in API. `quaternion_raw_frames.json` covers stored nonunit/zero parent frames and explicitly normalized member frames. Geometry fixtures cover projected tangents/arcs, all winding choices, arbitrary/zero axes, axial offsets and intersections; reuse Python 2D geometry. |
| Full inertia tensors (`inertia_tensor.js`) | `inertia_tensor.py`: equivalent for covered PSD tensors | `inertia.json` differentially covers rotated small SPD, rank-2/rank-1 and zero tensors. Isolated JS fix preserves Python's PR #61 scale-aware pseudoinverse. |
| Prediction, previous poses, PBD velocities (`commonSystems.js`) | `common_systems.py`: equivalent | `motion.json`, `rigid_members.json`; explicit member exclusions preserve kinematic zero-mass motion and world angular frames. |
| Rigid-member frames, endpoint reaction mapping (`rigid_bodies.js`) | `rigid_bodies.py`: equivalent | `rigid_members.json`, `distance_members.json` probe live attachments, world/local inversion and internal/external reactions. Foundation for cable/motor reactions. |
| Rigid-body synchronization (`commonSystems.js`) | `common_systems.py`: equivalent | `rigid_members.json`: body deltas, member offset velocities, spool references, repeated/paused sync; no hidden post-constraint resync. |
| One-axis spool state/helpers (`hangprinter_spools.js`) | `spools.py`: equivalent | `spool_projection.json`, `rigid_members.json`: tilted axes, swing/velocity projection and reference transport. Position/torque motor free-twist integration is covered. |
| Cable components/path construction (`cable_joints_core.js`, `createCablePaths.js`) | `cable_joints_components.py`, `create_cable_paths.py`, `cable_frames.py`: equivalent for valid authored paths | Construction fixtures cover local/world joints, live tilted member planes, intermediate wraps, hybrid knots, stored overrides, endpoint cuts, empty paths, parameter clamps and zero/infinite stiffness. Python factories intentionally keep the World out of data components. |
| Attachment cache (`cable_attachment_cache_system.js`) | `cable_attachment_cache_system.py`: equivalent | `cable_cache_members.json` covers member-local vs world orientation and moving parents for 200 steps; ownership tests check copies and mutable cache identity. Register after attachment rebuilding, before friction. |
| Dynamic attachments and hybrid transitions (`cable_joints_core.js`) | `cable_attachment_update_system.py`: equivalent for covered Hangprinter configuration | Motion/member/clamp/transition fixtures cover world vs onboard frames, rolling non-slip payout, skew planes, center placeholders, layered hybrid winding, clamp reactions before phase projection, hysteresis, degeneracy, feature flags and pause/step counter. Seven scenarios now run 200 steps. |
| Dynamic split/merge (`cable_joints_core.js`) | `cable_topology.py`: equivalent for covered valid paths | Thirteen fixtures cover ordered live-span splitting, cascading merge traversal, layered/rolling endpoint radii, stale tilted member frames, machine isolation, allocation/removal and feature flags. Attachment/cache/friction/solver/PBD/encoder integration and repeated topology cycles run 200 steps. Preserve component/list/point identity and total cable length; allocate only successful splits. The app retains its existing disabled default. |
| Layer/ramp winding (`cable_joints_core.js`) | `cable_layering.py`: equivalent for covered mappings | Signed forward/inverse mappings, radius/ramp transitions, rotation prediction and clamp inversion. `cable_winding.json` covers both endpoint signs, negative stored length, zero-radius/linear limits, wrap boundaries and the 2048-layer cap; motion/clamp fixtures cover integration. |
| Friction redistribution (`cable_friction_system.js`) | `cable_friction_system.py`: equivalent | Friction fixtures cover equal extension, capstan bounds, free rolling spools, fixed attachments, slack, zero-rest spans, arbitrary 3D pinholes and dt-scaled ordered chain iterations, including 200 steps. Moving attachment/cache/friction integration is covered. |
| XPBD cable solve (`cable_joints_core.js`) | `pbd_cable_constraint_solver.py`: equivalent for covered specialized dynamics | Body/spool/pinhole fixtures exercise tensor reactions, direct and indirect one-axis dynamics, holding release, closed-loop stiffness, torque-load maps, damping, alternating per-path iterations and force transfers. Four solver integration scenarios run 200 steps. |
| Cable over-correction (`pbdResolveCableOverCorrections.js`) | `pbd_resolve_cable_over_corrections.py`: equivalent for covered reactions | Shared-correction averaging/gates, tensor host/member reactions, hybrid-only pinhole coupling, duplicate joint membership and last-path metadata. Runs immediately after the cable solver in integration fixtures. |
| Distance XPBD (`commonSystems.js`) | `common_systems.py`: equivalent | `distance_members.json`: off-center tensor corrections, accumulated multipliers and internal endpoints. Used by fixtures; not currently registered by the Hangprinter app. |
| Ball/obstacle collisions and bump (`cable_joints_3d/`) | `pbd_ball_collisions.py`, `ball_obstacle_bump_system.py`: equivalent for covered core sphere contacts | Ordered unequal/zero/negative-mass pairs, query-store reordering, coincident/separated/touching gates, geometric obstacle contacts, contact-specific friction and raw-hit filtering, rotated tensor angular impulses and post-PBD bump/encoder order. Four fixtures run 200 repeatable steps. Not registered by the Hangprinter app. |
| Optional slack (`cable_slack_system.js`) | `cable_slack_system.py`: equivalent | Pinhole tension equalization and literal attachment-gated loose transfer, shortened/zero paths, ordered chains, rest conservation and pause. Four fixtures run 200 repeatable steps, including active attachment/cache/XPBD/PBD/encoder integration. Preserve the single-pass 3D policy rather than Python 2D's dt-scaled iterations. |
| Position motors (`hangprinter_stepper_motor.js`) | `stepper_motor.py`: equivalent for covered integration | Open/closed-loop torque and pose updates, live member aggregate inertia with physical-mass fallback, host reaction, member-local vs standalone integration and cable/PBD/encoder ordering. Standalone/member/cable scenarios run 200 steps. |
| Torque motors (`torqueModeSystem.js`) | `torque_mode_system.py`: equivalent for covered integration | Droop, windage/friction/cogging defaults and overrides, signed/implicit cable loads, drive-only host reaction, mode transitions and member-local integration. Update after PBD velocities; five scenarios run 200 steps. |
| Encoder unwrapping (`commonSystems.js`) | `common_systems.py`: equivalent | `spool_projection.json`, `rigid_members.json`: several turns, fallback axes and parent/reference motion. Position/torque motor and constraint encoder integration is covered. |
| Missed-step state (`motor-diagnostics.js`) | `motor_diagnostics.py`: equivalent for covered state | Persistent full-turn encoder baselines, current/peak counts, half-step rounding, machine resets, torque transitions and member-local fallback. Diagnostic reads preserve their state updates. |
| Effector frames/extrusion (`hangprinter_extruder.js`) | `extruder.py`: equivalent for covered state | Authored triangle frames, center/root/tip/cold offsets, numeric machine-key order, degenerate/missing source fallback and live constrained members. Authored USD bindings and full-machine command deposition are covered. |
| USD cable initialization (`usd/cable_scene_baker.js`) | `usd/cable_scene_loader.py`, `usd/value_readers.py`: equivalent for covered baking | Native pxr.Usd stage, reuse tangent/arc/layer helpers; authored/manual/automatic/derive-all policies, layered radii, parent frames and width overrides. Same hp4/hp3/rigid-pinhole files and dedicated policy fixtures compare before ECS construction. |
| USDA machine builders (`app/scene/`) | `machine_scene.py`: equivalent for covered construction | Native pxr.Usd stage and shared value readers; body/gravity/material/axis state, rigid mass/tensor aggregation, member conversion, distance/cable joints, path initialization, extruder bindings and append/namespaces. Eight authored scenes plus strict double-precision and append fixtures compare initial ECS. |
| Commands (`remoteSpoolSystem.js`, `hangprinter_runtime.js`) | `remote_spool_system.py`, `machine_runtime.py`: equivalent for covered headless records | One queued record per step, pause/zero-dt, mode/reference updates, machine targeting, playback history/reset, callbacks and extrusion colors. Python deque ownership/API is intentional divergence; worker/backpressure transport is browser-only/not required. |
| Composition root (`sceneSystems.js`, `simulationSystems.js`) | `machine_simulation.py`: equivalent for the registered headless pipeline | Production JS and Python registration, exact 19-system order; initial extruder update, no global substep loop. HP3, HP4, rigid pinhole and double-authored minimal machines run 200 repeatable steps; HP4 commands exercise modes, extrusion, diagnostics, pause and distinct update/resource dt. |
| Snapshot / Rerun (`flightRecorderSnapshot.js`, `FLIGHT_RECORDER.md`) | `machine_snapshot.py`, `rerun_system.py`, `__main__.py`: equivalent for covered recording data | Six fixture families compare native frames/lengths/forces with production JS recorder data. Live body hierarchy, solver-sampled cable endpoints, copied snapshots, initial/every-step RRD, Z-up coordinates, pause/reset clocks, static clearing, encoder/motor/velocity and tool/extrusion state. Straight-span/point visuals and stable per-quantity plot paths are intentional presentation divergences. |
| Optional cable event buffer / console summaries (`cable_joints_core.js`) | intentional divergence | Python records deterministic ECS snapshots and primary Rerun traces instead of the optional JS `cableEventTrace*` buffer/console API. These diagnostics do not feed simulation state. Saved-RRD topology coverage checks joint identity and clearing; no physics field is removed. |
| Three.js renderer, DOM/pointer/UI, upload controllers, workers | browser-only/not required | Do not port. Render-only slack/wrap geometry may be reused for Rerun presentation. |
| Separate flipper demo application (`example_apps/js/flipper_3d/`) | browser-only/not required for this goal | Extended sector/border/flipper contact machinery is outside the specialized Hangprinter app pipeline. The shared optional core sphere/slack systems above are covered; the existing JS demo regression remains required and passes. |

## Differential evidence

Ninety-six shared JSON fixtures under `tests/fixtures/python_3d_parity/` execute
production JS in Node (`tests/parity3d/oracle.mjs`) and the native Python engine.
Adapters construct/serialize state; they contain no physics oracle formulas.
Snapshots compare initial state and every timestep, including named relationships,
query order, poses/velocities, attachments, cable lengths/forces, spool/motor/encoder
state, effector frames, command playback/callbacks and entity-keyed torque loads.
Topology fixtures also compare every live entity, allocator state and creation/
removal order; deleted entities cannot survive as empty snapshot rows.
Structural fields compare exactly; quaternions
compare up to sign. Nonfinite physics state fails. Guard checks ensure targeted
constraints, transitions and reactions activate. Forty-two scenarios run 200 steps
and require exact repeatability within each engine; a further stiff-motor probe
runs 200 steps at 20 microseconds. See `tests/parity3d/README.md` for fixture coverage.
The recording oracle also calls the production JS flight-recorder snapshot for
six existing fixture families. It compares frame trees, live effector frames,
span endpoints, commanded/actual/geometric lengths and force telemetry. Browser
sag subdivisions and wrap tessellation are projected out explicitly; no physical
field is dropped. Native snapshots own their data and recording is read-only.

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
- Quaternion vector transforms use the stored quaternion without implicit
  normalization, matching JS. Frame helpers and member constructors normalize
  explicitly where required. The raw-parent attachment fixture failed before
  removing Python's hidden normalization; owned float outputs leave inputs intact.
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
- Native Rerun has a stable path for each quantity/segment, so entering torque
  mode or changing topology cannot relabel earlier plot indices. World transforms
  are relative to live rigid parents; velocity traces expose the stored ECS state.
- Dynamic splitting preserves attachment-array references through the inner
  splitter loop. Reverse guide insertion order can split the kept span again;
  copying those arrays or stopping after the first guide changes the topology.
  Merge traversal revisits shortened paths when neighboring stored lengths turn
  negative. Removed joints leave every component store; entity IDs are not reused.
- Optional sphere contacts retain the reference's simple model: ordered cached
  `PositionComponent` poses, coincident-center pairs skipped, and geometric
  obstacle projection regardless of ball mass. They do not redirect rigid-member
  reactions or replace the specialized spool model. Bump friction is the mean of
  contact/obstacle coefficients, uses relative surface velocity and applies world
  inertia tensors after PBD velocity reconstruction. Contact buffers keep list
  identity; contact directions are owned copies and remain unchanged by bump.
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
- Aborted dynamic split: JS allocated a machine-tag-only entity before checking
  available rest/wrap lengths. An insufficient-rest regression demonstrated the
  unused entity and advanced allocator. Allocation now follows those checks in
  both engines; successful topology and the full flipper trajectory remain stable.

## Reviewable slices and next dependencies

| PR | Covered slice | Depends on |
| --- | --- | --- |
| [#62](https://github.com/tobbelobb/hp-sim5/pull/62) | Differential oracle, rigid frames/sync, spool projection, distance/encoders | main after #61 |
| [#63](https://github.com/tobbelobb/hp-sim5/pull/63) | Geometry, cable construction/cache/friction | #62 |
| [#64](https://github.com/tobbelobb/hp-sim5/pull/64) | Signed winding, attachments/clamps, hybrid transitions | #63 |
| [#65](https://github.com/tobbelobb/hp-sim5/pull/65) | Cable solving, load telemetry and shared over-correction | #64 |
| [#66](https://github.com/tobbelobb/hp-sim5/pull/66) | Position/torque integration and body reactions | #65 |
| [#67](https://github.com/tobbelobb/hp-sim5/pull/67) | Commands, effector/extrusion state and missed-step diagnostics | #66 |
| [#68](https://github.com/tobbelobb/hp-sim5/pull/68) | Native USD baking, construction, semantic composition and full-machine differentials | #67 |
| [#69](https://github.com/tobbelobb/hp-sim5/pull/69) | Read-only snapshots, richer primary Rerun and headless recording CLI | #68 |
| [#70](https://github.com/tobbelobb/hp-sim5/pull/70) | Live split/merge, entity lifecycle oracle and coupled topology cycles | #69 |
| [#71](https://github.com/tobbelobb/hp-sim5/pull/71) | Sphere contacts, tensor bump and single-pass slack differentials | #70 |

The implementation checklist now covers the meaningful Hangprinter pipeline and
the shared optional engine systems. Review and merge the stacked slices in order;
incorporate review corrections with differential evidence. Full-machine tests use
the app's production registration, while optional systems have explicit fixture
registration and remain absent from the app pipeline.

## Completion gate

Python loads the same relevant USDA machines headlessly and runs the meaningful
JS pipeline. Differential checks prove initial construction, one/multiple steps,
rigid poses/velocities, cable lengths/forces/friction, spool/motor/encoder and
effector/extruder state, including representative complete machines. Remaining
differences are browser-only or documented intentional divergences. Existing JS
and Python tests pass. Keep this checklist open until the whole gate is met.

Implementation audit through #71: same authored HP3/HP4/pinhole inputs, native ECS
construction, exact production system order, shared engine/command state,
representative 200-step machines, coupled topology/friction/motor/constraint
scenarios, and primary native Rerun/CLI are covered. Final validation is 344
simulator Python tests, 287 autocal Python tests, and 650 JS tests (one skipped);
JS production is unchanged by #71. The stack remains open for review and merge,
so this audit does not claim integration into main or approval of the changes.
