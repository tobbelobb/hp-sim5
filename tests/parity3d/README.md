# Headless differential 3D checks

Run with Node on PATH and the repository Python environment:

```bash
.venv/bin/python -m pytest tests/python/cable_joints_3d/test_differential_parity.py -q
```

Every fixture executes the **production JavaScript implementation**, then the
native Python implementation, and compares initial state and every timestep.
`contract.json` lists the state fields to compare; adapters only construct ECS
components, invoke systems and serialize fields. Relationships use fixture
names, quaternions use XYZW order and compare up to sign. Query and solver order
remain observable. Arrays are copied when snapshots are taken.
Per-path knot-map keys also use fixture entity names.
Machine-keyed vector/source maps retain authored machine names. `effectorRotations`
probes the production frame estimator; `motorDiagnostics` reads production reports
before serializing their state changes. `removeComponents` and
`resetMotorDiagnostics` exercise encoder fallback and baseline resets.
`commands` seeds the production command queue; `commandState` and `observeCommands`
record playback state and callback values at invocation time. Per-step
`commandActions` exercise append, queue/history clearing, restoration, mapping reset,
direct processing and event/history options. `initializeExtruder` performs the same
zero-dt initialization as the app. `mapResources` seeds machine-keyed JS Maps and
Python dictionaries, including authored extrusion colors.
`usdBake` sends the same authored path (or in-memory source) to the production JS
baker and native `pxr.Usd` baker. It compares resolved joint world/local points,
rest lengths and stored wraps before ECS construction. Optional `deriveAll` and
`cablePathHalfWidthOverride` retain load-time policy semantics.
`scenes` executes native USD baking and each production ECS builder independently.
`pipeline: true` uses actual production registration in both languages and records
the system order; it forbids a fixture-supplied substitute system list.

For JSON snapshots of a single fixture:

```bash
node tests/parity3d/oracle.mjs tests/fixtures/python_3d_parity/rigid_members.json
PYTHONPATH=src/python:tests/python/cable_joints_3d .venv/bin/python -m parity_harness tests/fixtures/python_3d_parity/rigid_members.json
```

Fixtures specify `entities`, `systems` in execution order, `steps` with `dt`, and
explicit numeric `tolerance` (`atol`, `rtol`). Optional `resources`, `set` mutations
before each step, `initialSet` mutations before rigid initialization, deferred `addComponents`, `queries`, `initializeRigidBodies`
and `attachments` exercise state transitions and frames. `createPaths` invokes
production path construction and assigns names to generated paths; `geometry`
probes production operations listed in `geometry_contract.json`. Cable snapshots
include rest, stored and geometric lengths as well as force state. No viewer,
browser or worker is needed. Unknown components/systems and nonfinite outputs
fail. The material parameters stiffness/compliance alone permit positive
infinity, encoded as the string `"Infinity"` (rigid/zero-stiffness limits).
Nonfinite poses, lengths, forces and other numerical state still fail.
Systems may be names or `{ "name": "CableAttachmentUpdateSystem", "args": [false] }`
constructor definitions. `snapshotResources` selects simple resources to record;
`cableRotations` invokes production stored-length prediction at each snapshot.
`snapshotEntityMaps` records entity-keyed solver load resources using named
relationships. `entityResources` seeds those maps from `{kind, values}` definitions
at initialization or before a timestep (`kind` selects JS Map or object); Python
uses dictionaries. Optional torque-tuning fields serialize absent/None as null. The solver reads the resource `dt`; fixtures set it explicitly
without changing the reference World scheduling API.
Optional `tolerance.fields` selects qualified component fields by suffix, ignoring
array indices, and overrides their absolute/relative bounds. Unselected fields
retain the default bound. Structural/type checks and nonfinite rejection still
apply. Full-machine cases use explicit physical-unit bounds listed in
`PYTHON_3D_PARITY.md`; deposited extrusion lengths retain the default bound.

Current coverage:

- `usd_bake_hp4_rigid_body`, `usd_bake_hp3_rigid_body`, and the rigid pinhole case:
  complete authored cable initialization before scene construction.
- `usd_bake_frames` and policy/width variants: hierarchical frames, parallel/skew
  rolling tangents, layered endpoint radii, authored/manual values, auto wraps,
  forced rederivation and width override. Both bakers reject invalid manual data,
  stored lengths/modes, missing radii and duplicate joint relationships.
- `usd_scene_*`: eight authored files, a strict double-precision construction
  fixture and appended machines execute both production scene builders. They
  compare component presence, relationships, aggregate tensors/masses, material
  state, rigid members, attachments/rest/stored lengths, solver settings and
  extruder bindings. `snapshotMapResources` preserves machine-color maps.
  Native USD float32 opinions differ from JS literal doubles; authored initial
  states use atol `5e-10` / rtol `6e-8`. The double-precision fixture and native bake
  cases retain atol `1e-10` / rtol `1e-9`.
- `machine_pipeline_*`: the production 19-system pipeline runs HP3, HP4, rigid
  pinhole and a double-authored minimal machine for 200 steps, with exact
  within-engine repeatability. Full authored input uses documented per-field
  bounds; the same machines with shared double-authored input retain strict engine
  tolerances over 20 steps. No loader changes authored precision or adds substeps.
- `machine_pipeline_hp4_commands`: full-machine command playback, position/torque
  transitions, encoder diagnostics, extrusion at the pre-prediction tip, pause
  and zero update dt with a retained cable dt resource.
- `pbd_small_rotations`: recover representable quaternion-vector rotations when
  the scalar rounds to one; preserve sign and the existing cutoff.
- `commands_state`, `commands_playback`, and `commands_numeric_order`: one record
  per step, pause/zero-dt, motor/reference transitions, multiple machines per axis,
  cached mapping reset, queue/history restoration, callback timing and colors.
- `commands_extrusion_order` and `commands_cable_pipeline`: deposition before
  current-step physics, live effector state, commands with motor/cable/encoder
  integration and diagnostics. Both also run 200 repeatable timesteps.
- `motor_diagnostics_encoders`, `motor_diagnostics_frames`, and
  `motor_diagnostics_cables`: full-turn encoder slips, peak/current counts, JS
  rounding, machine resets, mode transitions, fallback angles and moving members.
- `extruder_frames`, `extruder_degenerate`, and `extruder_fallback`: authored
  triangle selection, all rotation conversion branches, numeric machine-key order,
  degenerate/missing sources, unrotated fallback offsets and pause.
- `extruder_rigid_constraints` and `extruder_rigid_fallback`: live body transforms
  after constraints with deliberately stale member ECS positions.
- `motion`: prediction, world angular frames, PBD velocities, kinematic/static,
  grabbed, zero-dt, pause/error behavior.
- `inertia`: small rotated SPD and rotated rank-2/rank-1/zero PSD inverse moments.
- `query_order`: smallest-store insertion order, including stable query ties.
- `rigid_members`: moving/rotating parents, member offsets, projected spools,
  reference transport, live attachment frames and internal/external reactions.
- `distance_members`: off-center rigid reactions with anisotropic inertia,
  compliant accumulated multipliers, internal endpoints and successive steps.
- `spool_projection`: tilted axes, swing rejection, off-axis angular velocity,
  encoder fallback axes and angle unwrapping through several turns.
- `geometry`, `geometry_degenerate`: plane-projected tangents/arcs in all winding
  directions, axial offsets, intersections and finite coincident-guide fallback.
- `cable_construction` and variants: live tilted member axes, local/world joints,
  initial wraps and hybrid knots, layering toggle, authored knots/overrides,
  path splitting at attachments, empty paths and parameter limits.
- `cable_cache_members`: live world poses and member-local orientation through
  parent motion, constraint-like pose edits and pause/resume.
- `cable_friction`, its layering-off variant, and `cable_friction_chain`: capstan
  rolling/pinhole friction, frictionless and free rolling guides, attachment
  barriers, slack/zero-rest spans, changed attachments and dt-scaled ordered
  redistribution through several guides.
- `cable_winding`: signed stored-length prediction at both ends, nonlinear ramps,
  wrap boundaries, negative stored length, linear/zero-radius limits and layer cap.
- `cable_attachment_motion` and its layering-off variant: world prediction,
  rolling payout, moving hybrid endpoints, parallel/skew wrap planes and
  attachment/cache/friction ordering.
- `cable_attachment_members`: carrier compound motion affects external spans,
  leaves onboard spool winding unchanged, then a commanded spool turn changes
  winding and encoder state. Separate knot angles share one spool across paths.
- `cable_attachment_clamp`: rolling and layered hybrid rest-length clamps change
  world/member-local orientation before knot phase projection.
- `cable_hybrid_transitions`: unwrapping/rewrapping at both ends, hysteresis,
  degenerate attachments, zero radius, flags and pause/transition-step counting.

- `stepper_state`: constructor defaults and state mutations; motor integration
  remains outside this fixture.
- `cable_solver_bodies`: full tensor reactions, external/internal members, damping,
  ordered per-path iterations, force resets, independent resource/update dt and pause.
- `cable_solver_spools`: direct tilted one-axis dynamics, holding release,
  component/global closed-loop stiffness, torque loads on mobile/static bodies and
  encoder turns. Static virtual loads also guard per-iteration accumulation.
- `cable_solver_pinhole` and its stale-inlet variant: upstream/downstream
  backdriving, rolling/hybrid spools, torque load deferral, force transfer and
  attachment/cache/friction/solve integration.
- `cable_zero_stiffness`: zero vs near-zero stiffness, with and without damping.
- `position_motor_standalone`, `position_motor_members`, `position_motor_cables`:
  open/closed-loop drive, torque-mode exclusion, zero inertia, tilted references,
  live aggregate inertia and preserved mass, parent reaction rotation/velocity,
  member vs standalone integration, commands and full cable/PBD/encoder order.
  A separate 100 Nm stress case uses 20 microsecond timesteps.
- `torque_motor_standalone`, `torque_motor_loads`, `torque_motor_members`,
  `torque_motor_cables`, `torque_motor_pinhole`: speed droop, friction/windage/cogging
  defaults and overrides, zero inertia, explicit/implicit signed cable loading,
  negative/malformed load coefficients, Map/object resources, drive-only housing
  reactions, torque/position transitions and complete motor/cable/PBD/encoder order.
- `cable_over_correction` and its member/pinhole variants: actual shared pushes,
  tensor rotor/host reactions, hybrid-only pinhole coupling, layered tangent
  rebuilding, duplicate joints, last-path metadata, zero stiffness and pause.

Rigid-member, distance, spool, cache, friction-chain, two moving-attachment,
four cable-solver, three over-correction, three position-motor and five
torque-motor fixtures run for 200 steps;
both engines must reproduce their own snapshots exactly on a second run.

These fixtures establish the covered authored-machine pipeline. Dynamic split/merge,
optional collision fixtures and richer Rerun recordings remain on the checklist
in `PYTHON_3D_PARITY.md`.
