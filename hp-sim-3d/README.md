# 3D Hangprinter simulator

hp-sim-3d is the 3D Hangprinter application built on the shared JavaScript
cable-joints PBD/XPBD code in `src/js/cable_joints_3d/`. Machine geometry and
physics properties come from USDA scenes; application controllers turn print
commands into motor input and render the resulting ECS world with Three.js.

The simulation is intentionally specialized for Hangprinter cable paths,
rigid assemblies, and driven spools. It is not a general-purpose rigid-body
joint engine.

The [Rerun flight recorder](FLIGHT_RECORDER.md) records geometry, line lengths,
and cable forces at every physics timestep, with live viewing and saved `.rrd`
files for replay.

## Start here

From the repository root, run `npx vite` and open
<http://localhost:5173/hp-sim5/hp-sim-3d/>. The default is Hangprinter v4. Use
**Print Logo** or **Print Squares** for a built-in print, **Upload File** for
G-code/command logs or an authored USDA, and **Machines** to add/remove presets.
Expand **▼** for **Pause**, **Reset**, **Finish ASAP**, **Rerun** and view tools.
**Show Forces** is a rendering overlay; the recorder logs forces independently.
**Line Layering** changes the winding model and rebuilds cable initialization.
**Closed Loop Motors** changes position-drive behavior. Keep these settings the
same when comparing runs; quality checks and reference paths aid inspection.

To record a browser print, run
`.venv/bin/python scripts/hangprinter_flight_recorder.py`, open its printed viewer
URL, then click **Rerun** in the simulator before starting the print. This records
JS physics. For native Python physics, use the CLI or API below. Both read the
same authored machine data, but the native CLI consumes timestep-scheduled motor
JSON rather than G-code and has no browser print/upload UI.

The [recording guide](FLIGHT_RECORDER.md) has complete copy-and-run recipes for
browser logo recording, native settling, two-second motion/extrusion, torque
transitions, live connections and replay. It also explains the effector marker,
plot units, output-file naming and common connection problems. Opening an RRD
replays saved state; it does not run or record another simulation.

### Authored machine presets

All files are under `public/usd_scenes/`. The native CLI selects one by path.
The scene's default root is used automatically; `--scene-prim` overrides it.

| Browser label | USDA file |
| --- | --- |
| Hangprinter v4 (default) | `hp4_rigid_body.usda` |
| Hangprinter v3 | `hp3_rigid_body.usda` |
| Four High Anchors | `four_high_anchors_rigid_body.usda` |
| CubeCorners | `cubecorners_rigid_body.usda` |
| Slideprinter Multi Unit | `slideprinter_multi_unit_rigid_body.usda` |
| Slideprinter Original | `slideprinter_rigid_body.usda` |
| Slideprinter (hexagon) | `slideprinter_hexagon_rigid_body.usda` |
| Slideprinter (single pinholes) | `slideprinter_single_pinholes_rigid_body.usda` |

Initial-construction differentials cover these catalog scenes. The complete
machine timestep suite specifically covers HP3, HP4, single-pinhole Slideprinter
and a minimal fixture; do not infer equal trajectory coverage for every preset.

### Manual acceptance checklist

Work through these checks one run at a time. Keep the machine, settings,
command source and RRD path with any result you report.

- [ ] **Browser baseline:** print the HP4 logo; pause/resume and reset. Confirm
  the effector/toolpath behaves as expected, and note any quality/missed-step report.
- [ ] **Browser recording:** start the receiver, connect Rerun, print again,
  disconnect, then open the printed RRD path. Check consecutive simulation steps,
  cable forces and length traces. Repeat with **Finish ASAP**; recording can slow
  execution, but it retains every physics step.
- [ ] **Native construction/settling:** record 200 steps of the same HP4. Find
  `world/machines/default/effector` (orange point/axes), body/member frames and
  nine cable paths. Select `sim_step` 0–200 / `sim_time` 0–0.4 seconds.
- [ ] **Native movement/extrusion:** run the guide's 1,000-command ramp. Compare
  first/last effector positions, zoom to see about 29 mm travel, and inspect ten
  deposition points totaling 0.01 m, encoder tracking and zero missed steps.
- [ ] **Torque/position:** run the guide's D-axis example. Check motor mode/torque
  traces, force response, loss of commanded-length/error traces in torque mode,
  and their return when position mode resumes.
- [ ] **Another machine and lifecycle:** record HP3 or single-pinhole Slideprinter
  into a separate file. In the browser, add/remove machines and reset while
  recording; check file boundaries and names. For native append/replacement,
  follow the [API example](../README_adv.md#append-or-replace-authored-machines).
- [ ] **Numerical confirmation:** run the focused differential commands below;
  shape/plot similarity alone cannot establish ordering/frame/constraint parity.

```bash
.venv/bin/python -m pytest -q \
  tests/python/cable_joints_3d/test_machine_scene_parity.py \
  tests/python/cable_joints_3d/test_machine_pipeline_parity.py \
  tests/python/cable_joints_3d/test_machine_lifecycle_parity.py \
  tests/python/cable_joints_3d/test_machine_snapshot.py \
  tests/python/cable_joints_3d/test_machine_recording.py
```

These execute the live JS oracle in Node and/or read real saved RRD data. The
[harness README](../tests/parity3d/README.md) explains fixtures, numerical bounds
and running the whole 3D suite. The [main README](../README.md#tests) lists the
full JS, Python and autocal suites.

## Runtime and Scene Construction

The native Python headless entry point loads the same authored machine files:

```bash
PYTHONPATH=src/python .venv/bin/python - <<'PY'
from cable_joints_3d.machine_simulation import load_machine_world

world = load_machine_world('public/usd_scenes/hp4_rigid_body.usda')
for _ in range(200):
    world.update(world.get_resource('dt'))
PY
```

It registers the meaningful simulation systems in the JS app's order. Optional
`recording=` accepts a Rerun recording stream. See the
[Python parity checklist](../PYTHON_3D_PARITY.md) for covered machines, numerical
bounds, fixture coverage and explicit differences. For primary native Rerun recording,
use `PYTHONPATH=src/python .venv/bin/python -m cable_joints_3d PATH --steps 200`;
the [recording guide](FLIGHT_RECORDER.md) describes saved files, commands and live sinks.

`app/hp-sim-3d.js` boots the application assembled by `app/appBootstrap.js`.
The bootstrap is a composition root for controllers that own machine loading,
print jobs, workers, feature flags, quality checks, inspection tools, and view
state.

`app/setupScene.js` is a small scene-loading coordinator. Scene construction is
split into these phases:

1. `app/scene/machineSceneSpec.js` normalizes the supplied USDA stage, reads the
   selected scene root, and validates the machine specification.
2. `app/scene/machineScenePipeline.js` discovers scene prims and builds an
   entity plan using the feature-specific builders in `app/scene/`.
3. `app/scene/entityPlanApplier.js` applies resources, entities, and post-apply
   hooks to the ECS world.
4. `app/sceneSystems.js` registers the input and simulation systems once, in
   execution order.

`app/machineSceneController.js` owns catalog and uploaded machines, bakes cable
scene data before loading, and can append multiple namespaced machines to the
same world. The first machine supplies global physics resources such as gravity
and the fixed timestep derived from USDA `timeCodesPerSecond`.

The renderer is stored as the `renderSystem` world resource rather than as a
simulation system. `app/runner.js` advances physics and invokes rendering
separately, so ASAP playback can simulate several steps between rendered
frames.

## Simulation Step Order

`World.update(dt)` runs systems in the order registered by
`app/sceneSystems.js`. For an active local simulation step, the order is:

1. Process pointer/input state.
2. Save previous final positions and orientations.
3. Consume queued spool commands and run position-mode stepper motors.
4. Apply gravity, then integrate predicted linear and angular poses.
5. Sync rigid-body members from their parent bodies and enforce spool-axis
   projections.
6. Rebuild cable attachment geometry and cache attachment data.
7. Redistribute adjacent cable-segment rest lengths in
   `CableFrictionSystem` before constraint solving.
8. Solve cable constraints using each path's `solverIterations`, then resolve
   cable over-corrections.
9. Derive linear and angular velocities from the corrected poses.
10. Run torque-mode motors using cable loads recorded by the cable solver.
11. Update extrusion, encoders, and missed-step diagnostics.

The central position solve still has the usual PBD/XPBD shape:

```txt
save previous pose -> integrate pose -> solve constraints -> update velocities
```

Torque mode is a specialized velocity update after that sequence. Its angular
velocity change affects subsequent simulation steps.

There is no global substep loop around the complete system sequence. The runner
repeatedly calls `world.update(dt)` with the fixed scene timestep, either from a
real-time accumulator or in time-budgeted ASAP batches. Cable paths can request
different solver iteration counts through `cablePath:solverIterations`, while
friction work per step is scaled with `dt`.

## Cable Paths

A `CablePathComponent` joins ordered cable segments and stores link types,
winding direction, stored length, stiffness/compliance, damping, cable width,
and its solver iteration count.

Before the position solve, `CableFrictionSystem` redistributes rest length
between adjacent segments. With no active friction barrier it tends toward
equal PBD extension; at pinholes and non-free rolling links it limits the
tension ratio using the configured coefficient of friction and wrap angle.
Cable attachment updating also handles rolling geometry, hybrid link state,
and optional line layering.

The cable solver maps ordinary rigid-body-member endpoints to the parent body,
applies translational and tensor-based angular corrections, and treats supported
spool endpoints as a special one-axis rotational degree of freedom. For
torque-mode spools it also records cable load torque and, where available,
effective stiffness and damping for `TorqueModeSystem`.

## Rigid Bodies and Members

The rigid assembly model uses:

```txt
RigidBodyComponent
RigidBodyMemberComponent
RigidBodySyncSystem
```

`app/scene/rigidBodyBuilder.js` turns an authored rigid-body group into one
simulated parent entity. It computes aggregate mass, center of mass, linear
velocity, and inertia tensor from the members. Member inertia tensors are
rotated into the aggregate frame and combined with parallel-axis terms.

The original entities become zero-mass, non-gravity-affected members with
authored local positions and orientations. `RigidBodySyncSystem` derives their
world transforms and velocities from the parent pose. Cable and distance
constraint helpers normally redirect a member endpoint's reaction to that
parent body.

This is equivalent to a rigid body with attachment frames, not a collection of
independent bodies connected by XPBD fixed joints. Member attachment is
therefore hard and kinematic; it has no per-joint compliance or solver
iteration setting.

## Spools and Motors

A physical motor housing is fixed to its Hangprinter body, while its spool rotor
has one rotational degree of freedom. hp-sim-3d represents that behavior with a
specialized member model:

- `SpoolStateComponent` stores the spool axis and reference orientation.
- `StepperMotorSystem` handles position-commanded motors. Open-loop mode uses
  holding and damping torque; closed-loop mode directly projects to the target
  twist.
- `TorqueModeSystem` handles torque-commanded motors after the cable solve and
  includes the solver's cable load.
- Rigid-body-member spools integrate their free twist in member-local space.
- `RigidBodySyncSystem` recomposes the world orientation from the parent body
  and local member orientation.
- `constrainSpoolOrientation()` removes swing, and
  `constrainSpoolAngularVelocity()` removes off-axis angular velocity.
- Motor paths apply a custom equal-and-opposite rotation or angular impulse to
  the parent body when a spool is a rigid-body member.

The spool is therefore not a generic XPBD hinge. Direct projection is cheap and
stable, but removing off-axis motion is not the same as solving a bearing
constraint and can discard angular energy without exposing the corresponding
bearing reaction.

## Inertia Tensors

The 3D ECS uses a full 3x3 inertia tensor. `MomentOfInertiaComponent` stores the
local tensor and its inverse; scalar constructor input remains supported as an
isotropic tensor. `src/js/cable_joints_3d/inertia_tensor.js` supplies world-space
tensor transforms, inverse-inertia application, axis-effective inertia, and
parallel-axis aggregation.

Cable constraints, distance constraints, over-correction resolution, collision
response, and motor/body reaction paths use this tensor logic. Spool drive and
spool-axis constraints intentionally reduce it to effective inertia about the
allowed axis, because the current rotor model exposes only that one degree of
freedom.

## Remaining Hinge-Joint Gap

If bearing reactions or off-axis rotor dynamics become important, the next
model should be:

```txt
body <-> motor housing: fixed attachment
motor housing/body <-> spool rotor: hinge joint + angular motor
```

That requires a rotor entity independent of the body, fixed/hinge constraints
between bodies, and a motor constraint that shares corrections through the
existing inverse-inertia tensors. Compliance, damping, and stored constraint
multipliers could then model bearing softness and losses and expose bearing
reaction forces or torques. More solver iterations or whole-step substeps may
also be needed for a stiff hinge. The current one-axis projection remains a
useful simple mode.
