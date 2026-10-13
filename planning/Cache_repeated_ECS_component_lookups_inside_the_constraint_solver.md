# Cache repeated ECS component lookups inside the constraint solver

## Reason and architectural boundary

Profiling baseline: `0b5afc52`. The solver consumes 32–49% of physics time;
`World.getComponent` alone accounts for 9.1% of CPU self time in the layered HP4
logo profile. That percentage spans all systems, so it is evidence for investigating
lookups, not a promised solver speedup. See [measured results](../research/headless-js-profiling.md).

Start with [the simulator architecture](../hp-sim-3d/README.md#simulation-step-order)
and [app ownership](../hp-sim-3d/app/README_for_coders.md). Browser and headless
worlds share `app/simulationSystems.js`; this work belongs in the 3D physics
library. The 3D ECS re-exports the shared 2D `World`, whose component access is two
Map lookups. Keep the first change concentrated in the solver rather than adding
a global ECS cache or app-level plumbing.

## Files and intended changes

- [`src/js/cable_joints_3d/cable_joints_core.js`](../src/js/cable_joints_3d/cable_joints_core.js):
  in `PBDCableConstraintSolver.update`, reuse component stores/references across
  `applyConstraint`, previous/current pose accessors, and member-spin helpers.
  Start with hot endpoint components: position, orientation, mass, inertia,
  rigid membership, cable link, stepper and previous-final pose.
- [`src/js/cable_joints/ecs.js`](../src/js/cable_joints/ecs.js):
  use the existing `getComponentStore()` API; changing this shared implementation
  should not be necessary. Fetch stores afresh for each solver update, preserving
  absent-store/component behavior.
- [`src/js/cable_joints_3d/rigid_bodies.js`](../src/js/cable_joints_3d/rigid_bodies.js)
  is a possible second change if profiles still show nested lookup costs in
  endpoint mapping/transforms. Keep access explicit and preserve existing callers;
  do not temporarily override `world.getComponent` to inject a cache.
- [`tests/js/cable_joints_3d/pbdCableConstraintSolver.test.js`](../tests/js/cable_joints_3d/pbdCableConstraintSolver.test.js)
  and [`tests/js/hp-sim-3d/rigidBodyConstraintSolver.test.js`](../tests/js/hp-sim-3d/rigidBodyConstraintSolver.test.js):
  add focused coverage for any new cache lifetime, particularly missing components
  and replaced worlds/components. Existing torque, rigid-member and pinhole cases
  should retain their current expectations.

## Tripwires

- Cache component references, not their numerical values. The solver mutates
  positions, orientations and member-local orientations after each constraint;
  subsequent joints and iterations must observe those writes. Cached world poses,
  angular denominators or transformed inertia are a separate optimization.
- Entity identity has several roles: original attachment member, mapped solver
  endpoint, rotor with a free spin degree of freedom, and parent reaction body.
  `resolveRigidBodySolverEndpoint` also depends on the counterpart: internal
  member-to-member spans differ from external spans. Do not collapse these roles
  into one endpoint record keyed only by the original entity.
- A member's stored world pose can lag its parent during solving. Keep the
  transform helpers' parent/local composition instead of reading member `pos`
  directly as a substitute for `getEntityWorldPosition`.
- `World.clear()` retains systems/resources but clears component stores and
  resets entity IDs. Scene replacement/append and topology changes make caches
  retained across updates unsafe without a separate lifecycle design.
- Preserve the alternating forward/backward traversal, per-path iteration counts,
  first-iteration force reporting, pinhole force transfer, motor holding-torque
  release and torque-load resources. Lookup hoisting should not reorganize the solve.
- Benchmark direct store access before building per-entity records: constructing
  another Map/object graph each step can offset the lookup saving.

## Validation and relation to the other plans

Use [the headless profiling recipe](../tests/benchmark/HEADLESS_JS_PROFILING.md)
with the frozen inputs in `output/research/sessions/2026-10-13-headless-js-profiling/inputs/`.
Take fresh plain baselines, then use `--compare` with identical settings across
HP3/HP4, logo/autocal, layering on/off. Compare uninstrumented repeat distributions;
reprofile separately to confirm that the intended lookup cost fell.

The [differential harness](../tests/parity3d/README.md) checks every timestep and
scene lifecycle, complementing the benchmark's 1,000-step checkpoints. Use
`.venv/bin/python` for its focused solver/machine/lifecycle checks. Keep lookup
changes separate from scratch-object reuse and attachment memoization so each
change has its own numerical and timing evidence.
