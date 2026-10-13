# Reuse temporary vectors and quaternions in rigid-body transforms and attachment calculations

## Reason and architectural boundary

Profiling baseline: `0b5afc52`. Native allocation sampling estimated 258–1,682 KiB
allocated per update. World-position/orientation helpers, attachment geometry,
plane-basis construction and inertia transforms were leading sites. Reported GC
was only about 0.8–1.7% of replay time: target object creation and repeated math,
not GC tuning. See [measured results](../research/headless-js-profiling.md).

Read [the rigid-member model](../hp-sim-3d/README.md#rigid-bodies-and-members)
and [inertia-tensor architecture](../hp-sim-3d/README.md#inertia-tensors). These
helpers serve the solver and attachments as well as `RigidBodySyncSystem`, so
the dedicated sync-system timing understates their total cost. Physics uses the
repo's own `Vector3`/`Quaternion`, not Three.js math objects.

This is spread across shared math helpers and their consumers. Start with one
hot caller chain; do not convert every allocation site in one change.

## Files and intended changes

- [`src/js/cable_joints_3d/rigid_bodies.js`](../src/js/cable_joints_3d/rigid_bodies.js):
  provide explicit output/scratch paths for world-pose and local/world attachment
  transforms, then migrate a hot consumer. Existing allocating calls must continue
  returning independent values; member poses must still compose parent and local frames.
- [`src/js/cable_joints_3d/quaternion.js`](../src/js/cable_joints_3d/quaternion.js)
  and [`vector3.js`](../src/js/cable_joints_3d/vector3.js):
  add narrowly needed destination operations, such as quaternion-vector transform
  into an existing vector. Preserve the original arithmetic and documented aliasing
  behavior rather than replacing the math implementation wholesale.
- [`src/js/cable_joints_3d/cable_joints_core.js`](../src/js/cable_joints_3d/cable_joints_core.js):
  reuse caller-owned temporaries in `calculateAttachmentPoints`, `_updateAttachmentPoints`
  or a solver transform chain. Separate scratch intermediates from attachment,
  force and local-coordinate objects that survive the call.
- [`src/js/cable_joints_3d/geometry3.js`](../src/js/cable_joints_3d/geometry3.js)
  and [`inertia_tensor.js`](../src/js/cable_joints_3d/inertia_tensor.js) are follow-up
  targets supported by the allocation profiles: reuse basis/projection temporaries
  and local/world inertia-transform intermediates. Make these separate measured
  changes after the first transform conversion.
- [`src/js/cable_joints_3d/commonSystems.js`](../src/js/cable_joints_3d/commonSystems.js):
  migrate `RigidBodySyncSystem` only if needed to exercise the new helper API;
  retain member/spool projection and synchronized-pose bookkeeping.
- Tests under `tests/js/cable_joints_3d/` (`vector3`, `quaternion`, `inertiaTensor`,
  tangent/attachment tests) and `tests/js/hp-sim-3d/rigidBodySyncSpoolAxis.test.js`:
  exercise destination aliasing and ownership as well as numerical outputs for
  whichever helper chain changes.

## Tripwires

- Existing pose/attachment helpers return detached objects; callers mutate them.
  Returning a component's live vector or one globally shared temporary silently
  changes ownership. Prefer explicit caller-owned destinations/workspaces and keep
  A/B endpoints and nested helper calls from overwriting one another.
- Retained values need independent storage: `jointLocals`, correction arrays/maps,
  component attachment points, previous poses and force vectors cannot point at
  scratch objects reused by the next joint. Over-correction gathers multiple deltas
  before applying their averages.
- Custom `Vector3.set(v)` returns `undefined`, unlike `Quaternion.set(q)`;
  `Vector3.cross(v)` allocates. Existing `crossVectors(a,b)` is not safe when its
  destination aliases an input because it writes coordinates before reading all
  source coordinates. Verify aliasing rather than assuming Three.js conventions.
- `_getPlaneNormal` can return a component-owned normal or the shared default.
  Treat inputs as read-only even when other branches return freshly allocated vectors.
- Preserve normalization, multiplication order, degenerate-input fallbacks and
  arithmetic order. Replacing `Math.hypot` or algebraically simplifying inertia math
  is a separate numerical change, even if an allocation profile points nearby.
- Rigid-body members are kinematic attachment frames; spool members retain local
  twist. Avoid mutating parent/local quaternions while forming inverse transforms,
  or substituting stale member world components for composed transforms.

## Validation and relation to the other plans

Use [the profiling recipe](../tests/benchmark/HEADLESS_JS_PROFILING.md): fresh plain
baselines and `--compare` for the same frozen HP3/HP4 logo/autocal inputs and
layering settings. Repeat plain timings first; use a separate `--heap-sampling`
run to verify reduced allocation at the converted sites, then inspect CPU/system
attribution. Reduced sampled bytes alone is not a speed result.

Run focused ownership/aliasing tests and the applicable [per-step differential
checks](../tests/parity3d/README.md) with `.venv/bin/python`. Keep this change
independent of component-lookup caching and attachment-result caching. The last
two plans share `cable_joints_core.js`; scratch-backed results must never become
persistent cached results without independent ownership.
