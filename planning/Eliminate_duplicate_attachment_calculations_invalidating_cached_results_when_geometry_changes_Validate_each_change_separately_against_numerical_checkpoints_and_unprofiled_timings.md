# Eliminate duplicate attachment calculations with explicit invalidation

Implemented 2026-10-13: correction-local tangent reuse and joint-local endpoint
preparation were validated separately. All eight workload/settings cases retain
exact checkpoint hashes; final median physics times are 6.2–13.2% lower than a
fresh baseline. No persistent cache was introduced. See the
[implementation and validation results](../research/attachment-calculation-reuse.md).

## Reason and architectural boundary

Profiling baseline: `0b5afc52`. Attachment updates consume roughly 25–33% of
physics time and over-correction another 7.5–12.2%; `calculateAttachmentPoints`
is a CPU and allocation hotspot shared by both. The profiles establish cost,
not a measured cache hit rate or guaranteed benefit. See
[measured results](../research/headless-js-profiling.md).

Start with [the documented step order](../hp-sim-3d/README.md#simulation-step-order)
and cable-path/rigid-member sections. The shared production pipeline is:
integration → rigid sync → attachment update → attachment history snapshot →
friction → solver → over-correction. Later systems often need different geometry
from earlier ones, even within the same timestep.

The existing `CableAttachmentCacheSystem` stores **historical link poses** used
to measure rotation/translation and cable transfer. It is not a memoization cache.
Keep that responsibility intact; [app ownership](../hp-sim-3d/app/README_for_coders.md)
also places this work in the physics library, not browser controllers.

## Files and intended changes

- [`src/js/cable_joints_3d/pbdResolveCableOverCorrections.js`](../src/js/cable_joints_3d/pbdResolveCableOverCorrections.js):
  begin with the bounded duplicate: `update()` computes current attachments to
  identify over-corrected joints, then `calculateJointCorrection()` computes them
  again. Carry those results into correction calculation; all corrections are
  gathered before poses are changed, giving this reuse a short, explicit lifetime.
- [`src/js/cable_joints_3d/cable_joints_core.js`](../src/js/cable_joints_3d/cable_joints_core.js):
  a separate follow-up can share endpoint preparation between `_updateAttachmentPoints`
  and `calculateAttachmentPoints`, which currently repeat world poses, normals and
  frame/delta-angle work within one joint calculation. Prefer passing prepared
  inputs for that call before introducing geometry state retained across systems.
- [`src/js/cable_joints_3d/cable_attachment_cache_system.js`](../src/js/cable_joints_3d/cable_attachment_cache_system.js):
  preserve when and how `prevCableAttachmentTime*` snapshots are taken. If broader
  memoization eventually spans this boundary, history writes must participate in
  invalidation; moving this system to obtain cache hits changes the physics.
- [`tests/js/cable_joints_3d/pbdResolveCableOverCorrections.test.js`](../tests/js/cable_joints_3d/pbdResolveCableOverCorrections.test.js)
  and attachment/history/hybrid/topology tests in the same directory: extend the
  affected cases to catch stale results after pose, winding or topology changes.
  For a broader cache, add scene reset/append coverage through the existing machine
  lifecycle differential suite.

The first change is concentrated in over-correction, with no persistent cache
needed. A cache shared across systems would spread into geometry mutation and
lifecycle owners; undertake that only if subsequent profiles justify the complexity.

## Tripwires and invalidation inputs

- `joint.attachmentPoint*_world` is intentionally the pre-solve attachment state
  used for over-correction's `preLen`. The current tangent calculation supplies
  `postLen`. Do not overwrite the former with memoized post-solve results or replace
  the latter with the earlier attachment-update output.
- The solver forms `jointLocals` once from initial attachments, then transforms
  those fixed local points using each successive corrected pose. This is not the
  same operation as re-deriving tangents with `calculateAttachmentPoints`.
- Current position/orientation alone is not a sufficient key. Inputs include
  parent-body pose, member-local pose, prior link poses and joint attachment points,
  local cable-plane normals, radius, stored winding length, cable width/layering,
  link types/direction and path/link index. Endpoint frame selection also depends
  on whether the counterpart is on the same rigid body.
- `_updateAttachmentPoints` changes rest/stored lengths and may correct rotor
  orientation while clamping rest length; hybrid transitions and split/merge alter
  path topology. Friction redistributes rest lengths, and solver corrections change
  poses joint by joint. Object identity and a once-per-step cache stamp miss these
  in-place writes; any broader cache must account for its actual dependencies.
- Radius arguments matter: attachment update supplies effective radii explicitly,
  while over-correction omits them and derives current layered radii. Preserve this
  distinction. Sharing by endpoint entity alone also conflates path-dependent state.
- Keep shared results read-only and owned for their whole lifetime, especially
  if the temporary-vector plan introduces scratch destinations. A cached result
  overwritten by the next helper call is worse than a missed cache hit.
- Default production attachment registration disables merge/split, but core tests
  and feature overrides exercise them. Scene replacement retains systems while
  reusing entity IDs. A persistent cache requires topology/world-lifecycle handling.

## Validation and relation to the other plans

Use [the profiling recipe](../tests/benchmark/HEADLESS_JS_PROFILING.md) and existing
frozen inputs under `output/research/sessions/2026-10-13-headless-js-profiling/inputs/`.
Measure the bounded over-correction reuse separately from endpoint-preparation
reuse or a broader cache. For each, compare exact checkpoint hashes using
`--compare` and plain repeat distributions across both machines, workloads and
layering settings; use CPU/system profiles separately to confirm fewer calculations.

The [differential harness](../tests/parity3d/README.md) checks every timestep,
including history, topology and scene lifecycle; use `.venv/bin/python` for it.
The benchmark's 1,000-step checkpoints alone can miss transient cache errors.
Layering-off HP3 autocal already differs by up to 1,080° from its original layered
trace, so compare each setting to its own baseline. Its frozen waits are a stress
replay, not evidence of a successful new adaptive collection.
