# Attachment transform scratch, 2026-10-13

The first caller chain in the [temporary-vector/quaternion plan](../../planning/Reuse_temporary_vectors_and_quaternions_in_rigid-body_transforms_and_attachment_calculations.md)
now uses caller-owned destinations in the PBD cable solver. World pose helpers
accept optional destinations; attachment helpers additionally accept scratch
from `createAttachmentTransformScratch()`. `Quaternion.transformVectorInto()`
supports an input/destination alias and preserves raw quaternion arithmetic.
Calls without destinations still return detached values.

Each solver owns pose scratch and separate A, B and coupled-spool destinations.
Joint locals and mapped endpoint results retain independent vectors. Geometry,
inertia transforms, rigid-body synchronization, lookup caching and attachment
result caching were left for separate changes.

## Plain timings and numerical equality

Fresh baselines at `0cfe920c` and the working-tree optimization used the same
frozen HP3/HP4 logo/autocal inputs from the [preceding profiling investigation](../../research/headless-js-profiling.md).
Each case ran three plain repetitions of a **30,000-step prefix** (60 simulated
seconds), with a separate 5,000-step VM warmup and fresh measured worlds.
The [profiling recipe](HEADLESS_JS_PROFILING.md) was followed sequentially on an
i7-12700K with Node 24.14.1/V8 13.6. Tests and simulation jobs did not compete
with benchmark runs.

| Case | Before median, seconds | After median, seconds | Physics time reduction |
| --- | ---: | ---: | ---: |
| HP3 logo, layering on | 2.844 | 2.872 | −0.99% |
| HP3 logo, layering off | 2.313 | 2.320 | −0.29% |
| HP3 autocal, layering on | 2.507 | 2.479 | +1.10% |
| HP3 autocal, layering off | 2.741 | 2.808 | −2.44% |
| HP4 logo, layering on | 17.182 | 16.719 | +2.70% |
| HP4 logo, layering off | 12.652 | 12.123 | +4.18% |
| HP4 autocal, layering on | 16.646 | 16.223 | +2.54% |
| HP4 autocal, layering off | 16.478 | 16.473 | +0.03% |

All repetitions and profiler runs match their baseline checkpoints **exactly**,
including positions, orientations, cable state, forces, motor state and saved
encoder observation times. Inputs/settings and baked scenes are unchanged.
The 48 plain trajectories plus six attribution trajectories measured 1,620,000
updates, excluding warmup.

HP4 logo and layered autocal show a modest improvement. HP3 timing changes are
mixed, including a 2.44% higher median for unlayered autocal; this change does
not establish a speedup across every workload. Repetition ranges and individual
timings are retained in the evidence. Prefix measurements do not establish
performance over the full autocal schedules, fresh collector behavior or hardware.

## Separate allocation and CPU/system attribution

The layered HP4 logo allocation sample estimates **9.31% fewer total allocated
bytes** (51.58 GB → 46.78 GB over 30,000 updates). Allocation attributed to the
world-position and world-orientation helper stacks fell by approximately 51%
and 38%, respectively. Attachment-transform helper stacks also lost their
intermediate allocations; independent output vectors still allocate at their
callers. V8 sampling estimates allocation over time, not retained heap or exact
object counts, and attribution can move when functions inline.

Separate system instrumentation recorded PBD solver time of 8.491 → 7.945
seconds, a 6.43% reduction. CPU profiles still identify `applyConstraint`, ECS
lookups and vector lengths among the main costs; its inclusive time was
6.796 → 6.611 seconds. Profiled runs provide attribution; the plain timing
table supplies the throughput evidence.

## Validation and evidence

- All 56 JavaScript 3D suites passed (323 tests), followed by a passing additional
  raw-quaternion destination test: 324 tests validated in total.
- `.venv/bin/python -m pytest tests/python/cable_joints_3d/test_differential_parity.py -q`:
  all 160 checks passed, including member/spool frames, moving attachments,
  pinhole coupling, multiple solver iterations and repeated 200-step sequences.
- New tests cover vector destination aliasing, raw/zero quaternions, composed
  live member poses with stale cached components, fallbacks, retained endpoint
  ownership and shared solver reuse across worlds.
- `git diff --check` passed.

The [session report](../../output/research/sessions/2026-10-13-reuse-attachment-transform-scratch/report.md)
and [machine-readable summary](../../output/research/sessions/2026-10-13-reuse-attachment-transform-scratch/summary.json)
retain repetition distributions, exact checkpoint hashes, source/input hashes,
frozen inputs, before/after source snapshots, raw profiles and sequential runners.
These local generated artifacts are ignored by Git. Measured source hash prefixes
are `bd4847f81f25` before and `20207f026fac` after.
