# PBD constraint hotspot optimization

## Objective

Evaluate two exact-preserving ideas for the JavaScript 3D PBD cable solver:
skip settled constraints when their dependencies have not changed, and cache
per-update endpoint mappings/component references and per-path coefficients.

## Constraints and method

- Keep the existing two-sweep schedule, solve order, constants and arithmetic.
- Use frozen HP3/HP4 logo and autocal workloads, 30,000 steps each, layering on.
- Run three sequential repetitions per case with a 5,000-step warmup.
- Require exact equality at every 1,000-step numerical checkpoint.
- Retain hypotheses that miss their speed or correctness target as rejected
  experiments; remove those changes from the final implementation.

## Tested hypotheses

| Experiment | Hypothesis | Outcome | Evidence |
| --- | --- | --- | --- |
| B0 | Fresh baseline from the current tracked revision establishes timing and checkpoint references. | Complete | [`baseline/`](baseline/), revision `1268cb8e90fd77d9b4a1aae3e15a210ea01b7b68`, source SHA-256 `6a1c9e3271e8e2b210706aa4994b514c3868e78be3908ab09264afd2a1f70ba1`. |
| E1 | Skip a constraint that was slack/settled while versions of its endpoints and adjacent pinhole-related endpoints remain unchanged. | Rejected for performance. Exact checkpoints matched, but median whole-physics time regressed 3.1–7.1%; implementation removed. | [`h1/`](h1/). |
| E2 | Cache endpoint-to-solver-body mapping, live component references and inverse mass, and compliance/damping coefficients once per update/path. | Retained. All final checkpoints match exactly; final repeated batch measured 1.2–3.6% lower whole-physics time. A preceding batch measured 5.8–9.7%, so the gain is timing-sensitive and the lower final batch is the conservative result. | [`final/`](final/), optimized source SHA-256 `fc9d7c813ec8a140e9cd26d799966a648ea9e51cc4ef021b82ff8e7d184881e8`. Earlier isolated cache-only batch: [`h3/`](h3/). |
| E3 | Add reusable scratch vectors and in-place translation corrections on top of E2. | Rejected for performance. Checkpoints matched, but it did not improve over E2; removed. | [`h3-scratch/`](h3-scratch/). |
| E4 | The later nullish fallback (`complianceValue = compliance ?? 0.0`) caused the large later-batch slowdown. | Not supported. A controlled A/B/A HP4 logo run measured 14.047 s and 14.209 s with the fallback versus 13.940 s without it. The prior form was 1.3% faster than the mean current-form median; the two current medians differ by 1.1%, and per-run timing ranges overlap. All checkpoints matched. | [`compliance-abba/`](compliance-abba/); source hashes `fc9d7c81` (A) and `b4ba1d7c` (B). |

The kept code is in `src/js/cable_joints_3d/cable_joints_core.js`. It refreshes
all contexts for each solver update, retains live component objects, and does
not cache changing positions, orientations, transformed inertia or attachment
geometry. The two-sweep solve and traversal order remain unchanged.

## Final timing comparison

Medians are from three plain runs. Each run replays the same frozen input and
compares every checkpoint against its matching baseline.

| Machine/workload | Baseline, s | Kept change, s | Change |
| --- | ---: | ---: | ---: |
| HP3 logo | 2.526 | 2.467 | −2.3% |
| HP3 autocal | 2.197 | 2.170 | −1.2% |
| HP4 logo | 14.834 | 14.333 | −3.4% |
| HP4 autocal | 14.029 | 13.521 | −3.6% |

The weighted total of the four case medians fell 3.3%. The four case changes
average 2.6%. The earlier cache-only batch produced larger reductions, but the
final repeated batch did not reproduce them; treat 1.2–3.6% as the measured
conservative range. These are whole-physics timings, not solver-only timings.

The source-hash difference between those batches was isolated. The only solver
change after the earlier batch defaulted missing compliance to zero before
precomputing path coefficients; production paths already have numeric
compliance. The A/B/A result above shows no measurable slowdown from that
fallback. The old form was 1.3% faster than the mean of the current-form
medians; the same current source varied by 1.1% between the two A medians, and
individual timing ranges overlapped. The early-to-late batch difference was
about 4–7% across workloads. The remaining discrepancy is consistent with
run-condition variability, but its cause was not instrumented and cannot be
attributed more specifically.

All final checkpoint hashes match baseline hashes exactly for all three
repetitions in all four workloads. Focused validation passed 35 tests across
`pbdCableConstraintSolver.test.js`, `rigidBodyConstraintSolver.test.js`, and
`hybridEndpointSymmetry.test.js`; `node --check` and `git diff --check` passed.

## References

- [Headless JS profiling report](../../../../research/headless-js-profiling.md):
  original hotspot shares, profiler recipe, and frozen-input provenance.
- [Frozen workloads](../2026-10-13-headless-js-profiling/inputs/): the exact
  HP3/HP4 command and event streams used here.
- [Benchmark runner](../../../../scripts/profile_headless.mjs): warmup,
  checkpoints, timing collection and exact-comparison implementation.
- [Solver source](../../../../src/js/cable_joints_3d/cable_joints_core.js): the
  retained implementation; baseline code was revision `1268cb8e`.

## Limitation and next decision

The frozen workloads preserve inputs and make numerical comparison decisive,
but they do not measure a collector adapting to changed trajectories or validate
hardware behavior. The active-set approach did not pay for itself here; further
large gains likely need a different solve formulation or a validated reduction
in constraint work.
