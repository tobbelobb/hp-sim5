# PBD constraint hotspot optimization

We tested whether settled-constraint skipping and per-update invariant caching can speed up the JavaScript 3D PBD solver without changing physics. Across frozen HP3/HP4 logo and autocal replays, the retained endpoint/path cache matched all checkpoints exactly and reduced median whole-physics time by 1.2–3.6% in the final batch. A prior batch measured larger gains that did not reproduce. Active-set skipping regressed timing, and scratch-vector reuse added no reliable gain. Frozen replays do not validate adaptive collector behavior or hardware accuracy.

## Objective and acceptance criteria

Evaluate two exact-preserving ideas for the solver: skip settled constraints
while their dependencies remain unchanged, and cache per-update endpoint
mappings/component references and per-path solve coefficients. Keep the current
two-sweep schedule and traversal order. Accept only exact checkpoint equality
and repeatable improvement in uninstrumented timings.

## Method

Each of HP3 logo, HP3 autocal, HP4 logo and HP4 autocal used the saved frozen
input with layering on, 30,000 steps, three sequential repetitions and a
5,000-step warmup. Checkpoints every 1,000 steps include the solver's resulting
trajectory state. Baseline was freshly measured from revision
`1268cb8e90fd77d9b4a1aae3e15a210ea01b7b68`. The [research notebook](research.md)
records source hashes, all tested hypotheses and evidence directories.

## Results

The retained optimization hoists endpoint-to-solver-body mapping, endpoint
component references/inverse mass and compliance/damping coefficients out of
the per-sweep apply path. It refreshes those caches for every solver update and
keeps pose values live. It does not change the sweep schedule, constraint order,
geometry calculations or inertia math.

| Machine/workload | Baseline median, s | Optimized median, s | Whole-physics change |
| --- | ---: | ---: | ---: |
| HP3 logo | 2.526 | 2.467 | −2.3% |
| HP3 autocal | 2.197 | 2.170 | −1.2% |
| HP4 logo | 14.834 | 14.333 | −3.4% |
| HP4 autocal | 14.029 | 13.521 | −3.6% |

The weighted total of the four case medians fell by 3.3%; the mean of the four
case percentage changes is 2.6%. All three repetitions in each case had an
exact checkpoint hash match. A preceding isolated cache-only batch measured
5.8–9.7% reductions, but a second final-source batch produced 1.2–3.6%; the
larger estimate did not reproduce and should not be used as the expected gain.
The measurements are whole-physics time, not solver-only time.

### Source-change isolation

Between the two cache-only benchmark batches, the only relevant source change
was a nullish fallback when precomputing path compliance. We tested the old and
current forms in an A/B/A sequence on the same HP4 logo input, each with three
repetitions, 30,000 steps and the same warmup/checkpoint contract:

| Variant | Source hash prefix | Median physics time |
| --- | --- | ---: |
| Current A | `fc9d7c81` | 14.047 s |
| Prior B | `b4ba1d7c` | 13.940 s |
| Current A2 | `fc9d7c81` | 14.209 s |

All trajectories matched exactly. The prior version was 1.3% faster than the
mean of the two current medians, while current A-to-A medians differed by
1.1%; individual repetition ranges overlapped. This does not support the
fallback as the cause of the earlier 5.8–9.7% versus 1.2–3.6% gap. Current-
source timing differed by several percent between the early and late batches
across all workloads; run-condition variability is the likely explanation, but
the cause was not directly measured.

Settled-constraint skipping also matched all checkpoints, but increased median
whole-physics time by 3.1–7.1%. Version tracking and dependency bookkeeping cost
more than it saved on these workloads; this implementation was removed.
Reusable apply-constraint scratch vectors were tested on top of the cache. They
did not improve the cache-only timing consistently and were removed as well.

## Validation

- 35 focused Jest tests passed across the 3D PBD solver, rigid-body solver and
  hybrid endpoint symmetry suites.
- `node --check src/js/cable_joints_3d/cable_joints_core.js` passed.
- `git diff --check` passed.
- The final benchmark matrix and per-run checkpoint files are in [`final/`](final/).
- Cache-only and rejected experiment data are retained in [`h3/`](h3/),
  [`h1/`](h1/) and [`h3-scratch/`](h3-scratch/).

## References

- [Headless JS profiling report](../../../../research/headless-js-profiling.md)
  identifies the original solver share and documents the benchmark protocol.
- [Frozen workloads](../2026-10-13-headless-js-profiling/inputs/) provide the
  exact command/event streams used here.
- [Benchmark runner](../../../../scripts/profile_headless.mjs) calculates
  physics timings and rejects any checkpoint mismatch.
- [Solver source](../../../../src/js/cable_joints_3d/cable_joints_core.js)
  contains the retained optimization; baseline source was revision `1268cb8e`.

## Limitation

The saved command streams make runs directly comparable but cannot show whether
a live collector would make different movement or settling decisions after
physics changes. These runs do not establish hardware accuracy.
