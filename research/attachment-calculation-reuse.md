# JavaScript attachment calculation reuse, 2026-10-13

Two call-local JavaScript physics optimizations were implemented and validated separately. Over-correction now reuses post-solve tangents while gathering corrections, and attachment updates share endpoint poses, normals, frames and rotation deltas within each joint calculation. Eight frozen HP3/HP4 logo and autocal workloads cover layering on and off. Three unprofiled repeats per stage matched their setting-specific checkpoint hashes exactly. Endpoint preparation reduced median physics time a further 1.9–10.5%; the combined changes reduced it 6.2–13.2% versus the fresh baseline. Final verification passed 168 JavaScript tests and 176 every-timestep differential/pipeline checks. The bounded stage also passed three extended lifecycle cases. Separate CPU/system attribution covers representative layered HP3 logo and HP4 autocal workloads. These are single-host sequential measurements with possible JIT and time-order effects; frozen autocal waits are stress replays, not new adaptive collections or hardware validation.

## Implementation

Current tangents and their length are carried from over-correction detection into correction gathering. Pre-solve joint attachments remain available for detecting taut-to-slack transitions. All corrections are still gathered before any pose changes. The former redundant correction calculation path was removed.

A private endpoint-preparation helper feeds both standalone tangent calculation and attachment updates. The update passes its explicit effective radii to a private geometry helper; standalone callers still derive current layered radii. Prepared inputs expire after each joint calculation, and over-correction results expire after each update. History snapshot order, fixed solver-local points, topology handling and scene lifecycle are preserved. No persistent cache or invalidation key was added.

## Plain timings

| Case | Baseline seconds | Bounded seconds | Prepared seconds | Preparation reduction | Combined reduction |
| --- | ---: | ---: | ---: | ---: | ---: |
| hp3-logo-on | 2.831 | 2.884 | 2.657 | 7.9% | 6.2% |
| hp3-logo-off | 2.392 | 2.356 | 2.125 | 9.8% | 11.1% |
| hp3-autocal-on | 41.862 | 40.816 | 38.634 | 5.3% | 7.7% |
| hp3-autocal-off | 44.136 | 41.811 | 40.023 | 4.3% | 9.3% |
| hp4-logo-on | 17.117 | 16.206 | 15.898 | 1.9% | 7.1% |
| hp4-logo-off | 12.886 | 11.675 | 11.182 | 4.2% | 13.2% |
| hp4-autocal-on | 45.161 | 43.240 | 41.203 | 4.7% | 8.8% |
| hp4-autocal-off | 44.726 | 43.887 | 39.287 | 10.5% | 12.2% |

Three fresh-world repeats, separate 5,000-step warmup, Node 24.14.1 on Intel i7-12700K. All runs were sequential without our tests/profiles competing for CPU. The baseline source hash matches the original profiling source. Speed claims use physics batch timers only. All final repeat ranges lie below the fresh baseline ranges. Preparation repeats also lie below bounded repeats in seven cases; the layered HP4 logo ranges overlap. The bounded change alone ranges from 1.9% slower to 9.4% faster, so its speed evidence is mixed.

## Verification

- Bounded: 167 JavaScript tests; 179 differential/pipeline/lifecycle checks, including extended lifecycle repeatability.
- Prepared: 168 JavaScript tests; 176 default differential/pipeline/lifecycle checks, including 200-step repeatability and 1,000-step HP4 command settling.
- Targeted regressions detect four tangent calculations on the original source versus two after bounded reuse, and four endpoint-position preparations versus two after preparation reuse.
- Every plain repetition and separate attribution run matches the relevant checkpoint hash exactly; every-timestep native comparisons cover transient history, hybrid, topology and lifecycle behavior.
- `git diff --check` passed.

## Evidence

- [Session report](../output/research/sessions/2026-10-13-attachment-reuse/report.md): repeat distributions, attribution and source hashes.
- [Original profiling](headless-js-profiling.md): frozen workload origins, numerical contract and previous attribution.
- [Profiling recipe](../tests/benchmark/HEADLESS_JS_PROFILING.md): measurement and exact comparison protocol.
- [Differential harness](../tests/parity3d/README.md): every-timestep independent Python/JavaScript comparisons.

## Cumulative follow-up with the two earlier optimizations

After this worktree's changes were combined with per-update component-store reuse and attachment-transform scratch reuse, all eight frozen HP3/HP4 logo and autocal cases passed exact baseline checkpoints on each of three repeats. Cumulative unprofiled median physics time fell 6.7–16.0% per workload, with a 9.0% median across the eight cases. The complete per-workload medians, repeat ranges, hashes and test results are in the [cumulative validation report](../output/research/sessions/2026-10-13-three-optimizations/report.md). This measures the three-commit state directly; isolated percentages from separate studies should not be added.
