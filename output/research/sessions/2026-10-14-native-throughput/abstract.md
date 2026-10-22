# Native Rust headless throughput versus Wasm and Python

Status: complete.

## Abstract

Native Rust commit ab833759 was benchmarked against the exact preceding Rust/Wasm headless default at 7905e64f, and against the former Python default for fresh research trials. Three sequential repetitions covered HP3/HP4, layering on/off, frozen logo and autocal schedules, continuing worlds, every-step fresh telemetry, real six-point firmware collections, and the standalone autocal service with and without recording. Native shared-pipeline throughput improved 1.145–1.235×, with an aggregate ratio of 1.181×. Recording reduced end-to-end gains; short real firmware collections showed only 1.02–1.03× without recording and approximately 0.99× with recording. Fresh numerical trials improved 3.11–7.17× over Python; recorded trials improved 1.46–2.12×. Repeats were deterministic, but five long physics comparisons exceeded the declared pointwise tolerance. These host measurements do not establish calibration accuracy or complete autocal optimization speedup.

## Topics

Rust, native, Wasm, headless, throughput, HP3, HP4, Rerun, autocal, recording, firmware

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Native improves shared-pipeline throughput. | N01 | Supported for fixed schedules; numerical agreement mixed | [Physics matrix](physics/matrix.json) |
| Recording/transport reduce gains. | N02, N05 | Supported | [Consumer matrix](consumer-matrix.json), [service matrix](autocal-service-matrix.json) |
| Fresh trials improve over Python. | N03 | Supported for 200-step trials | [Consumer matrix](consumer-matrix.json) |
| Fresh firmware collections improve. | N04 | Mixed; no useful recorded gain | [Consumer matrix](consumer-matrix.json), [measurement audit](live-numerics.json) |

## References

- [Full report](report.md): results, numerical drift and limits.
- [Research notebook](research.md): decisions and budgets.
- [Source manifest](source-manifest.json): measured revisions and compiled artifact.
- [Frozen input origins](../2026-10-13-headless-js-profiling/inputs/origins.json): schedules and provenance.
- [Earlier Wasm investigation](../2026-10-14-rust-engine/report.md): prior engine change; older timings were not reused.
