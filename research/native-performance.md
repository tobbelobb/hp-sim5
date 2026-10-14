# Native Rust headless performance, 2026-10-14

Native Rust commit ab833759 was benchmarked against the exact preceding Rust/Wasm headless default at 7905e64f, and against the former Python default for fresh research trials. Three sequential repetitions covered HP3/HP4, layering on/off, frozen logo and autocal schedules, continuing worlds, every-step fresh telemetry, real six-point firmware collections, and the standalone autocal service with and without recording. Native shared-pipeline throughput improved 1.145–1.235×, with an aggregate ratio of 1.181×. Recording reduced end-to-end gains; short real firmware collections showed only 1.02–1.03× without recording and approximately 0.99× with recording. Fresh numerical trials improved 3.11–7.17× over Python; recorded trials improved 1.46–2.12×. Repeats were deterministic, but five long physics comparisons exceeded the declared pointwise tolerance. These host measurements do not establish calibration accuracy or complete autocal optimization speedup.

The comparison uses exact commits `7905e64f` (previous defaults) and `ab833759`
(native), the same host/dependencies, unchanged 2 ms steps and three sequential
repeats per case. The full report records timings, settings, source/artifact
hashes, preflighted collections, numerical differences and limits.

| Workflow | HP3 throughput ratio | HP4 throughput ratio |
| --- | ---: | ---: |
| Physics-only logo, layering on / off | 1.21× / 1.20× | 1.15× / 1.16× |
| Physics-only autocal replay, layering on / off | 1.23× / 1.19× | 1.15× / 1.14× |
| Continuing 1,000 steps, recording off / on | 1.27× / 1.12× | 1.16× / 1.07× |
| Standalone autocal service, recording off / on | 1.18× / 1.06× | 1.20× / 1.02× |
| Fresh 200-step trial, recording off / on | 3.11× / 1.46× | 7.17× / 2.12× |
| Actual six-point RRF collection, recording off / on | 1.02× / 0.99× | 1.03× / 0.99× |

Fresh trials compare against their former **Python** default. Other rows compare
against the fuller **Wasm** engine. The matched 40-second HP4 frozen collection
replay is 1.09× faster without Rerun and 1.05× with it; its full numerical trace
is 1.08× faster. All timers and repeat ranges are in the report.

Native saves 13–19% of physics time on the eight primary workloads. That does
not imply the same whole-job saving: recorded HP4 six-point collection physics
fell from 3.38 to 2.94 seconds, but whole-job medians were 17.97 versus 18.12
seconds. Recording and collector/firmware work dominate that case. Three repeats
and desktop activity do not support treating roughly 1% differences as a useful
gain or a proven regression. Complete fitting/automatic-acceptance autocal runs
were not timed.

Five long physics cases exceed the declared `atol=rtol=1e-6` tolerance; HP3
native/Wasm encoder differences reach about 3.1°. Current Wasm matches all exact
baseline checkpoint hashes, and every backend’s repeated trajectories are
stable. The short controlled/fresh comparisons pass; all 24 real collections
complete with identical selected measurements and step counts. No calibration
accuracy or bitwise cross-target trajectory claim follows.

- [Full report](../output/research/sessions/2026-10-14-native-throughput/report.md)
- [Physics matrix](../output/research/sessions/2026-10-14-native-throughput/physics/matrix.json)
- [Consumer matrix](../output/research/sessions/2026-10-14-native-throughput/consumer-matrix.json)
- [Autocal service matrix](../output/research/sessions/2026-10-14-native-throughput/autocal-service-matrix.json)
- [Source manifest](../output/research/sessions/2026-10-14-native-throughput/source-manifest.json)
- [Numerical audit](../output/research/sessions/2026-10-14-native-throughput/consumer-numerics.json)
- [Previous Wasm study](rust-wasm-engine.md)
