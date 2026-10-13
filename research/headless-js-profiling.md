# Headless JavaScript physics profiling, 2026-10-13

Headless profiling now uses the browser's existing `PerformanceMonitor` and the
production HP3/HP4 system pipeline. The
[profiling recipe](../tests/benchmark/HEADLESS_JS_PROFILING.md) covers repeatable
timings, native Node CPU profiles, sampled allocations, GC events and numerical
comparisons. No physics optimization was made in this investigation.

Eight cases cover HP3/HP4, layering on/off, 60-second firmware logo prefixes and
complete saved autocal movement/settling schedules. Each case has three plain
repeats and separate system, CPU and allocation runs: **48 measured trajectories
and 7,544,232 physics updates**. All profiler modes match their case's baseline
numerical checkpoints exactly. Measurements ran sequentially on an i7-12700K,
Node 24.14.1/V8 13.6, with a separate 5,000-step VM warmup and fresh measured worlds.

| Machine | Workload | Simulated seconds | Median physics seconds, layering on | Layering off |
| --- | --- | ---: | ---: | ---: |
| HP3 | Logo | 60.000 | 2.747 | 2.191 |
| HP3 | Autocal replay | 976.542 | 39.219 | 40.693 |
| HP4 | Logo | 60.000 | 16.223 | 12.280 |
| HP4 | Autocal replay | 160.830 | 43.541 | 43.765 |

The constraint solver is the largest system in seven cases. Attachment updates
are slightly larger for the unlayered HP3 logo. Representative layered profiles:

| System | HP3 logo | HP4 logo | HP3 autocal | HP4 autocal |
| --- | ---: | ---: | ---: | ---: |
| Constraint solver | 37.8% | 47.7% | 35.9% | 48.1% |
| Attachment update | 28.9% | 25.4% | 30.9% | 25.4% |
| Over-correction | 12.2% | 10.4% | 10.9% | 9.9% |
| Friction | 3.6% | 3.1% | 4.0% | 3.1% |
| Rigid-body sync | 2.4% | 4.2% | 2.6% | 4.2% |

CPU self-time hotspots include `applyConstraint`, ECS `getComponent`, vector
length calculations, `calculateAttachmentPoints`, and world-position/orientation
transforms. The latter also execute inside the solver and attachment systems;
the dedicated rigid-body sync row does not capture their complete cost.

Sampled allocation estimates range from **258 KiB to 1,682 KiB per update**.
Leading sites include world-position/orientation helpers, ECS queries, attachment
calculations, plane-basis construction and inertia transforms. These estimates
retain samples for objects collected during the run. They describe allocation
over time, not heap growth. Reported GC contributes approximately **0.8–1.7%** of
plain replay wall time; GC event durations do not represent all concurrent GC
CPU work. Allocation is substantial, but collector pauses are not the dominant
measured cost.

The layered HP3 autocal replay reproduces all 1,956 saved encoder reads within
1.14e-13 degrees. Its unlayered fixed-command replay diverges by up to 1,080
degrees from the original reads. It is a repeatable stress workload, not a
validated replacement collection. The historical HP4 trace lacks explicit
encoder-response records, so its checks establish repeat/profiler equality,
without claiming encoder parity against that older trace.

Layering removal improves logo throughput but changes little in these autocal
timings and can change physical behavior substantially. These are frozen
schedules; a fresh collector could choose different settling waits. Physics
timings exclude construction, observations, firmware, transport, rendering and
recording. Observations are reported separately, and CPU samples exclude startup
and warmup. Future optimization comparisons must preserve inputs and numerical
behavior before accepting speed changes.

The [full report](../output/research/sessions/2026-10-13-headless-js-profiling/report.md)
and [matrix](../output/research/sessions/2026-10-13-headless-js-profiling/matrix.json)
retain all 19 system timings, CPU/allocation summaries, GC/memory measurements,
checkpoint files and source/input hashes. Raw profiles, frozen workloads and the
sequential runner are in that session directory. Input origins identify the
actual RRF logo files and preceding HP3/HP4 autocal sessions.

Verification: 164 Jest tests across 28 suites and four native Node harness/profile
tests passed; `git diff --check` passed. The next step is one isolated optimization
followed by the same numerical comparisons and uninstrumented timing repeats.
