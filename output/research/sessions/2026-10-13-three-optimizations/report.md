# Cumulative validation of three headless physics optimizations

Status: complete.

## Abstract

This experiment checked the cumulative effect of three JavaScript simulation optimization commits on exact trajectory behavior and unprofiled physics timing. Frozen HP3/HP4 logo and autocal cases matched exact baseline checkpoints; cumulative median physics time fell 6.7–16.0%, with a 9.0% median across the eight workloads. Six other cataloged 3D machine designs—Four High Anchors, CubeCorners, and four Slideprinter variants—also matched the unoptimized `0cfe920c` checkout exactly through 200 production-pipeline steps with small commanded spool moves. The timing result is a single-host, fixed-prefix benchmark; no separate performance timings were collected for those six designs.

## Cumulative plain timings

Each comparison used the same workload/input and baked scene contract as the original baseline. Runs were sequential, unprofiled, and performed before tests ran. Physics time is the median of three fresh-world repeats; ranges are min–max across those repeats.

| Workload | Baseline median (s) | Cumulative median (s) | Cumulative reduction | Cumulative range (s) |
| --- | ---: | ---: | ---: | ---: |
| HP3 logo, layering on | 2.747 | 2.552 | 7.1% | 2.526–2.590 |
| HP3 logo, layering off | 2.191 | 2.040 | 6.9% | 2.018–2.083 |
| HP3 autocal replay, layering on | 39.219 | 36.005 | 8.2% | 35.943–36.012 |
| HP3 autocal replay, layering off | 40.693 | 37.976 | 6.7% | 37.513–37.977 |
| HP4 logo, layering on | 16.223 | 14.591 | 10.1% | 14.506–14.886 |
| HP4 logo, layering off | 12.280 | 10.804 | 12.0% | 10.738–10.893 |
| HP4 autocal replay, layering on | 43.541 | 39.264 | 9.8% | 38.900–39.618 |
| HP4 autocal replay, layering off | 43.765 | 36.763 | 16.0% | 36.234–38.874 |

The unweighted median of the eight per-workload reductions is 9.0%. Do not add the isolated percentages reported by the three optimization studies: those used separate runs and their effects may interact. This matrix measures the cumulative source state directly.

## Numerical behavior

All eight workloads passed `scripts/profile_headless.mjs --compare` against the original profiling baseline. Every one of the three repeats per workload matched the exact checkpoint hash, with matching inputs, scene bake, timestep, warmup and checkpoint interval. This verifies the current combined source over the tested fixed 30,000-step prefixes and the saved autocal command/settling traces.

The combined JavaScript test suite passed 764 tests across 160 suites (one skipped). The native differential, machine-pipeline and lifecycle parity suites passed 176 checks, with three deselected. The latter compare independent native/Python calculations at every simulated step for the covered scenarios.

## Other 3D machine designs

The app catalog includes Four High Anchors, CubeCorners and four Slideprinter scene variants in addition to HP3/HP4. For each of these six designs, the production system pipeline ran 200 steps at 2 ms with a small move command on its spool axes. Every per-step snapshot from the optimized source matched the unoptimized `0cfe920c` JavaScript oracle exactly. The commanded body moved in all six; final constraint forces were nonzero for 16/16, 8/16, 3/3, 12/12, 11/12 and 6/6 joints respectively, in the same order as the scene list. Raw result hashes and command targets are in [non-hp-scenes.json](non-hp-scenes.json).

An initial scene-fixture attempt omitted the production-pipeline flag, so it only exercised scene construction and was discarded. The results above are from the corrected run with production systems registered and actual spool commands.

## Limitations

Measurements were sequential on one Intel i7-12700K with Node 24.14.1/V8 13.6. They cover three repeats per 60-simulated-second prefix, not full adaptive autocalibration, firmware/transport costs or hardware behavior. The HP4 unlayered autocal repeat range is wider than the other cases; its median improvement is still positive in this sample, but the range is a reminder that one host/run set does not establish a universal speedup.

## Reproducibility and evidence

- [Research notebook](research.md) records the objective and constraints.
- [Raw cumulative matrix](cumulative-matrix.json) records all runs, source/input hashes, runtime environment and checkpoint hashes.
- [Per-case logs and checkpoints](cumulative/) retain command output and exact checkpoint files.
- [Sequential runner](run_matrix.py) reuses the original headless profiling inputs and compares each run to the baseline matrix.
- [Original baseline matrix](../2026-10-13-headless-js-profiling/matrix.json) and [profiling method](../../../../tests/benchmark/HEADLESS_JS_PROFILING.md).
- [Other-machine 200-step comparisons](non-hp-scenes.json) against the unoptimized `0cfe920c` source.
- [Individual attachment reuse results](../2026-10-13-attachment-reuse/report.md) validate the two changes in the third commit separately from the first two optimizations.
