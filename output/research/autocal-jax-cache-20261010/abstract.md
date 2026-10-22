# Autocal persistent-cache performance and quiet native logging

Status: complete.

## Abstract

Five complete 11-dataset calibration-log regression batteries compared a fresh persistent cache, cache disabled, and warm persistent caching using JAX/jaxlib 0.9.2. Cache-off runs took 97.82 and 96.48 seconds; warm-cache runs took 53.68 and 55.83 seconds. Mean completion time fell from 97.15 to 54.75 seconds, saving 42.40 seconds (43.64%, 1.77x speedup). All 55 calibration children exited zero, all reference comparisons passed, and parsed final and per-iteration numerical outputs were exactly identical. A warm unsuppressed battery emitted 292 PJRT compatibility warnings; the warm battery with final defaults emitted zero. Twelve JAX/cache tests passed. Autocal now restores persistent caching by default and sets overridable TF_CPP_MIN_LOG_LEVEL=2, retaining native errors, Python warnings and ordinary stderr. The warning is an upstream unimplemented compatibility-check notice; JAX cache keys include library/backend versions. Timing includes ordinary background load and one off-run overlapped small tests; these measurements demonstrate benefit on this fixture suite, not universal performance or hardware accuracy.

## Topics

Autocalibration, JAX, PJRT, persistent compilation cache, regression replay, CPU timing, native logging, numerical reproducibility.

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Persistent caching reduces end-to-end regression time | C1–C5 | Supported: warm-cache mean 43.64% lower | [Analysis](analysis.json) |
| Caching changes fitted results or causes failures on this suite | C1–C5 | Rejected for 55 tested children; no differences or failures | [Numerical comparisons](analysis.json), per-child logs in C1–C5 |
| Native log level 2 silences the compatibility warning without changing JIT output | D2, C5 | Supported: zero warnings, identical outputs | [Reproduction](warning-repro.json), [final battery](C5/summary.json) |
| Explicit cache/logging overrides and Python diagnostics remain functional | D3 | Supported by four fresh-process settings cases | [Checks](checks.md) |

## References

- [Full report](report.md): timing, validation and implementation details.
- [Notebook](research.md): objective, accepted steering and experiment choices.
- [Previous investigation](../autocal-comfortable-force-20261010/report.md): initial warning reproduction and provisional opt-in decision.
- [Provenance](provenance.json): interpreter, dependency versions, revision, workers and CPU affinity.
- [Benchmark wrapper](benchmark.py): observational instrumentation around the unchanged regression main entry point.
- [Analysis script](analyze.py): exact numerical comparison and timing arithmetic.
- [Implementation diff](changes.diff): cache/logging defaults, docs and test changes.
- [JAX cache warning issue #36294](https://github.com/jax-ml/jax/issues/36294): upstream cache-hit warning report.
- [JAX 0.9.2 cache-key source](https://github.com/jax-ml/jax/blob/jax-v0.9.2/jax/_src/cache_key.py): library/backend versions are hashed into cache keys.
- [XLA pinned-source compatibility notice](https://github.com/openxla/xla/blob/187a5eb58277a85847d1516bd1e20b7faf03d5ef/xla/python/pjrt_ifrt/pjrt_executable.cc): an unimplemented version check emits the warning rather than detecting an actual mismatch.
