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

## Complete regression batteries

| ID | Cache mode | Elapsed seconds | PJRT warnings | Reference results |
| --- | --- | ---: | ---: | --- |
| C1 | cold-on | 83.071 | 120 | 11/11 |
| C2 | off | 97.823 | 0 | 11/11 |
| C3 | warm-on | 53.676 | 292 | 11/11 |
| C4 | off | 96.478 | 0 | 11/11 |
| C5 | warm-on | 55.829 | 0 | 11/11 |

Every calibration child returned zero. All 11 summaries exactly matched their references in every battery. The analysis separately compares final anchors, radii and fit score, and each iteration's anchors, radii, fit score, rank score and history rank score across arms; every value and iteration count was identical. These are parsed numerical outputs, not byte equality of timestamped logs or a proof that every possible cache use is safe.

C1 used a new dedicated cache. Its 120 warnings were from cache hits within the parallel battery, including executables shared between fixture runs. C3 and C5 reused that cache. Mean warm-cache runtime excludes C1's population cost. All five runs used the production runner's normal 11-worker concurrency and full fitting settings; no datasets were trimmed, no optimization tolerances changed, and dependencies were not upgraded. The runner SHA256 remained ed3495226c101c221ea6e1e2d1f1705ac54adf63111526e8bcd43a6597f990a4 throughout. Fixture hashes, individual child timings, complete stdout/stderr, scratch paths and parsed outputs are retained in each experiment directory. The benchmark imports and calls the unchanged regression main entry point, wrapping run_autocal only to observe its results.

The machine exposed 20 logical CPUs. Background load was not controlled. The small cache/JAX test suite overlapped C4; that off-run nevertheless completed within 1.35 seconds of C2. Thus the large measured saving is clear, while the exact percentage should be treated as a fixture-suite measurement rather than a precise universal speedup.

## Why the warning is harmless in these runs

The inspected XLA source logs the compatibility message when its version-check callback reports Unimplemented. It is not evidence of a detected version mismatch. JAX's persistent-cache key independently hashes the jaxlib version, backend platform/version, computation, compiler options, flags and device information. The measured equality across cached and uncached fits supports keeping this cache on for this workload.

D2 reused the same tiny JIT executable with native log levels 1, 2, and 1. Warm level-1 runs printed three compatibility warnings; level 2 printed none. All returned 100.0 and exited zero. This distinguishes logging from numerical execution.

## Final change and diagnostic scope

Restored JAX_COMPILATION_CACHE_DIR's default to output/autocal-jax-cache and JAX_PERSISTENT_CACHE_MIN_COMPILE_TIME_SECS to 0 via setdefault. JAX_ENABLE_COMPILATION_CACHE=false still disables disk caching; custom directories and thresholds are respected. In-process JIT caching remains active.

Added TF_CPP_MIN_LOG_LEVEL=2 via setdefault before JAX initialization. It suppresses native C++ INFO and WARNING messages, including this notice, while native ERROR/FATAL messages remain enabled. This is a native severity setting, not an exact-message filter; other native warnings are also quiet by default. Python warnings and unrelated stderr remain visible and are tested. TF_CPP_MIN_LOG_LEVEL=1 restores native warnings; 0 also restores INFO. Explicit user settings take precedence. No global stderr interception, private cache monkeypatch, executable-version-check bypass or package upgrade was introduced.

C5 exercised these final production defaults across all eleven fixtures: full pass, identical results and zero PJRT warnings. Twelve objective/cache tests passed, and git diff --check passed. Source edits are uncommitted changes based on 0c922efd007f3491453c706e2cc801af0c49aa98.

## Scope

No simulator movement, fresh sensor collection or hardware experiment was performed. These tests validate replay performance and numerical consistency on the eleven reference fixtures. The earlier max-force changes remain untouched. Keeping cache off merely to avoid this warning costs about 42 seconds per measured battery, so the provisional opt-in cache decision was reversed.
