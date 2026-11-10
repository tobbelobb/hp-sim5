# JAX cache reuse on the recorded HP3 extended reference

Status: complete.

## Abstract

Does the autocal JAX cache help the recorded HP3 no-buildup reference, and do
executables warmed by the regression battery transfer? Fifteen offline replays
used the regression runner's unchanged launch and comparison helpers with fixed
radii supplied for anchor-only log parsing. Two-run mean times were 24.19 s
with caching disabled, 23.35 s with the regression-warmed cache at its original
path, and 16.79 s after warming on the reference dataset. Dataset warming saved
7.39 s (30.57%). Compiler diagnostics showed 4/16 objective variants reused
from the battery and 16/16 from the dataset-warmed cache. All numerical results
matched, including full-precision fit checkpoints and the original score 8161.
The cache is a general compilation optimization whose variants depend on array
shapes and static solver settings. A methodology correction was necessary:
JAX 0.9.2 also hashes an auxiliary cache path, so relocated cache copies were
excluded from transfer conclusions. Timings measure replay computation on one
CPU, with two repeats per main arm; they exclude collection and recording time.

## Valid timing and reuse measurements

| Cache state | Experiments | Seconds | Mean seconds | Objective variants already cached |
| --- | --- | --- | --- | --- |
| Disabled | D1, D5 | 24.2483, 24.1296 | 24.1889 | None |
| Empty, enabled | D3 | 24.9601 | 24.9601 | None |
| Warmed only by eleven-dataset regression battery, same path | D12, D13 | 23.2897, 23.4021 | 23.3459 | 4/16, confirmed by D14 |
| Warmed on this reference dataset, same path | D4, D7 | 16.8623, 16.7258 | 16.7940 | 16/16, confirmed by D15 |

Dataset warming saved 7.3949 s (30.57%), or a 1.4403x speedup. Battery warming
saved 0.8431 s (3.49%) in these samples. This smaller difference is less robust
to timing noise; the four reused objective executables establish transfer
independently of wall time. Both diagnostic runs also reused a scalar/array
conversion executable. D14 compiled twelve new objective variants; D15 had
zero persistent compilation misses. Diagnostic timings are excluded from means.

## Method and numerical checks

Revision: `9422620c`, JAX/jaxlib 0.9.2, NumPy 2.4.3, SciPy 1.17.1, repository
local `.venv/bin/python`, CPU backend. Each replay launched in a fresh process
sequentially. Thread variables OMP/OPENBLAS/MKL/NUMEXPR were fixed to one.
This is recorded simulator evidence, not hardware validation.

The [input](../../extended-autocal/no-buildup-reference-001/sweeps.json) contains
seven sweeps of twenty points. Each isolated copy uses automatic full-auto
replay: start with three saved sweeps, then append the remaining four. The
runner uses `--sim --machine-type hangprinter_4 --find-radii off --base-radii 30
--buildup-factor 0 --no-collect`; the HP3 alias canonicalizes to this solver.
Radii and buildup match the recorded configuration. No collection, force
retuning, machine commands, or Rerun recording were performed. Production
autocal and regression runner code were unchanged.

The runner parser requires radii in the summary and per-iteration log. Autocal
omits them when they are fixed. The first parser attempt therefore rejected
even a self-comparison. [checks.py](checks.py) supplies only the known fixed
30 mm radii, then uses the unchanged parser/reporting functions. Original logs
remain unmodified and [preliminary results](preliminary-results.json) preserve
the failed parsing attempt. Comparisons use a 1e-6 mm total-parameter tolerance
and strict score mismatch handling; results are exact, not merely within tolerance.

All fifteen replays exit zero and pass comparisons against D1 for the final
summary and five iterations. Full-precision run costs, quality/ranking scores,
covariance metrics and uncertainties match. Run settings differ from the
original recording only in explicit CLI radii/buildup versus loading the same
values from the dataset; original numerical metrics match exactly.

D10 is the full-precision cache-off checkpoint baseline. D15 is the validated
same-path warm-cache checkpoint run. All seven fit artifacts, including both
iteration-three recovery fits, match exactly in anchors, calibration values,
costs, information matrices, covariance, confidence intervals, warnings, spool
models and fit structure after removing calibration timestamps. These fields
also match the original recorded artifacts. See [analysis.json](analysis.json).

Final score remains **8161**, with exactly the original M669 command:

```
M669 A0.00:-2443.33:-279.56 B708.79:401.97:-279.56 C-2294.40:1381.21:-279.56 D235.94:124.93:1075.91
```

Caching neither causes nor repairs this poor fit. The previous force-tuning
fix affects new collection; replay retains the old recorded measurements.

## Cache transfer and the directory correction

D2 and D6 initially copied the regression-warmed cache into new trial
directories. D8/D9 added detailed compiler logging and checkpoints, then D11
copied a newly populated diagnostic cache to another directory. They still
compiled sixteen variants. Those results initially looked like no transfer,
but the experiment changed a cache-key input along with cache contents.

Installed JAX `compiler.py` lines 258–278 set
`xla_gpu_per_fusion_autotune_cache_dir` to a directory beneath the persistent
cache, even when compiling for CPU. This is enabled by the default
`jax_persistent_cache_enable_xla_caches` value. `cache_key.py` hashes serialized
compile options without clearing that path. Consequently, changing the cache
directory changes keys. The issue was cache relocation, not evidence that the
cache encodes datasets or calibration answers. These relocated trials are
retained as controls and excluded from transfer conclusions.

[transfer.py](transfer.py) corrects this by moving the original regression cache
aside, putting a copy at its original path, running one trial, archiving the
mutated copy, and restoring the original in a `finally` block. It repeats with a
fresh copy for D12–D14. SHA-256 manifests verify the original cache contents are
restored after every trial. D15 uses the original D3/D4/D7 target cache path.
Detailed logging now confirms four battery-to-reference objective hits and
sixteen reference-to-reference hits. No production cache option was changed.

## Why the optimization generalizes

In [ellipse_objective_jax.py](../../../autocal/ellipse_objective_jax.py), packed
drive/sensor lengths, deltas, masks, uncertainty arrays, bounds and current
anchor estimates are dynamic JAX inputs. Dataset names and paths are not
objective parameters. The cached object is an executable that evaluates the
objective and its gradient on current inputs; it is not a solved calibration.

Compiled variants specialize on array shapes and dtypes, plus static values
such as dimensions, anchor constraint mode, filtering schedule, metric and
loss settings. Packed shapes depend on sweep count and maximum valid points
per sweep. Some static scales are also derived from the data (`l2_scale`,
`floor_norm`), so equal point counts alone do not guarantee reuse. JAX's keys
add compiler options, library/backend versions and device configuration.

Thus it is a general optimization across datasets using this JAX objective.
A new dataset can reuse compatible variants, compile missing ones, and cache
those for later launches. This reference demonstrates both partial reuse from
different datasets and full reuse after its own first run. It does not prove a
fixed speedup for every dataset or for solver modes that do not use JAX.
Keep the cache at a stable path with current JAX defaults.

## Tested hypotheses

| Hypothesis | Experiments | Outcome |
| --- | --- | --- |
| Caching preserves calibration outputs. | D1–D15 | Supported for this dataset; exact numerical comparisons and full-precision checkpoints. |
| Battery warming transfers all required variants. | D12–D14 | Mixed: partial, 4 of 16. |
| Dataset warming speeds replay. | D1/D5 vs D4/D7, D15 | Supported; 30.57% measured saving. |
| Cache files can be relocated without changing usable keys. | D2/D6/D8/D9/D11 vs D12–D15 | Rejected with current auxiliary-cache defaults. |

## Limits and evidence

Two sequential repeats per main arm on one CPU estimate computation time for
one recorded dataset, not live simulation/collection/recording time. Different
data, filtering, solver options, JAX versions or devices may need new variants.
The smaller 3.49% battery transfer timing difference has limited precision.
Zero process errors alone do not establish accuracy; explicit numerical
comparisons establish preservation, while the score still diagnoses a bad fit.

- [Analysis](analysis.json), [results](results.json), [D14](D14/summary.json),
  [D15](D15/summary.json): measured times, exact comparisons and valid compiler counts.
- [Original comparison](original-comparison.txt): original recording versus current replay.
- [Provenance](provenance.json), [integrity](integrity.json): input hashes, code revision
  and original regression cache preservation.
- [benchmark.py](benchmark.py), [transfer.py](transfer.py),
  [warm_precision.py](warm_precision.py), [diagnostics.py](diagnostics.py),
  [checks.py](checks.py), [analyze.py](analyze.py): reproducible experiment and analysis scripts.
- [Previous battery report](../autocal-jax-cache-20261010/report.md): cache was warmed
  only by the eleven regression fixtures before this investigation.
- [JAX persistent compilation cache documentation](https://docs.jax.dev/en/latest/persistent_compilation_cache.html):
  compiled program reuse, key inputs and diagnostic settings.
- [JAX 0.9.2 compiler source](https://github.com/jax-ml/jax/blob/jax-v0.9.2/jax/_src/compiler.py):
  auxiliary cache directories in compile options, confirmed against installed source.
- [JAX 0.9.2 cache-key source](https://github.com/jax-ml/jax/blob/jax-v0.9.2/jax/_src/cache_key.py):
  compile-option hashing and debug metadata exclusion, explaining relocation misses.
