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

## Topics

autocalibration, HP3, Hangprinter, JAX, CPU, persistent compilation cache,
cache transfer, shapes, static arguments, replay, numerical reproducibility

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Cache modes preserve calibration outputs. | D1–D15 | Supported for this dataset. | [Analysis](analysis.json), [original comparison](original-comparison.txt), [D15](D15/summary.json). |
| The regression-warmed cache transfers fully to the reference. | D12–D14 | Mixed: partial reuse, 4 of 16 objective variants. | [D14 compiler counts](D14/summary.json). |
| Warming on this dataset reduces replay time. | D1/D5 vs D4/D7, D15 | Supported: 30.57% saved, all variants reused. | [Analysis](analysis.json), [D15](D15/summary.json). |
| Copying cache files to another directory preserves usable keys. | D2/D6/D8/D9/D11 vs D12–D15 | Rejected under current JAX defaults. | [Report](report.md), [relocated D11](D11/summary.json). |

## References

- [Full report](report.md): method, correction, measurements and limitations.
- [Research notebook](research.md): decisions and accepted steering.
- [Input dataset](../../extended-autocal/no-buildup-reference-001/sweeps.json): simulated recorded measurements.
- [Provenance](provenance.json): source SHA-256, revision 9422620c, original battery cache manifest.
- [Preceding cache battery investigation](../autocal-jax-cache-20261010/report.md): eleven-dataset timings and cache provenance.
- [JAX persistent cache documentation](https://docs.jax.dev/en/latest/persistent_compilation_cache.html): compiled-program reuse and key inputs.
