# Independently testable encoder calibration stages

Status: complete.

## Abstract

Encoder autocalibration was split into modeled-length transformation, fitting and assessment, frozen sweep planning, prediction evaluation and pure history ranking. Versioned JSON artifacts allow downstream experiments without rerunning fitting or collection. Eleven recorded simulated baselines reproduced current references and showed 20 adjacent score/parameter-error disagreements, with physically worse final selections on ten datasets. The final implementation preserves every returned and intermediate calibration exactly. A small metadata-copying optimization reduced mean fitting time from 3.290 to 2.992 s in seven alternating pairs; the complete regression matrix changed from 201.512 to 192.633 s (4.4% faster). Frozen planning, prediction evaluation and ranking take about 32.7 ms, 14.5 ms and 2.27 microseconds. All 337 Python tests pass. Accuracy is preserved rather than improved; evidence is simulated, runtime samples are bounded, and fitting must keep borrowed nested measurement metadata read-only.

## Topics

autocalibration, encoder, CDPR, modular pipeline, stage replay, runtime, metadata copying, selection regret

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Existing ranking can favor physically worse parameters | E01 | Supported | [Physical audit](E01/physical-audit.json), [baseline results](E01/results.json) |
| Filtering can repeatedly fail to restore constraints | E01 | Supported: fifth HP3 has six failed recovery attempts | [Recovery audit](E01/recovery-audit.json), [log](E01/fifth_hp3_dataset.log) |
| Stage extraction preserves calibration and permits downstream-only trials | E02a, E03, E04, E08 | Supported on all 11 fixtures; frozen fit replay exact | [Paired results](E02a/results.json), [stage checks](E03/benchmark.json), [final comparison](E08/comparison.json) |
| Information reuse removes redundant planning work without changing ranks | E03, E06, E07 | Supported; standalone extraction initially had inconclusive full-fit timing | [Alternating planner measurements](E07/benchmark.json) |
| Borrowing read-only metadata accelerates fitting without changing lengths or calibration | E07, E08 | Supported within measured battery | [All-dataset transformations](E07/transforms.json), [fit pairs](E07/benchmark.json), [full regression](E08-regression.log) |
| Raw-rank-only selection improves the small frozen case | E05 | Supported for one fixture; not accepted as a general policy | [Frozen-history pilot](E03/raw-rank-pilot.json) |

## References

- [Research notebook](research.md): objective, steering, budgets, failed driver trial and decisions.
- [Final regression command, runtime and status](E08-results.json), [full output](E08-regression.log): all eleven reference checks pass with zero deltas.
- [Source/input provenance](provenance.json), [source patch](final-source.patch), [saved source](source/): baseline revision, final source hashes, dataset/reference hashes and numerical environment.
- [Prior rejected selection and solver experiments](../autocal-ranking-bounded-updates-negative-results-b7deb1bc/report.md): why a single-fixture accuracy improvement is insufficient for changing selection.
