# Future-sweep selection versus refreshed autocal references

Status: complete.

Retrospective summary of saved evidence, written 2026-10-08.

## Abstract

Eleven new main-branch reference logs were generated with source and environment provenance, then reproduced exactly in a bounded two-worker regression run. Against that baseline, research_findings_1 changed final history selection while preserving every intermediate fit. Summed returned parameter error fell from 885.158 to 816.936 mm, a 7.71% reduction. Three datasets improved, five worsened and three were unchanged. This is a net gain on this battery with explicit regressions, not a uniform accuracy improvement. Candidates were evaluated on different future-sweep sets. Earlier full-concurrency runs suffered native crashes; the successful bounded runs do not establish a root-cause fix or guaranteed crash prevention.

## Topics

autocalibration, HP3, Slideprinter, whole-sweep prediction, history selection, regression references, concurrency

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| The refreshed main references reproduce exactly | Baseline verification | Supported across 11 datasets | [Baseline log](baseline-verification-bounded.log) |
| Future-sweep history selection improves all datasets | research_findings_1 comparison | Mixed; 3 improve, 5 worsen, 3 unchanged | [Comparison](comparison.json) |
| Two-worker scheduling fixes the native crash cause | Bounded baseline and branch runs | Inconclusive; successful runs only | [Provenance](provenance.json) |

## References

- [Per-dataset errors, selections and validation-set limitations](report.md)
- [Machine-readable branch comparison](comparison.json)
- [Source, runtime and scheduling controls](provenance.json)
