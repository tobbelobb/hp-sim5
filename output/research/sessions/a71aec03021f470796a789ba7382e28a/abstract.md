# Autocal history-ranking baseline, unfinished record

Status: incomplete record; final research outcome not documented.

Retrospective summary of saved evidence, written 2026-10-08.

## Abstract

The notebook records a baseline audit of historical Slideprinter histories. Repeated ten_points_bigger_deltas runs selected physically worse candidates with 10.96–14.90 mm parameter regret. It proposes whole-sweep prediction and bounded residual updates, and records a native HP4 collection job plus a running regression baseline. The last saved decision still awaits those results. There is no final report or documented completion of the proposed selector experiments. Although exit.json records a successful process exit, that does not resolve the unfinished research record. Treat this as useful baseline evidence and a record of intended experiments, rather than a completed assessment of a new algorithm.

## Topics

autocalibration, Slideprinter, history ranking, ground-truth regret, proposed held-out validation, HP4 collection

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| History ranking can prefer physically worse fits | Historical baseline audit in notebook | Supported in saved audit | [Notebook](research.md) |
| Held-out prediction improves selection under noise and winding | None completed in saved record | Not tested to completion in this record | [Proposed hypotheses](research.md) |
| Bounded residual updates improve robustness | None completed in saved record | Not tested to completion in this record | [Proposed hypotheses](research.md) |
| Native collection supports follow-up validation | d2e98d4fa0f7491cbaaede24521eecfa | Started; final outcome not summarized | [Notebook](research.md) |

## References

- [Saved hypotheses, baseline measurements and outstanding decisions](research.md)
