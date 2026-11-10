# Autocal history ranking with held-out sweeps

Status: complete.

Retrospective summary of saved evidence, written 2026-10-08.

## Abstract

This investigation reproduced disagreement between calibration ranking and known physical geometry, then tested synthetic sweep selection and whole-sweep prediction. On seventh HP3, the selected fit had 100.737 mm more summed anchor error than another candidate. A synthetic held-out scorer separated exact and perturbed geometry by 40.55x in robust prediction cost, but poor starts still produced physically bad fits. The archived implementation added future-sweep history selection with the existing rank as fallback. Its static regression replays did not measure an accuracy gain from that gate. Native HP4 collection was cancelled before a complete sweep after tracking losses. The report supports ranking diagnostics and a synthetic discriminator; it does not establish reliable calibration accuracy.

## Topics

autocalibration, HP4, HP3, Slideprinter, candidate ranking, held-out sweeps, D-optimal selection, outliers, poor starts

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| History ranking can select worse known anchors | E1 | Supported on recorded fixtures | [Measurements](report.md#measurements) |
| Held-out prediction distinguishes exact and perturbed geometry | E2b | Supported in one synthetic test; integrated accuracy unmeasured | [Notebook](research.md) |
| D-opt ranking remains reliable from poor starts | E2a | Not supported; all poor-start fits physically bad | [Measurements](report.md#measurements) |
| Production HP4 collection yields a usable sweep | E3 / 4b4d1aeb4eed4f3fafe9c45393c02f52 | Inconclusive; cancelled before sweep completion | [Notebook](research.md) |

## References

- [Archived implementation, measurements and limits](report.md)
- [Hypotheses, E0–E3 decisions and retained collection path](research.md)
