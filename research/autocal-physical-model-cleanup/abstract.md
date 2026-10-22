# Autocal ellipse flex removal and zero-tension validation

Status: complete, 2026-10-22.

## Abstract

Lane 05 found that ellipse flex correction never applied on the reference corpus
because the collector writes no per-point tensions. This integration removes its
two CLI switches and ellipse fitting/validation plumbing, preserving independent
legacy point and plotting consumers. Failed zero-tension extrapolation now stops
collection, and explicit projected fits require projected angle fields. Fit-free
before/after comparisons match exactly in 72 checks across twelve fixtures. Two
full recorded-data replays preserve reference results and collection counts.
Python and collector checks pass. A small saved-geometry probe finds no valid
sweeps and a zero numerical gradient in two cost-100 production failures, even
with sweep rejection disabled. That narrows the next solver investigation without
changing solver or filtering policy. Calibration evidence is recorded simulation;
no new movement, hardware measurement or accuracy/speed gain was established.

## Topics

autocalibration, physical models, flex removal, zero-tension projection, CLI cleanup,
reference regression, invalid geometry, underconstrained sentinel, solver stagnation

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Removing inert ellipse flex preserves current objective and replays | C05-01, C05-02 | Supported on tested data | [Report](report.md) |
| Invalid projection can be rejected before recording/fitting | C05-03, C05-04 | Supported by code checks | [Report](report.md) |
| Turning off sweep rejection makes saved sentinel geometries valid | C05-05 | Rejected for two checked production failures | [Report](report.md) |
| Different solver starts cure the original failure trajectory | None | Not tested | [Next investigation](report.md#solver-follow-up) |

## References

- [Report](report.md): changes, metrics, source links and raw evidence.
- [Notebook](research.md): scope, decisions and failure preservation.
- [Verification script](verify.py): fit-free before/after and saved-geometry checks.
