# Why the fifth HP3 reference trajectory differs today

Status: complete.

Retrospective summary of saved evidence, written 2026-10-08.

## Abstract

Isolated March and current source replays used the same present-day numerical environment. They produced identical intermediate fits, both differing from the historical trajectory by 390.788 mm in the diagnostic mean. March code still returned the saved 70.314 mm calibration; current history selection returned 70.787 mm. The large trajectory penalty is dominated by underconstrained attempts that are excluded from final selection. Subsequent tracked algorithm changes are therefore unnecessary to reproduce the trajectory difference. Numerical environment or execution differences remain a leading explanation, but the original runtime lacked enough provenance to establish the cause. Source rollback alone did not restore the historical trajectory.

## Topics

autocalibration, HP3, historical reference, numerical environment, sentinel fits, regression metrics

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Later source changes caused the historical trajectory penalty | March df699275 and current isolated replay | Rejected as a necessary cause | [Results](results.json) |
| The large trajectory penalty measures returned calibration quality | Reference/current metric decomposition | Rejected | [Report](report.md#what-the-regression-row-measures) |
| The exact original runtime difference is identified | Historical provenance inspection | Inconclusive; original environment missing | [Replay](report.md#controlled-replay) |

## References

- [Metric decomposition, source history and replay controls](report.md)
- [Controlled comparison measurements](results.json)
- [Preserved historical settings](reference-settings.json)
