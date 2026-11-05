# HP3 force and friction robustness program

Status: guide-friction investigation complete and independently reviewed.
No production autocal improvement was adopted.

## Abstract

The program refreshed the parameter-variation agenda against earlier research,
then tested encoder-only HP3 force handling under guide-friction and sensor-noise
variation. Nominal defaults, a bounded-preload prototype and a matched noise-only
control each stopped automatically with all anchors below5mm. Four active-guide
trials failed collection before fitting; both preload versions failed to recover
the development case. A constructed native probe transmitted spool load with
macroscopic local cable slack, limiting interpretation of the guide stress as a
real-machine robustness test. Separate reviewers checked source provenance,
original asset hashes and saved-data arithmetic. The report-only finding was
admitted after recovery from a root execution-attachment failure. The original
research files survived and are pushed with a verified raw-artifact archive.


## Topics

HP3, autocalibration, force tuning, guide friction, motor friction, sensor noise,
robustness, candidate tails, native model validation, recovery.

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Bounded preload recovers active-guide collection | dev1-active-default, dev1-bounded-preload, dev1-consistent-preload | Rejected in tested model | [Producer report](../attempt-939c073f49004eb6949910600200be51/report.md) |
| Nominal and matched noise-only defaults finish below5mm per anchor | nominal-numeric, eval0-noise-only-default | Supported within this scope | [Measurements](../attempt-939c073f49004eb6949910600200be51/evidence/measurements.json) |
| Encoder-only hardware robustness is established | None | Not tested | No hardware experiments |
