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

## Measured results

| Completed run | A/B/C/D error mm | Points | Simulated seconds |
| --- | --- | ---: | ---: |
| Nominal default | 3.841/3.599/3.464/1.861 | 140 | 4183.208 |
| Nominal bounded preload | 3.873/3.719/3.366/1.980 | 140 | 4152.904 |
| Matched noise-only default | 3.709/4.349/3.987/1.478 | 140 | 4278.630 |

The active-guide development default/v1/v2 and held-out default collected
0/20/20/40points before failure, with no applied geometry. Large intermediate
cold-fit errors were rejected; no completed run applied an anchor above20mm.

## Decision

Keep current production defaults. Do not promote the bounded-preload prototype
as a robustness improvement. Validate the guide transport/spool-load model before
using that stress to infer hardware failure. The next useful implementation
experiment must target a verified, separate modeled variation and demonstrate
a paired improvement in failures or applied geometry, with relevant checks.

## References

- [Producer report](../attempt-939c073f49004eb6949910600200be51/report.md): exact design, failures, sources and limits.
- [Durable original archive](../../../../research/programs/program-0a82df7fd4b34b8181716630492a1af4/recovery-originals/README.md):407 original files with verified restoration.
- [Program archive](../../../../research/programs/program-0a82df7fd4b34b8181716630492a1af4/report.md): scoped memory, fixed submission and review.
- [Research notebook](research.md): interview, decisions and recovery chronology.
