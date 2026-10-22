# Faster encoder autocalibration through exact spool-model dataset reuse

Status: complete.

## Abstract
This investigation asks whether encoder-only CDPR calibration can obtain reliable results with less collection or computation. Eleven fresh browser-simulation fixture replays reproduce 20 score/physical-error disagreements and nine underconstrained recoveries, six unsuccessful. Two comparable-validation rules remain unreliable. Uniform and geometry-aware half-point selection each worsen anchors on three of four recorded fixtures; 18 analytical synthetic fits show conditional, mixed benefits. Profiling instead finds thousands of repeated spool-model dataset conversions. A small implementation caches up to 16 exact radius/buildup transformations per filter pass, enabling existing residual-cache reuse. All eleven candidate regressions preserve parameters, stopping decisions and 5,184 numerical log fields exactly. Isolated three-repeat comparisons reduce median full-auto replay time by 2.5% on layered_manual_some_bad_points, 10.4% on seventh_hp3_dataset. The retained change reduces fitting work; collection point counts and physical ranking remain unchanged. Evidence is recorded simulation and analytical synthetic data, not hardware or new-data positioning validation.

## Topics
CDPR, Hangprinter, HP3, Slideprinter, encoder-only autocalibration, exact spool cache, numerical speed, informative points, fewer points, common held-out sweeps, robustness, automatic stopping

## Tested hypotheses

| Hypothesis | Experiments | Outcome | Evidence |
| --- | --- | --- | --- |
| Current scores agree with physical accuracy | E01 | Rejected | [Ranking audit](E01-E01-baseline-audit.json), [recovery audit](E01-fragility.json) |
| A common last sweep improves final ranking robustly | E02 (E00 archived control) | Mixed; rejected as default | [Shared comparison](E02-common-audit.json) |
| Every-shared-sweep dominance avoids physical regressions | E02 | Rejected as a general guard | [Paired comparisons](E02-common-audit-paired.json) |
| Geometry-aware thinning preserves full-data accuracy | E03 | Mixed; rejected as default | [Recorded comparisons](E03-recorded-comparison.json), [18 synthetic fits](E03-synthetic-results.json) |
| Exact bounded transformed-dataset reuse preserves behavior and speeds fitting | E04–E06 | Supported on tested fixtures | [Profiles](E04-profile), [11 regressions](E05-cache-regressions.json), [timing record](E06-isolated.txt) |
| Exact keys, eviction and pass isolation work | E07 | Supported tool checks | [Spool tests](E07-spool-tests.txt), [322 Python checks](E07-python-tests.txt) |
| Joint nuisance-aware acquisition, continuous acquisition and settling reuse improve collection | none | Not tested | [Proposals](report.md#remaining-experiments) |

## References

- [Full report](report.md): method, measurements and limitations.
- [Research notebook](research.md): decisions, constraints, steering and budgets.
- [Source and dataset manifest](manifest.json): revision, hashes and numerical environment.
- [Retained source diff](final-source.diff): exact bounded cache and tests.
- [Prior collection investigation](../2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/report.md): earlier thinning failures and guarded settling already in the baseline.
