# Covariance metrics for encoder based CDPR autocalibration

Status: complete; no production calibration algorithm change was justified.

## Abstract

This study asks whether covariance metrics from Caverly et al.'s online CDPR self-calibration paper can improve hp-sim5's encoder-based autocalibration ranking. The full recorded regression matrix was replayed after fixing the regression tool's module invocation: all 11 outputs match their stored references exactly, while reproducing 20 adjacent score/physical-error disagreements across nine fixtures. Selected-history anchor-sum regret ranges from 0 to 163.828 mm. A heuristic that weighted the existing ellipse-feature information by point count and inverse encoder variance selected the physically best available history in only 2/11 fixtures and sometimes increased anchor RMSE substantially. Candidate planning itself took 0.039–0.819 s per fit in a 33/90-candidate audit, so it does not explain the tens-of-seconds replay runtime. The paper's PDOP/ODOP depend on joint pose/shared-bias covariance and cannot be transplanted as anchor covariance. No calibration selector change was retained; only the regression launcher was repaired. Evidence is simulated recorded data, not hardware.

## Topics

CDPR, Hangprinter, encoder autocalibration, covariance, PDOP, ODOP, D-optimal design, ranking regret, runtime

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Current score/physical disagreements reproduce on current source | E01 | Supported: 20 disagreements, 9/11 fixtures | [Full report](report.md), replay output under ignored `.regress_parallel_runs` |
| Simple point-count/noise weighted covariance improves history ranking | E03 | Rejected: physically best candidate in 2/11; several large regressions | [Covariance audit](E03-weighted-covariance-audit.json) |
| Candidate ranking cost explains slow full-auto replay | E04 | Rejected for tested saved fits: 0.039–0.819 s per fit | [Planning cost audit](E04-plan-cost-audit.json) |
| Paper's pose covariance is equivalent to existing anchor information | E02 | Rejected: different estimator and estimated quantity | [Paper PDF](../../../../autocal/papers/Covariance_based_data_quality_assessment_metrics.pdf), [analysis](report.md) |

## References

- [Full report](report.md)
- [Research notebook](research.md)
- Caverly, Cheah, Bunker, Patel, Sexton & Nguyen (2024; published online 2024, issue 2025), “Online Self-Calibration of Cable-Driven Parallel Robots Using Covariance-Based Data Quality Assessment Metrics,” DOI [10.1115/1.4065236](https://doi.org/10.1115/1.4065236); [local PDF](../../../../autocal/papers/Covariance_based_data_quality_assessment_metrics.pdf). Supports the paper method distinction and its covariance-based PDOP/ODOP claims.
- [Current regression runner](../../../../autocal/tools/regress_calibration_logs.py): fixture ground truth and score/accuracy audit.
- [Earlier ranking and cache study](../2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/report.md): controlled 11-fixture baseline, recovery fragility and 20 adjacent disagreements.
- [Earlier ranking negative results](../autocal-ranking-bounded-updates-negative-results-b7deb1bc/report.md): held-out prediction, age score, bounded updates and active movement failures.
