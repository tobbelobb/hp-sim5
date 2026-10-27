# Cheap analytical HP3 parameter screens expose misleading encoder fit scores

Status: complete, separate review pending.

## Abstract

Can varied physical assumptions be screened without paying for many full HP3
collections? Eight deterministic analytical worlds vary geometry, prescribed cable
compliance, reversal offsets, encoder noise and movement span. Thirty-two current
core fits compare lbfgsb/trf and 20/10 stored points per sweep using data-derived
initialization and common unseen sweeps. The matrix takes 118.24 seconds; only six
fits satisfy successful-optimizer and complete-coverage checks. Lower unseen
residual does not reliably imply better anchors. In a clean case, TRF's unseen RMS
falls from 0.02362 to 0.02234 mm after thinning while anchor RMSE rises from 5.89
to 40.20 mm. No default solver or thinning change passes the declared robustness
gates. A retained benchmark separates truth from inputs and preserves failures,
hashes and budgets. Two short native Rust probes confirm encoder sensitivity to
cable stiffness, with peak paired A difference 0.561 mm. These analytical core
fits bypass full-loop recovery, planning, consensus and stopping. They establish
neither faster collection nor hardware accuracy.

## Topics

HP3, encoder-only autocalibration, parameter variation, analytical benchmark,
cable compliance, hysteresis surrogate, solver comparison, point thinning,
held-out prediction, geometric accuracy, native Rust, reproducibility

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Archived fit scores always order geometric accuracy | PV-E01 | Rejected in saved evidence | [Arithmetic audit](PV-E01-historical-audit.json) |
| A cheap analytical screen exposes varied-world core-fit failures | PV-E02 | Supported in eight named analytical conditions | [Results](PV-E02/results.json), [summary](PV-E02-summary.json) |
| TRF or half density meets declared robust replacement gates | PV-E02 | Rejected for these core-fit gates | [Comparisons](PV-E02-delivery-summary.json) |
| Lower common unseen residual always identifies better geometry | PV-E02 | Rejected | [17 pairwise disagreements](PV-E02-summary.json) |
| Native encoders respond to changed cable stiffness | PV-E03 | Supported for a four-second probe | [Paired native summary](PV-E03-summary.json) |
| The benchmark establishes fewer collection points or hardware gains | None | Not tested | [Limitations](report.md) |

## Decision handoff

- Scope: eight analytical HP3 conditions and two short native stiffness probes at fixed radii, zero buildup and no layering.
- Review: producer evidence and tests are available; separate review pending.
- Decision: retain the benchmark, preserve calibration defaults.
- Next: compare matched nominal/soft native collections through ordinary fitting and recovery under a separate budget.

## References

- [Full report](report.md): numerical methods, results and limitations.
- [Research notebook](research.md): budgets, decisions, failures and next experiment.
- [Measured generator](PV-E02/benchmark-measured.py): exact numerical source at measurement.
- [Final numerical-source equality check](PV-T05-measured-source-contract.json): measured/delivered numerical AST comparison.
