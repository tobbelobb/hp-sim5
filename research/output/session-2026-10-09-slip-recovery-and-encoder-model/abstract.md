# Recovering autocal slips and retaining fixed encoder errors

Status: complete.

## Abstract

This investigation implements bounded reduced-force recovery after a fixed encoder slips and automatic closed-loop/layering/recording startup. An isolated production JS/RRF fault injection caused 8.01-degree fixed B drift. The collector restored position at lower commands and completed both directions, retaining 20 valid points with B span 0.47 degrees and maximum fixed error 0.38 degrees. A closed-loop bootstrap with automatic travel measurement completed 60 points. A real recorder connection preserved its session and acknowledged initial physics before G-code events. Trial-005 fixed encoders span at most 0.01 degrees; its final rank-zero fit had no valid next candidate. Finite-residual comparisons show that current objectives ignore per-point fixed encoder variation: perturbing raw fixed observations by ±0.7 degrees, or reversing fixed-angle associations while preserving modeled medians, leaves residuals identical. The recommended extension uses measured per-point fixed lengths and propagated uncertainty in both NumPy/JAX pointwise objectives and held-out validation. That objective extension is proposed, not implemented. No hardware recovery or improved calibration accuracy is established.

## Topics

autocalibration, slip recovery, fixed encoders, closed loop, line layering, Rerun recording, pointwise objectives

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Reduced force can recover a slipped direction | E1 | Supported | [Verified recovered dataset](overload-guard-recovery-verified/sweeps.json), [log](overload-guard-recovery-verified/collector.log) |
| Settings update UI and wait for recorder startup | E2 | Supported | [Bootstrap dataset](patched-bootstrap-auto-size/sweeps.json), [recording result](recording-startup/result.json) |
| The objective discards pointwise fixed encoder variation | E3 | Supported | [Finite residual audit](encoder-objective-audit.json) |
| Trial-005 stopped without a valid next candidate | E4 | Supported | [Trial-005 log](../../extended-autocal/trial-005/sweeps.full_auto.log) |

## References

- [Notebook](research.md), [report](report.md).
- [Preceding slipping investigation](../2026-10-09-trial004-upper-sweeps/report.md): original high-force failure.
- [Source revision](revision.txt) and [working patch](working.diff): tested production code.
- [Trial-005 dataset](../../extended-autocal/trial-005/sweeps.json): closed-loop fixed measurements.
- [Optimization overview](../../../autocal/optimization_algorithm_overview), [objective overview](../../../autocal/objective_functions_overview): current and proposed error treatment.
