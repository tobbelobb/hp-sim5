# Fixed-motor slipping corrupts upper autocal sweeps

Status: complete.

## Abstract

Trial-004 contained two upper sweeps with visibly irregular paths. Recorded commands and encoders, followed by isolated production JS physics and RRF collector experiments, identified loss of fixed B position: its encoder spanned 193.04 and 200 degrees. Simulator branch tensions exceeded B's modeled holding torque, while the collector accepted stationary encoders after slipping and repeatedly corrected B between points. A reproduced baseline matched the recorded first upper sweep to 0.01 degree. Lower default commands and a fixed-target encoder guard reduce B spans to 0.50 and 0.18 degrees in the two configurations; all six bootstrap directions also complete. A deliberate overload is rejected at 8.01 degrees before saving a dataset, with position-hold commands issued. No runtime force sensing is introduced: tensions are diagnostic simulator evidence only. Eighty-nine collector tests pass. These results establish collection behavior in the tested simulator geometry, not hardware safety or improved numerical calibration accuracy; a fresh visual capture and optimization remain untested.

## Topics

autocalibration, HP3, fixed anchors, motor slipping, encoder validation, force preload, RRF, Rerun replay, candidate ranking

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Upper sweeps lose fixed B position | E1 | Supported | [Recorded and reproduced encoder spans](sweep-metrics.json) |
| Applied cable load exceeds modeled B holding torque | E2 | Supported | [Command replay](replay-results.json) |
| Lower forces preserve fixed geometry | E3 | Supported | [Baseline and reduced-force datasets](reduced-force/sweeps.json) |
| Patched collector completes upper/bootstrap sweeps and rejects slipped fixed encoders | E4 | Supported | [Final metrics](sweep-metrics.json), [overload log](overload-guard-20x/collector.log) |
| Fits 4/5 are closer to known simulator geometry than fit 1 | E5 | Supported | [Fit comparison](fit-errors.json) |
| A low pullout command also finishes long bootstrap travel | E6 | Mixed | [Five-times-start failure](patched-bootstrap/collector.log), [ten-times-start failure](patched-bootstrap-10x/collector.log), [twenty-times-start success](patched-bootstrap-20x/sweeps.json) |

## References

- [Full report](report.md): cause, changes, measured behavior and limitations.
- [Research notebook](research.md): hypotheses, bounds and accepted steering.
- [Trial-004 dataset](../../extended-autocal/trial-004/sweeps.json) and [text log](../../extended-autocal/trial-004/sweeps.full_auto.log): original bad sweeps and numerical fits.
- [Original RRD](../../extended-autocal/trial-004/hangprinter-20261009T101909.456318Z-5eb541ea.rrd): recorded browser commands, encoders and scene settings.
- [Source revision](revision.txt) and [working changes](working.diff): production baseline and tested patch.
- [Preceding trial-003 investigation](../2026-10-09-trial003-size-measurement/report.md): active travel timeout handling.
