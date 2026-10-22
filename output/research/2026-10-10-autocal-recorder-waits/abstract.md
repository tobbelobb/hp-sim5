# Autocal return waits fixed by lossless recorder batching

Status: implementation complete; recorder replay and regression tests passed.

## Abstract

The recorded reference run showed a 16,162-command return burst delaying event logging by 21.88 seconds and blocking physics after about 0.64 simulation seconds. The implementation batches browser/headless event transport and writes complete logical event rows as RRD columns, preserving payloads, original timestamps, receipt ordering and clock fields. Bounded queues, counted post-write acknowledgements, partial-batch flushes and failure handling retain capture integrity. An isolated localhost replay of the exact recorded envelopes acknowledged the following physics sample in 0.175 seconds with every envelope preserved. Direct writer replay improved from 1.670 to 0.109 seconds (15.4x). Recorder, headless lifecycle and calibration guard tests passed. These measurements used no Viewer and synthetic physics samples; they do not establish full browser collection speed or calibration accuracy. Observed-clock waits and force/slip guards remain unchanged.

## Topics

Autocalibration, HP3, browser recording, force tuning, simulation clock, recorder backpressure, event batching, Rerun.

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Per-command recorder backlog explains return stalls | OBS-001 (existing-run observation) | Supported | [Original events](evidence.json), [report](report.md) |
| Columnar batching reduces writer overhead without losing original envelopes | EXP-001 (offline writer replay) | Supported | [Measurements](batch-benchmark/results.json) |
| Batching prevents a recorded command burst from blocking the following physics acknowledgement for seconds | EXP-002 (localhost recorder replay) | Supported | [Measurements](batch-benchmark/websocket/results.json) |
| Full browser collection speed and calibration decisions are unchanged | None | Not tested | [Limits](report.md) |

## References

- [Full report](report.md): diagnosis, implementation, measurements and limitations.
- [Research notebook](research.md): constraints and accepted steering.
- [Reference log](../../extended-autocal/no-buildup-reference-001/sweeps.full_auto.log): original waits.
- [Writer replay results](batch-benchmark/results.json) and [WebSocket replay results](batch-benchmark/websocket/results.json): exact event preservation and timing.
