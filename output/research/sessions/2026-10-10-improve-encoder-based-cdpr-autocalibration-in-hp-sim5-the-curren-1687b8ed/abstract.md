# HP3 encoder autocalibration: reference failure and runtime bottlenecks

Status: complete for the supplied reference; no source change justified.

## Abstract

This investigation asks why zero-buildup HP3 encoder autocalibration is slow and can return poor anchors, and whether fewer points are safe. The supplied browser-js reference recorded 4,706,150 physics steps over 9,412.3 simulated seconds and ended after eight sweeps with a rank-zero, underconstrained fit; its recorder marks autocal incomplete. Encoder noise was reported as zero and fixed-target errors stayed small, while later anchor estimates became geometrically implausible. Recovery fitting took about 1.35 seconds per exhausted pass, making collection and settling the likely primary wait cost. Detailed event timing could not be completed because streaming the 2.0 GB RRD did not return within the bounded tool window. Earlier recorded trials found fixed point thinning worsened anchor RMSE on three of four fixtures; a guarded settling change saved about 25% in separate fixed HP4 collection. No source changes or hardware measurements were made. A coverage-aware, collection-time-aware planner and adaptive point count remain untested.

Follow-up inspection found 473,771 RRD chunks; Rerun's object-store estimate was 2,232. Its supported optimizer rewrote a separate copy in 66.9 seconds, reducing chunk count by 99.7% and file size by 41.3% (1.9 to 1.1 GiB), without intentional sample deletion. Exact semantic comparison did not complete. Rerun supports sparse unchanged values through latest-at semantics; trimming quiet intervals from existing data is a lossy derived transform and must preserve event timestamps and mark gaps.

## Topics

autocalibration, HP3, encoder-only, anchor identifiability, rank deficiency, point count, collection time, sweep planning, Rerun profiling

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
|---|---|---|---|
| Supplied run completes automatic acceptance | E01 | Rejected | [Run log](../../../../extended-autocal/no-buildup-reference-002/sweeps.full_auto.log) |
| Fitting/recovery dominates wait | E02 | Rejected for supplied run; detailed RRD profile incomplete | [Recorder manifest](../../../../extended-autocal/no-buildup-reference-002/hangprinter-20261010T151618.949443Z-c9ffe4cd.json) |
| Fixed halving of points is robust | E03 | Rejected in prior recorded trials | [Prior report](../2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/report.md#disproved-ideas-and-remaining-experiments) |
| Selection score can disagree with physical accuracy | E04 | Supported by prior audits; consistent with this failed geometry | [Prior physical audit](../2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/E01-E01-baseline-audit.json) |
| Low encoder noise ensures identifiable geometry | E05 | Rejected for this simulated run | [Dataset](../../../../extended-autocal/no-buildup-reference-002/sweeps.json) |
| Chunk overhead materially inflates the RRD | E06 | Supported by Rerun stats | [Full report](report.md#rrd-size-and-whether-quiet-time-can-be-removed) |
| Supported RRD optimization reduces size | E07 | Supported as rewrite/size result; exact equivalence unverified | [Full report](report.md#rrd-size-and-whether-quiet-time-can-be-removed) |

## References

- [Full report](report.md): analysis, measurements and limitations.
- [Research notebook](research.md): experiment decisions and next step.
- [Supplied dataset](../../../../extended-autocal/no-buildup-reference-002/sweeps.json), [log](../../../../extended-autocal/no-buildup-reference-002/sweeps.full_auto.log), [manifest](../../../../extended-autocal/no-buildup-reference-002/hangprinter-20261010T151618.949443Z-c9ffe4cd.json): failed HP3 simulated run.
- [Prior October 8 report](../2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/report.md): ranking, thinning and settling evidence.
- [Rerun optimize-chunks guide](https://rerun.io/docs/howto/logging-and-ingestion/optimize-chunks) and [latest-at semantics](https://rerun.io/docs/concepts/logging-and-ingestion/latest-at): supported compaction and sparse component behavior.
