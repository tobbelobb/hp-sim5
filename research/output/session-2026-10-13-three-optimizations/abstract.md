# Cumulative validation of three headless physics optimizations

Status: complete.

## Abstract

This experiment checked the cumulative effect of three JavaScript simulation optimization commits on exact trajectory behavior and unprofiled physics timing. Frozen HP3/HP4 logo and autocal cases matched exact baseline checkpoints; cumulative median physics time fell 6.7–16.0%, with a 9.0% median across the eight workloads. Six other cataloged 3D machine designs—Four High Anchors, CubeCorners, and four Slideprinter variants—also matched the unoptimized `0cfe920c` checkout exactly through 200 production-pipeline steps with small commanded spool moves. The timing result is a single-host, fixed-prefix benchmark; no separate performance timings were collected for those six designs.

## Topics

HP3, HP4, headless simulation, exact trajectory checkpoints, component-store lookup, attachment transforms, performance.

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| The three cumulative optimizations preserve exact trajectory checkpoints and reduce median physics time across the eight frozen workloads. | CUM-1 | supported | [Cumulative report and matrix](report.md), [raw matrix](cumulative-matrix.json). |
| The three optimizations preserve commanded physics behavior in all other 3D machine designs in the app catalog. | CUM-2 | supported | [Cumulative report](report.md), [per-scene exact snapshots](non-hp-scenes.json). |

## References

- [Full report](report.md): cumulative measurements and limitations.
- [Research notebook](research.md): objective, constraints and experiment status.
- [Original profiling study](../2026-10-13-headless-js-profiling/report.md): frozen inputs and numerical baseline.
- [Attachment reuse study](../2026-10-13-attachment-reuse/report.md): evidence for the third optimization commit's two stages.
- [Headless profiling recipe](../../../../tests/benchmark/HEADLESS_JS_PROFILING.md): runner behavior and comparison protocol.
