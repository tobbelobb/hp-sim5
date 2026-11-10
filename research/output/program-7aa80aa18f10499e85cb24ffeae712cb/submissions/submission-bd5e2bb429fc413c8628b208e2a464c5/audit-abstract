# Cloud Research Program audit and repairs

Status: complete within tested scope; final production regression interrupted.

## Abstract

The managed CPU cloud could perform useful research, but its original setup and tools did not work smoothly. Actual authenticated workers and separate reviewers tested native experiments, telemetry, recordings, collection, autocalibration, Rerun, browser tools and cleanup. A production actor invoked all 23 supervisor tools through a launcher-owned MCP file relay while its command sockets were denied. A recorded/unrecorded 500-step native pair produced identical 501-row telemetry; six collection points validated; bounded full-auto produced 1,305,459 steps. Repairs address shared-output paths, test caches, review launch paths, CPU rendering/protocol setup and invalid status/history responses. The final delivery pass at the repaired runtime source passed 333 tests and 34 subtests. A fresh full-auto/browser regression reached its deadline during a stalled parent permission request, leaving final live verification incomplete. These results support the tested repairs, not optimal performance or calibration accuracy.

## Topics

Research Program, managed cloud, MCP, file relay, native simulation, Rerun, SwiftShader, autocalibration, lifecycle, strict JSON, review, cleanup.

## Tested hypotheses

See the [full report](report.md) for hypotheses, experiment IDs and measured outcomes.

## Decision handoff

- Scope: current CPU cloud, HP3/HP4 supported paths, runtime fixes through `7c0a353e`.
- Review: baseline and production evidence separately reviewed; final repaired-source review retained with the archive.
- Decision: publish tested repairs with the final live verification gap explicit.
- Next: bounded terminal full-auto, next-job cancellation and real browser history using a pre-permitted launcher/browser context.

## References

- [Full report](report.md) records measurements and limits.
- [Notebook](research.md) retains budgets and accepted steering.
