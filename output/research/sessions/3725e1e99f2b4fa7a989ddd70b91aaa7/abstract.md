# Native and Viewer MCP round-trip check

Status: complete.

Retrospective summary of saved evidence, written 2026-10-08.

## Abstract

A fresh three-step HP4 numerical experiment returned finite telemetry at 0.006 seconds. The existing candidate recording was then inspected at step 200 with zero Viewer warnings or errors. This checks both MCP servers; it makes no calibration-accuracy claim.

## Topics

integration verification, MCP, HP4, numeric telemetry, Rerun

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Both numerical and Viewer MCP servers work in one session | Fresh three-step check; 22f857e8987149ab93220cfc18f568a6 | Supported as a tool check | [Report](report.md) |

## References

- [Saved results](report.md)
- [Actual MCP calls](events.jsonl)
- [Tracked SDK and route verification](../../../../research/verification.md)
