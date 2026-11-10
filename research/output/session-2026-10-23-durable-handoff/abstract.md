# Research handoffs retain scoped evidence after worktree turnover

Status: complete.

## Abstract

Can another researcher inspect a useful result after its original worktree or
supervisor is gone? Task 006 added a standard-library archive CLI that copies
explicitly selected evidence, records hashes and relative references, and reads
saved autocal job outcomes offline. DH-E1 packaged task 004's saved consensus
result: 19 files, including eleven compressed inputs, totaling 4,509,141 bytes.
After relocating the bundle and deleting every temporary source path, all
selected hashes and its scoped decision remained readable. Focused tests also
preserved partial records for absent/damaged evidence and separated terminal job
observations from historical running/cancelling states. All 21 archive/handoff
tests passed. This demonstrates portability and historical lifecycle inspection,
not independent reproduction of the calibration result, current process liveness
or scientific success. Evidence selection remains explicit; runtime dependencies
and unselected document links are outside the bundle. No physics, fitting,
prediction evaluation or collection was performed.

## Topics

research archive, evidence portability, durable handoff, autocal job status,
worktree turnover, SHA-256, partial evidence, lifecycle acceptance

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Explicit selected evidence preserves a scoped result without its source paths | DH-E1 | Supported for the copied CM-E1 handoff | [Results](results.json), [recovery](evidence/recovered.json) |
| Saved lifecycle records can be read without treating CLI acceptance as science or stale running as liveness | DH-T1 | Supported for synthetic lifecycle and corruption fixtures | [Checks](verification.json), [tests](../../../../tests/python/test_research_handoff.py) |

## Decision handoff

- Scope: offline archive export/inspection and saved full-autocal job records; base revision 166f0352 and implementation hashes in verification.json.
- Review: this agent ran the focused tests and one byte-portability demonstration, then inspected the implementation/evidence. No independent scientific reproduction or live-job test.
- Decision: keep the explicit export/show/job route; retain live supervisor ownership semantics and report limitations with partial evidence.
- Next: task 005 can use the living research agenda. Revisit automatic reference discovery only if explicit selections prove insufficient in actual handoffs.

## References

- [Full report](report.md), [notebook](research.md): scope, implementation and limitations.
- [Demonstration](demonstrate.py), [export manifest](evidence/export-manifest.json): copied-file/source identities and original-to-relative mapping.
- [Producer CM-E1 report](../2026-10-23-consensus-membership/report.md): scientific decision reused, not numerically rerun.
- [Operator procedure](../../../../research/handoffs.md): export selection and offline job interpretation.
