# Bounded research workers across competing approaches

Status: complete for task 014. Manager release incomplete.

## Abstract

The manager now coordinates a small local worker pool through the existing launcher. The store and each program allocation default to one worker and one future review obligation. Store capacity supports up to four workers and sixteen review obligations. Fixture checks exercised actual CLI/file-host requests, real child processes and Git worktrees, with synthetic supervisor/world identities. They verified atomic charges, separate sessions and branches, cross-program backpressure and cancellation, shared-build exclusion, recovery without PID adoption, and attempt-specific premise review while retaining partial evidence. The successful fix pass covered 61 tests and 17 subtests; the final store/metadata pass covered 22 tests and 17 subtests. These results establish the fixture-tested coordination boundary. They do not establish live coding-client review, production-world isolation or calibration accuracy. Task 016 still owns that release evidence.

## Topics

research manager, worker pool, SQLite reservations, review backlog, worktree isolation, premise withdrawal, interrupted review

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| A small local pool can enforce owned worker/review capacity and preserve shared decisions | POOL-014 | Supported within fixture scope | [Checks](checks.json), [CLI evidence](cli-evidence.json) |
| A changed premise remains linked to its running attempt and partial evidence | POOL-014 | Supported within synthetic graph scope | [Final tests](pytest-final.txt), [Report](report.md) |
| This proves the live managed production research path | None | Not tested | Task 016 remains required |

## Decision handoff

- Scope: local SQLite/file host, existing launcher worktree creation and fixture children; world IDs and reviewer judgements are synthetic.
- Review: producer checks plus parent read-only audit. Passing unaffected cases were reused. Unknown orphan activity remains unknown after a recorded handoff.
- Decision: complete task 014; keep default concurrency and backlog at one. Raise store and allocation capacity explicitly when parallel work is useful.
- Next: task 015 must hold shared-build/evaluation exclusion while checking a combined patch candidate. Task 016 must supply actual client and production-tool evidence.

## References

- [Full report](report.md): implemented behavior and limits.
- [Research notebook](research.md): budget, corrections and next decision.
- [Controller guide](../../../../research/program-controller.md): commands and ownership boundaries.
- [Preceding bounded-controller report](../2026-10-24-bounded-controller/report.md): task 013 foundations reused here.
