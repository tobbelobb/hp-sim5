# Finite research allocations and recovery across manager interruptions

Status: complete for task 013; the full manager release remains incomplete.

## Abstract

Can a manager continue distinct investigations within finite allocations and
recover without duplicate dispatch or false physical-stop claims? Task 013 adds
durable task, wall and review reservations, bounded queues, retry limits,
pause/stop controls and explicit recovery handoffs. The affected suite passed
140 tests and 34 subtests. Follow-up suites passed 35 controller/manager cases,
two supervisor cases and five final boundary cases. The actual CLI/file-host
transcript completed three distinct synthetic investigations with separate
fixture review and reconstructed two prior admissions in a fresh host. Worker
launcher and reviewer deadlines remained effective without host polling in
process fixtures. Recovery checked process birth and current supervisor identity
without adopting or signalling historical PIDs. Unknown cleanup blocks new work
until an explicit bounded handoff. These checks use fixture children and saved
synthetic evidence, not actual coding-client judgement or production science.
No numerical work ran. Missing live dependencies and client startup remain
limitations for the task 016 release gate.

## Topics

research manager, finite allocations, wall deadlines, review reservations,
recovery, process ownership, pause, cancellation, controller interruption

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Finite reservations, backlog and retry limits prevent duplicate or unfunded dispatch | CTRL-013 fixture suite | Supported within deterministic controller and local process tests | [Checks](checks.json), [affected tests](pytest-final.txt) |
| Owned launcher/reviewer deadlines survive absent host polling and preserve honest cleanup limits | CTRL-013 deadline fixtures | Supported for fixture subprocesses and the existing launcher cleanup path | [Controller follow-up](pytest-followup.xml) |
| A fresh manager can continue distinct investigations and retain reviewed results | CTRL-013 CLI transcript | Supported with actual CLI/file host and fixture worker/reviewer children | [Transcript](cli-transcript.json) |
| The full chat-to-production-worker release works here | None | Not tested successfully; live prerequisites remain | [Report](report.md) |

## Decision handoff

- Scope: SQLite/controller, actual CLI and file host, existing launcher code with fixture runtime/client, synthetic worker/reviewer subprocesses.
- Review: retained test results and process/graph outcomes inspected by implementation and parent agents; no numerical reproduction or independent model review.
- Decision: task 013 is complete. Keep numerical estimates separate from supported tool limits and retain unknown physical-stop boundaries.
- Next: task 014 adds the bounded worker pool. Task 015 covers combined patch integration. Task 016 still requires actual release proof.

## References

- [Full report](report.md): implementation, checks and remaining limits.
- [Research notebook](research.md): budget and decisions.
- [Controller guide](../../../../research/program-controller.md): current operations and recovery workflow.
- [Source hashes](source-hashes.json): exact source and task documentation reviewed.
