# Program-bound manager tools for an existing research chat

Status: complete for task 011. Full manager release remains incomplete.

## Abstract

This session tested whether a repository-connected coding chat can create/open a
research program and operate its durable graph, workers and separate reviews
through structured tools without manual service setup or a manager physics world.
A file-queue host retains the actual dispatcher/reviewer child ownership across
fresh CLI processes. The affected integration passed 107 tests and 34 subtests;
eighteen unrelated tests were deselected. Four later client/configuration checks
also passed. Eleven manager cases cover question-only entry, concurrent opens,
program-bound identities, responsive worker status, pause without cancellation,
explicit cancellation/stop, fixed-review admission and zombie-host rejection.
The actual MCP SDK completed initialize/list/call/exit using a diagnostic timer
only in the test wrapper. Stock SDK startup and a genuine coding-chat session
remain unproved because of recorded sandbox/runtime limitations. No numerical
experiments or actual model reviews ran. Tasks 012-016 still own remaining manager
implementation and live release proof.

## Topics

research manager, chat entry, CLI, MCP, program attachment, worker ownership,
file IPC, separate review, pause, cancellation, Codex, Claude, sandbox prerequisites

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Fresh chat/tool processes can share one program controller and retained worker ownership without duplicate dispatch or manager physics startup | QA-011-1 | Supported by real local protocols with fixture children | [Checks](checks.json), [test transcript](pytest-final.txt), [historical question example](question-example.json) |
| Fixed separate-review cancellation and admission remain available through program-bound CLI tools | QA-011-1 | Supported with fixture reviewer judgements, not live-model review | [Fixture history](cli-review-fixture.json), [tests](../../../../tests/python/test_research_program_manager.py) |
| The actual manager MCP adapter handles SDK initialize/list/call/exit | QA-011-2 | Supported with an explicit test-only timer accommodation | [Wire record](manager-protocol.json), [wrapper](sdk-timer-test.py), [prior SDK diagnosis](prior-sdk-findings.json) |
| Stock SDK startup and the full chat-to-production-worker release work in this sandbox | None | Not tested successfully; prerequisites remain | [Prior runtime observation](prior-runtime-prerequisites.json), [manager release contract](../../../../research/program-tasks/MANAGER.md) |

## Decision handoff

- Scope: actual local CLI/store/controller and SDK protocols, synthetic worker/reviewer fixtures and supported-client command planning.
- Review: implementation author inspected the retained checks. No numerical reproduction, actual coding-chat conversation or independent model judgement occurred.
- Decision: task 011 is complete as a manager interface capability. Keep the full manager readiness notice until remaining implementation and task 016 live proof pass.
- Next: task 012 can use these tools for strategy and steering. Task 013 must add runtime deadlines, reservation/restart reconciliation and autonomous bounded continuation.

## References

- [Full report](report.md): behavior and current limits.
- [Notebook](research.md): budget, accepted steering and corrected startup failure.
- [Manager guide](../../../../research/program-manager.md): current commands and question-only prompt.
- [Client plans](client-plans.json): supported argv/configuration only; no client launched.
- [Preceding review capability](../2026-10-23-reviewed-claim-admission/report.md): fixed submissions and admission contract reused here.
