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

## Implementation and checks

The existing-chat route runs hp-sim5-research-program --question or --program,
which opens the store and automatically starts one locked host when necessary.
Its private file queue checks host generation, program and controller identity.
A bound tool cannot override program_id or invoke the unbound open operation.
Three concurrent fresh openers share one host and controller. Another program
gets its own identity; shutting down one controller does not retire another.
Zombie/dead process states fail the liveness check without changing stable
birth-identity equality. Historical workers are never adopted or signalled.

The host keeps the actual WorkerDispatcher and ReviewController instances alive,
observes their children and serves short structured operations. Opening or
attaching starts no worker, physics runtime or Viewer. A worker_start goes through
the existing launcher with its bounded assignment. Pause only prevents new
worker/review dispatch. Worker_cancel, review_cancel and stop request explicit
cancellation. Cancelled reviewers cannot supply a review receipt. Producer exits
never admit a claim. Tests seal synthetic files, execute a separate fixture
reviewer and admit its fixed report through the controller only after completion.

Dedicated Codex/Claude command construction and auth reuse coding_client.py.
Forwarded Codex provider/model handling was factored there from the unchanged
worker behavior. Both clients attach only hp_sim5_program. Codex also disables
the configured worker physics and Rerun servers. The final configuration followup
uses the existing Codex approval/startup convention and the supported Claude
command/args descriptor. Its four affected checks passed after the 107-test
integration; all unchanged passing store/worker/review checks were reused.

The SDK wire test runs the real MCPServer, validators and stdio protocol against
scripts/research_program_mcp.py. Its external 10 ms timer wakes the event loop in
this sandbox where asyncio internal self-pipe sends fail with EPERM. This timer
is confined to the retained test wrapper. Production SDK/adapter source received
no wakeup workaround. The wire record includes a real overview and a structured
bound-program error. The prior client's nested app-server writable-state failure
is reused as the reason an actual coding-chat smoke conversation was unavailable.
The earlier prerequisite file names mcp 1.29 before recovery; prior-sdk-findings.json
and this session's wire test supersede that SDK limitation with actual mcp 2.3.0.
No credentials, environment overrides, installs or network approvals were added.

The initial seven startup failures came from using Path.open with opener, which
it does not accept. Built-in open fixed that code error. Subsequent focused checks
passed and the final affected integration passed 107 tests, 18 deselected and
34 subtests in 17.69 seconds. The later four client checks passed in 0.67 seconds.
The exact commands and stdout are in checks.json and the transcripts. The
question/fixture snapshots are retained historical test records after orderly
host shutdown, not active live research programs or newly measured science.

## Limits

The host is a cooperative same-UID local tool, not protected evaluation. Private
files and birth identities reduce accidental cross-control; the same UID can
still edit files or call store APIs directly. Timeout can leave a mutation outcome
unknown, so callers must inspect state before retrying. A lost host does not adopt
old children; stale active reservations block dispatch. Runtime allocation records
are worker contracts, not hard spend or movement guarantees. Independent worker
wall enforcement and recovery belong to task 013. Strategy automation, coordinated
parallel workers and combined-patch delivery remain 012, 014 and 015. The complete
manager remains incomplete until all those capabilities and task 016 live proof
pass. No physics/motor/cable/dynamics source changed, so the physics delivery gate
does not apply to this task. Numerical budget and use were zero.
