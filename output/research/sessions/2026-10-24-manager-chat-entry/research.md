# Task 011 notebook

Objective: operate durable research programs from an existing chat through CLI
and through a dedicated supported-client/MCP manager entry, with no manager world.

Development budget: at most 90 wall minutes of implementation/check work on this
resumed iteration. Zero numerical experiments are needed for interface checks.
Actual fits, numerical replays, physics runs/steps, movements and collections: zero.
No physics/dynamics source changes; the physics delivery gate does not apply.

Accepted steering: resume unfinished 011, preserve the partial manager/reviewer
changes, reuse supported client/auth helpers, auto-start an owned tool controller
for existing-chat CLI calls, bind program identity, pause only new dispatch,
retain live-release requirements and record SDK/client environment limits.

QA-011-1: fresh-process CLI and actual store/host lifecycle checks with fixture
workers/reviewers. Initial suite exposed an incorrect Path.open(opener=...) call:
seven startup failures, three passing client checks. Changed it to built-in open.
Then ten focused tests passed in 7.96s. Adding positive fixed-review cancellation
and admission yielded eleven passing tests in 10.45s. Those are local protocol
and ownership checks, not scientific findings or actual client conversations.

QA-011-2: real MCP SDK initialize/list/call/exit against the actual manager adapter.
A timer exists only in the test wrapper to wake the sandbox-blocked internal
asyncio self-pipe. SDK and production adapter are unchanged. Stock stdio startup
and a genuine coding-chat smoke session are not established by that check.

Prior overnight client/physics prerequisite observations are reused, not rerun:
../../program-task-loop-2026-10-23/prerequisites.json and
../../program-task-loop-2026-10-23/sdk-protocol-followup/findings.json.
The full manager remains incomplete until 012-016 implementation/live proof.

Final QA-011-1 affected integration passed 107 tests, 18 deselected and 34 subtests
in 17.69 seconds. The subsequent supported-client configuration followup passed
four affected cases in 0.67 seconds. All unaffected passing tests were reused.
Retained the actual SDK wire exchange, historical question-only overview, fixed
review fixture history and actual client command plans. No actual clients ran.

Source freshness: parent confirmed main/HEAD/origin at
7840797b5a81d265ebb0c5091d3e0b6322ef9a6d before this resumed assignment.
No fetch, push or network install was attempted. Parent handles publication.
Decision: complete 011, retain the full manager readiness notice, and hand off 012.
No second task starts in this invocation.
