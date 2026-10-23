# 009: Dispatch and observe the existing research worker

Outcome: the controller launches a bounded assignment through
`hp-sim5-research-agent` and records its real session, process and submission
artifacts. Build on 007, the existing launcher, services ownership helpers and
[the execution architecture](../architecture.md). Do not replace the worker.

Implement a small adapter for the existing `--batch --prompt-file --session-dir`
route and supported backend/provider/model, worktree, machine, viewer/recording
options. Use an argv array and exact prompt file. Respect the launcher's new
session-directory requirement and shared output behavior; resolve actual returned
paths across re-exec/worktree creation. Preserve existing authentication and
client flags; never provision a new API dependency merely to dispatch a worker.

Record assignment/attempt/session/worktree IDs, owner identity, source revision,
argv/settings, start/exit observations, logs/events/final-message and launcher
manifests. Start/status/cancel must remain responsive without waiting on the
whole investigation. Expose owned worker sessions for human inspection.

Done when:

- The launch plan invokes the existing executable and preserves a supplied
  assignment and its budgets. The manager does not create a second physics world.
- Fake launcher fixtures exercise normal submission, nonzero exit, missing
  executable/authentication, partial logs and interrupted startup. A zero exit
  or a success token never becomes an accepted scientific result.
- Cancellation reaches the launcher/supervisor cleanup path, preserves artifacts
  and reports cleanup failures. Ownership includes birth identity or equivalent;
  a reused historical PID is not adopted or signalled.
- A real launcher dry-run verifies argument compatibility for supported clients;
  record unavailable client/runtime prerequisites honestly. Actual live worker
  evidence is mandatory in 016 and cannot be inferred from fake children.

Verify the adapter and required existing launcher regressions once. Keep one
active worker initially. Bounded parallelism and restart reconciliation have
their own dependent tasks, rather than hidden untested behavior here.

## Delivered

Task 009 is complete. [The adapter guide](../program-workers.md) documents the
actual `ProgramStore.workers(root)` start/status/cancel/sessions API, durable
attempts, absolute prompt/session paths and ownership boundary. One active
reservation is enforced across a store. The launcher writes an early non-secret
session manifest with actual paths after worktree re-exec. Interrupted runtime
and Viewer startup retain cleanup ownership, and cleanup errors remain explicit
while other owned services still receive cleanup attempts.

[The verification record](../../output/research/sessions/2026-10-23-program-worker-dispatch/report.md)
contains four actual launcher dry-run outputs, fixture/launcher check counts,
test-tool provenance and runtime limitations. Final adapter/launcher checks:
57 passed, 19 deselected, 17 subtests; the later attempt-import validation check
passed all 10 adapter tests with 17 subtests. The earlier 17 store/graph tests
passed and are reused. Existing launcher regressions used actual launcher code
with fake clients/services; dry-runs started neither coding client nor physics.
The development numerical budget was zero. Runtime allocations are preserved
worker contracts, with hard lifecycle budgets/reconciliation left to task 013.
No live research or scientific admission is claimed; task 016 remains mandatory.
