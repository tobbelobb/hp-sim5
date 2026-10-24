# Finite controller allocations and recovery

The manager conversation chooses distinct assignments. The file host validates
and dispatches that finite selection through the existing worker launcher. It
never restarts an unfinished prompt to keep a conversation running.

An assignment still needs a positive finite `wall_minutes` contract. Before a
sequence, reserve its worker time, separate review time and cleanup allowance:

```bash
./hp-sim5-research-program tool CONNECTION allocate --params '{"task_limit":2,"wall_minutes":24,"review_minutes":3,"cleanup_seconds":190,"retry_limit":2,"review_backlog_limit":1}'
./hp-sim5-research-program tool CONNECTION run_within_allocation --params '{"allocation_id":"RETURNED_ALLOCATION_ID","assignment_ids":["FIRST_READY_ASSIGNMENT","SECOND_READY_ASSIGNMENT"]}'
./hp-sim5-research-program tool CONNECTION runtime_status
```

For example, two assignments with eight worker minutes each reserve 22 minutes
including their reviews. The 24-minute allocation leaves two minutes for manager
interaction and dispatch overhead. Choose these numbers from the workload. This
example sets no policy for fits, physics runs or collection. Supported simulated
autocal CLI speedup and recording guidance still apply.

`run_once` dispatches one selected assignment against `allocation_id`. The older
`worker_start` operation uses the same controller. Without an explicit grant it
creates one task with the assignment's wall time, five review minutes and 190
cleanup seconds. These defaults do not cap the assignment's research duration.
Use an explicit grant for a different review or cleanup requirement.

The controller reserves capacity before spawning. A task charge and its wall
reservation remain spent after a failed or cancelled start. An assignment is
reserved once. An explicit execution retry uses a new bounded assignment and
`retry_of` naming the previous attempt. `retry_limit` bounds total launches in
that retry chain and separate reviewer launches per task. Neither loop retries
automatically. Scientific inconclusive limits remain the strategy's own rule.

One worker and one separate reviewer can run at once by default. An in-flight
worker holds a future review obligation before it produces any submission files.
With the default backlog limit of one, the queue waits for its fixed submission,
separate review and `review_apply` before dispatching the next worker. Reviews
spend their reserved time; cancelled reviews do not refund it. `fund_review`
explicitly allocates review of retained historical evidence after an interrupted
or ended grant. It transfers every predecessor obligation for the same attempt
in one transaction. Admission reconciliation repairs a cached reservation if a
host dies immediately after the admission commits.

`pause` blocks new worker and reviewer dispatch. Running work continues within
its existing deadlines. `resume` permits dispatch again. An accepted explicit
human instruction clears the pending queue for manager selection under that
instruction, while its running investigation continues. Notes and hunches retain
the queue. A changed assignment still has to fit the remaining grant.

`cancel_attempt` and `worker_cancel` request the existing launcher's shutdown.
`stop_allocation` ends one grant while leaving the program open. `stop_program`
and `stop` stop new work and cancel owned workers and reviewers. Poll status to
observe cleanup. `shutdown` cancels owned work and retires the host without
erasing history. A question can remain unresolved with no funded work.

The allocation deadline covers worker activity and review. Cleanup has its own
allowance beyond that deadline. The host polls deadlines. The worker launcher
also arms an absolute POSIX deadline, including startup and worktree preparation,
and carries that same timestamp through worktree re-execution. Expiration enters
its existing interruption and cleanup path. Cleanup disables the alarm and
ignores a second SIGTERM until cleanup finishes. A separate review guard owns the
coding-client process group and enforces its wall deadline even after host loss.
Review records retain the coding-client `argv` and separate `guard_argv`; their
recorded process identity belongs to the guard.

These limits bound dispatch, concurrency, task count, review backlog and owned
process wall time. They do not promise hard aggregate movement or spend limits.
Production supervisor jobs enforce their supported limits, such as autocal job
wall timeout and physics-step settings. Arbitrary shell work is unmetered here.
`runtime_status` keeps allocation estimates, observed worker wall duration,
authenticated supervisor observations and `worker_usage` reports separate.
A worker report never becomes a measured count or a scientific admission.

A forced launcher-group kill, cleanup errors or a missing exit manifest leave
physical and recording shutdown unverified. The CPU/process slot can be released,
but new dispatch waits for an explicit bounded handoff acknowledging that unknown
boundary. The controller never silently calls that a clean physical shutdown.

## Reopen after an interruption

Opening the same program in another chat reuses the live file host and its owned
children. `recover` inspects their birth identities and existing session and
connection records. When available it reads authenticated supervisor status,
without treating a saved final snapshot as a live world or a conversation resume
as a physics checkpoint.

After host loss, a new host marks old allocations and unowned attempts
`interrupted`. It retains charged reservations, partial artifacts and observations.
A live foreign controller retains its reservation. An expired lease or historical
PID alone never authorizes adoption or a signal, including when that PID has
been reused. Unsupported reconnection stays interrupted or unknown. It does not
replay a queued prompt.

```bash
./hp-sim5-research-program tool CONNECTION recover
./hp-sim5-research-program tool CONNECTION recover --params '{"handoff_attempt_id":"INTERRUPTED_ATTEMPT","reason":"Recorded why the unknown physical state permits this new bounded handoff"}'
```

If the crash occurred before an attempt ID was recorded, use
`handoff_allocation_id` instead. Then create a distinct bounded assignment and
fund it. The historical attempt keeps `physical_shutdown: unknown`; this command
records a handoff, rather than asserting that an orphan stopped. Existing
launcher and supervisor tools remain the owners of physical cancellation.

The store and private IPC files are a cooperative same-UID boundary. Exported
historical PIDs never provide process authority. Stronger isolation is separate
work. The [fixture transcript](../output/research/sessions/2026-10-24-bounded-controller/cli-transcript.json)
shows three distinct assignments, separate fixture reviews and a fresh host
continuing after two results. The [report](../output/research/sessions/2026-10-24-bounded-controller/report.md)
records fault checks and limits. Genuine coding-client and production-tool
release evidence remains task 016.

## Small worker pool

Pool settings persist for the entire store, across host restarts and unopened
programs. The store defaults to one worker and one future review obligation.
Set both limits deliberately, then select each program allocation's capacity:

```bash
./hp-sim5-research-program tool CONNECTION configure_pool --params '{"worker_limit":2,"review_backlog_limit":2,"reason":"Compare two bounded routes concurrently"}'
./hp-sim5-research-program tool CONNECTION allocate --params '{"task_limit":2,"wall_minutes":24,"review_minutes":3,"worker_limit":2,"review_backlog_limit":2}'
./hp-sim5-research-program tool CONNECTION run_within_allocation --params '{"allocation_id":"ALLOCATION","assignment_ids":["ROUTE_A_ASSIGNMENT","ROUTE_B_ASSIGNMENT"]}'
```

The store supports one through four workers and one through sixteen outstanding
review obligations. Each program allocation still defaults to one worker and one
review obligation. Its worker limit applies across that program's allocations.
Both store and program limits must permit dispatch. Increasing worker capacity
alone never bypasses the default review backlog. Submitted, unapplied results
hold their review obligations, including in stopped or unopened programs.
Cancel/failure releases an unused future obligation; charged worker/review time
remains spent. A partial result can still receive separately funded review.
One separate reviewer slot is shared across the store.

The retained file host runs dispatch, review, steering and polling on the single
SQLite owner thread. Capacity checks and fresh ledger charges commit together
under `BEGIN IMMEDIATE`, before spawning. Queue dequeue and its worker charge
commit together. Independent callers use the file-host CLI/MCP interface;
sharing a SQLite connection across request threads is unsupported. The low-level
Python helpers do not create another scheduling service.

With a store cap above one, manager dispatch always requests the existing
launcher's fresh worktree. Its Git branch, supervisor/world, session directory
and attempt artifact directory belong to that attempt. `runtime_status` reports
store limits, future obligations, reservation holders with program/record IDs,
and unknown ownership. A fresh manager can locate an unopened program that
holds capacity. Saved reservations count until that program is recovered, and
historical PIDs never grant adoption or signalling authority. Program stop and
attempt cancellation affect only that program's owned children. They do not
stop another program's worker or reviewer.

The launcher's output directory, `RRF/build`, dependency links and starting
native addon are shared. Workers must read shared datasets without modifying
them and write new artifacts only in their own session. Native rebuild outputs
are worktree-local, but shared RRF builds, dependency changes and combined
candidate evaluation require `settings.shared_runtime_write: true`. The
controller reserves this setting exclusively against every worker and reviewer,
including the startup gap before a process record exists. A queued exclusive
assignment waits intact for compatible capacity. For task 015, the store-side
hooks are `program_pool.check_worker(..., exclusive=True)` and
`program_pool.check_review`, called within the transaction that records the
integration/evaluation lease. An integration implementation must retain that
lease through its checks; task 014 does not run or integrate combined patches.
These checks require same-UID cooperation. They do not prevent a worker from
editing shared files outside the API or provide adversarial evaluation isolation.

Attempts snapshot the exact route-plan premise IDs, including funded rival
plans. A later withdrawal/applicability event appends an attempt-specific
`premise_conflicts` entry with its assignment, premise and event citations. The
manager queues review while leaving the worker and its partial evidence intact.
The graph still retains shared input identities and conflicting findings; two
results using the same dataset do not become independent confirmations.

An orphaned reviewer can remain live even after recovery marks its run
`interrupted`. Its guard has its original deadline, but no new host adopts it.
Unknown reviewer records and uncached startup charges continue holding reviewer
capacity. After inspecting that boundary, explicitly record a new bounded
handoff with `recover` using `handoff_review_run_id`, or
`handoff_review_charge_id` when a charge has no cached run ID, plus a reason.
These operations reject active/current-owned or live foreign-controller work,
retain charged seconds and record `reviewer_shutdown: unknown`. They do not
assert that the historical guard stopped. An acknowledged orphan can overlap a
new owned process; slot limits cover controller reservations and owned work,
not an assertion that unknown external activity has ceased. The same limitation
applies to an explicit worker handoff after unknown physical cleanup.
