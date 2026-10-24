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

## Implemented behavior

`ProgramController` records finite task/wall grants, separate review reservations,
cleanup allowance, leases, ordered manager selections and retry/backlog limits.
`run_once` and `run_within_allocation` dispatch through the existing worker
adapter. The default remains one worker and one separate reviewer. Task charges
survive cancelled/failed startup. A reviewed submission clears its obligation;
new historical-review funding atomically transfers predecessor obligations.
Admission reconciliation repairs the reservation cache if a host dies after
admission commits. Repeated unchanged polling leaves semantic revisions stable.

The existing `hp-sim5-research-agent` receives an absolute deadline, carried
through worktree re-execution. Its POSIX alarm enters the existing cleanup path.
Cleanup disables the activity alarm and ignores repeated SIGTERM until that
cleanup finishes. The separate review guard owns the coding-client process
group and enforces its deadline after host loss. This adds no physics runtime.

Pause blocks new dispatch while owned work continues. Stop/cancel delegates to
the existing launcher/supervisor path, retains partial files and exposes cleanup
errors or missing manifests as unknown boundaries. A released process slot does
not prove the world or recorder stopped. Unknown cleanup requires a recorded
bounded handoff before another worker starts. Explicit human direction holds
pending queue selections without cancelling unrelated active work.

Recovery reads leases, process birth identity, session/connection records and,
when reachable, authenticated current supervisor status. Historical/final
snapshots never become live-world authority. Foreign PIDs are not adopted or
signalled. Lost ownership becomes interrupted/unknown and requires a distinct
new assignment. Invalid worker options are rejected before funding a startup.
Stopped/expired allocations retain late results without silently admitting them.

## Verification and evidence

- The affected suite passed 140 tests, with 18 deselected and 34 subtests, in
  38.97 seconds. Exact argv and retained output are in [checks.json](checks.json)
  and [pytest.xml](pytest.xml).
- Follow-up controller/manager checks passed 35 cases in 21.35 seconds. They
  include host crash, retry limits, independent deadlines, review funding/cache
  recovery, steering and the fresh-host transcript. See
  [follow-up JUnit](pytest-followup.xml).
- Two supervisor cases passed in 0.16 seconds, with 17 deselected. Five final
  boundary cases passed in 0.63 seconds, with 15 deselected. The final cases cover
  invalid settings before funding, atomic transfer, current supervisor identity
  and unverified review ownership. See [supervisor checks](pytest-supervisor.xml)
  and [boundary checks](pytest-final-boundaries.xml).
- The [CLI transcript](cli-transcript.json) records three distinct assignments,
  three fixture admissions and two host generations. The fresh host observed
  the first two admissions before continuing the third assignment. Actual
  command argv, structured stdout, return codes and final graph are retained.
  Reviewers inspected frozen synthetic fixture hashes; they are not model review.
- An earlier broad selector reached three physics-dependent tests and failed at
  missing `pxr` import after 69 passing cases. The [failure log](affected-tests.txt)
  is retained. No physics ran. The supported affected selector then passed.
- Parent reviewed the implementation and read-only audit findings. Follow-up
  fixes addressed continuous SQLite writes, historical-review funding bypass,
  double-counted obligations, unresolved cleanup dispatch, transfer atomicity,
  admission-cache recovery, cleanup signal races and independent reviewer timing.

This handoff reused the passing checks. It did not rerun unchanged suites.
Compilation, queue/docs/source integrity, catalog regeneration and final diff
checks are recorded in checks.json.

## Limits and next decision

Task/wall/concurrency/backlog reservations and owned-process deadlines are
implemented. Aggregate shell movement, computation and spend remain unmetered.
Worker-reported counts stay reports. Supported production-tool per-job limits
apply only to the calls that enforce them. The cooperative same-UID store and
worktrees are not adversarial isolation.

Development numerical budget and use were zero fits, predictions, replays,
physics runs, steps, movements and collections. No physics/dynamics math changed,
so the physics delivery gate does not apply to this controller task. Research
runtime defaults still permit targeted fits, headless/native physics and
simulated collection under explicit finite allocations and supported speedup.

The environment lacks USD/pxr, Rerun, cargo/native build and node dependencies.
The installed nested Codex client fails app-server initialization with read-only
state; Claude is absent. Stock MCP stdio is restricted by the sandbox's asyncio
socket wakeup denial. Prior diagnostic accommodations are not release proof.
No keys, HOME/CODEX_HOME overrides, network installs or new approval waits were
used. These limitations remain genuine task 016 prerequisites.

Parent verified upstream main matches base
`891aef26a82d5b2096c9f1b3e93a419effc58b36`. This task makes one local commit;
the parent publishes after review. Task 014 is the next eligible implementation.
The full manager remains incomplete until actual task 016 evidence passes.
