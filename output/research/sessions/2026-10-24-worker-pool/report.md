# Bounded research workers across competing approaches

The manager now coordinates a small local worker pool through the existing launcher. The store and each program allocation default to one worker and one future review obligation. Store capacity supports up to four workers and sixteen review obligations. Fixture checks exercised actual CLI/file-host requests, real child processes and Git worktrees, with synthetic supervisor/world identities. They verified atomic charges, separate sessions and branches, cross-program backpressure and cancellation, shared-build exclusion, recovery without PID adoption, and attempt-specific premise review while retaining partial evidence. The successful fix pass covered 61 tests and 17 subtests; the final store/metadata pass covered 22 tests and 17 subtests. These results establish the fixture-tested coordination boundary. They do not establish live coding-client review, production-world isolation or calibration accuracy. Task 016 still owns that release evidence.

## Implemented behavior

`configure_pool` persists the store-wide worker and future review caps. Each
allocation retains its own default of one. Worker reservation and queue dequeue
commit together on fresh records under `BEGIN IMMEDIATE`; reviewer startup is
charged before spawning too. A failed start does not refund charged time.
Unopened programs retain their reservations. Cross-program worker stop/cancel
and separate reviewer ownership remain scoped to their program and attempt.

With a store worker cap above one, manager dispatch requests the existing
launcher's fresh worktree. The two-route fixture used the production
`create_worktree` function and retained distinct work branches, session paths,
artifact paths and synthetic world IDs. Its concise [CLI evidence](cli-evidence.json)
records the actual worker argv, allocations, reservations and final cancelled
states. It queued a third task, held it under explicit steering, cancelled one
worker and then stopped the other through the owning program.

Shared output, RRF builds, dependencies and the starting native addon remain a
cooperative filesystem boundary. `settings.shared_runtime_write` reserves worker
and reviewer exclusion, including the charged startup gap. Task 015 must extend
these store-side lease checks to combined candidate evaluation and retain its
lease for the entire check. Task 014 does not integrate candidates.

Attempts snapshot premise and route-plan IDs. A later graph withdrawal queues
an attempt-specific review entry while retaining the worker's artifacts. Two
opposed synthetic producer assessments remain visible with one shared source
identity. They remain unreviewed reports rather than accepted knowledge.

Recovery never adopts saved PIDs. Unknown worker cleanup blocks new dispatch
until an explicit bounded handoff. Orphan reviewer records and uncached review
charges hold reviewer capacity until their explicit handoff. That handoff retains
the charge and records shutdown as unknown. It does not assert that a live
orphan guard or world stopped. Capacity guarantees concern controller
reservations and owned children; acknowledged unknown activity can still exist.

## Verification and corrections

All exact argv and results are recorded in [checks.json](checks.json). Raw pytest
stdout and JUnit files are retained, including useful failures. Initial and
affected failure outputs use gzip to preserve their exact original bytes.

- Initial pool pass: 9 passed and one fixture-owner setup failure. The fixture
  had represented a live owner as dead without changing its process birth.
- Affected integration: 75 passed, 16 failed, 5 deselected and 14 subtests passed.
  It exposed stale cached local backlog after recovery, missing interrupted
  startup status, and older fixtures that omitted exit manifests without an
  explicit unknown-cleanup handoff. Those cases were corrected. The fixtures
  now record honest handoffs instead of weakening the cleanup gate.
- Fix pass: 61 passed, 1 deselected and 17 subtests passed in 35.20 seconds.
  It covered two-worker orchestration, concurrent file requests, global/per-
  program review backpressure, cancellation, source conflicts, startup-gap
  exclusivity and review recovery with and without an inserted guard.
- Final focused pass: 22 passed, 20 deselected and 17 subtests passed in 10.02
  seconds. It covered store/graph import, pinned premise metadata, actual
  worktree/session separation, pool defaults/range checks, host restart and
  reservation-holder reporting. Passing unchanged manager/review cases from the
  affected run were reused.

Parent read-only audit identified startup-gap exclusion, queue dequeue ordering,
review-charge recovery and local backlog-cache risks. The implementation and
focused regressions resolve those findings. No unchanged passing suite was
repeated for handoff. Compilation, queue/docs/artifact integrity, catalog
regeneration and `git diff --check` are recorded in the final integrity record.

## Budget and limits

Development allocation was at most 90 minutes plus handoff. Numerical allocation
and use were zero fits, predictions, replays, physics runs/steps, movements and
collections. No dynamics or motor math changed, so the physics delivery gate
does not apply. Runtime program research allocations retain their existing
finite workload-specific budgets; no blanket numerical prohibition was added.

These are real local CLI/process/Git checks with fake supervisors, world IDs and
reviewer judgements. Existing client authentication and launcher physics/session
paths are preserved. No live calibration or production-world isolation claim is
made. Missing nested-client writable-state access and physics/runtime
prerequisites remain task 016 limitations. No API key provisioning,
HOME/CODEX_HOME override, installation, shell fetch/push or network approval was
used. The parent supplied upstream freshness at base
`aea237e65d5589eb433a90c345fb7f2e364bf6a9`.

## Next decision

Proceed to task 015 for explicitly checked combined patch candidates. Full
manager release remains incomplete until task 016 supplies the required live
chat, worker, production-tool, review, graph and replanning evidence.

## References

- [Controller guide](../../../../research/program-controller.md): supported pool commands, shared paths and handoff semantics.
- [Task card](../../../../research/program-tasks/014-parallel-approach-coordination.md): acceptance criteria.
- [Preceding controller report](../2026-10-24-bounded-controller/report.md): reused ownership, finite allocations and deadline foundation.
- [Exact checks](checks.json) and [CLI evidence](cli-evidence.json): POOL-014 fixture verification.
- [Source hashes](source-hashes.json): producer source identity for review.
