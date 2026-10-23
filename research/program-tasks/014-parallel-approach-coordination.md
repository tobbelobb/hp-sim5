# 014: Coordinate a small bounded worker pool

Outcome: the manager can pursue related approaches concurrently without duplicate
leases, shared-world interference or contradictory unrecorded decisions. Extend
013's working serial controller with a default concurrency of one and a small
configurable pool; avoid a distributed scheduler.

Each worker uses the existing launcher's separate worktree/session/supervisor.
Respect shared output/runtime links: independent code worktrees do not imply
isolated artifacts or evaluation state. Use stable attempt ownership and reserve
parent resources atomically for both experiments and pending review.

Done when:

- Two different routes can run under one program allocation with distinct
  worlds, sessions, artifact paths and work branches; duplicate claims/tasks
  remain distinguishable from duplicate leases of the same assignment.
- Concurrent dispatch/review/steering cannot oversubscribe slots/reservations or
  lose decisions. An affected worker can be cancelled without signalling another
  program's worker. Stop-program handles every owned worker and pending review.
- Shared evidence retains its source identity; conflicting findings and overlapping
  datasets stay visible. A result can invalidate another task's premise and queue
  its review without silently deleting that worker's partial evidence.
- Focused two-worker fake-launcher integration, conflict, recovery and cancellation
  tests pass. Existing live-world isolation checks may be reused if code/settings
  and claim match; a second costly physics trial is not owed solely for concurrency.
- Document supported pool limits, controller concurrency guarantees and any
  cooperative shared-filesystem limitations in the chat status/brief.

Do not add remote scheduling or parallel nested manager hierarchies. Reuse the
same program/graph/review records so a fresh manager has one coherent overview.
