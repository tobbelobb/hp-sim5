# Execution and recovery

Dispatch goes directly to the existing researcher launcher. There are no route
plans, runtime allocations, experiment counters or review-funding records.
`ProgramController` only configures concurrency and observes lost owners.

Commissioned researchers default to separate worktrees. Worker concurrency defaults to one. `configure_pool` can select up to four;
parallel launches use separate worktrees. Shared runtime changes and combined
checks retain exclusive access. Optional assignment wall timeouts still request
normal launcher cleanup; cancellation and reviewer/check timeouts remain active.

Historical PIDs never authorize signals. Unknown cleanup blocks capacity reuse
until an explicit recovery handoff records what happened. The store retains its
client transport across host recovery.
