# Dispatch through the existing research worker

Task 009 adds `ProgramStore.workers(root)` and the standard-library
[WorkerDispatcher](../src/python/hp_sim5_research/program_workers.py).
It launches `hp-sim5-research-agent --batch --prompt-file FILE --session-dir DIR`
with an argv array. Only that existing launcher starts physics, collection and
Viewer services. The adapter inherits its normal client authentication; it never
creates credentials, changes client homes or uses a model API directly.
The full manager is still subject to [task 016's live gate](program-tasks/016-live-manager-release.md).

The API is for a retained controller in one Python process. The chat commands
and lifecycle controller are subsequent tasks. For an existing ready assignment:

```python
from pathlib import Path
from hp_sim5_research.program import ProgramStore

store = ProgramStore(Path.cwd() / '.research-programs')
workers = store.workers(Path.cwd())
attempt = workers.start('PROGRAM_ID', 'ASSIGNMENT_ID', settings={
    'backend': 'codex', 'worktree': True, 'viewer': 'none',
    'machine': 'hp3', 'record': False,
}, instructions='Inspect the named saved inputs; preserve failed evidence.')
workers.status('PROGRAM_ID', attempt['id'])
workers.sessions('PROGRAM_ID')  # Saved paths and explicit current ownership
workers.cancel('PROGRAM_ID', attempt['id'])  # Requests cleanup and returns
# Keep polling status until terminal; then close the store.
```

Assignments must already be `ready`, with an active program, live/promising
approach and a finite allocation including positive `wall_minutes`. Start reserves
one slot transactionally across the entire store, records the attempt and marks
the assignment running before spawning. The exact UTF-8 prompt file contains the
assignment snapshot, program constraints, IDs, allocation, source revision and
session path, followed by additional instructions without shell interpolation.
`assignment.json` and the prompt SHA-256 retain that contract. These allocations
are recorded worker instructions; hard elapsed-time and workload enforcement is
task 013. No automatic restart or continuation loop is included.

Settings preserve launcher defaults unless supplied: `backend` (`codex`/`claude`),
`model_provider` (Codex only), `model`, `worktree`, `worktree_dir`, `physics_backend`,
`machine`, `viewer`, `record`, `dont_ask`, `goal_mode`, `sidebar_section` and
`client_args`. `client_args` is an argv list forwarded unchanged after `--`,
including the launcher's normal provider/model precedence and authentication.
Use client configuration/environment for secrets, not credential text in argv.
An executable override exists for fixtures; production defaults to the repository
launcher. Start does not run doctor or preflight or wait for a client to finish.

## Observations and ownership

The durable attempt ID links to the supplied assignment ID. The attempt records
controller identity, launcher PID, birth ticks, boot ID, host, UID and process
group, source revision/dirty flag, settings/argv and dated reservation/spawn/exit
observations. Missing executables retain a `launch_failed` attempt and logs;
client/authentication failures retain the nonzero exit and stderr.

The manager creates only its private attempt directory under
`STORE/workers/ATTEMPT_ID/`. The requested worker session is a new absolute path
under `output/research/sessions/ATTEMPT_ID/`; the launcher creates it. Keeping
both prompt and session paths absolute preserves them through worktree re-exec
and shared `output` symlinks. A new, non-secret launcher `session.json` is written
before runtime startup. It returns the actual repository/worktree path, branch,
source revision and launcher session ID. The adapter uses the returned repository
path as `worktree_id`; `launch.json` supplies the runtime's distinct session ID.
It never reads the bearer token in `connection.json` into program memory.

Status inventories launcher manifests, raw events, final message, report/notebook,
abstract, attachment/connection files and runtime/Viewer logs alongside controller
stdout/stderr. It records paths and byte counts, and reads only three manifests,
bounded to 1 MiB each. It does not hash growing recordings or ingest telemetry.
Partial manifests become diagnostics; incomplete JSONL does not block inspection.
Files remain provisional/partial for running, failed or cancelled attempts.
Their presence, finalized size or a worker success token is not scientific review.

Attempt execution states are `starting`, `running`, `cancelling`, `submitted`,
`failed`, `cancelled`, `launch_failed` and `ownership_lost`. A zero exit means
`submitted` and assignment execution `completed`; review remains `unreviewed`
and knowledge `unassessed`. The adapter never adds or accepts graph claims.
The [task 010 controller](program-review.md) now owns fixed evidence submissions
and reviewed admission separately from these execution observations. An externally
changed terminal assignment state is preserved when the child exits.

Cancel sends SIGTERM only to the current birth-verified launcher. Its existing
`finally` path stops the client, runtime and Viewer and preserves artifacts.
Cleanup errors are recorded in `exit.json`, and each remaining owned service
still receives its cleanup attempt.
The launcher also now retains partial runtime/Viewer startup ownership when
interrupted. Cancel and status return promptly; callers must continue polling.
Default cleanup grace is 190 seconds, allowing the existing runtime's bounded
shutdown request, wait and termination fallback. On a subsequent poll after that
grace, only the verified launcher group may be killed; this explicitly reports
cleanup failure and unverified detached services. A returned launcher `exit.json`
records that its cleanup path returned, not independent proof of every service's
health. A missing manifest leaves cleanup unverified.

Linux `/proc` birth identity is required. A different dispatcher or imported
snapshot can inspect saved sessions but cannot adopt or signal their PIDs.
A stale active reservation blocks dispatch rather than guessing that the process
is owned. A reused PID/birth mismatch is never signalled. Retain this controller
until workers exit; task 013 adds restart reconciliation and independent deadlines,
and task 014 adds a bounded pool. Same-UID workers and shared output are not
adversarial isolation; filesystem permissions and hashes prevent accidents only.

## Verification scope

[Focused tests](../tests/python/test_research_program_workers.py) use actual
ProgramStore/graph APIs and real child processes with fake launchers. The existing
[launcher regression suite](../tests/python/test_research_agent.py) uses its real
launcher and fake coding clients/services for lifecycle checks. Physics-only
imports are deferred so these checks can run without USD/Rerun. Real launcher
dry-runs verify both client argument routes, prompt bytes, worktree planning and
forwarded flags; they do not start a coding client or physics world.
The [task 009 record](../output/research/sessions/2026-10-23-program-worker-dispatch/report.md)
states checks and unavailable prerequisites. Live research/review proof remains
mandatory in task 016.
