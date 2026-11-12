# Research workers

`program.workers.WorkerDispatcher` starts `hp-sim5-research-agent` with a question,
session directory and execution settings. An optional `settings.machine` (a
`machines/catalog.json` name or nickname) is validated and written into the
worker's prompt and contract; the worker builds that world with `configure_session`. The existing
launcher owns physics and runtime jobs. The program records commands, revisions,
process identities, execution results and cleanup status.

A completed process has an unreviewed result. Status includes a pending manager
question when the worker needs a decision; `worker_answer` takes its attempt ID,
question ID and answer. Cancellation targets only owned, verified processes.
Recovery preserves partial evidence and requires a handoff for unknown ownership.

The default is one worker. Parallel workers need separate worktrees and worlds;
shared dependency/build changes require an exclusive reservation. Shared-UID
worktrees organize work but do not isolate hostile code.
