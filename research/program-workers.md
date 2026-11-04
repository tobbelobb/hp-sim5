# Research workers

Managers dispatch through the bound program controller:

| Operation | Purpose |
| --- | --- |
| `worker_start` / `run_once` | Start a funded assignment |
| `worker_status` | Observe progress, clarification and cleanup |
| `worker_answer` | Answer the pending question by attempt ID and question ID |
| `worker_cancel` | Request cancellation; poll until cleanup completes |

Use `capabilities` for arguments. The existing launcher owns physics and tools;
an exited worker still needs separate evidence review.

## Clarification

Workers ask the manager, not the human. When status returns `manager_question`,
call `worker_answer` with `attempt_id`, `question_id` and `answer`. Resolve it
within the brief and the selected human communication policy. Waiting consumes
the worker's existing allocation.

For direct Python dispatch, mailbox format, artifacts or ownership diagnostics,
read the [dispatcher reference](program-worker-reference.md). For manager
operations, see [the tool guide](program-manager.md).
