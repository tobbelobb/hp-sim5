# Research manager tools

Start with the [question-first guide](program.md). The current coding chat is the
manager; ordinary CLI tools connect it to a small persistent controller. Local
`--codex` or `--claude` opens that manager chat. Cloud `--cloud` uses the existing
chat's authenticated subagents and never opens another coding CLI.

## Open a program

```bash
./hp-sim5-research-program --codex --question 'Your question'
./hp-sim5-research-program --claude --program PROGRAM_ID
# The cloud entry prompt uses this internal command:
./hp-sim5-research-program --cloud --question 'Your question'
```

The response includes program_id, attachment.connection and the overview. Opening
dispatches no workers. A new program has no allocated investigation. The host
starts automatically and retains child ownership across separate CLI calls.
Each store has one host and one client transport. A different transport requires
another --store directory; recovery retains the recorded mode even after the
old host stops.

The attachment, capabilities and overview.manager report client_transport.
Keep the bound mode during recovery. Local command sandboxes and hidden PIDs
do not imply cloud execution. The entry route selects the mode; it is never
guessed from Codex-specific environment variables or available skills.
Clients establish availability through the file
protocol; an invisible host PID is not a dead controller. A held controller lock
prevents replacement, while host-generation checks reject stale attachments.

## Call tools

Use the returned attachment.connection path as CONNECTION:

```bash
./hp-sim5-research-program tool CONNECTION capabilities
./hp-sim5-research-program tool CONNECTION overview
./hp-sim5-research-program tool CONNECTION query --params '{"query":"open_questions"}'
./hp-sim5-research-program tool CONNECTION OPERATION --params 'JSON object'
```

Capabilities lists actual parameters. Successful calls return ok:true; failed
calls return an error and exit 2. Tool calls do not wait for a worker to finish.
Poll the returned attempt/review IDs in later calls. A timed-out mutation has an
unknown outcome: inspect state before retrying it.

| Purpose | Operations |
| --- | --- |
| Strategy and memory | overview, query, question, approach, route_plan, route_decide, commission, assignment, decision, steering |
| Worker | worker_start, worker_status, worker_cancel, worker_sessions |
| Evidence and review | artifact, claim, derive, relate, submit, submission_verify, review_start, review_status, review_cancel, review_apply, strategy_result |
| Budgets and stopping | allocate, run_once, run_within_allocation, runtime_status, worker_usage, fund_review, pause, resume, stop_program, recover, shutdown |
| Save results | save (files list, commit boolean) |

The manager normally handles these arguments. Read [strategy](program-strategy.md),
[review](program-review.md) or [controller](program-controller.md) when needed.
Choose one finite investigation plus review and startup margin. Keep numerical
workload, wall time and cleanup limits explicit. Arbitrary shell spend and
movement are not hard aggregate quotas. The default is one worker.

## Manager CLI, worker MCP

The manager uses a private file queue, so it needs no MCP SDK or local socket.
Its former MCP attachment route and --serve option have been removed. Local
manager clients disable inherited research MCP entries and use CLI tools.
Workers retain the existing physics supervisor and its MCP attachment. Cloud
workers can call scripts/research_attach.py through their command tools; they
must use the private connection created for their own attempt. No new physics
runtime is introduced.

## Results and lifecycle

An exited worker is unreviewed. Submit a fixed claim and selected evidence, run
a separate reviewer, and apply a valid review before treating the finding as
reviewed. Review does not normally repeat expensive experiments. Physics changes
still need npm run test:physics. Accepted findings and code readiness are distinct.

Pause holds new dispatch while existing work continues. Stop requests owned
cleanup; poll status to observe completion. Shutdown also retires this program's
controller. Closing a local manager client alone does not stop the host.
Cloud cancellation additionally needs the parent to interrupt its actual cloud
actor and acknowledge that interruption to the bridge.

After cleanup, call the controller as a separate step:

```bash
./hp-sim5-research-program tool CONNECTION save --params '{"commit":true}'
```

The host performs the scoped Git operation in its own execution context, so a
local manager's read-only .git mount does not prevent the commit. Host permissions
still apply; Git errors retain their diagnostics. The terminal convenience command
`hp-sim5-research-program save PROGRAM_ID --commit` uses the same host operation.
See the [save policy](program.md#save-research-in-git). Durable IDs and notes
help a later conversation in the same checkout; they do not restore physics or
unowned processes. Historical PIDs never authorize signals.

Parallel configuration and combined patch operations are advanced optional tools;
see [pool/controller](program-controller.md) and [integration](program-integration.md).
They are not required for the first useful program. No automatic merge or push
is part of the research controller.
