# Operate a research program from chat

An existing repository-connected Codex or Claude Code chat can manage a program
with its shell tool. Give it your question and this prompt:

```text
Read research/prompts/research-manager.txt and manage this question:
Can encoder-only HP3 calibration improve from the same measurements?
```

The agent runs `./hp-sim5-research-program --question 'Your question'`. This
creates durable memory and starts a local tool host automatically. The JSON
result includes the program ID, program-bound connection path, manager session,
overview paths and next allocation. The overview includes a cited strategy brief,
also written to brief.md beside the full overview. A new question starts unallocated. The
manager chooses and records an assignment before dispatching it. No scientific
result exists merely because the program opened.

Tasks 007 through 015 are implemented. The full manager release remains blocked
at [task 016](program-tasks/016-live-manager-release.md). The actual
[live pilot](../output/research/sessions/2026-10-24-live-manager-release/report.md)
verified CLI dispatch, graph/lifecycle commands and a fresh host, but could not
complete production science or coding-client review in this environment.

You can then prompt the agent naturally:

| Say | Manager action |
| --- | --- |
| Continue within the current allocation. | Inspect results and reservations, then select bounded work. |
| What do we know, and what is unknown? | Query scoped support and open questions, with citations. |
| Why pursue that approach? | Explain the route rationale, rivals, cost and discriminating observation. |
| Try this idea using the same measurements. | Record steering and revise pending work while retaining evidence. |
| Pause new work. | Hold dispatch while running work keeps its deadlines. |
| Stop that worker, or stop the program. | Cancel owned work and report observed or unknown cleanup. |
| Resume program PROGRAM_ID. | Reopen durable state and reconcile ownership before dispatch. |

The agent handles tool arguments. Keep the program ID when moving to another chat.

Open the same program in another chat with
`./hp-sim5-research-program --program PROGRAM_ID`. Both chats receive the same
controller identity while it is alive. Opening a program dispatches no workers.
The default dispatcher allows one active worker across the store. A separate
program gets its own controller identity. Use `--store PATH` before the command
to choose another local store.

## Use the CLI tools

Replace `CONNECTION` with the returned attachment.connection path. CLI requests
and responses are JSON. A failed call has `ok: false` and an error code/message,
and exits with code 2. Invalid calls do not silently succeed.

```bash
./hp-sim5-research-program tool CONNECTION capabilities
./hp-sim5-research-program tool CONNECTION overview
./hp-sim5-research-program tool CONNECTION query --params '{"query":"open_questions"}'
./hp-sim5-research-program tool CONNECTION approach --params '{"question_id":"QUESTION_ID","title":"Inspect saved candidates","reason":"Resolve membership before new fitting","record_id":"saved-route"}'
./hp-sim5-research-program tool CONNECTION assignment --params '{"approach_id":"saved-route","objective":"Compare frozen memberships with their original baselines","allocation":{"wall_minutes":5,"fresh_fits":0},"record_id":"compare-membership"}'
./hp-sim5-research-program tool CONNECTION transition --params '{"record_id":"compare-membership","target":"ready","reason":"Bounded saved-evidence comparison"}'
./hp-sim5-research-program tool CONNECTION worker_start --params '{"assignment_id":"compare-membership","settings":{"backend":"codex","viewer":"none","worktree":true}}'
./hp-sim5-research-program tool CONNECTION worker_status --params '{"attempt_id":"RETURNED_ATTEMPT_ID"}'
```

The example's zero fresh fits is specific to inspecting saved evidence. Choose
finite workload limits for the actual question. The [finite controller](program-controller.md) reserves tasks, worker wall time,
separate review capacity and cleanup before dispatch. Numerical counters remain
contracts unless the supported production tool enforces them.
The CLI never waits for the worker investigation to finish. Later calls reach
the same host, which retains the actual dispatcher/reviewer child processes and
polls their exits. Worker session links point to the existing launcher artifacts.

`capabilities` lists the actual Python parameter names for every operation. These
operations are implemented:

| Work | Operations |
| --- | --- |
| Program and graph | overview/status, capabilities, query, question, approach, assignment, artifact, claim, derive, relate, assess, review_event, transition |
| Worker | worker_start, worker_status, worker_cancel, worker_sessions |
| Fixed separate review | submit, submission_verify, submission_invalidate, review_plan, review_start, review_status, review_cancel, review_apply |
| Strategy | strategy, route_plan, route_decide, commission, strategy_result |
| Human direction | steering, decision |
| Runtime | allocate, run_once, run_within_allocation, runtime_status, worker_usage, fund_review, recover |
| Combined candidates | integration_prepare, integration_check, integration_status, integration_release, integration_recover |
| Lifecycle | pause, resume, cancel_attempt, stop_allocation, stop_program, stop, shutdown |

The [strategy guide](program-strategy.md) describes route plans, rival
discriminators, automatic reviewed outcomes, withdrawal review, futility and
accepted steering. `steering` also accepts `intent`, `links`, `provenance`,
`next_assignment_id` and `allocation`. Notes and hunches preserve provenance; an
explicit instruction can replace the next pending allocation without stopping
unrelated active work. `review_start` accepts the
same settings as `review_plan`; see [review contracts](program-review.md).
`query` retains scoped evidence and source links, as in the [graph guide](program-graph/README.md).

Pause prevents new worker/reviewer dispatch. Running work continues. Use
worker_cancel or review_cancel for an explicit cancellation. Stop prevents new
dispatch and requests cancellation of this controller's owned children. Poll
worker_status/review_status until cleanup completes. Shutdown also cancels owned
children and retires the program's tool controller after they exit; it does not
erase records. The per-store host exits after all opened program controllers
have shut down. Closing a chat alone leaves the host and workers available to
another chat. Cancellation of a production worker allows the existing launcher's
cleanup grace, currently 190 seconds by default.

## Open a dedicated manager chat or attach MCP

Use the existing installed client and its authentication:

```bash
./hp-sim5-research-program --codex --question 'Your question'
./hp-sim5-research-program --claude --program PROGRAM_ID
./hp-sim5-research-program --serve --program PROGRAM_ID
./hp-sim5-research-program --codex --question 'Your question' --dry-run
```

The dedicated entry shares command/auth helpers with the existing worker and
reviewer. It starts no continuing physics world or Viewer. Codex defaults to its
existing ChatGPT sign-in; no new model API or credentials are required. Client
arguments after `--` use the supported client's normal options. `--dry-run`
creates no store, files or processes. Ordinary single-agent investigation still
uses [research-program.txt](prompts/research-program.txt) or
`hp-sim5-research-agent` as before.

`--serve` prints attachment/session paths and leaves the file host running.
The manager session contains attachment.toml for Codex and attachment.json for
Claude Code, pointing to `scripts/research_program_mcp.py CONNECTION` over stdio.
The adapter offers program_overview, program_capabilities and program_call.
It proxies the same host used by CLI calls. Physics MCP remains a separate worker
service. The ordinary-chat CLI needs only Python's standard library; MCP needs
the repository's supported MCP SDK.

## Current boundaries and verification

The private file queue needs no socket. One host lock prevents duplicate hosts.
Every request checks the host generation, bound program and controller identity.
A stale attachment fails explicitly; reopen the program to obtain its current
attachment. Reopening after host loss does not adopt or signal old worker PIDs.
A fresh host reconciles leases, process birth identities and session/connection
records. Unverified work stays interrupted; a new bounded handoff must acknowledge
unknown physical state before dispatch resumes. No saved PID authorizes signals. Request timeout means the outcome may be unknown. Inspect durable
status before retrying a mutation. Same-UID callers can edit local files or use
the store APIs directly. This is a cooperative tool boundary, not adversarial
isolation.

Task 011 verifies fresh-process CLI calls, retained fake-worker ownership,
program-bound attachment sharing, graph/review errors, pause/cancel/stop behavior,
client argv and actual MCP SDK initialize/list/call handling. These are interface
checks, not live scientific or release evidence. In this command sandbox the
SDK's stock stdio loop cannot wake after thread work because its internal socket
send is denied. The recorded protocol check uses a diagnostic timer around the
unchanged SDK. The task 016 stock initialize attempt also timed out after eight
seconds. Actual dedicated Codex and separate reviewer startup reach a read-only
app-server error. The pilot fixed a configuration bug: Codex validates disabled
MCP transports, so manager/reviewer settings now replace entire legacy server
tables with valid disabled stdio entries. This avoids inherited HTTP conflicts.
Installed-client parser checks and focused tests passed. No new keys or
HOME/CODEX_HOME overrides were introduced.

[Combined candidate preparation and checks](program-integration.md) consume sealed
reviewed submissions at an explicit base. Their separate worktrees and polled
checks preserve the human checkout and scientific admissions. No research patch
is automatically merged or pushed. Pool configuration and its limits are
documented in [program-controller.md](program-controller.md).
`run_within_allocation` executes the
manager's finite selection of distinct assignments; it does not resume a prompt
in a restart loop.
Task 016 still requires the genuine client, worker, production-tool, review and
replanning pilot. The full manager remains incomplete until that live proof passes.
