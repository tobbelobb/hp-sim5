---
name: cloud-research-client
description: Launch or resume an hp-sim5 Research Program in the current authenticated cloud chat. Use when asked to "Launch a Research Program" on a question; local users start through the launcher script.
---

The current cloud conversation is the manager. Use the existing checkout and
this chat's authenticated subagents; no additional login, API key or coding CLI.
All commands below run from the repository root.

## Open or resume

For the human's supplied question:

```bash
./hp-sim5-research-program --cloud --question 'the human research question'
# Resume an existing program instead:
./hp-sim5-research-program --cloud --program PROGRAM_ID
```

Keep the returned program ID and `attachment.connection`. The response includes
`manager_instructions`: follow that shared manager role contract. The local
launcher injects the same canonical instructions into its manager client.
This skill supplies the cloud-specific operation; no entry prompt, supplement
or operating guide needs to be loaded separately.

Call `tool CONNECTION capabilities` to discover arguments and `tool CONNECTION
runtime_check` before allocating experiments. If setup is incomplete, read
[setup](references/setup.md) and repair it before dispatching another attempt.
The explicit cloud entry loads the skill's `scripts/env.sh` automatically.
Ordinary development commands need to source it in their own shell.

## Triage a mission prompt

Before commissioning new work, call `prompt_portfolio` and compare the
registered missions in `research/prompts/` with the current evidence,
unresolved decision, tool readiness and bounded cost. Reject prompts likely to
repeat reviewed work. When one wins, load it with `prompt_id` and
`include_text:true`, pass the exact text to the worker, and record that
`prompt_id` on `route_plan`. If none fits, state why a custom assignment is
better. Do not put Manager, Reviewer or other role plumbing back in the mission
portfolio.

## Dispatch workers and reviewers

The controller and existing research launcher own assignments, deadlines,
physics services, fixed submissions and admission. `worker_start`/`run_once`
and `review_start` queue a model request; they do not invoke another Codex CLI.
For each request:

1. List pending requests:

   ```bash
   .venv/bin/python .agents/skills/cloud-research-client/scripts/cloud_client_bridge.py list
   ```

   Confirm its session belongs to an owned live attempt/review. Retained queue
   files after host loss are historical; do not redispatch them automatically.
2. Read the exact queued prompt and instructions at the returned paths. Spawn
   an actual authenticated subagent (`collaboration.spawn_agent` in Codex) with
   `fork_turns="none"`. Supply those files' contents once, the requested
   `final_output` path and, for workers, `tool_connection_path`. The actor must
   write its own real final response to `final_output`. Do not pass this manager
   skill or inherit the manager conversation into its context.
3. Give workers the file-relay commands below. Reviewers receive no physics
   tool relay; they inspect their fixed submission.
4. Bind the actual returned actor ID:

   ```bash
   .venv/bin/python .agents/skills/cloud-research-client/scripts/cloud_client_bridge.py bind REQUEST_ID --agent-id ACTUAL_AGENT_ID
   ```

5. Wait for that actor to finish and write its final response, then acknowledge:

   ```bash
   .venv/bin/python .agents/skills/cloud-research-client/scripts/cloud_client_bridge.py complete REQUEST_ID --agent-id ACTUAL_AGENT_ID
   ```

   Poll controller worker/review status and retain the shared submit/review/apply
   sequence. An actor's successful exit is not scientific acceptance. Preserve
   invalid review responses and commission a correction; never rewrite a verdict.

### Worker tool attachment

Include these commands with the worker handoff, substituting its returned
`tool_connection_path` for `TOOL_CONNECTION`:

```bash
.venv/bin/python .agents/skills/cloud-research-client/scripts/cloud_tool_bridge.py list TOOL_CONNECTION
.venv/bin/python .agents/skills/cloud-research-client/scripts/cloud_tool_bridge.py call TOOL_CONNECTION runtime_status
.venv/bin/python .agents/skills/cloud-research-client/scripts/cloud_tool_bridge.py call TOOL_CONNECTION research_preflight
# Assigned Viewer tools, when enabled:
.venv/bin/python .agents/skills/cloud-research-client/scripts/cloud_tool_bridge.py list TOOL_CONNECTION --server rerun
```

`call` takes `--params` with a JSON object and returns the real MCP result,
including `isError` and `structuredContent`. The launcher-owned relay exposes
only hp_sim5 and the assigned Rerun attachment. Actor commands need no sockets.
A timed-out mutation may have completed: inspect status before retrying it.
Do not print bearer credentials. The launcher owns attachment cleanup.

## Cancellation and completion

Request `worker_cancel`/`review_cancel`, interrupt the bound actor, verify that
it stopped, then acknowledge with `cloud_client_bridge.py cancelled REQUEST_ID
--agent-id ACTUAL_AGENT_ID`. Poll launcher cleanup separately and retain partial
evidence. The launcher/reviewer guard owns the recorded deadline; no bridge
acknowledgement means actor shutdown is unconfirmed.

Follow the returned shared manager instructions for result delivery and continued
iterations. Keep the parent manager chat active while brokering actors. Platform
permissions apply to all actors; review instructions do not create a separate
OS sandbox. The controller retains the permissions of the process that opened
it. For socket/Git denial or a stale controller, read
[permission recovery](references/permission-recovery.md).

For maintenance only, see [adapter boundary](references/boundary.md).
