# Research in the current cloud chat

The current authenticated chat is the research manager. It delegates bounded
investigations and separate reviews to real cloud subagents. It does not start a
second Codex/Claude CLI, request another login or require a model API key.

## Start

Prepare native dependencies with `bash scripts/cloud/install_research.sh` if
needed. Source scripts/cloud/env.sh in each shell when those toolchain settings
are needed. No research-client installation or PATH wrapper is required.

Ask the cloud chat to read research/prompts/research-manager.txt and this guide,
then give it the research question. It opens the controller with:

```bash
./hp-sim5-research-program --cloud --question 'Your research question'
# Or reopen in the same checkout:
./hp-sim5-research-program --cloud --program PROGRAM_ID
```

--cloud selects the client transport for the retained controller. Later tool
calls need only its returned CONNECTION path. Recognizable managed cloud runtimes
select cloud mode automatically; --cloud also works without those markers.
Interactive --codex/--claude manager launches are refused in cloud mode before
creating state. CLI model/provider options do not configure cloud subagents.
The original direct research-agent interface remains available locally.

## Dispatch a real actor

The manager uses ordinary program CLI tools; there is no manager MCP server.
The controller and existing research launcher still own assignments, deadlines,
physics services, fixed submissions and admission. When they need a model,
they invoke cloud_client_bridge.py directly. The request waits in
output/cloud-research/client-queue; it does not invoke an executable named codex.

For each worker or reviewer:

1. Allocate finite work, review, startup and cleanup capacity. Start it through
   worker_start/run_once or review_start.
2. Run `.venv/bin/python scripts/cloud/cloud_client_bridge.py list`. Confirm the
   queued session belongs to a currently owned live attempt/review. Retained
   requests after host loss are historical; never redispatch them automatically.
3. Read the exact prompt and instructions at the returned paths. Dispatch an
   actual authenticated subagent with collaboration.spawn_agent. Include the
   requested final_output path; the actor must write its real final response
   there. Workers use their private connection through scripts/research_attach.py.
   Never print bearer credentials. No nested coding CLI is allowed.
4. Bind the actual returned actor ID, wait for the actual actor to finish, then
   acknowledge completion:

```bash
.venv/bin/python scripts/cloud/cloud_client_bridge.py bind REQUEST_ID --agent-id ACTUAL_AGENT_ID
.venv/bin/python scripts/cloud/cloud_client_bridge.py complete REQUEST_ID --agent-id ACTUAL_AGENT_ID
```

Use a fresh context for reviewers. Preserve invalid responses and commission a
bounded correction instead of editing a reviewer's verdict. checks.asset_ids
names selected manifest.assets entries; manifest.json and candidate.diff are
metadata. Execution completion is not scientific acceptance: retain submit,
review and review_apply.

For cancellation, request worker_cancel/review_cancel, interrupt the bound cloud
actor, verify its stopped state, then acknowledge with the bridge's cancelled
operation. Poll launcher cleanup separately. Keep partial evidence. The bridge
has no independent ten-minute deadline: the worker launcher/reviewer guard owns
the recorded allocation. Its short cancellation acknowledgement window remains;
no acknowledgement means shutdown is unconfirmed.

## Save and continue

Keep the manager chat active through each bounded cycle. Before ending, stop owned
work, check cleanup, update reports and call `hp-sim5-research-program tool CONNECTION
save --params '{"commit":true}'`. Include additional small files with a files list.
The controller saves small results in research/programs and commits only that
archive; no push occurs. The host still needs Git write permission. Any Git
failure is reported with its diagnostic rather than treated as a saved commit.
Continue in a long chat or start a later chat from those committed results.
Moving an unfinished live program between environments is not a primary workflow.

The platform's permissions apply to all actors. Read-only review is an instruction
plus fixed-evidence integrity checks, not a new OS sandbox. This workflow makes no
promise of an independent daemon after the chat stops. The
[preserved pilot](../../research/cloud-client-pilot/2026-10-24/README.md) records
actual bounded work with the earlier prototype; the current transport still
needs a cloud trial. Historical failures remain evidence, not setup requirements.
