# Research in the current cloud chat

The current authenticated chat is the research manager. It delegates bounded
investigations and separate reviews to real cloud subagents. It does not start a
second Codex/Claude CLI, request another login or require a model API key.

## Start

Prepare dependencies with `bash scripts/cloud/install_research.sh`. This installs
Python packages (including usd-core), locked npm dependencies, Rust with LLVM
tools, and the native/firmware builds. Saving these scripts in Git does not
install them in an existing cloud environment; configure its setup step or run
the installer there. No research-client installation or PATH wrapper is required.

Use the cloud entry prompt:

```text
Read research/prompts/research-manager.txt and manage this question: [question].
```

That prompt opens the controller with:

```bash
./hp-sim5-research-program --cloud --question 'Your research question'
# Or reopen in the same checkout:
./hp-sim5-research-program --cloud --program PROGRAM_ID
```

--cloud selects the client transport for the retained controller. Later tool
calls need only its returned CONNECTION path. The controller passes the mode to
its children through HP_SIM5_RESEARCH_CLIENT. There is no automatic cloud
detection or dependency on Codex-home paths, platform environment markers,
available skills or a particular sandbox. --cloud and local --codex/--claude
are mutually exclusive. CLI model/provider options do not configure cloud actors.
The original direct research-agent interface remains available locally.

The cloud program entry and each cloud worker automatically source
scripts/cloud/env.sh. New workers reload it even when their controller predates
installation. Ordinary development commands still need to source it themselves.
The program helper uses system Python when .venv is absent; opening a durable
program alone does not establish that physics dependencies are installed. This
Python helper/controller is not a separate Codex manager session.

Before allocating experiments, run:

```bash
./hp-sim5-research-program runtime-check --cloud
# From an open program, the same check in its controller context:
./hp-sim5-research-program tool CONNECTION runtime_check
```

The check imports dependencies, loads the native addon and probes local sockets
without starting physics or moving a world. Cloud dispatch repeats it before
charging an attempt. Repair setup failures before commissioning retries.
For a stale native addon with dependencies already installed, source env.sh and
run `npm run build:native -- --no-pgo`. headless-js still uses the Rust native
engine; it is not a fallback that avoids the native build.

Record `execution: {"machine":"hp3"}` (or hp4) in commission. It sets the actual
launcher machine; conflicting worker settings are rejected. Alternatively pass
`settings: {"machine":"hp3"}` to worker_start/run_once. Cloud workers require an
explicit machine rather than silently defaulting to HP4.

If launcher_sockets is blocked with EPERM, the launcher cannot start its own
supervisor in that command context. Dependency installation, sourcing env.sh,
and another child process cannot grant socket permission. Run the launcher in a
context that allows local sockets or change the environment's command policy.
Supervised start_autocal/start_collection helps when only the model's commands
are restricted and its attached supervisor is already running in an allowed
context. It cannot rescue a supervisor whose own socket creation is denied.
Run research_preflight on that attached supervisor before collection.

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
   actual authenticated subagent using this chat's subagent mechanism
   (collaboration.spawn_agent in Codex). Include the
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
