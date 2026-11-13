# Platform adapters are instructions, not another agent runtime

The same program CLI and roles work wherever an authenticated agent can read the
checkout and run Python. The current conversation is the manager. Hand it the CLI's
returned instructions and task; use the platform to start actual researchers and
fresh reviewers. Program membership records are not proof an actor is running.

## T3 Code

Use the current thread for the manager. Prepare independent handoffs with
`hp-sim5-research-program --resume PROGRAM_ID --agent QUESTION`. Use native
subagents for suitable same-provider work. Use T3 delegation for another provider,
settings, background work, or an isolated workspace. Follow the live T3 tool's
returned lifecycle instructions. Do not launch nested Codex or Claude CLIs.

For code-writing work, have T3 create and bind a worktree and prepare dependencies.
A worktree starts from committed source; commit required implementation first, or
use the platform's supported dependent-work base. Do not try to construct a child's
worktree in its prompt. Once the child is in its workspace, it binds the prepared
investigation before writing evidence:

```bash
./hp-sim5-research-agent --resume INVESTIGATION_ID --workspace
```

This changes the initially reserved artifact path to the same relative path in the
child's checkout. It refuses to relocate existing evidence. Return the actual
artifact directory and commit to the manager; integrate useful evidence with code.
The manager owns the shared agenda and topic syntheses. The standalone launcher's
`--worktree` is for terminal use; T3 already owns worktrees for its children.

## Codex Cloud

Use the current cloud conversation and authenticated subagents. No extra Codex CLI,
model login, API key, relay queue for model work, or cloud-specific manager process
is involved. Workers receive the exact question, short role, ID, and artifact path;
pass a fresh context, not the manager history. Parallel workers can each start their
own runtime. Keep concurrent code changes in isolated workspaces when the platform
supports them; otherwise divide files carefully or serialize writes.

The standard library suffices for programs and saved-evidence research. Simulation
needs the project's Python, Node, native Rust, and RRF builds. The optional Linux
x86_64 setup is:

```bash
bash research-program/cloud/install.sh
source research-program/cloud/env.sh
.venv/bin/python scripts/research_runtime_check.py
```

Setup downloads dependencies and needs network access. Source env.sh before starting
the runtime supervisor; it inherits that shell's tool paths. Include that source
command in simulation workers' tasks: fresh contexts do not inherit the manager's
shell. If the native build is stale, run `npm run build:native` in the sourced
shell or let world configuration
rebuild it. If a supervisor started without the toolchain, stop it and restart it
from a sourced shell. Socket restrictions apply to the supervisor's
actual launch context. The file tool transport lets a socket-denied worker call a
supervisor launched in a permitted context; it cannot grant socket privileges to a
supervisor that is itself forbidden to bind sockets. Inspect a failed readiness
check before dispatching more work. Browser work additionally needs Chromium;
headless numerical research does not.

Hosted workspaces are disposable. Commit useful small evidence and upload large
recordings to durable storage before the workspace goes away. Credentials and live
connections cannot be resumed on another host. Reopen the committed program by its
directory; use `--agent-id` to reconstruct member metadata.

## Anthropic infrastructure

The same existing-chat handoffs work with Claude Managed Agents that have a checkout,
shell access, and the required dependencies. Set the short manager role as the agent's
system instructions and supply the returned program task as its message. Use that
platform's supported multi-agent/delegation mechanism; keep its session IDs in the
platform. The program CLI and file tools need no Anthropic SDK or model credentials.

For Claude Agent SDK deployments, include project settings/skills (`setting_sources`
in Python or `settingSources` in TypeScript) and the tools required for the task.
The .claude/skills links expose the same portable skills used by Codex. If a managed
service uploads skills instead of discovering repository directories, upload
.agents/skills/research-program and .agents/skills/research-corpus, and include the
referenced research-program directory in the environment. Use a self-hosted sandbox
when native toolchain or background-service requirements exceed the managed sandbox.
This repository supplies a filesystem/shell contract, not an API provisioner.
Managed Anthropic execution has not been exercised by the local fixture tests.

Official references: [Managed Agents](https://platform.claude.com/docs/en/managed-agents/overview)
and [Agent SDK skills](https://platform.claude.com/docs/en/agent-sdk/skills).

## On-demand experiment tools, on every platform

```bash
./hp-sim5-research-agent --resume INVESTIGATION_ID --start-runtime
# Copy the returned tool_connection path into TOOL_CONNECTION below.
.venv/bin/python scripts/research_tools.py list TOOL_CONNECTION
.venv/bin/python scripts/research_tools.py call TOOL_CONNECTION runtime_status
.venv/bin/python scripts/research_tools.py call TOOL_CONNECTION configure_session --params '{"configuration":{"machine":"hp3"}}'
.venv/bin/python scripts/research_tools.py call TOOL_CONNECTION research_preflight
./hp-sim5-research-agent --resume INVESTIGATION_ID --runtime-status
./hp-sim5-research-agent --resume INVESTIGATION_ID --stop-runtime
```

The supervisor initially has no world. configure_session creates one; tools list
shows the exact supported arguments and units. Add `--viewer headless` to runtime
startup when Rerun inspection is useful. Tools return actual MCP results including
isError and structuredContent. A failed tool call exits nonzero while retaining its
JSON response on stdout. Long jobs return IDs; inspect and explicitly cancel
jobs as needed. File calls default to 120 seconds; use --timeout up to 14500 only
for a tool that needs it. After a timeout, outcome can be unknown: inspect status
before retrying a mutation. Separate investigations have separate descriptors,
worlds, job IDs, and artifact paths. Do not point two researchers at the same ID.

The actor or manager stops its detached runtime before finishing or after platform
cancellation. Stop requests finalize partial evidence; inspect runtime status until
shutdown is confirmed. Optional --wall-deadline UNIX_SECONDS bounds a runtime that
could outlive its actor. A stopped runtime can be started again against the same
investigation, with a fresh world and retained previous evidence.
