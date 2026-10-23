# 011: Launch and operate the manager from chat

Outcome: a user with only a question can enter a distinct manager conversation
and operate the durable program through tools. The manager delegates through 009;
it is not merely the worker with a longer research prompt. Follow the required
chat routes in [MANAGER.md](MANAGER.md).

Provide a manager tool surface over the existing store, graph, dispatch and review
APIs, via CLI and a small MCP adapter. Reuse supported coding-client setup/auth
and attachment conventions from scripts/research_agent.py where useful. Add a
manager entry prompt and `hp-sim5-research-program --codex/--claude` plus service
attachment support without starting a continuing experiment world for the manager.
Avoid duplicating client command construction; factor shared helpers only as
needed and preserve existing researcher entry behavior.

An already-running repository-connected chat must also be able to create/open a
program, start an owned controller when necessary, dispatch and poll a worker,
inspect graph/review state, steer and stop through the CLI tools it already has.
No prerequisite manual terminal, new web UI, installation of a new chat product
or dynamic MCP registration can be mandatory for that route. MCP attachment is
an additional supported path for clients that offer it.

Done when:

- A documented question-only chat prompt creates/opens a program and exposes a
  manager overview, with actual program/session links and the next allocation.
- Manager tools cover status/graph queries, assignments/dispatch, reviews,
  steering and lifecycle controls as they become implemented. Unavailable later
  capabilities are explicit, not silent placeholders pretending to succeed.
- A second chat can attach/open the same program without starting duplicate
  workers or inheriting another program's control identity. Worker physics MCP
  is separate from manager orchestration tools.
- Protocol/CLI/client-command tests verify structured errors, nonblocking status
  and that the manager entry starts no physics. A real coding-chat smoke session
  is recorded when available; final live proof remains required in 016.

Update the quickstart and distinguish implemented manager commands from future
design examples. Keep ordinary single-agent research prompting usable.
