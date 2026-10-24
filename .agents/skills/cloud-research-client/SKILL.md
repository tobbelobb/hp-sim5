---
name: cloud-research-client
description: Operate hp-sim5 research programs in an authenticated cloud chat without another login or API key, using real cloud subagents for bounded worker and reviewer requests.
---

Read scripts/cloud/RESEARCH.md before using this prototype.
Source scripts/cloud/research-env.sh in each shell. Use the existing checkout and repository manager CLI/MCP. Do not create a generic worktree. A parallel research pool's deliberate worker worktrees are a separate feature requirement.

Use only the authentication already supplied by this cloud chat. Never request an API key or separate login. Do not invoke a second standalone Codex or Claude CLI. The research shell's codex executable is explicitly the cloud-session batch adapter.

For this mode, delegate each verified pending worker/reviewer request to an actual authenticated cloud subagent through collaboration.spawn_agent. The controller and existing hp-sim5-research-agent launcher continue owning jobs and physics. Use an active parent chat as the broker. No synthetic child output or fabricated CLI transcript may count as a model response.

1. Open/resume a program and inspect its capabilities. Allocate explicit finite work and review capacity with startup margin.
2. Let its normal worker_start/run_once/review_start create the client request. List requests with `.venv/bin/python scripts/cloud/cloud_client_bridge.py list`.
3. Confirm the request's session belongs to an owned live attempt/review in the current controller generation. Never redispatch retained queue files after snapshot/host loss without explicit recovery.
4. Dispatch a real cloud subagent with the exact queued prompt and instructions. Workers attach to the launcher's private connection through scripts/research_attach.py. Never print runtime bearer credentials. No nested coding CLI or additional login.
5. Bind the actual returned agent ID using the bridge bind operation. Use a fresh context for separate reviewers. State read-only review intent, but do not claim OS read-only isolation; the platform permission profile applies.
6. Wait for actual actor completion and its real final-output file. Then call bridge complete. Poll actual worker/review status and keep the official fixed-submission/admission guards. A zero client exit is not scientific acceptance.
7. For review JSON, checks.asset_ids must name only manifest.assets entries. manifest.json and candidate.diff are metadata, not asset IDs. Never repair a reviewer's verdict yourself. Preserve a rejected review and commission a bounded correction.
8. For cancellation, request the controller stop, interrupt the actual bound cloud agent, verify its interrupted/completed state, acknowledge bridge cancelled, and poll launcher cleanup. Keep partial evidence and distinguish model stop from native cleanup. If a cloud actor fails, cancel and retain that failure; do not synthesize successful output.
9. Before ending, stop owned work, retain/export memory and regenerate the session catalog. Never claim physics checkpoints or cloud actor sessions were restored by reopening graph memory.

The broker requires the current cloud chat to have authenticated subagent tools and remain active while work is outstanding. It is not an unattended background model daemon. CLI model/sandbox overrides do not configure cloud actor sessions. The prototype is an opt-in versioned client transport, not a released first-class repository backend or task 016 completion.
