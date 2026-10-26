---
name: cloud-research-client
description: Operate hp-sim5 research programs in an authenticated cloud chat without another login or API key, using real cloud subagents for bounded worker and reviewer requests.
---

Read scripts/cloud/RESEARCH.md. Open with `hp-sim5-research-program --cloud --question TEXT` or `--cloud --program ID`. Use the existing checkout and ordinary program CLI. Source scripts/cloud/env.sh when the installed toolchain needs it. No manager MCP server or client installation is needed. Do not create a generic worktree.

Use only the authentication already supplied by this cloud chat. Never request an API key or separate login. Do not invoke a second standalone Codex or Claude CLI or shadow codex on PATH. The launcher/reviewer invokes the cloud queue bridge directly. The current authenticated conversation is the manager.

For this mode, delegate each verified pending worker/reviewer request to an actual authenticated cloud subagent through collaboration.spawn_agent. The controller and existing hp-sim5-research-agent launcher continue owning jobs and physics. Use an active parent chat as the broker. No synthetic child output or fabricated CLI transcript may count as a model response.

1. Open/resume a program and inspect its capabilities. Allocate explicit finite work and review capacity with startup margin.
2. Let its normal worker_start/run_once/review_start create the client request. List requests with `.venv/bin/python scripts/cloud/cloud_client_bridge.py list`.
3. Confirm the request's session belongs to an owned live attempt/review in the current controller generation. Never redispatch retained queue files after snapshot/host loss without explicit recovery.
4. Dispatch a real cloud subagent with the exact queued prompt and instructions. Workers attach to the launcher's private connection through scripts/research_attach.py. Never print runtime bearer credentials. No nested coding CLI or additional login.
5. Bind the actual returned agent ID using the bridge bind operation. Use a fresh context for separate reviewers. State read-only review intent, but do not claim OS read-only isolation; the platform permission profile applies.
6. Wait for actual actor completion and its real final-output file. Then call bridge complete. Poll actual worker/review status and keep the official fixed-submission/admission guards. A zero client exit is not scientific acceptance.
7. For review JSON, checks.asset_ids must name only manifest.assets entries. manifest.json and candidate.diff are metadata, not asset IDs. Never repair a reviewer's verdict yourself. Preserve a rejected review and commission a bounded correction.
8. For cancellation, request the controller stop, interrupt the actual bound cloud agent, verify its interrupted/completed state, acknowledge bridge cancelled, and poll launcher cleanup. Keep partial evidence and distinguish model stop from native cleanup. If a cloud actor fails, cancel and retain that failure; do not synthesize successful output.
9. Before ending a useful iteration, stop owned work, update reports, regenerate the session catalog and run `hp-sim5-research-program save PROGRAM_ID --commit`. Add needed analysis/input files with --file and inspect reported omissions. This commits only research results and never pushes. Never claim physics checkpoints or cloud actors were restored by reopening graph memory.

Keep the current chat active through bounded cycles. CLI model/sandbox overrides do not configure cloud actors. The controller owns deadlines; the bridge adds no independent ten-minute timeout. Long chats with subagents and committed results are the target; independent services and moving unfinished programs between machines are not. Live trials of the current version remain distinct from fixture tests and the preserved earlier pilot.
