# Existing-login cloud client prototype

Use only the authentication already supplied by this cloud chat. No additional CLI login or model API key is part of this mode.

## Install from Git

From the repository root:

```bash
bash scripts/cloud/install_research.sh
source scripts/cloud/research-env.sh
```

This runs the standard cloud dependency installer and creates an opt-in client
wrapper outside the checkout. It leaves existing credentials and other project
skills unchanged. A modified wrapper is preserved. For a machine whose ordinary
dependencies are already prepared, `.venv/bin/python scripts/cloud/install_research_client.py`
installs just the wrapper.

The code, skill and setup instructions are versioned, so a new checkout can
rebuild this mode without an environment snapshot. Saving the managed environment
draft is separate from publishing its prepared filesystem. This chat's tools can
save that draft but expose no snapshot publication action. Do not assume a
Save/Publish button will appear in the chat.

Ask an authenticated cloud agent with subagent tools to read
`.agents/skills/cloud-research-client/SKILL.md` and manage the research question.
The agent performs the controller and dispatch steps below.

The existing research manager/controller still owns assignments, budgets, fixed submissions, admission and cancellation. The existing research launcher still owns each physics/RRF supervisor. A local batch-client compatibility adapter queues the model-client part for the authenticated parent chat's real collaboration tools. It does not invent coding-client output, run a substitute physics engine or accept producer claims.

Initialize each shell with `source scripts/cloud/research-env.sh`. The `codex` executable on this shell's PATH is explicitly the cloud-session adapter, not the standalone CLI. It supports batch worker/reviewer clients only. Do not run the dedicated interactive `--codex` manager route in this mode. Manage from this existing cloud chat with `hp-sim5-research-program` CLI/MCP.

For every queued request, the parent chat must inspect `cloud_client_bridge.py list`, dispatch a real authenticated cloud agent, bind its actual actor ID, wait for its actual completion and call `complete`. The actual cloud actor must write the real final output requested by the launcher/reviewer. A zero client exit is execution completion only. Keep the existing fixed-submission and separate-review admission steps.

Example bridge commands, replacing IDs with actual returned IDs:

```bash
.venv/bin/python scripts/cloud/cloud_client_bridge.py list
.venv/bin/python scripts/cloud/cloud_client_bridge.py show REQUEST_ID
# Use the cloud collaboration.spawn_agent tool with the queued prompt/instructions.
.venv/bin/python scripts/cloud/cloud_client_bridge.py bind REQUEST_ID --agent-id ACTUAL_CLOUD_AGENT
# Only after that actual agent finishes and writes its real final-message file:
.venv/bin/python scripts/cloud/cloud_client_bridge.py complete REQUEST_ID --agent-id ACTUAL_CLOUD_AGENT
```

Use a fresh context for separate reviewers and do not pass producer conclusions as instructions. In review JSON, checks.asset_ids may contain only manifest.assets IDs. Metadata names are not evidence IDs.

For cancellation, first use the manager worker_cancel/review_cancel operation. The bridge records cancel_requested. Interrupt the actual bound cloud actor through the collaboration tool and confirm its stopped state, then acknowledge with `cloud_client_bridge.py cancelled REQUEST_ID --agent-id ACTOR`. Do not assert model cleanup from launcher cleanup alone. The bridge records cancel_unconfirmed if no actor acknowledgement arrives. Keep the parent active while bounded work is running.

This prototype needs an active cloud chat capable of authenticated subagent dispatch. A sleeping/closed chat is not a daemon broker. CLI model/sandbox options do not reconfigure the cloud session. The cloud platform permission profile applies; reviewer read-only behavior is instructed and fixed-evidence hash guards remain active, but no additional OS read-only sandbox is created. Same-UID shared files are cooperative, not adversarial isolation.

No permanent Codex-home write grant is needed by this transport because it does not start another standalone Codex CLI. The earlier filesystem-grant experiment was a diagnostic, not a dependency. Environment setup cannot change the platform's filesystem policy. Avoid file-only permission grants in this environment because they exposed a sandbox mount bug.

This versioned prototype is an opt-in client transport. It is not a first-class backend in the research program. Do not mark task 016 released based on this pilot alone. Preserve the actual tests, scope and retained failures.

## Verified pilot, 2026-10-24

The [preserved pilot](../../research/cloud-client-pilot/2026-10-24/README.md)
contains fixed submissions, review responses, admissions, telemetry, recordings
and the program audit export.

Program `program-387d94c6067647b6964aa267122d7989` retained two actual managed investigations with separate authenticated cloud reviewers and two `reviewed/keep` admissions. The production worker ran HP3 experiment `8f977de942c64d7391f63c3198ba0c9d`, 100 steps at 2 ms, with 101 telemetry and RRD samples and a readable blueprint. The other worker used saved evidence only. Both were report-only engineering findings, not calibration improvements.

The fixed review guard rejected one real review for citing candidate.diff as an evidence asset. It remained failed. A separately funded reviewer correction passed through the unchanged guard. Retry/funding guards required ending the old allocation before the new bounded review handoff. Do not weaken guards or silently alter actor output.

A third real cloud worker was deliberately cancelled with zero numerical work. Its actual cloud actor reached interrupted state, the controller recorded cancelled, and the real launcher returned 130 with launcher_finally_completed. A fresh manager host reconstructed both reviewed admissions, three worker-history entries and zero active workers. Pause/resume worked. No world restoration or unattended broker continuation is claimed.

The inherited standalone CLI login still returned HTTP 401. That is a separate client-authentication boundary, not a Git/network problem. This cloud transport avoids the additional CLI entirely. No API-key requirement was saved and no extra login was performed.

On a new task, recover the existing program and verify ownership first. Do not treat saved queue files or historical actor IDs as live requests. For an actor failure or usage limit, preserve the actual platform failure, cancel the attempt and acknowledge the stopped actor; do not create successful final output.
