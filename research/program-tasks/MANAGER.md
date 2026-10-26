# Research manager architecture

Current direction, 2026-10-26. This supersedes the earlier local-only completion
contract. Historical task cards and pilot reports retain what was actually tried.
"Release" in those cards meant completing their feature acceptance checklist;
it does not describe another service or deployment the human must manage.

The human gives a research question to one coding chat. That conversation is the
manager. It chooses bounded investigations, delegates to research workers, asks
separate reviewers to inspect fixed evidence, updates what is known and decides
what to try next. Human steering, explanations and continuation stay in that chat.

## Keep the layers small

- The manager conversation owns reasoning, priorities and human interaction.
- The deterministic controller owns memory, assignments, deadlines and owned
  process cleanup. It does not run a separate reasoning loop.
- The existing hp-sim5-research-agent owns each physics/RRF supervisor and
  collection/recording lifecycle. Preserve its direct local human interface.
- Workers and reviewers use local installed clients or the current cloud chat's
  authenticated subagents. Cloud has no nested standalone coding CLI.
- A completed iteration saves small relevant results in Git as a separate step.

Use ordinary CLI calls from the manager. No manager MCP adapter, PATH disguise,
extra login, API key or Codex-home permission changes are part of cloud research.
Local users launch hp-sim5-research-program --codex/--claude; cloud users invoke
research/prompts/research-manager.txt in the existing chat. That prompt selects
--cloud. The controller inherits and forwards the explicit choice; no environment,
skill or sandbox heuristic selects the mode.
A long active cloud chat with many bounded subagent assignments and saved results
is the intended use case. An independent background service is not a target.

Start with one worker plus a separate reviewer. Parallel pooling and combined
patch integration already exist but are advanced tools; do not expand them until
the basic local and cloud cycles have been tested. Cross-machine import/recovery
is secondary. Existing hashes must still identify exactly what a reviewer saw;
relocation must not silently rewrite fixed evidence or preserve reviewed status
for changed material.

## Evidence and authority

Retain useful questions, alternatives, claims, evidence and decisions in SQLite;
keep datasets and recordings outside the graph. Distinguish simulation, synthetic,
analytic and hardware evidence. A successful process exit is not a scientific
finding. Separate review fixes evidence versions and does not automatically repeat
expensive experiments. Findings and code readiness remain distinct.

The manager chooses finite allocations with review, startup margin and cleanup.
The controller enforces owned deadlines and concurrency. Numerical estimates and
arbitrary shell spend must not be described as hard aggregate quotas. No transport
adds its own unrelated ten-minute timeout. Cloud cancellation must interrupt the
real actor and separately observe launcher cleanup. Parent platform permissions
apply; review instructions do not establish an OS read-only sandbox.

Do not automatically merge or push research patches. Save results under
research/programs/PROGRAM_ID and commit just those files; preserve unrelated work.
Physics delivery still requires npm run test:physics.

## Verify behavior

Test locally first: one question, worker, fixed submission, separate review,
support query and changed next decision; then bounded production-tool use,
controlled cancellation and a fresh manager conversation in the same checkout.
Check the original direct research-agent launcher independently.

Repeat that cycle in cloud using the existing authenticated conversation and real
subagents. No fixture transcript substitutes for actual model/tool use. The
preserved prototype pilot establishes a small successful cloud path, not that the
current version or every platform permission profile works. Keep live failures.
See research/program.md and scripts/cloud/RESEARCH.md for current commands.
