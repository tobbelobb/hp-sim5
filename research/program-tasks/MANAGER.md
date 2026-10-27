# Research manager architecture

Current direction, 2026-10-27. This supersedes the earlier local-only completion
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
- Workers and reviewers use the selected client transport. The local path uses
  installed clients and their existing authentication.
- A completed iteration saves small relevant results in Git as a separate step.

Use ordinary CLI calls from the manager. Local users launch
hp-sim5-research-program --codex/--claude. The optional platform adapter and its
operating rules live in [scripts/cloud/RESEARCH.md](../../scripts/cloud/RESEARCH.md);
its [boundary map](../../scripts/cloud/BOUNDARY.md) identifies removable code,
prompts and the explicit transport selection seam. The shared controller retains
that choice through recovery.

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
arbitrary shell spend must not be described as hard aggregate quotas.
Transport-specific deadline and actor-cancellation rules belong to the adapter.
Review instructions do not establish an OS read-only sandbox.

Do not automatically merge or push research patches. Save results under
research/programs/PROGRAM_ID and commit just those files; preserve unrelated work.
Physics delivery still requires npm run test:physics.

## Verify behavior

Test locally first: one question, worker, fixed submission, separate review,
support query and changed next decision; then bounded production-tool use,
controlled cancellation and a fresh manager conversation in the same checkout.
Check the original direct research-agent launcher independently.

Repeat the same cycle for each supported adapter with its actual authenticated
actors. No fixture transcript substitutes for model/tool use. See
[the visual audit](../reports/research-program-cloud.html) for the current cloud
path's measured scope and remaining verification gap.
