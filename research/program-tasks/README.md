# Build the chat-driven research manager, one outcome at a time

The target is a manager between the human's coding chat and the existing
`hp-sim5-research-agent`: it maintains queryable knowledge and competing
approaches, dispatches investigations, reviews evidence, changes priorities and
recovers across conversations. [MANAGER.md](MANAGER.md) defines this release.
The [original design](../../research-program-design.md) is background; current
human steering and the release contract govern implementation.

In an ordinary repository-connected coding-agent chat, repeat:

```text
Read research/program-tasks/PROMPT.md and do one task
```

[tasks.json](tasks.json) owns order, dependency eligibility, budgets and status;
[progress.txt](progress.txt) is append-only history. A fresh agent in the same
checkout can continue from the preceding local commit. A fresh checkout must
first incorporate preceding task commits; the prompt itself does not publish
work. Each iteration implements one useful outcome, verifies it and commits
only its own changes. Push only when authorized by the human. Do not run the
next task inside the same invocation.

Tasks 001–006 are completed foundations. Tasks 007–016 are required manager
implementation, not optional proposals. Tasks 007–011 now supply storage, graph,
dispatch, review and [manager chat entry](../program-manager.md). Task 012 is next;
only cards with completed dependencies are eligible. Always use tasks.json for
current eligibility. The full manager remains incomplete until task 016 passes.
There is no separate proposal/promotion barrier between the implementation cards.
Implementation choices can evolve, but graph, manager/worker separation and chat
entry cannot be replaced by prompt-only equivalents and marked complete.

| Task | Concrete outcome |
| --- | --- |
| [001](001-evidence-first-review.md) | Done: evidence-first claim review |
| [002](002-short-decision-handoff.md) | Done: scoped decision handoffs |
| [003](003-one-next-question.md) | Done: one economical assignment from a broad question |
| [004](004-saved-evidence-pilot.md) | Done: broader consensus comparison and scoped negative decision |
| [006](006-read-ended-job-evidence.md) | Done: portable evidence and historical job status |
| [005](005-follow-explicit-dependencies.md) | Done: living Markdown agenda and dependency exercise |
| [007](007-durable-program-store.md) | Durable program state and usable create/overview CLI |
| [008](008-queryable-evidence-graph.md) | Typed evidence graph, queries, withdrawal and alternative support |
| [009](009-existing-worker-dispatch.md) | Dispatch/status/cancel through hp-sim5-research-agent |
| [010](010-reviewed-claim-admission.md) | Fixed submissions, separate review and controller-owned admission |
| [011](011-chat-manager-entry.md) | Manager chat launcher, CLI/MCP tools and existing-chat operation |
| [012](012-manager-strategy-and-steering.md) | Overview, approach selection/pruning, scouting and human steering |
| [013](013-bounded-controller-and-recovery.md) | Finite orchestration, budgets, interruption, stop and restart recovery |
| [014](014-parallel-approach-coordination.md) | Small isolated worker pool and coherent shared decisions |
| [015](015-tested-patch-integration.md) | Combined patch candidates, conflicts and relevant delivery checks |
| [016](016-live-manager-release.md) | Actual chat-to-worker/review/graph/replanning pilot and release docs |

The final gate must demonstrate actual coding-client interaction and production
worker tools. Fake children and graph fixtures are useful earlier verification,
not proof that the manager release is complete. Missing live prerequisites block
016 honestly. Until it passes, call the existing layer foundations/single-agent
workflow. After it passes, report `MANAGER_RELEASE_COMPLETE` with evidence.

## Cost and evidence rules

Development defaults allow up to 90 minutes, six fresh fits, two bounded native/
headless trials of at most 10,000 steps each and one simulated collection.
Task 016 allows up to 120 minutes for its live integration pilot. These are
ceilings, not targets or service-enforced quotas. Choose a lower concrete budget
before numerical work; use fixtures for store/controller correctness. Runtime
research programs have their own recorded allocations chosen for their question;
missing human caps do not prohibit necessary targeted experiments.

Use useful saved evidence first, then the cheapest probe that resolves the
specific gap. Use supported --speedup settings, normally --speedup 25 for a
simulated autocal CLI; headless native Rust already integrates as fast as possible.
Read [recording-cost guidance](../autocal-recording-performance.md) before choosing
capture/compute budgets. Keep collection sweeps/points, physics steps/simulated
time and wall time explicit. Preserve partial and unsuccessful trials.

Review the producer's exact claim, code/inputs/configuration and checks. Inspect
adequate evidence instead of automatically duplicating physical runs. A concrete
discrepancy, relevant change or new generalization can justify a fresh check;
state the uncertainty it resolves. Required delivery checks are additional to
exploratory budgets. Physics/motor/transport/dynamics changes require the real
`npm run test:physics` gate; extra substeps, startup or parity are no substitute.

The manager reuses the existing worker launcher, supervised jobs, worktrees,
recorders and client authentication. Its controller may orchestrate distinct
bounded assignments and separate reviews, as explicitly authorized here. It
must not create a second physics runtime or an outer prompt-restart loop to
continue one unfinished conversation. Do not start a world or Viewer merely to
edit these backlog files.

## What the existing work changes about this queue

The notes below retain the foundation phase's evidence and sequence. Current
eligibility and release scope are in tasks.json and MANAGER.md above.

- [Current autocal prompt](../prompts/autocal.txt): starts with recorded/synthetic
  work and uses collection when warranted. Keep that ordering.
- [Parameter-variation prompt](../prompts/autocal_parameter_variation_focus.txt):
  wider physical variation matters, but collection is expensive and physical
  ground truth is unavailable to a real calibration algorithm. Keep scope limits
  visible; do not turn every task into a world-variation matrix.
- [Existing reviewer](../prompts/reviewer.txt): its questions about relevance,
  reproduction, confounds and maintainability survived task 001, which fixed the
  assumption that the latest commit is necessarily the result under review.
- [Lane 04 integration](../autocal-ranking-lane04-integration.md): saved histories
  already contain useful noise metrics and support selection-only comparisons.
  Changes that affect the fitting trajectory still need appropriate replay.
- [Lane 02 integration](../autocal-filtering-lane02-integration.md): a supposed
  regression compared an individual fit with final consensus. Check attribution
  and provenance before scheduling more experiments.
- [Recording costs](../autocal-recording-performance.md): recording mode materially
  affects workload cost. For later experiments use `none` for numerical sweeps,
  `events` routinely, and detailed physics recording when the question needs it;
  the measured ratios are not general full-autocal speedups.
- [Archive implementation](../../src/python/hp_sim5_research/archive.py): the
  catalog already embeds whole short abstracts and preserves evidence links.
  Additional Markdown fields need no new catalog parser or database.
- [Tooling wish list](../prompts/researcher-tooling-wish-list.md): lost multi-hour
  runs are a real cost. Its requests are historical observations, not proof that
  every feature is still missing. Inspect current code before implementing one.

The inspiration remains modest: Ralph's one-task feedback loop, Pharos's
distinction between meaningful progress and fact counts, and SHEAF's pruning of
unneeded dependencies. Sources:
[Ralph](https://www.aihero.dev/getting-started-with-ralph),
[Pharos progress assessment](https://github.com/frenzymath/Pharos/blob/main/agents/skills/main/math/assessing-genuine-mathematical-progress/SKILL.md),
[SHEAF](https://github.com/frenzymath/SHEAF).
Their larger runtime architectures are not prerequisites for these tasks.

## What the sanity check established

The [task-003 worked proposal](003-one-next-question.md#implemented-prompt-and-worked-desk-check)
has now been executed on its three frozen histories. Restricting consensus to
finite-prediction candidates reduced summed per-anchor error from 10.894 to
6.989 mm on five-points and 4.670 to 2.552 mm on layered; seventh HP3 was unchanged
at 197.408 mm. These are selection-only simulated results, not RMSE, fresh fitting
or hardware accuracy. Baselines match saved producer values and input hashes.

Task 004 starts from those results instead of repeating their desk checks. The
[reproduction record](../../output/research/sessions/2026-10-23-program-task-sanity/report.md)
contains copied inputs and the six-evaluation script. Do not count duplicate
histories from cache/performance trials as independent configurations. The
external worktree inputs were available locally but not in shared output; this
made durable handoff recovery a concrete task rather than a hypothetical feature.

Task 004's [broader comparison](../../output/research/sessions/2026-10-23-consensus-membership/report.md)
is complete. Across eight additional dataset names, finite-prediction-only
membership improved one, worsened two and left five unchanged. The aggregate
gain was only 0.303245 mm summed anchor error; changed cases had no common
unseen prediction fold. Retire the blanket default-change proposal and preserve
current policy. The task-specific archived inputs make this result reproducible;
task 006 has now delivered the general [handoff/recovery workflow](../handoffs.md).
Its relocated CM-E1 bundle retained nineteen selected files, eleven input hashes
and the scoped decision after temporary source deletion; focused tests cover
partial/corrupt evidence and stale historical jobs. Task 005's agenda is next,
using this scoped negative result without repeating its numerical work.
