# One useful research-workflow change at a time

This is a revisable development backlog. Start with the small change that helps
now, observe its use, and change the queue. The
[architecture proposal](../../research-program-design.md) is background, not a
list of infrastructure we must eventually build.

In an ordinary coding-agent conversation at the repo root, say:

```text
Read research/program-tasks/PROMPT.md and do one task.
```

Repeat that instruction for the next iteration, in the same conversation or a
fresh one. [tasks.json](tasks.json) owns priorities, dependencies and status;
[progress.txt](progress.txt) is append-only history. Edit the queue and task
briefs whenever our judgment changes. Completed work can be superseded without
rewriting its history. The one-task prompt requests a local commit, never a push.

Tasks 001-003 are committed. Their runtimes (1m31s, 3m28s and 8m10s) suggest that
the initial cards were too finely split for these agents. The remaining cards
cover useful outcomes through research, implementation where justified, checks
and a decision. A quick success is fine; runtime is not a quality metric and
there is no minimum duration. Combine shared-context work instead of padding it.
Cards remain revisable options. Park work whose benefit disappears.

| Task | Outcome |
| --- | --- |
| [001](001-evidence-first-review.md) | A review prompt that checks the actual claim and reuses valid evidence |
| [002](002-short-decision-handoff.md) | A short decision/next-experiment handoff in the existing abstract |
| [003](003-one-next-question.md) | A prompt that turns a broad question into one economical assignment |
| [004](004-saved-evidence-pilot.md) | Broader replay rejects the blanket consensus restriction; preserve defaults |
| [006](006-read-ended-job-evidence.md) | Preserve usable research evidence and status across worktree/session turnover |
| [005](005-follow-explicit-dependencies.md) | Use a living research agenda to prioritize the next question and trace its evidence |

## Cost and evidence rules

The queue defaults now allow up to 45 minutes, six fresh fits and two bounded
headless physics trials of at most 10,000 steps each, with no new collections.
These are ceilings for exploratory work, not amounts to spend or measured
runtime quotas. Per-task overrides keep the agenda and recovery work numerical-
free. Plan a specific lower budget before a probe, using saved artifacts first.
The user authorized fittings and simulations for this sanity check; the revised
queue makes bounded numerical work available in subsequent research tasks too.

Use the cheapest evidence that can answer the particular question:

1. Read the relevant report, inputs, source and existing check results.
2. Recompute only the needed analysis from saved artifacts when necessary.
3. Run focused tests for changed behavior. Use synthetic fixtures for bookkeeping.
4. Refit or recollect only when the scientific question actually needs it and
   the assignment budgets it. Small iteration counts can still be slow. If a
   feedback loop changes, frozen ranking cannot stand in for its replay.

One producer check and one evidence/code review at a meaningful result boundary
usually suffice. The reviewer inspects the producer's measurements, commands,
inputs and revisions; another full run is not the default price of independence.
A concrete discrepancy, changed relevant input, unavailable evidence or new
generalization claim can justify an additional check. State which uncertainty it
resolves. Reading a report is not independent numerical reproduction; label the
extent of checking accurately.

Evidence reuse requires matching relevant code, inputs, configuration and claim.
A relevant change can invalidate a previous result; an unrelated documentation
edit need not invalidate physics measurements. Reuse saves work, not uncertainty.
The root instructions still require `npm run test:physics` for physics, motor,
cable transport or scene dynamics changes. Budget required delivery checks
separately from exploratory trials, run the applicable gate once for the final
candidate, and do not substitute a mock, startup or extra substeps.

Do not start the research launcher, Viewer, doctor or preflight merely to edit
these files. Actual research continues through the existing client and supervisor;
this backlog adds no outer continuation loop for a research conversation.

## What the existing work changes about this queue

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
task 006 still owns the general handoff/recovery workflow, followed by task 005's
agenda using this scoped negative result.
