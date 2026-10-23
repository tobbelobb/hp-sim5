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

The first three tasks are ready in dependency order. Later tasks are proposals
with activation conditions. An agent may promote one when recorded evidence
satisfies its condition, or park it when the benefit disappears. Proposed tasks
are options, not promises. There is no need to finish this entire list.

| Task | Small outcome |
| --- | --- |
| [001](001-evidence-first-review.md) | A review prompt that checks the actual claim and reuses valid evidence |
| [002](002-short-decision-handoff.md) | A short decision/next-experiment handoff in the existing abstract |
| [003](003-one-next-question.md) | A prompt that turns a broad question into one economical assignment |
| [004](004-saved-evidence-pilot.md) | Try the handoff on a few existing investigations and prune this backlog |
| [005](005-follow-explicit-dependencies.md) | Add minimal dependency lookup only if the pilot needs it |
| [006](006-read-ended-job-evidence.md) | Recover saved job outcomes only if lost status is a current obstacle |

## Cost and evidence rules

All six seed tasks have a budget of **zero new physics simulations, fresh fits or
collections**. Saved report/JSON inspection, cheap score calculations, text
fixtures and fake child-process tests are sufficient. A numerical investigation
that needs more becomes its own scoped task; it does not hide inside this work.

Use the cheapest evidence that can answer the particular question:

1. Read the relevant report, inputs, source and existing check results.
2. Recompute only the needed analysis from saved artifacts when necessary.
3. Run focused tests for changed behavior. Use synthetic fixtures for bookkeeping.
4. Refit or recollect only when the scientific question actually needs it and
   the assignment explicitly budgets it. Small iteration counts can still be slow.

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
cable transport or scene dynamics changes. Those changes are outside these seed
tasks. Do not replace that delivery gate with a mock, startup or extra substeps.

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
  reproduction, confounds and maintainability are worth keeping. Its assumption
  that the latest commit is the result under review needs fixing.
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
