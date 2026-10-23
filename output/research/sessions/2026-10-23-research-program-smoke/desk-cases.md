# Question-only entry: two manual desk cases

These cases were applied by the current agent after reading the entry,
assignment and reviewer prompts. They are manual workflow checks, not independent
agent sessions, model evaluations or new calibration/physics experiments.

## RP-D1a: the question is already answered within a saved scope

Input:

```text
Read research/prompts/research-program.txt and investigate:
Should autocal consensus exclude every fit without a held-out prediction?
```

Practical decision: whether to adopt that unconditional membership rule.
Leading explanation: excluding unvalidated fits might prevent bad consensus.
Rival: the newest fit has more training data and no subsequent sweep; absence of
a prediction need not mean its geometry is bad. The relevant scope is frozen
final selection with the chosen fit's spool parameters, not a changed loop.

Assignment selected: inspect the existing CM-E1 report, saved results and current
selection source before spending on another experiment. Budget: three files,
zero fits, predictions, physics or collection. The prior research decision
already answers unconditional adoption, so this iteration can close on evidence.

Execution: catalog search found the [CM-E1 abstract](../2026-10-23-consensus-membership/abstract.md).
Inspected its [report](../2026-10-23-consensus-membership/report.md) and
[results.json](../2026-10-23-consensus-membership/results.json). The saved summary
contains eight additional dataset names: one improvement, two regressions and
five unchanged. The two worsening anchor-error deltas are +0.502997755 and
+0.094156302 mm; the improvement is -0.900398730 mm. These are sums of per-anchor
distances, not coordinate RMSE. Every changed case has an empty common-unseen-fold
list. Current history_selection.py matches CM-E1's recorded analysis hash.
The [inspection record](desk-inspection.json) preserves identities and the exact
saved fields used. No producer probe was rerun.

Answer: retain current defaults. The unconditional nonworsening claim fails on
this simulated corpus. The earlier two gains retain their different-history
scope. No hardware effect or predictive generalization is established. The
[existing agenda](../../../../research/autocal-agenda.md) already records this
decision; its common-unseen-sweep question remains the next allocation, rather
than repeating the rejected blanket comparison.

Self-review target: this answer, the linked CM-E1 report/results, and current
history_selection.py at base 0606f031. Saved producer evidence was inspected;
five-versus-eight dataset distinctions, metric, output stage and source hash
match the claim. No dynamics changed and no new accuracy gain is asserted.
Verdict: keep this scoped answer. No additional fit or independent duplicate
run resolves a remaining gap in the unconditional-adoption decision.

## RP-D1b: a different question needs new evidence

Input:

```text
Read research/prompts/research-program.txt and investigate:
Would a lighter HP4 effector improve stability at the default physics settings?
```

This is a separate topic. The autocal membership agenda cannot answer it.
Read the [HP4 instability abstract](../2026-10-16-hp4-effector-instability/abstract.md),
its linked [single-step follow-up](../2026-10-16-hp4-one-step-autocal/report.md)
and [load-bookkeeping abstract](../2026-10-16-hp4-load-bookkeeping/abstract.md).
The old four-substep settling result is explicitly superseded as a default
recommendation. The current repository instructions require default-one-substep
production checks for delivered dynamics changes. The reports do not contain a
controlled lighter-mass comparison. Source scene mass fields alone do not
establish that lower mass improves a stiff, discrete-time solver.

Evidence assessment: unresolved. Leading explanation: lower inertia could let
the effector settle faster; rival: lower inertia can increase numerical stiffness
and rocking at a fixed integration step. Earlier failures also had distinct
causes, so the size-tuning failure cannot simply be equated with corner rocking.
These are hypotheses, not a newly observed mass effect.

Smallest proposed next experiment, not tested: a matched baseline/half-mass
headless HP4 comparison at one substep, preserving geometry, cable/motor settings,
initial state and command schedule, scaling corresponding inertias consistently.
Use at most two 10,000-step trials with the same bounded Print Logo prefix;
record displacement/speed/rotation, finite state and missed steps, followed by
the same settling observation. Any mismatch in initial state/workload blocks a
paired claim. A useful improvement needs bounded print motion and lower settling
motion without new instability or missed steps; a mixed result remains mixed.
This would establish only a simulated effect at those two masses, not hardware
safety or a universal optimum. No mass sweep or default change is authorized by
this desk smoke case. A delivered physics change would additionally need the
repository's complete `npm run test:physics` gate; the paired probe cannot replace it.

Default-budget outcome: no fresh physics allowance was supplied, so complete the
evidence assessment and return the proposed pair and required runtime/budget.
Do not start a world or silently select four substeps. A real follow-up should
put its evidence and allocation in a relevant HP4 agenda rather than replacing
the human's question with autocal priorities.

Self-review target: the unresolved answer and proposed comparison above. Verdict:
keep the scope-limited assessment; the lighter-mass hypothesis remains untested.
The next budget/runtime is explicit. This checks the budget and topic boundary,
not whether the proposed physical outcome will occur.
