# Encoder-only autocalibration: research agenda

Updated 2026-10-23; evidence inspected at `fd10fd94`. Owner: the researcher
continuing the [autocal assignment](prompts/autocal.txt). This is a revisable
allocation decision, not a new calibration policy.

## Goal and current decision

Improve calibration accuracy, reliability and time while preserving one-click
use and automatic stopping. The deployable algorithm sees encoders and commanded
movements; simulated anchor truth is available only to evaluate experiments.
Track anchor error, radius error, prediction error, failures, movements and elapsed
time separately. A better fit cost or successful CLI exit does not settle this goal.

**Keep the current consensus defaults. Fund one bounded search for compatible
unseen sweeps before another membership or ranking rule.** The latest comparison
rejects a blanket exclusion of unvalidated fits; it leaves predictive safety of
the actual applied average unresolved. A further threshold search on these same
fixtures would not answer that uncertainty.

## Evidence that changes the allocation

| Evidence and scope | What it supports or challenges | Dependency and limitation |
| --- | --- | --- |
| [CM-E1 report](../output/research/sessions/2026-10-23-consensus-membership/report.md#cm-e1-broader-accuracy-comparison), [saved results](../output/research/sessions/2026-10-23-consensus-membership/results.json): eleven frozen simulated histories, producer `72523b9d`, analysis `5400cefc` | Among eight additional dataset names, finite-only membership improves one, worsens two and leaves five unchanged. Sum of per-anchor distances: 401.100276 → 400.797031 mm. Rejects the unconditional nonworsening claim. | The three overlapping names are sensitivity cases, not independent support. No fresh fitting, changed stopping or hardware evidence. |
| [CM-E1 prediction coverage](../output/research/sessions/2026-10-23-consensus-membership/report.md#prediction-guardrails-and-source-compatibility), [input identities](../output/research/sessions/2026-10-23-consensus-membership/inputs.json) | Every changed comparison has zero sweeps unseen by both complete applied models. Finite member scores do not validate the average plus its chosen spool parameters. This is the immediate evidence gap. | Selected single-fit scores match producer values, but their future folds can train consensus members. Input and scoring source revisions differ; compatibility was checked only for the reported frozen evaluations. |
| [Negative ranking study](../output/research/sessions/b7deb1bcc9cb418388273e2b7b9adc8c/report.md#reproduction-of-the-existing-failures), source `aaf25cf7` | Reports 22 adjacent ranking/physical-direction disagreements, prediction selection worse on 5/11 datasets, and aggregate composite parameter error +200.701 mm after removing the age bonus. Challenges a generic ranking-only fix. | Historical report inspected; its paired-regressions JSON is absent here. Composite anchor-plus-weighted-radius error is different from CM-E1's anchor-only metric. Shared fixture names do not create independent worlds. |
| [Refreshed-reference comparison](../output/research/reference-refresh-october-7-2026/report.md#branch-result), `b60856d9` versus `4daac40c` | Reports a 7.71% aggregate composite-error gain, alongside three improvements and five regressions. Supports keeping scoped gains while investigating validation comparability. | Candidates used different future folds. The report is available; comparison.json is absent here. This and the negative study test different revisions/selection policies, so their aggregates cannot be pooled. |
| [Lane 02 attribution](autocal-filtering-lane02-integration.md#the-layered-reference-comparison), baseline `87093cdd`; [Lane 01 truth audit](autocal-solver-lane01-integration.md#the-scene-claim) | An individual fit versus final consensus explains the alleged layered fallback gap. Nominal scene geometry supports existing HP3 truth; agreement with a historical fit is not ground truth. | These block spending on a solver rollback or scene correction justified by those comparisons. They do not establish universal solver equivalence or recover unrecorded browser overrides. Raw linked integration artifacts are not present in this checkout. |
| [Fixed-encoder model audit E3/E4](../output/research/2026-10-09-slip-recovery-and-encoder-model/report.md#optimization-audit-and-recommendation) | Reports finite residuals unchanged under pointwise fixed-angle perturbations. Supports a distinct model-adequacy route. The same report records failure with fixed encoders constant to 0.01°, so slip alone is insufficient. | Historical simulated evidence, not a current sensitivity experiment or accuracy gain. Current source still packs sweep-level fixed deltas; quantify relevance before implementing a larger objective change. |

The motivating three-history screen's two gains remain scoped reported evidence
in the [queue account](program-tasks/README.md#what-the-sanity-check-established).
Its original session is absent in this checkout. CM-E1 contains different producer
histories for the three overlapping dataset names; they cannot reproduce or erase
those earlier gains. Use CM-E1's archived inputs for the next decision.

## Competing routes and priority

| Order | Route and credible explanation | Cheapest discriminator and obstacle | Allocation now |
| --- | --- | --- | --- |
| 1 | Comparable validation of the applied anchor-plus-spool model. The apparent ranking signal may reflect different folds or training overlap. | Find matching saved later whole sweeps outside the union of all contributing training sets; score frozen arms on the same data only if identities match. Such data may not exist. | One assignment below. It can close this route without changing defaults. |
| 2 | Model adequacy and identifiability. Biased lengths, winding or ignored fixed-angle variation may let physically wrong geometry predict well. | If common-fold evidence is unavailable or inconclusive, scope a separate synthetic sensitivity/NumPy–JAX parity assignment using the fixed-encoder audit and Lane 01 truth constraints. | Preserve as an alternative; no objective implementation or unconstrained nuisance parameters funded yet. A failed selector does not falsify encoder-only calibration. |
| 3 | Solver/filter robustness. Poor optima or coherent bad sweeps may dominate before selection. | Match individual fits to individual fits and final outputs to final outputs, then use solver traces or a controlled bad-sweep case. | Behind the validation/model question. Lane 02's attribution gap does not justify a fallback/default change. |
| 4 | Faster or more informative collection and eventual hardware validation. Coverage may be inadequate, and simulation may omit systematic error. | Identify a specific unresolved measurement before collecting; retain physical guards and budget movements. | Defer new collection. Recording throughput and accepted simulated jobs establish tool capability, not accuracy or hardware safety. |

No external paper was needed to choose among these concrete gaps. The current
source and saved contradictory results determine this allocation; no uninspected
literature claim supplies empirical support.

## Next assignment: common unseen sweeps for changed applied models

**Proposed; not tested.** Question: can compatible saved observations distinguish
the predictive behavior of the two frozen CM-E1 arms on the three changed cases?
Leading explanation: existing overlapping folds obscure a usable predictive
distinction. Rival: both arms share model bias, so fair prediction remains
uninformative about physical accuracy. The model-adequacy route comes next if
this inexpensive feasibility check cannot distinguish them.

1. Start at the [CM-E1 manifest](../output/research/sessions/2026-10-23-consensus-membership/inputs.json),
   results and audit for `flexible_lines_2000`, `ten_points_w_buildup` and
   `ten_points_w_buildup2`. Decode their three gzip-compressed history envelopes
   with `read_stage_artifact` after verifying compressed and raw SHA-256 values.
   Preserve each arm's saved applied anchors and the chosen fit's own spool model,
   merge/spread thresholds, residual/noise settings and original training IDs.
2. Search only the producer session named by those manifest entries and matching
   repository datasets for later sweep candidates: at most twelve additional
   dataset/manifest files in total. Match observation contents, prefix identity,
   machine, frame/units, encoder origin, spool/layering and collection settings;
   matching filenames or renamed sweep IDs are insufficient. Reject unexplained
   settings changes. Hash the retained files and compare sweep contents against
   every contributing anchor fit and the chosen spool fit in both arms.
3. Freeze a common whole-sweep set for each compatible case before scoring either
   arm. It must be outside the union of both arms' training data, including spool
   fitting and any recorded preprocessing/selection use. Unknown lineage means
   unverified holdout, not unseen data. If there is no eligible set, return the
   missing identity/data explicitly and stop that case without fitting a new one.
4. If eligible data exist, use the existing `score_future_sweeps` conventions on
   the two actual applied models with identical observations/settings, excluding
   no bad results. Report robust normalized prediction cost and sweep/point
   counts, finite/invalid outcomes, training overlap and source compatibility.
   This is residual-based prediction, not newly measured encoder forecast error.
   Preserve CM-E1's separate anchor errors as evaluation context; truth cannot
   choose an arm or tune a deployable rule.

Predeclared decisions: a missing/unknown common fold leaves predictive safety
inconclusive and parks membership work in favor of scoping route 2. An invalid
prediction rejects that arm for the tested fold. A strict finite cost difference
larger than `1e-8 * max(1, abs(cost_a), abs(cost_b))` establishes only a scoped
prediction difference; otherwise record a tie. This tolerance separates numerical
ties, not practical accuracy. Even uniformly better prediction cannot undo the
two observed anchor regressions or authorize a blanket default change. A mixed
result or prediction/anchor disagreement raises the priority of model adequacy.
Do not invent a new membership rule in this assignment.

Budget: 30 minutes total, including source/input checks and the handoff; three
base histories, twelve additional files and at most six frozen prediction
evaluations (two per eligible dataset). Use the saved applied plans; zero fresh
fits, physics runs/steps, collections or movements. No Viewer or recorder. Stop
at the cap and preserve missing evidence. Required delivery checks are additional
if later implementation expands scope; physics changes require `npm run test:physics`.
The [recording-cost guidance](autocal-recording-performance.md) applies if a later
assignment needs simulation, not as a speed estimate for this one.

Return one decision with exact commands, input hashes, training/content overlap,
settings and source revisions, cost/count/invalid measurements, failures and
scope limits in the existing research.md/report.md/abstract.md. Use
[explicit handoff export](handoffs.md) for selected portable inputs and
[evidence-first review](prompts/reviewer.txt). Do not schedule duplicate fitting
or collection to create review independence.

## Withdrawal exercise: what if CM-E1 is unavailable?

This is a desk counterfactual, not a retraction of any saved result.

Withdraw CM-E1's accuracy comparison and prediction-coverage measurements. The
specific two-regression claim no longer supports retirement, and the exact three
cases above lose their priority. The initial two gains remain a lead, not a
validated policy. The other historical reports still report mixed ranking
outcomes and incomparable future folds; they support retaining defaults while
uncertainty is resolved, but do not reproduce CM-E1 or prove its conclusion.

Replace the next allocation with recovery/identity review of the archived CM-E1
inputs and results using the [handoff procedure](handoffs.md), before spending on
the three-case prediction probe. If recovery fails, propose a separately bounded
broader comparison; do not silently reinstate the blanket proposal or rerun
expensive evidence. Lane 02's attribution correction and Lane 01's nominal truth
remain relevant to solver/scene decisions, regardless of the membership finding.
The model-adequacy route also remains open. Restore CM-E1 only after the specific
withdrawn evidence is available and checked; retain this exercise as history.

## Keep this note useful

At a meaningful result boundary, update only the affected evidence link, scope,
priority and next assignment. Mark reported, inspected and independently
recomputed evidence separately. Keep rejected routes and their reopening
conditions. Before repeating an experiment, state which relevant input,
hypothesis or scope changed. Then update its session abstract and regenerate
the catalog with `.venv/bin/python scripts/research_index.py`.

Use the [next-assignment prompt](prompts/next-assignment.txt) with this note and
the human's latest steering; it proposes one assignment without launching it.
The [agenda exercise record](../output/research/sessions/2026-10-23-autocal-agenda/report.md)
states what was actually checked. No graph parser, registry or automatic
dependency invalidation is needed for the two decisions exercised here.
