# Handoff 06: Movement planning, recovery and stopping

Copy this file's task into one researcher session. Start independently now; no results from the other five lanes are prerequisites. Status: prepared, experiments not run.

Your lane ID is `06-planning-stopping`. Read [the parallel execution contract](README.md) and the [original assessment](../report.md), then execute the bounded first screen below. Follow repository and research AGENTS instructions. Use `.venv/bin/python`. Preserve the user's KEEP annotations.

## Question and ownership

Investigate whether these options should be retained, simplified, removed, hard-coded or developed further, and whether they improve robust physical accuracy or speed:

`--candidate-deltas`, `--candidate-count`, `--delta-min`, `--delta-max`, `--fd-eps-mm`, `--regularization`, `--no-exclude-existing`, `--existing-tol-mm`, `--min-fixed-delta-spacing-mm`, `--top-k`, `--sparse-recovery`, `--sweep-points`, `--max-steps`, `--stop-cost`, `--stop-std-mm`, `--patience`.

You own these assessments and lane-local scripts/results only. Keep every other lane's numerical settings at the pinned baseline. Cross-lane combinations are follow-up proposals, not prerequisites for this screen. The old experiment labels are E6 and E7; use your lane ID in actual experiment IDs.

## Start and isolation

Set `FLAG_LANE=06-planning-stopping` and follow the setup in README.md. Your output root is `output/research/sessions/2026-10-22-autocal-flags-06-planning-stopping/`. Create a new trial subdirectory per invocation. Never reuse or overwrite another lane's dataset, logs, generated config, recorder, cache or result files. If this session directory already exists, continue its notebook deliberately or choose a unique suffix and record it.

Capture or reuse your own verified baseline artifacts from copies of the three development fixtures. Use the fixture's `DatasetSpec` and confirm artifacts match baseline source/options. No waiting for a common artifact producer is required. Three baseline full-auto captures maximum; reuse already available matching frozen artifacts whenever possible. The extra execution budget below excludes those captures but includes its own controls. Set a 15-minute timeout per replay and a 60-minute initial compute ceiling including captures; preserve partial evidence and narrow scope if exceeded.

## Initial screen

1. Trace information/covariance construction, candidate feasibility, duplicate exclusion, sparse recovery, consensus and stopping precedence. Finite-difference epsilon here is not the optimizer differentiation step. Confirm whether top-k changes collection or only output.
2. Planning screen: at most twelve plan-stage executions, baseline plus grid 21/81 and one epsilon or ridge sensitivity control on three frozen fits. Hold candidate range and all fitted parameters fixed. Record feasible candidate counts, ranks, selected sweep and information conditioning. Avoid an uncontrolled range/grid/spacing Cartesian product.
3. With saved prefix histories, evaluate patience 1/3/5 and possible cost/std thresholds against independent error. These are counterfactual stopping checks; prove the saved information is sufficient to reproduce decisions, otherwise label approximation and propose a full replay. No refitting to an unseen suffix.
4. At most three additional fit/recovery executions may probe prefixes known to trigger underconstraint. Count actual triggers and outcomes; include corrupted endpoints if constructing synthetic controls. Recovery success does not establish the final calibration is accurate.
5. Design later acquisition comparisons for 10/20/40 points and movement choices; do not run fresh collection in the initial screen. Old six-point trials fail the min_points=10 guard, and previous thinning harmed recorded fits. Replay cannot measure the benefit of a newly selected sweep that was never collected.
6. Write proposals for nuisance-aware information and calibrated uncertainty stopping if ordinary epsilon/ridge tuning is insufficient. Keep hard acquisition budgets regardless of stopping-policy results.

Plan overrides use `candidate_count`, `candidate_deltas`, `delta_min`, `delta_max`, `fd_eps_mm`, `regularization`, `exclude_existing`, `existing_tol_mm`, `min_fixed_delta_spacing_mm`, `top_k`. Changing epsilon requires recomputing information; the planner handles saved-info reuse conditionally. Read `planning_pass.py`, `active_learning.py`, `autocal.py`, and recovery code.

## Evidence and decision

Apply README.md's common metrics, holdout rules, failure accounting and acceptance thresholds. Concurrent timings are screening evidence only. List feature activation and literal usage separately; grep mentions, defaults and successful tool checks do not establish accuracy.

Conditional covariance and two agreeing biased fits can both give false confidence. Measure premature acceptance and expected physical prediction gain, not just information score.

Write `research.md`, `abstract.md`, `report.md`, a machine-readable trial table and exact commands/overrides under your output root. In the report, answer each owned flag's five questions: actual-use evidence; purpose/removal case; sound idea versus implementation limits; interactions; evidence for the default or the next test. Give a disposition per flag (`KEEP user decision`, `retain`, `remove candidate`, `hard-code candidate`, `needs redesign`, or `inconclusive`) and cite experiment IDs. Put ideas needing deeper research in `ideas.md`, with a falsifiable hypothesis, prerequisites, experiment and rejection criterion.

Do not change shared runtime defaults while other lanes run. Produce `proposed-changes.md` or a patch confined to your separate worktree if supported by evidence; changes are not a prerequisite for a useful negative result. Finish with the exact next integration experiment involving another lane, if needed. Update your discoverability records per research/AGENTS.md; do not rewrite the six handoffs or shared flag inventory.
