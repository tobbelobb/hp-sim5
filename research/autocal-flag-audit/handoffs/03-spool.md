# Handoff 03: Spool refinement, priors and scale coupling

Copy this file's task into one researcher session. Start independently now; no results from the other five lanes are prerequisites. Status: prepared, experiments not run.

Your lane ID is `03-spool`. Read [the parallel execution contract](README.md) and the [original assessment](../report.md), then execute the bounded first screen below. Follow repository and research AGENTS instructions. Use `.venv/bin/python`. Preserve the user's KEEP annotations.

## Question and ownership

Investigate whether these options should be retained, simplified, removed, hard-coded or developed further, and whether they improve robust physical accuracy or speed:

`--scale-fix`, `--spool-outer-iters`, `--spool-inner-iters`, `--r0-prior-sigma-mm`, `--b-prior-sigma`, `--objective-schedule`. KEEP radius/buildup fitting, seeds and bounds are controlled interactions.

You own these assessments and lane-local scripts/results only. Keep every other lane's numerical settings at the pinned baseline. Cross-lane combinations are follow-up proposals, not prerequisites for this screen. The old experiment labels are E3; use your lane ID in actual experiment IDs.

## Start and isolation

Set `FLAG_LANE=03-spool` and follow the setup in README.md. Your output root is `output/research/sessions/2026-10-22-autocal-flags-03-spool/`. Create a new trial subdirectory per invocation. Never reuse or overwrite another lane's dataset, logs, generated config, recorder, cache or result files. If this session directory already exists, continue its notebook deliberately or choose a unique suffix and record it.

Capture or reuse your own verified baseline artifacts from copies of the three development fixtures. Use the fixture's `DatasetSpec` and confirm artifacts match baseline source/options. No waiting for a common artifact producer is required. Three baseline full-auto captures maximum; reuse already available matching frozen artifacts whenever possible. The extra execution budget below excludes those captures but includes its own controls. Set a 15-minute timeout per replay and a 60-minute initial compute ceiling including captures; preserve partial evidence and narrow scope if exceeded.

## Initial screen

1. Verify scale-fix activation with radii fitted. Level 1 affects alternating refinement; 2 polishes on the last filter pass; 3 polishes on every pass. Verify actual resolved schedules and pass counts.
2. Screen off/1/2/3 on three representative frozen fits, twelve fits including controls. Keep anchor solver budgets, filtering schedule, origin convention and scoring policy at baseline.
3. Plot or tabulate physical error, held-out error, data objective and prior objective along the scale direction for the most revealing case. Count attempted/accepted polish steps and total evaluations. No physical scale improvement is implied by objective improvement alone.
4. Design, but do not automatically run, follow-ups: outer 1/3/6 with inner 30; inner 10/30/60 at outer 3; radius prior None/1/5 mm; buildup prior 0.03/0.1/0.3 when k is actually fitted. Use truth away from prior means/bounds. Only cross individually promising controls.
5. Own objective-schedule follow-up: default pointwise sequence versus prefit-first or position-last. If reconstruction looks promising, distinguish residual fidelity from nested-solver failure and write an idea note. Lane 05 does not tune objective schedules.

Fit override examples: `{"scale_fix": []}`, `{"scale_fix": [2]}`, `{"spool_outer_iters": 1}`. Read `filter_pass.py`, `alternating_refinement.py`, `scale_polish.py`, `initialize_pass.py`. Keep lane 02 filter schedules and lane 01 solver budgets fixed.

## Evidence and decision

Apply README.md's common metrics, holdout rules, failure accounting and acceptance thresholds. Concurrent timings are screening evidence only. List feature activation and literal usage separately; grep mentions, defaults and successful tool checks do not establish accuracy.

A flat anchor/radius scale valley may require independent metric information or a joint well-scaled fit. More polish iterations cannot establish identifiability.

Write `research.md`, `abstract.md`, `report.md`, a machine-readable trial table and exact commands/overrides under your output root. In the report, answer each owned flag's five questions: actual-use evidence; purpose/removal case; sound idea versus implementation limits; interactions; evidence for the default or the next test. Give a disposition per flag (`KEEP user decision`, `retain`, `remove candidate`, `hard-code candidate`, `needs redesign`, or `inconclusive`) and cite experiment IDs. Put ideas needing deeper research in `ideas.md`, with a falsifiable hypothesis, prerequisites, experiment and rejection criterion.

Do not change shared runtime defaults while other lanes run. Produce `proposed-changes.md` or a patch confined to your separate worktree if supported by evidence; changes are not a prerequisite for a useful negative result. Finish with the exact next integration experiment involving another lane, if needed. Update your discoverability records per research/AGENTS.md; do not rewrite the six handoffs or shared flag inventory.
