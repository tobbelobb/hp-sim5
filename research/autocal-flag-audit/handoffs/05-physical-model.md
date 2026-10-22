# Handoff 05: Elasticity, origin conventions and geometry assumptions

Copy this file's task into one researcher session. Start independently now; no results from the other five lanes are prerequisites. Status: prepared, experiments not run.

Your lane ID is `05-physical-model`. Read [the parallel execution contract](README.md) and the [original assessment](../report.md), then execute the bounded first screen below. Follow repository and research AGENTS instructions. Use `.venv/bin/python`. Preserve the user's KEEP annotations.

## Question and ownership

Investigate whether these options should be retained, simplified, removed, hard-coded or developed further, and whether they improve robust physical accuracy or speed:

`--flex`, `--spring-k-multiplier`, `--low-anchor-z`, `--theta0-mode`, `--project-zero-tension`. KEEP spool geometry/modes are controlled interactions. Lane 02 owns raw/mean policy; lane 03 owns objective schedules.

You own these assessments and lane-local scripts/results only. Keep every other lane's numerical settings at the pinned baseline. Cross-lane combinations are follow-up proposals, not prerequisites for this screen. The old experiment labels are E5; use your lane ID in actual experiment IDs.

## Start and isolation

Set `FLAG_LANE=05-physical-model` and follow the setup in README.md. Your output root is `output/research/sessions/2026-10-22-autocal-flags-05-physical-model/`. Create a new trial subdirectory per invocation. Never reuse or overwrite another lane's dataset, logs, generated config, recorder, cache or result files. If this session directory already exists, continue its notebook deliberately or choose a unique suffix and record it.

Capture or reuse your own verified baseline artifacts from copies of the three development fixtures. Use the fixture's `DatasetSpec` and confirm artifacts match baseline source/options. No waiting for a common artifact producer is required. Three baseline full-auto captures maximum; reuse already available matching frozen artifacts whenever possible. The extra execution budget below excludes those captures but includes its own controls. Set a 15-minute timeout per replay and a 60-minute initial compute ceiling including captures; preserve partial evidence and narrow scope if exceeded.

## Initial screen

1. Audit fixture tension, stiffness, raw-angle, gearing and base-length metadata. Choose only inputs where the feature can activate and its assumptions can be checked. Record unavailable metadata as a limitation.
2. Allocate the initial maximum twelve fit executions to one supported question, rather than five weak tests. With suitable stiffness/tension data, compare flex off and on at multipliers 0.5/1/2 over three snapshots. If those prerequisites are absent, use the budget for zero/infer origin controls or fixed/free low-anchor-plane controls instead, stating why.
3. Origin test: construct clearly labeled synthetic copies with consistent angle-origin shifts and metadata; compare zero/infer against known geometry. Preserve originals. Missing inference inputs must produce a classified unsupported case, not a failed accuracy claim.
4. Plane test: use correct known shared height, a deliberately wrong height and unconstrained baseline, holding radius policy fixed. Inspect bias/conditioning. Geometry names in old HP3 fixtures are aliases, not proof of HP4 scene provenance.
5. Zero-tension projection acts during collection. Static replay cannot apply the flag retroactively to prove a collection benefit. Audit equations and saved projected/raw fields now; specify a later independent-world, varied-preload off/on by flex off/on acquisition test. Initial scope contains zero fresh collection or hardware trials.

Fit overrides: `use_flex`, `spring_k_multiplier`, `low_anchor_z`, `theta0_mode`. Read `flex.py`, `spool_model.py`, `ellipse_cost.py` and collector projection code. If a synthetic control generator is needed, keep it under your lane directory and document its equations/truth.

## Evidence and decision

Apply README.md's common metrics, holdout rules, failure accounting and acceptance thresholds. Concurrent timings are screening evidence only. List feature activation and literal usage separately; grep mentions, defaults and successful tool checks do not establish accuracy.

Elasticity and origin corrections are useful physical ideas but can absorb geometry error or double-count one another. Validate on held-out tensions/origins, not just training cost.

Write `research.md`, `abstract.md`, `report.md`, a machine-readable trial table and exact commands/overrides under your output root. In the report, answer each owned flag's five questions: actual-use evidence; purpose/removal case; sound idea versus implementation limits; interactions; evidence for the default or the next test. Give a disposition per flag (`KEEP user decision`, `retain`, `remove candidate`, `hard-code candidate`, `needs redesign`, or `inconclusive`) and cite experiment IDs. Put ideas needing deeper research in `ideas.md`, with a falsifiable hypothesis, prerequisites, experiment and rejection criterion.

Do not change shared runtime defaults while other lanes run. Produce `proposed-changes.md` or a patch confined to your separate worktree if supported by evidence; changes are not a prerequisite for a useful negative result. Finish with the exact next integration experiment involving another lane, if needed. Update your discoverability records per research/AGENTS.md; do not rewrite the six handoffs or shared flag inventory.
