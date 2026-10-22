# Handoff 02: Noise normalization and robust filtering

Copy this file's task into one researcher session. Start independently now; no results from the other five lanes are prerequisites. Status: prepared, experiments not run.

Your lane ID is `02-filtering`. Read [the parallel execution contract](README.md) and the [original assessment](../report.md), then execute the bounded first screen below. Follow repository and research AGENTS instructions. Use `.venv/bin/python`. Preserve the user's KEEP annotations.

## Question and ownership

Investigate whether these options should be retained, simplified, removed, hard-coded or developed further, and whether they improve robust physical accuracy or speed:

`--threshold`, `--filter-schedule`, `--pointwise-residual`, `--pointwise-filtering`, `--no-pointwise-filtering`, `--pointwise-global-mad`, `--pointwise-per-sweep-mad`, `--sweep-wise-filtering`, `--no-sweep-wise-filtering`, `--sweep-metric`, `--sigma-source`, `--sigma-floor-mm`, `--sigma-used-mm`, `--use-noise-mean`, `--use-raw-lengths`. KEEP `--line-width` is an interaction control.

You own these assessments and lane-local scripts/results only. Keep every other lane's numerical settings at the pinned baseline. Cross-lane combinations are follow-up proposals, not prerequisites for this screen. The old experiment labels are E2; use your lane ID in actual experiment IDs.

## Start and isolation

Set `FLAG_LANE=02-filtering` and follow the setup in README.md. Your output root is `output/research/sessions/2026-10-22-autocal-flags-02-filtering/`. Create a new trial subdirectory per invocation. Never reuse or overwrite another lane's dataset, logs, generated config, recorder, cache or result files. If this session directory already exists, continue its notebook deliberately or choose a unique suffix and record it.

Capture or reuse your own verified baseline artifacts from copies of the three development fixtures. Use the fixture's `DatasetSpec` and confirm artifacts match baseline source/options. No waiting for a common artifact producer is required. Three baseline full-auto captures maximum; reuse already available matching frozen artifacts whenever possible. The extra execution budget below excludes those captures but includes its own controls. Set a 15-minute timeout per replay and a 60-minute initial compute ceiling including captures; preserve partial evidence and narrow scope if exceeded.

## Initial screen

1. Trace threshold units and which ellipse fits use it. Main CLI 250 and low-level 0.01 defaults must not be treated as interchangeable. Map actual sigma sources, available noise means and fallbacks on each fixture.
2. First screen the four point/sweep filtering on/off combinations on three frozen fits: twelve fits including controls. Retain the same sigma source, schedule and residual metric. Measure false rejection and information loss as well as error.
3. Select at most one follow-up family for the next researcher: global/per-sweep MAD; the three sweep metrics; auto/point/origin/min sigma; unset/0.1/1 mm sigma overrides; raw/mean; Sampson/Euclidean; threshold 25/250/2500 after units are known; shortened filter schedule. Document the exact next matrix but do not run all families within this first screen.
4. With no refitting, inspect activation, normalized residuals, retained endpoints, coherent bad sweeps and heteroscedasticity. A scale that lowers normalized cost by enlarging its denominator is not an accuracy win.
5. Investigate positive-default spelling cleanup only as a proposal. Test main and full-auto variant inheritance before recommending removal.

Fit overrides use `pointwise_filtering`, `sweep_wise_filtering`, `pointwise_global_mad`, `pointwise_residual_mode`, `residual_threshold`, `use_noise_mean`, `sigma_source`, `sigma_floor_mm`, `sigma_used_mm`. Inspect `fit_stage.py` for accepted names. Own filter-schedule tests; lane 03 holds it fixed.

## Evidence and decision

Apply README.md's common metrics, holdout rules, failure accounting and acceptance thresholds. Concurrent timings are screening evidence only. List feature activation and literal usage separately; grep mentions, defaults and successful tool checks do not establish accuracy.

Robust statistics may have a sound purpose while masking a wrong physical model. Write a separate idea note when filtering deletes the observations that reveal model error.

Write `research.md`, `abstract.md`, `report.md`, a machine-readable trial table and exact commands/overrides under your output root. In the report, answer each owned flag's five questions: actual-use evidence; purpose/removal case; sound idea versus implementation limits; interactions; evidence for the default or the next test. Give a disposition per flag (`KEEP user decision`, `retain`, `remove candidate`, `hard-code candidate`, `needs redesign`, or `inconclusive`) and cite experiment IDs. Put ideas needing deeper research in `ideas.md`, with a falsifiable hypothesis, prerequisites, experiment and rejection criterion.

Do not change shared runtime defaults while other lanes run. Produce `proposed-changes.md` or a patch confined to your separate worktree if supported by evidence; changes are not a prerequisite for a useful negative result. Finish with the exact next integration experiment involving another lane, if needed. Update your discoverability records per research/AGENTS.md; do not rewrite the six handoffs or shared flag inventory.
