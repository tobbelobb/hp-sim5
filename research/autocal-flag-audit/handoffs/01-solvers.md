# Handoff 01: Solver algorithms and budgets

Copy this file's task into one researcher session. Start independently now; no results from the other five lanes are prerequisites. Status: prepared, experiments not run.

Your lane ID is `01-solvers`. Read [the parallel execution contract](README.md) and the [original assessment](../report.md), then execute the bounded first screen below. Follow repository and research AGENTS instructions. Use `.venv/bin/python`. Preserve the user's KEEP annotations.

## Question and ownership

Investigate whether these options should be retained, simplified, removed, hard-coded or developed further, and whether they improve robust physical accuracy or speed:

`--solve-restarts`, `--solve-iterations`, `--solve-optimizer`, `--trf-solver-fallback`, `--no-trf-solver-fallback`, `--optimizer-mode`.

You own these assessments and lane-local scripts/results only. Keep every other lane's numerical settings at the pinned baseline. Cross-lane combinations are follow-up proposals, not prerequisites for this screen. The old experiment labels are E1; use your lane ID in actual experiment IDs.

## Start and isolation

Set `FLAG_LANE=01-solvers` and follow the setup in README.md. Your output root is `output/research/sessions/2026-10-22-autocal-flags-01-solvers/`. Create a new trial subdirectory per invocation. Never reuse or overwrite another lane's dataset, logs, generated config, recorder, cache or result files. If this session directory already exists, continue its notebook deliberately or choose a unique suffix and record it.

Capture or reuse your own verified baseline artifacts from copies of the three development fixtures. Use the fixture's `DatasetSpec` and confirm artifacts match baseline source/options. No waiting for a common artifact producer is required. Three baseline full-auto captures maximum; reuse already available matching frozen artifacts whenever possible. The extra execution budget below excludes those captures but includes its own controls. Set a 15-minute timeout per replay and a 60-minute initial compute ceiling including captures; preserve partial evidence and narrow scope if exceeded.

## Initial screen

1. Trace actual optimizer activation, bounds, stopping and fallback configuration. Record inherited environment and resolved mode. Inspect historical failures and compiler-crash notes before choosing starts.
2. On three frozen fits (one per development fixture), screen baseline 4 restarts/400 iterations against 2/400 and 4/200. This is nine fit executions including controls. Leave all spool, filtering and scoring settings fixed.
3. Spend at most three additional fits on the most informative missing question: 1 restart, 100 iterations, trf/lm, or a mode/fallback comparison. Choose based on activation or observed failures; explain which questions remain untested. Do not squeeze every optimizer into an underpowered matrix.
4. For modes and fallback, use separate CLI processes or a harness reproducing process configuration. `optimizer_mode` and fallback are not accepted fit-stage overrides. Record retry triggers and invalid restarts, including zero triggers.
5. Compare exact/JAX/legacy objective and derivative behavior at fixed parameters before claiming a numerical discrepancy. Preserve native crashes and failed children.

Fit override example: `{"solve_restarts": 2, "solve_iterations": 400}`. Use `autocal/ellipse_solver.py` and `autocal/ellipse_objective_jax.py` for activation. No changes to spool iteration budgets.

## Evidence and decision

Apply README.md's common metrics, holdout rules, failure accounting and acceptance thresholds. Concurrent timings are screening evidence only. List feature activation and literal usage separately; grep mentions, defaults and successful tool checks do not establish accuracy.

Adaptive budgets or a simpler default may replace manual tuning if robustness survives. Legacy may remain useful as an independent diagnostic even if slower.

Write `research.md`, `abstract.md`, `report.md`, a machine-readable trial table and exact commands/overrides under your output root. In the report, answer each owned flag's five questions: actual-use evidence; purpose/removal case; sound idea versus implementation limits; interactions; evidence for the default or the next test. Give a disposition per flag (`KEEP user decision`, `retain`, `remove candidate`, `hard-code candidate`, `needs redesign`, or `inconclusive`) and cite experiment IDs. Put ideas needing deeper research in `ideas.md`, with a falsifiable hypothesis, prerequisites, experiment and rejection criterion.

Do not change shared runtime defaults while other lanes run. Produce `proposed-changes.md` or a patch confined to your separate worktree if supported by evidence; changes are not a prerequisite for a useful negative result. Finish with the exact next integration experiment involving another lane, if needed. Update your discoverability records per research/AGENTS.md; do not rewrite the six handoffs or shared flag inventory.
