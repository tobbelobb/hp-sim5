# Handoff 04: Ranking, run ensembles and CLI cleanup

Copy this file's task into one researcher session. Start independently now; no results from the other five lanes are prerequisites. Status: prepared, experiments not run.

Your lane ID is `04-ranking-cli`. Read [the parallel execution contract](README.md) and the [original assessment](../report.md), then execute the bounded first screen below. Follow repository and research AGENTS instructions. Use `.venv/bin/python`. Preserve the user's KEEP annotations.

## Question and ownership

Investigate whether these options should be retained, simplified, removed, hard-coded or developed further, and whether they improve robust physical accuracy or speed:

`--fit-structure`, `--full-auto-run`, `--shotgun`, `--stage-artifacts`, `--full-auto-log`, `--verbose`, `--robust-debug`, `--no-robust-debug`, `--debug-sweep-actions`, `--residuals-csv`, `--report`, `--plot-residual-histogram`, `--output-with-explanations`, `--write-sweep-config`, `--extended-reference`, `--extended-reference-ws`. Preserve all user KEEP recording aliases.

You own these assessments and lane-local scripts/results only. Keep every other lane's numerical settings at the pinned baseline. Cross-lane combinations are follow-up proposals, not prerequisites for this screen. The old experiment labels are E4; use your lane ID in actual experiment IDs.

## Start and isolation

Set `FLAG_LANE=04-ranking-cli` and follow the setup in README.md. Your output root is `output/research/sessions/2026-10-22-autocal-flags-04-ranking-cli/`. Create a new trial subdirectory per invocation. Never reuse or overwrite another lane's dataset, logs, generated config, recorder, cache or result files. If this session directory already exists, continue its notebook deliberately or choose a unique suffix and record it.

Capture or reuse your own verified baseline artifacts from copies of the three development fixtures. Use the fixture's `DatasetSpec` and confirm artifacts match baseline source/options. No waiting for a common artifact producer is required. Three baseline full-auto captures maximum; reuse already available matching frozen artifacts whenever possible. The extra execution budget below excludes those captures but includes its own controls. Set a 15-minute timeout per replay and a 60-minute initial compute ceiling including captures; preserve partial evidence and narrow scope if exceeded.

## Initial screen

1. Trace tail-ratio, sweep-bias and tail-factor penalty activation and all later ranking/validation gates. Audit the three active shotgun metric variants, their resolved settings and recorded winner frequencies.
2. On frozen candidate histories, compare fit-structure off/1/2/3. Recompute affected metrics, then rerank without changing anchors or spool parameters where mathematically valid. Count actual selection changes, invalid predictions and known-truth selection regret. If saved histories lack necessary diagnostics, mark the limitation rather than fabricating a ranking-only result.
3. Budget at most twelve frozen-history policy evaluations initially (four policies on three histories). Up to three reconstruction fits may replace missing diagnostics; record them separately. Do not tune solver/filter/model values.
4. Compare standalone versus ensemble choices using the available same-history candidates. A true whole-run ensemble comparison may require new full-auto replays; propose it separately instead of treating a frozen-history test as complete equivalence.
5. Audit diagnostics, report paths, recorder ownership and variant parser precedence. Produce a compatibility/removal table, including positive-default forms referred by lane 02. No concurrent shared parser or logging edits. All proposed simplifications must preserve KEEP capability and reconstruction of resolved settings.

History `evaluate` requires original history, `rank` requires evaluated history and accepts no overrides. `fit_structure` is not a history-replay keyword: reconstruct scoring using `_autocal_common.py` and saved metrics or recompute fit-stage metadata. Read `history_selection.py`, `shotgun.conf`, parser helpers and recorder modules.

## Evidence and decision

Apply README.md's common metrics, holdout rules, failure accounting and acceptance thresholds. Concurrent timings are screening evidence only. List feature activation and literal usage separately; grep mentions, defaults and successful tool checks do not establish accuracy.

Tail diagnostics can detect mismatch without being good rank penalties. An ensemble that exploits a biased score can cost more and select worse geometry.

Write `research.md`, `abstract.md`, `report.md`, a machine-readable trial table and exact commands/overrides under your output root. In the report, answer each owned flag's five questions: actual-use evidence; purpose/removal case; sound idea versus implementation limits; interactions; evidence for the default or the next test. Give a disposition per flag (`KEEP user decision`, `retain`, `remove candidate`, `hard-code candidate`, `needs redesign`, or `inconclusive`) and cite experiment IDs. Put ideas needing deeper research in `ideas.md`, with a falsifiable hypothesis, prerequisites, experiment and rejection criterion.

Do not change shared runtime defaults while other lanes run. Produce `proposed-changes.md` or a patch confined to your separate worktree if supported by evidence; changes are not a prerequisite for a useful negative result. Finish with the exact next integration experiment involving another lane, if needed. Update your discoverability records per research/AGENTS.md; do not rewrite the six handoffs or shared flag inventory.
