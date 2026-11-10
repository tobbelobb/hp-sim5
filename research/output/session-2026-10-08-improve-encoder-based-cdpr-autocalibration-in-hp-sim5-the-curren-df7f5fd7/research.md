# Modular encoder autocalibration

Objective: make fitting, sweep planning and history selection independently reusable and measurable; preserve one-click behavior, runtime and all regression outcomes.

Accepted steering: user prioritizes separate testable components (Flyvbjergian legos), asks for reproduction before changes, and prohibits slower calibration or worse regression results.

Source baseline: 8811814c1c80ac52ebb5dabf380713f14e8d68a4. Existing untracked research/prompts/autocal_flyvbjerg.txt belongs to user; leave untouched.

Constraints and budgets: offline replay first, 11 recorded simulated datasets, two concurrent calibration processes; up to three full matrix phases plus focused stage trials. No hardware movements. New simulated collection only if needed for a changed measurement/solver claim. Preserve input/reference logs and failures. Native launcher world is fresh and not a checkpoint.

Hypotheses:
- H1: current ranking can prefer physically worse anchors (E01 supported; 20 adjacent mismatches; final regret in 10/11 datasets).
- H2: separating fit/plan/selection preserves exact calibration behavior and lets downstream trials skip fitting (E02a/E03 supported; all 11 paired logs identical).
- H3: reuse of the already computed information matrix reduces planning work with identical scores (E03/E06/E07 supported for frozen planning; extraction-only full-fit timing mixed).
- H4: borrowing read-only nested metadata within fitting improves runtime with exact results (E07/E08 supported).

Prior evidence: [rejected selection/solver trials](../autocal-ranking-bounded-updates-negative-results-b7deb1bc/report.md), [reference audit](../../reference-refresh-october-7-2026/report.md). Both caution that lower residuals need not improve physical parameters.

Next decision: completed stage extraction and no-worsening verification; future accuracy proposals can use frozen artifacts before the full-matrix acceptance gate. Selection policy remains unchanged.

E01 complete: all 11 datasets reproduce October 8 references exactly. Total matrix wall time 201.512 s. Ranking audit reproduces ten_points_bigger_deltas score-selected error 18.207 mm vs best 3.306 mm. Final prediction selection also remains imperfect. Numerical source changes now begin; frozen baseline logs retained.

E02 trial harness failed to resolve the renamed frozen baseline logs (None reference path); calibration subprocess logs are preserved under E02. This is an experiment-driver failure, not a negative calibration result. Corrected path lookup before rerun. Full matrix budget extended to four phases to retain this failed trial and obtain paired measurements. Brief concurrent tests/benchmark invalidate E02 timing; use isolated E04 for final timing.

E03 complete: recorded first fit 4.259 s; frozen planner median 0.0347 s; reevaluation 0.0149 s; pure ranking 2.36 microseconds. Fit replay, plan ranks and history keys match exactly. Initial three-pair full-plan timing was 3.023→3.051 s under shared load, so it does not establish no slowdown; rerun isolated after matrix finishes. Paired cached/recomputed planner median 0.03292/0.03365 s. E05 raw-rank frozen-history pilot selects iteration 1 instead of 3; full-precision error 3.365 vs 18.247 mm, but this is one fixture and prior matrix evidence rejects generalizing the policy. Keep the production policy unchanged.
No fresh collection is needed because no sensor, measurement model, optimizer or movement policy changes; reuse recorded simulated sweeps and test stage contracts directly.

E01 fragility reproduction: fifth_hp3_dataset has three underconstrained selected iterations and six failed recovery attempts; two attempts per episode do not re-establish constraints. Across six fixtures, 16 recovery attempts and six exhausted episodes occur. The current two-worker runs show no process crashes; previous full-concurrency crashes are archived evidence, not reproduced here.
E02a complete: all 11 summaries and intermediate fits exactly equal E01 (zero parameter/error/trajectory deltas). Wall 202.144 s versus baseline 201.512 s (+0.31%); initial overlap with other checks prevents treating this as a definitive timing comparison. E04 uses no concurrent tests or benchmarks.

E04 actual regression command passed all 11 with zero deltas, wall 206.313 s. E06 seven alternating full-plan pairs preserve exact results, but old median 3.197 s/new 3.346 s and wide pair differences (-0.333 to +0.623 s) leave no-slowdown inconclusive. Source profiling shows 736 modeled-length transformations and 2.285 profiled seconds in deepcopy (of 7.334 total; profiling overhead included). H4 proposed: borrow read-only nested measurement metadata within fitting while copying rewritten records and config, preserving public full-copy semantics, to remove a measurable bottleneck. Expand compute budget to six matrix phases only if this bounded stage experiment supports adoption. E07 independent transform and paired-fit trial next; E08 final matrix conditional on exact outputs and faster fit.

E07 supports H4: all seven alternating full-fit pairs are faster with identical anchors, costs and planner ranks; old/new mean 3.290/2.992 s (-9.1%), medians 3.255/3.078 s (-5.5%). All 11 complete dataset transformations compare exactly with unchanged inputs. Public deep-copy isolation and record/config mutation isolation are tested. Full Python suite now 337 passed, two slow collection tests deselected. Accept metadata borrowing provisionally only if E08 full regression remains exact and improves matrix runtime.

E08 complete: actual regression CLI returns 0 / ALL DATASETS PASS, wall 192.633 s versus E01 201.512 s (4.4% faster). Every summary and iteration compares exactly. H4 accepted; all modular and copying changes retained. No selection/solver policy change accepted. Final objective complete for independently testable stages and measured no-worsening; remaining accuracy issue explicitly preserved. Next decision: use frozen artifacts for a new score/identifiability hypothesis, requiring full-matrix acceptance before promotion.

Final CLI smoke: independent plan command succeeds; one-iteration opt-in export chooses stages_1 instead of overwriting prior artifacts and skips M669 application. Figure inspected. git diff --check passes. Catalog regenerated.
