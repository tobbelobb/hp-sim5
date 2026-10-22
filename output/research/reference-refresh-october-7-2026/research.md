# October 7 reference refresh

Objective: generate references at main b60856d9, verify zero regression deltas, then compare research_findings_1.
Hypothesis: contemporaneous references remove runtime drift from the comparison; branch changes only final history selection.
Constraints: offline --no-collect replay; local .venv/bin/python; retain historical references; no algorithm changes.
Compute budget: three complete regression batteries (generation, baseline verification, branch comparison), with follow-ups only for failures or unclear accuracy results.
Accepted steering: user explicitly authorized creating dated reference logs and switching to research_findings_1 after baseline verification.
Next decision: complete baseline generation, inspect every dataset result before promoting references.

Generation complete: all 11 calibration logs have summaries and iterations. Old-reference comparison exits 1 as expected. Created 11 dated logs and called resolve_reference_log_path for every DatasetSpec; each selects its October 7 log. Next: run independent full baseline verification.

First baseline verification: 10 datasets exact with zero deltas; flexible_lines_2000 child exited -11 before any calibration result. Aggregate zero is insufficient because this run exits 1. Retrying full command unchanged before branch switch.

Second full baseline verification: same flexible_lines_2000 native crash (different iteration), all other datasets exact. Six-dataset Slideprinter diagnostic with PYTHONFAULTHANDLER=1 passes with all deltas zero. Next full verification retains native crash diagnostics; no source changes. Multiprocessing semaphore access is available, and autocal planning/filter passes explicitly use serial restarts, so process-pool fallback is not the demonstrated cause.

Native diagnostic full run failed on sixteenth_hp3_dataset_even_more_pressure inside JAX tracing with "Garbage-collecting" at mesh.__hash__. This occurs in another dataset and no longer singles out flexible data. Testing temporary sitecustomize hook to disable automatic cyclic GC in runner/children, with solver code and numerical settings unchanged. Hook is confined to ignored research artifacts and will be identical for baseline/branch.

GC-disabled run did not resolve native memory corruption: JAX compilation aborted with double free (-6) in sixteenth dataset. Restoring normal GC. Next run uses temporary startup hook limiting only outer regression dataset executor to two workers; calibration solver/math untouched. Same hook will be used for branch comparison.

Full baseline verification PASS: all 11 summaries and iteration parameters exact, every true-error delta zero, final_score 0.000, exit 0. Temporary outer-executor limit=2; normal GC. User condition for branch switch is satisfied. Next: checkout research_findings_1 and compare under identical scheduling.

Branch comparison complete: all 11 dataset runs completed; exit 1 is expected reference mismatch, with no runtime errors. Selected-error sum 885.158483 → 816.935891 mm, delta -68.222593 (-7.7074%). 3 better, 5 worse, 3 unchanged. All intermediate fits/scores identical; mean-delta sum exactly zero. Predictive helper tests: 2 passed. Current checkout research_findings_1. New reference logs remain uncommitted. Final report and machine-readable comparison retained.
Next decision: requested task complete. Potential future work: common validation folds and full candidate-specific line models; native JAX crash investigation remains separate.

Accepted steering, October 7: user requested a bug report for another debugging agent, not a fix. Reviewed captured failures and source paths; no additional calibration experiments run. Created research/bug-reports/autocal-native-crashes-october-7-2026/README.md with native failure traces, run matrix, exact commands, failed GC experiment, limits of successful scheduling workaround and proposed debugging/acceptance checks. Copied raw runner outputs, partial child logs, provenance and bounded-worker hook into repository evidence with SHA-256 manifest. Current main is 31e81bed, after predictive selection and reference commits; historical failing baseline remains b60856d9. Next decision: hand off native debugging; root cause remains unknown.
