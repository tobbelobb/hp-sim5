# fifth_hp3 reference audit

Objective: explain the March 23 reference, its code provenance and why current regressions differ.
Hypotheses: trajectory metric is dominated by rejected sentinel estimates; reference/current differ through solver or numerical environment rather than deliberate abandonment of a superior selected calibration.
Constraints: offline replay only; no hardware or live collection; no production source changes.
Compute budget: start with existing logs, then a few isolated single-dataset replays; cap if uninformative.
Evidence: reference introduced df699275, renamed 84930240; current existing log fifth_hp3_dataset_08bb67bc; pre-tip log fifth_hp3_dataset_d45c9a41.
Findings: reference selected iter 3, 70.314 mm; current selected iter 2, 70.787 mm. Iteration mean delta 390.788 is largely failed-fit trajectories, not returned calibration.
Next decision: replay March code under current dependencies and inspect default L-BFGS-B differences.

Completed experiments: March source df699275 and current HEAD 4daac40c, isolated snapshots with identical recorded dataset/arguments and current local venv. Both exit 0. Seven logged estimates and JSONL primary costs are identical, both mean delta +390.788445. March returns iter 3 (70.314 mm); current predictive history selection returns iter 2 (70.787 mm). Archived historical settings exactly match March replay settings. Numerical environment/execution drift is leading explanation; historical runtime/source hashes were not recorded, so precise cause is unresolved. No production edits made.
Artifacts: report.md, results.json, preserved reference log/JSONL/settings, complete March/current source snapshots and replay outputs.
Next decision: audit is complete; separating returned-error accuracy from failed-fit trajectory statistics and recording runtime provenance are recommended follow-up work.
