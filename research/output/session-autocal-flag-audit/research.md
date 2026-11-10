# Research notebook

Date: 2026-10-22. Status: static audit complete; experiment handoff ready.

Objective: assess the difficult autocal options and design experiments for the
next researcher. Preserve the user's KEEP marks and avoid running the full matrix.

Constraints: documentation/tooling only, no movement, no firmware or physics
changes. Numerical experiment budget used: zero. Static evidence search:
13,863 files, allowed extensions and 8 MB size cap, recorded in usage.json.

A01: AST parser extraction and literal evidence search completed. Result is the
92-spelling inventory, with parser locations, defaults and example excerpts.
This is an observability audit, not evidence of numerical feature activation.

A02: traced fitting, spool-origin, scale-polish, ranking, planning and stopping
entry points; reviewed prior abstracts/reports and fixture provenance. No numerical
flag was established as safe to delete unconditionally. Positive default switches
are compatibility-audit candidates. Threshold is an ellipse-validity gate, not a
verified no-op. Physical model switches require acquisition-aware testing.

Accepted steering: focus on robust/precise/fast calibration; design experiments
rather than running them all; preserve KEEP features. The report distinguishes
existing research findings from E1–E7 proposed work.

Next decision: run the bounded scale-fix screen, then decide whether it justifies
validation or whether to move to filtering and solver budgets. Use frozen stage
artifacts and independent errors; do not select a default on fit score alone.

Evidence: [report](report.md), [inventory](inventory.md), [raw audit](usage.json),
[audit script](audit_usage.py). Source revision is in usage.json; user flag document
was untracked at the start of this audit. No pre-existing file content was removed.

## Steering: six concurrent researchers

Prepared six independent handoffs under [handoffs/](handoffs/README.md). Split
E1/E2/E3/E4/E5 and combined E6/E7; assigned exclusive flag ownership and separate
session roots, datasets, logs and JAX caches. Each researcher can acquire its own
baseline; source stays fixed while numerical work runs. Parallel times are
screening evidence and require later isolated confirmation. No experiments were
launched. Handoff validation covers links, embedded Python syntax and flag coverage.
Next decision belongs to each lane's bounded initial screen, followed by a common
integration review rather than simultaneous default edits.
