# Coverage allocation versus force tuning

Started 2026-10-27 12:22:11 UTC. Deadline 13:07 UTC. Reserve last eight minutes for checks and handoff.

Objective: improve calibration behavior using saved native HP3 inputs. Primary hypothesis CVF-E01 is that six independent fixed-target sweeps improve geometry at the same 60-point budget. Rival CVF-E02 is that force tuning, rather than layout, controls the poor native fits.

Budgets: four collections, 24 ordinary fit stages, 4M physics steps. Native Rust via production headless host, no shared builds or dependencies. One-click automatic stopping stays in place. No truth in collection, fitting, seeds or policies.

Predeclaration: see [coverage contract](coverage-contract.json). Delta=min40mm, quarter minimum observed drive span, centered on each saved size-tuned fixed offset. Both physical directions count, so five points/direction gives ten per sweep. Compare ordinary lbfgsb/trf to saved stripped fits, require at least20% conventional scene-home RMSE gain in both worlds, no geometry/rank/coverage regression, and at most2x wall/step work. Conventional truth lacks settled reference pose.

Approach ledger:
- Coverage allocation, running. Make a10-15minute cut/continue decision.
- Automatic force tuning, not tested. Use remaining two collections if coverage is weak.
- More random seeding, rejected as a fresh explanation from source inspection. Current restarts already sample uniform full bounds after the production initial guess. No seed experiment run.

Runtime: cloud network policy unrestricted/enforced, no credentials needed. Actual Node23.11.1 verified after sourcing scripts/cloud/env.sh. Agent command socket creation, bind and HTTP round-trip passed. Supervisor preflight uses research_attach.py.

12:26 source/log clarification before fitting: HP3 clamps lower fixed target to0. Nominal effective levels0/24.7958mm; soft17.8430/97.8430mm. Requested negative nominal level is preserved in contract. Both remain independent configurations. Supervisor preflight passed. First MCP wrapper failed after capabilities because the SDK field is is_error; corrected wrapper and completed all checks. No movement in preflight.

12:28:40 cut/continue decision. CVF-E01 rejected in tested scope. Extra fixed levels at equal60points failed paired20% exploratory criterion, regressing2/4 solver/world conventional geometry comparisons despite smaller residual. Both collections completed,33.84/77.03s,4fit stages. Proceed CVF-E02 existing automatic force tuner, original3x20 layout and sensor1N. No default changed. See force-contract.json.

Count correction: two of four coverage geometry comparisons worsen, not three. Nominaltrf improves3.68%, softlbfgsb improves15.13%; neither passes20%, and the other two regress. Use per-anchor Euclidean RMSE, not coordinate-wise RMSE. See measured-summary.json for exact values.

12:35 accepted steering decision-b044bb8844bb44fbb318395b8b877e4a expands cap to exactly6collections/6Msteps, still24fit stages and13:07deadline. Both automatic-force originals exhausted1Msteps, nominal30points, soft40points; do not call these accuracy negatives. CVF-E03 uses two fresh worlds and exact encoder-derived learnedforces, skipping repeated tuning. Each capped1.55Msteps. State history differs, production size tuner remains. If these fail, no further expansion.

12:38 to12:40 results. CVF-E03 completed60points in each fresh world with exact learnedforces, no retuning. Nominal ordinary LBFGSB hit sentinel; actual existing restore_sweeps then restore_points recovered3.29mm. Soft ordinary10.43/11.64mm. No solver change justified. CVF-E04 froze models and predicted reconstructed complete original-force sweeps,20/20 nominal and40/40 soft coverage; default recovered residual RMS0.02045/0.10302mm. Baseline softTRF predicts0/40, counted as failure. See exact holdout script/results. Reference pose limitation and state-history differences remain.

12:45 to12:48 production result. Separate commits39510991 optional per-config targets and25822cbb durable force-selection checkpoint. The latter atomically persists exact forces/diagnostics/context before sweeps, distinguishes success/fallback/manual, refreshes staleauto metadata on manual rerun, reuses one metadata object and never auto-loads settings.69 JS behavior/CLI and17Python focused tests pass,5slow deselected. Native measurements preceded delivery checkpoint; no seventh collection. No physics/core/sharedbuild change. No full-loop stopping gain claimed. Do not push before parent review.

Final usage:6collections,10fitstages,4,166,884 total physics steps, including627 regression steps,433.691measuredworldwallseconds. Original deadline13:07UTC retained; revised limits6/24/6M. Preserve all incompleteforce trials. Keep robustness fix; optional targetsyntax separate; no default solver/pointcount/stop change. Next question is cheaper force search with matched physical quality. No further collections needed.
