# Lower sensor measurement force on HP3

Status: in progress. No calibration result is available yet.

Fixed hypothesis H1: a default sensor force of min(forceMax,4*forceMid), instead of 5*forceMid, improves applied geometry in the declared motor/noise worlds without new failures and with at most 1.20 times completed baseline simulated effort. Candidate sensor-4mid-v1 was chosen before this assignment. Do not tune from new truth.

The three paired worlds and source hashes are in policy-frozen.json. All guide mu are zero. Motor perturbations and Gaussian encoder noise are hypothetical, not hardware characterized. Only the separate saved-evidence evaluator reads anchor truth. Explicit sensor overrides, pullouts, preloads, guards, solver, planning, selection and automatic stopping remain unchanged. Any anchor above 20 mm or incomplete calibration is a failure. Every anchor below 5 mm is good. RMSE is sqrt of the mean of four squared Euclidean anchor errors.

Native Rust, Node 23.11.1, speedup 25, no recording, full command/noise journals and passive initial/final snapshots. Two CPU cgroup; numerical jobs sequential. Limits: 7 collections, 80 fresh fits, 28M physics steps, 2000 points, 100 worker minutes including final 10 for reporting. Primary plan is six fresh runs, nominal/default then nominal/candidate, development/default then candidate, evaluation/default then candidate. Candidate freezes before any primary run.

Accepted human steering prioritizes avoiding incomplete or >20 mm calibrations, then accuracy and effort. Up to 20 percent extra simulated effort is acceptable. No further interview. After every numerical job the manager must remotely checkpoint the safe subset before another long job.

Previous study: [negative guide preload study](../attempt-939c073f49004eb6949910600200be51/report.md). Its original files are preserved under harness/original; original prepare.py is never executed. Session-local imports are rebased to this checkout to preserve ECS class identity. This study tests the independent motor friction route, with no active guide intervention.

Next decision: complete supervisor and command socket preflight, bounded torque-mode/readback probe, initial remote freeze checkpoint, then fresh nominal pair.

## Interrupted for handoff

Worker account usage limit prevented further work; user requested another agent after new upstream commits. No calibration collection started. Manager cancelled attempt and observed launcher_finally_completed. Upstream main at fbd2c744 includes our recovered evidence plus subsequent workflow/recorder changes. Reconcile these before continuing.
