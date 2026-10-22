# Trial-002 diagnosis notebook

Objective: measure recording performance from stopped captures and repair the
encoder-settling failure without changing the user's reset browser.

Hypothesis: the return routine waits for stability before removing active trial
force. Constraints: source wall timestamps determine ordering; requested speed
never serves as a clock; preserve both recordings; do not increase deadlines to
hide motion. Scope: no hardware, no full calibration, no agent headless collection.

Budgets: inspect selected event and clock columns from two stopped captures;
one isolated production JS probe, at most 95 simulation seconds; one synthetic
before/after control-flow reproduction; targeted collector/bridge regression tests.
No network-connected physics worlds or hardware were used for E3/E4.

E1: manifests, sizes and RRD clock samples confirm measured improvements. E2:
recorded commands prove that force was still active through the failed settling
wait. E3: production JS probe reproduces the timeout; position mode settles in
2 seconds. E4: original control flow fails, patched flow succeeds. The 91-test
collector/bridge regression run passes.

Accepted steering: the user stopped trial-002 and reset their simulator, and
authorized diagnosis and repair. No settings were changed in that browser.

Decision: hold motors before return settling and after zero-distance return;
retain existing deadlines and thresholds. Next validation is a fresh manual
visual collection using a new output directory. No result for that retry exists.
