Implemented a force-selection checkpoint so interrupted collection retains learned forces and diagnostics. It distinguishes successful tuning, fallback and manual selection, and never reuses saved settings automatically.

The equal 60-point coverage variant failed its declared gate. Replaying encoder-derived forces with existing recovery reduced conventional scene-reference errors to 3.29 mm nominal and 10-12 mm soft. Frozen models covered all 20/40 saved independent holdout points. These are limited simulation results, not full-auto acceptance or established absolute accuracy.

Validation:69 JS tests and17Python tests passed. Work used6collections,10fitstages and4,166,884 total physics steps, including627 regression steps. Source commits25822cbb and optional39510991 are separate and unpushed. See report.md and handoff.json for exact scopes, costs, failures and evidence.
