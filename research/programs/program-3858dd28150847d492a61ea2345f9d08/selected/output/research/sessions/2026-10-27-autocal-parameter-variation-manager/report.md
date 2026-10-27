# HP3 autocal robustness across parameter variation

Status: first iteration reviewed; native collection followup planned.

## Abstract

This program tests encoder-only HP3 calibration across varied parameter assumptions while preserving fixed radii, zero buildup and one-click stopping. A new analytical benchmark evaluates eight frozen worlds with 32 core fits in 118.24 seconds. Only six fits pass complete status and coverage checks. A clean TRF example improves unseen encoder residual from 0.02362 to 0.02234 mm after thinning while anchor RMSE worsens from 5.89 to 40.20 mm. A fresh reviewer recomputed the saved measurements and supported the scoped findings and benchmark candidate. Two short native Rust probes establish stiffness sensitivity, not calibration accuracy. A cloud review path bug was reproduced and fixed, with 51 tests passing and a successful real reviewer dispatch. No production solver or point-count default changed. Matched native collection is the next discriminator. The reference-003 raw dataset remains unavailable.

## Decision and evidence

Keep the bounded analytical benchmark in cc52307f. It writes separate observed encoder data and evaluation truth, frozen worlds and hashes, failed rows, coverage checks and partial results. All 64 benchmark/solver/stage tests pass. The complete candidate changes only the new tool, its tests and documentation.

The eight analytical worlds vary layout, prescribed compliance, direction offsets, noise and movement span. Four were designated holdout before fitting. They omit physical equilibrium and use cold data-derived seeds smaller than the true anchor scale. These core-fit failures do not demonstrate failures of the complete one-click loop, which also uses recovery, warm starts, planning and consensus. There is no measured collection speedup.

The independent reviewer verified the fixed submission and recomputed all saved input hashes, seeds, anchor RMSEs and unseen residuals. It confirmed six complete fits, 17 prediction/geometry reversals and rejection of all three predefined replacement gates. It also recomputed the native encoder difference from archived telemetry. Its [exact assessment](review-first.json) supports the scoped claim and marks candidate code ready, with no required checks.

![Saved analytical core-fit matrix](analytical-screen.png)

The native probe compares two copied HP3 scenes differing only in nine cable stiffness values, 20000 versus 8000 N. Both execute the same 4000 total fixed steps with 30 mm spools and no layering. Peak A encoder difference is 0.561257 mm, final difference 0.301964 mm. These four-second runs show sensitivity and are not completed collection or calibration.

## Cloud issue and repair

The first review never reached a model. The bridge rejected the normal .research-programs/reviews final-message path because it allowed output/ only. Commit ae0b20c4 narrowly permits final-message.md in repository-contained .research-programs*/reviews/review-run-<32hex> paths. The failing default-store fixture was retained, and all 51 transport/review-controller checks pass. A real fresh reviewer then dispatched, finished and received an applied admission through that path. [Issue record](cloud-environment-issues.json) and [test output](cloud-review-path-after.log) preserve the distinction between fixture tests and live completion.

An earlier model-capacity interruption lost no saved evidence. The same worker resumed only to write its genuine handoff. Dependency, toolchain and independent socket checks passed; no installation repair was needed.

## Scope and next decision

No default solver, point count or automatic stopping rule changed. The next useful experiment is paired native production collection in nominal and soft copied scenes, using matched forces and ordinary fitting/recovery. It should test the analytical-to-native gap within a separate finite budget. The requested reference-003 directory is absent and a bounded exact-hash search found no copy, so no direct win over that reference is claimed.

## References

- [Worker report](../attempt-3b2e360ec1834eee9bab4e06f6f880fd/report.md), [handoff](../attempt-3b2e360ec1834eee9bab4e06f6f880fd/handoff.json), [raw matrix](../attempt-3b2e360ec1834eee9bab4e06f6f880fd/PV-E02/results.json): producer evidence and provenance.
- [Exact independent review](review-first.json): saved-data recomputation, engineering assessment and limits.
- [Research notebook](research.md): budgets, steering, failed launch and repair decisions.
- [Literature note](literature.md): encoder-only ideas and source access limits.
- [Reference search](reference-search.json), [runtime checks](runtime-check.json), [resource quotas](runtime-resources.json): availability and environment observations.
