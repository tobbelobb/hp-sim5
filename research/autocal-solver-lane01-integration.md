# Lane 01 integration: HP3 truth and solver diagnostics

Status: complete, 2026-10-22.

## Abstract

The lane 01 report inferred a seventh-fixture scene mismatch from poor fits and
agreement with an older fitted reference. Static reconstruction from the scene at
first data commit `0a5a0f6d` and today's HP3 scene instead reproduces the existing
nominal effective-anchor truth within 0.002 mm rounding. Rescored saved fits have
63.84–65.05 mm anchor RMSE against that truth; the lane's smaller distances describe
reference-fit agreement. Five bounded frozen-fit replays preserve the lane's
numerical results while new traces establish actual solver methods and termination.
A separate scalar control exercises the production TRF retry; function-level tests
expose and fix an enabled-fallback bug that skipped finite but invalid converged
primary fits. Three full replay regressions pass without parameter or collection
changes. No physical-accuracy improvement, hardware result or new default is
claimed; original unrecorded browser settings remain a provenance limitation.

## The scene claim

The nominal truth constants are supported by scene geometry. Autocal anchors are
home-position pinhole-to-mover-attachment offsets, not raw world-space pinholes.
All nine HP3 cable branches give the same four effective anchors used by the
regression runner:

| Axis | Effective anchor, mm |
| --- | --- |
| A | (0, -1900, -280) |
| B | (1645.44826719, 950, -280) |
| C | (-1645.44826719, 950, -280) |
| D | (0, 0, 1900) |

The historical `examples/usd_scenes/hp3.usda` at `0a5a0f6d` and current
`public/usd_scenes/hp3_rigid_body.usda` agree. Maximum difference from the runner's
rounded B/C X is 0.001733 mm. A parametrized regression checks every current
branch. The historical source audit and raw scene hashes are in
[truth-verification.json](../output/research/sessions/2026-10-22-autocal-solver-integration/truth-verification.json).

This does not prove which browser overrides were active during original
collection. It disproves the claim that the inspected nominal scene differs by
60–90 mm. A calibration agreeing with a historical fit cannot justify redefining
truth to that fit. The original lane's reference distances should retain their
explicit label; they are not physical anchor errors.

| Seventh saved configuration | Anchor RMSE vs nominal scene truth, mm | Cost |
| --- | ---: | ---: |
| 4 restarts, 400 iterations | 63.8444 | 83.0780 |
| 1 or 2 restarts, 400 iterations | 65.0450 | 86.8402 |
| 4 restarts, 200 iterations | 64.3227 | 69.9269 |

These results leave the large seventh calibration bias unresolved. They do not
support changing the scene, constants or restart defaults. Investigating original
collection bias, elasticity, rigid-effector rotation and model assumptions needs
separate evidence. No single cause is established here.

## Small implementation changes

- Keep the restart, iteration, optimizer-mode and fallback defaults. The layered
  accuracy tradeoff and seventh bias do not establish a universally better setting.
- Preserve `solver` and compact `optimizer_runs` in calibration output and stage
  artifacts. Per-stage restart traces include resolved method, selected restart,
  budget, success, cost, iterations/evaluations, termination and retry attempt/use.
  Full-auto JSONL measured run entries retain those traces, including no-valid-run
  stops. Initial configuration-only run entries do not represent executed solvers.
- Clarify `--solve-restarts` and `--solve-iterations` help: later GNC stages use
  one warm start, iteration factors are 1/0.6/0.4, and seed/refinement solves cap
  budgets. No new flag or runtime path was added.
- An enabled TRF retry now checks usable geometry as well as finite optimizer
  values before accepting a nominally successful primary result. Previously a
  converged primary with zero valid sweeps bypassed the documented retry. Disabled
  fallback behavior and normal default fits remain numerically unchanged.
- Distinguish `trf_fallback_attempted` from `trf_fallback_used` in traces. A retry
  can run and lose. Absence of retry messages with the feature disabled gives no
  evidence for removing it.

Traces describe the reported calibration, not every nested spool/initialization
solve. Restart-specific masks and filter-stage changes mean costs are not always
comparable across entries. LSQ `nit` follows the existing Jacobian-evaluation
count convention. No timing superiority is claimed.

## Bounded follow-up findings

The original three cold control snapshots reproduce anchors, cost and radii
exactly after instrumentation. A fourth replay reproduces seventh's 200-iteration
result and reveals two first-stage restarts hitting their 200-iteration limits.
The final 80-iteration trim stage converges after 70 iterations. Thus an initial
suspicion that the reduced trim cap alone explains the change is not supported.
Earlier solves can change later starting basins; “400 did not bind” on a default
trajectory does not establish equivalence with 200.

A fifth replay reproduces the layered TRF result exactly and records six actual
TRF calls across the GNC stages, all converged. The lane's missing method trace is
resolved for this comparison, but one fixture still cannot justify retiring LM or
TRF. This investigation did not test optimizer-mode equivalence or held-out error.

A scalar quadratic with a one-iteration budget runs the production SciPy L-BFGS-B
and TRF paths. The primary hits its limit; the retry runs but is not selected.
This establishes route activation and attempt/use reporting, not calibration
recovery or physical accuracy. Failed/invalid-primary unit cases verify the
specific retry fix.

## Validation and evidence

| Hypothesis | Experiment ID | Outcome | Evidence |
| --- | --- | --- | --- |
| Nominal HP3 scene agrees with effective-anchor truth | T1 | Supported for historical/current source | [Derivation and rescoring](../output/research/sessions/2026-10-22-autocal-solver-integration/truth-verification.json) |
| A fitted reference establishes replacement simulation truth | T2 | Rejected as an inference | Same scene derivation; reference agreement is a separate metric |
| Diagnostic propagation preserves cold-fit numerics | D1 | Supported on three controls and two targeted variants | [Controls](../output/research/sessions/2026-10-22-autocal-solver-integration/replay-checks.json), [variants](../output/research/sessions/2026-10-22-autocal-solver-integration/followup-checks.json) |
| Lower seventh iteration budget is non-binding | I1 | Rejected on the measured trajectory | Variant trace has two first-stage limit terminations |
| Layered alternative actually invokes TRF | M1 | Supported for this replay | Variant trace |
| Retry activates and may retain a better primary | F1 | Supported as scalar route control | [Control](../output/research/sessions/2026-10-22-autocal-solver-integration/forced-fallback.json) |
| Finite converged but invalid primary bypasses retry | F2 | Reproduced by function-level test; fixed | `test_trf_fallback_retries_failed_or_invalid_primary` |

[134 targeted tests](../output/research/sessions/2026-10-22-autocal-solver-integration/pytest.log)
cover solver dispatch, fallback activation, calibration serialization, CLI, stage
replay, regression output and all nine scene branches. The
[three-fixture regression](../output/research/sessions/2026-10-22-autocal-solver-integration/regression.log)
passes with unchanged fitted parameters and all iteration parameters within the
reference tolerance; 21 sweeps and 350 points consumed match the references.
[JSONL readback](../output/research/sessions/2026-10-22-autocal-solver-integration/jsonl-checks.json)
confirms 26 measured run entries retain 156 restart trace records.
No physics, motor, cable transport or scene dynamics implementation was changed.

Source baseline: `4f175079aaefe20db697ac3d74fa66bd1084ebb9`; local changes are the
small diagnostic/fallback patch and documentation/tests. The lane's artifacts
remain untouched in `/home/torbjorn/repos/hp-sim5-worktrees/research-edf7c713/output/research/sessions/2026-10-22-autocal-flags-01-solvers/`.
That worktree's output directory was not symlinked into this checkout when read.
Artifact paths and hashes are recorded in the rescoring evidence. Scripts and the
[notebook](../output/research/sessions/2026-10-22-autocal-solver-integration/research.md)
are retained beside the evidence.

Further references: [original flag assessment](autocal-flag-audit/report.md),
[fixture provenance](../autocal/data/references/README.md),
[regression truth and metrics](../autocal/tools/regress_calibration_logs.py),
[solver implementation](../autocal/ellipse_solver.py), and
[calibration serialization](../autocal/calibrate.py).
