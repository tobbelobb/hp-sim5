# Autocal CLI changes

Run `.venv/bin/python autocal/autocal.py --help` for the everyday options and
`--help-research` for the full option list. The notes below record changes from the
flag audit.

## Investigation, 2026-10-22

The KEEP annotations above are preserved. Start with the
[flag assessment and experiment handoff](research/output/session-autocal-flag-audit/report.md).
It covers purpose, activation, removal candidates, coupled tuning, limitations
in the underlying ideas, and bounded experiments with acceptance criteria.

The [complete inventory](research/output/session-autocal-flag-audit/inventory.md) records parser
defaults and local literal-use evidence for 92 option spellings. The
[raw excerpts](research/output/session-autocal-flag-audit/usage.json) and
[reproducible audit](research/output/session-autocal-flag-audit/audit_usage.py) distinguish search
evidence from an assertion that an option ran or helped. Default-enabled features
can be used without appearing in commands.

No numerical experiments were run and no default was proved optimal. The proposed
first tests are scale-polish placement, filtering/noise normalization, and solver
budgets. No runtime flags or user KEEP decisions were changed.

Six researchers can start from the [parallel handoff index](research/output/session-autocal-flag-audit/handoffs/README.md),
which assigns exclusive topics, isolated outputs, budgets and integration checks.

Lane 01 follow-up: [truth verification and solver integration](research/autocal-solver-lane01-integration.md).
The nominal HP3 scene matches the existing effective-anchor truth; the seventh
fixture remains substantially biased against it. Defaults are retained. Solver
traces now expose methods, budgets, convergence and retry attempts; an enabled
fallback no longer skips a converged primary with zero valid sweeps.

## Lane 02 integration, 2026-10-22

Removed `--threshold` from the anchor-fit path after a 0.01 versus 250 replay
produced identical parameters and costs. Removed the misleading `--sigma-used-mm`
override and its plumbing; it did not override per-point normalization. Default
filtering and solver policies remain unchanged. Boolean filter switches retain
both spellings and full-auto inheritance. Older fit-stage snapshots are migrated
on replay. See [integration findings](research/autocal-filtering-lane02-integration.md).

Lane 03 integration removed `--scale-fix` and `--spool-inner-iters` from the
current interface. Per-pass scale polish and the existing cap of 30 are now fixed
behavior. See [integration evidence](research/autocal-spool-lane03-integration.md).
The original inventory and handoffs remain historical research records.

## Lane 04 integration

`--plot-residual-histogram` has been removed because it was parsed without a
consumer. Keep using `autocal/plot_residual_hist.py` with exported residual CSVs.
The help text above is the original audit snapshot, not current CLI help.
See [integration findings](research/autocal-ranking-lane04-integration.md).

## Free low-anchor heights, 2026-11-21

Removed `--low-anchor-z`. Hangprinter fits no longer give the low anchors one shared
height: every anchor coordinate is fitted except A.x and the top anchor's x and y.
The frame convention is origin at home, top anchor straight above it, A on -Y.

## Lane 05 integration, 2026-10-22

Removed `--flex` and `--spring-k-multiplier` from the ellipse/autocal pipeline,
including full-auto variants, fitting and history-validation plumbing. The
collector never supplied the per-point tensions this correction required.
Legacy point calibration and the explicit flex plotting helper remain separate.
`--theta0-mode` is retained with its existing default.
`--project-zero-tension` now rejects failed projections and requires projected
angle fields during fitting instead of silently using raw angles.

See the [integration report](research/output/session-autocal-physical-model-cleanup/report.md)
for preserved lane evidence, fit-free equivalence checks, regression results and
the saved-geometry sentinel probe. The earlier inventory is historical evidence.

## Lane 06 integration, 2026-10-22

Removed from the autocal and ellipse-planning CLI: `--stop-cost`, `--stop-std-mm`,
`--candidate-deltas`, `--fd-eps-mm`, `--no-exclude-existing`, `--existing-tol-mm`,
and `--top-k`. Candidate generation uses the grid/range controls; the composed
fit/plan uses the existing 1 mm information difference step, enabled duplicate
exclusion with 10 mm key tolerance, and a ten-entry diagnostic shortlist.
Independent stage APIs retain explicit experiment overrides.

One-sided `--delta-min`/`--delta-max` now override only that side of the normal
machine range. Invalid ranges are rejected. Exhausting spacing can relax spacing,
but cannot re-admit excluded duplicate sweeps. `--regularization` retains 1e-6
and now requires a finite positive CLI value; it is not calibrated uncertainty.
Patience remains 3, and sparse recovery remains opt-in.

### Stable-estimate stop

`--stable-step-mm` (default 0.5, 0 disables) stops collection once two
consecutive sweeps each move every anchor coordinate by at most that distance.
HP5 and HP5r add range coverage, uncertainty and residual requirements when
using spare sensor ranges. [HP5 stopping rules](autocal/README.md#hp5-stopping-rules).
The step rule replaced the `HP_SIM5_AUTOCAL_SPREAD_STOP_FRAC` agreement heuristic.

## Everyday and research help

`autocal.py --help` now shows routine configuration with a short usage line.
`autocal.py --help-research` lists every option, grouped by solver/recovery,
spool objectives, filtering, planning/comparisons and diagnostics. All flags remain
accepted directly and defaults are unchanged. Recovery and stopping orchestration
now live in `autocal/fit_recovery.py` and `autocal/stopping.py`.
