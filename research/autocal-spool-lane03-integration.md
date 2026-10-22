# Lane 03 integration: fixed scale polish and inner spool cap

The calibration interface now fixes the previous default behavior: uniform scale
polish after each spool filter pass, with a coordinate-descent cap of 30.
`--scale-fix` and `--spool-inner-iters` have been removed from the CLI, full-auto
variants, Python fit/plan/loop interfaces and supervised autocal settings. The
level-1 seed-rescaling path, level-2 last-pass scheduling and numeric-policy
metadata were deleted. Outer iterations, priors and objective schedules retain
their existing defaults and controls because lane 03 did not establish a general
replacement for them.

This is a simplification that preserves default numerical behavior, not a claim
that the default is optimal for every machine. The inner optimizer already stops
on parameter tolerances or stalled improvement; 30 remains its safety cap.

## Evidence checked

The supplied lane 03 report and proposed-changes document were read directly from
[the investigator's worktree report](/home/torbjorn/repos/hp-sim5-worktrees/research-85d64aa0/output/research/sessions/2026-10-22-autocal-flags-03-spool/report.md).
Contrary to the supplied path expectation, that session was not present under
this checkout's output directory. No investigator artifacts were modified.

A bounded integration investigation then used the current checkout (starting at
`eaae60c0b385ce561bc6df623b03452092567001`, with lane 02 integration already present).
Three validation fixtures outside lane 03's development set were fitted on all
but their final three sweeps. Those last three sweeps were held out without
refitting. Each pre-change prefix ran with default settings, scale polish off,
and inner cap 60. The simplified code then reran each default prefix: twelve
fits total, no fresh collection or hardware. All-pass refinement histories were
counted with a wrapper rather than relying on final-pass metadata alone.

| Fixture | Default physical total, mm | Polish off, mm | Maximum inner iterations at cap 30 | Cap 60 vs 30 | Simplified default vs original |
| --- | ---: | ---: | ---: | --- | --- |
| flexible_lines_2000 | 108.408 | 776.073 | 19 | Identical | Identical |
| ten_points_w_buildup | 8.240 | 301.254 | 16 | Identical | Identical |
| nineteenth_hp3_dataset_even_more_pressure | 175.942 | 175.942 | 11 | Identical | Identical |

“Physical total” is the regression runner's sum of anchor distances plus
2π times the radius absolute-error sum against simulated truth, not anchor RMSE.
Exact post-change equality was checked for anchors, radii, training cost, physical
errors, held-out median/p95 and all-pass inner iteration counts. All three
held-out sets contained 60 scored points; their membership was fixed before
comparison. Cap 60 also exactly preserved those values.

Default/off held-out p95 was 0.871/7.941 mm on flexible lines and 1.051/6.039 mm
on ten-point buildup. The pressure fixture was unchanged (8.700 mm p95). These
measurements support retaining polish, not deleting the remaining physical-model
investigations. The flexible prefix retains an existing cost sentinel of 100.0
even at the default; exact equivalence does not establish a valid/accurate final
calibration there. No timing speedup is claimed from these single runs.

## Compatibility and scope

Existing commands must omit the two removed options, including explicit
`--scale-fix 3` and `--spool-inner-iters 30`. Removed full-auto overrides and
supervisor settings are rejected. Tests and the supervisor command builder were
updated together so the supervisor cannot emit an obsolete CLI option.

Old fit-stage artifacts carrying the previous default `[3]`/30 are migrated at
the replay boundary. New override requests for removed controls are rejected.
An old artifact carrying a nondefault removed policy is rejected too: silently
replaying that policy as today's fixed policy would invalidate research
comparisons. Stored research evidence and reference logs remain unchanged.

The surviving `final_scale_polish` diagnostics and per-pass attempted fields
continue to describe actual work. Dead per-iteration `scale_fix_3` placeholders
that always said disabled were removed. User KEEP capabilities are preserved.
No physics, motor, cable transport or scene dynamics implementation changed.

## Reproduction and retained evidence

- [Integration notebook](../output/research/sessions/2026-10-22-autocal-spool-simplification/research.md).
- [Validation harness](../output/research/sessions/2026-10-22-autocal-spool-simplification/harness/validate.py): exact options, reference fixture selection, holdout split, counting wrapper and metrics.
- [Pre-change trial records](../output/research/sessions/2026-10-22-autocal-spool-simplification/before.log) and [post-change records](../output/research/sessions/2026-10-22-autocal-spool-simplification/after.log), with per-trial stage artifacts/results beneath their respective directories.
- [Lane 03 handoff](autocal-flag-audit/handoffs/03-spool.md) and [reference provenance](../autocal/data/references/README.md).

The pre-change `off` and `inner60` controls require the recorded pre-change
source. The new runtime intentionally cannot rerun removed policies. Restore
that source in an isolated checkout for such comparisons; do not silently strip
nondefault settings from their snapshots.

## Final verification

All 12 recorded regression fixtures pass with exact compared calibration
summaries and per-iteration parameters, unchanged stopping/consumption and
collection totals of 85 sweeps and 1,630 points. The complete non-slow autocal
suite passes 461 tests (two slow cases excluded), and the non-slow supervisor
suite passes 37 tests (five slow cases excluded). The first sandboxed autocal
run failed 13 socket-opening checks; all passed when rerun with localhost
sockets available. The initially interrupted supervisor lifecycle attempt was
also superseded by a complete passing run outside the sandbox.

- [Recorded regressions](../output/research/sessions/2026-10-22-autocal-spool-simplification/regression.log).
- [Autocal tests](../output/research/sessions/2026-10-22-autocal-spool-simplification/autocal-tests-unsandboxed.log).
- [Supervisor tests](../output/research/sessions/2026-10-22-autocal-spool-simplification/supervisor-tests-unsandboxed.log).

These checks establish preserved baseline behavior and integration, not an
improvement in physical accuracy or runtime. Further removal of outer budgets,
priors or objective modes requires their remaining investigations.
