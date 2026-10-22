# Autocal physical-model cleanup after lane 05

Status: implemented and verified, 2026-10-22.

## Abstract

Lane 05 found that ellipse flex correction never activated on the twelve reference fixtures: the collector writes no assumed per-point tensions. This integration removes its two CLI switches and forwarding paths through ellipse fitting, solver workers, planning and history validation, and removes packed tension arrays and the JAX exclusion. Failed zero-tension extrapolation now stops collection; requested projected fits require projected angle fields, including when spool fitting is off. A fit-free comparison of the pre-change and current objectives gives exact equality in 72 checks over twelve fixtures. Two complete reference replays preserve parameters, selected results and collection counts exactly. The Python and collector checks pass. A small saved-geometry probe shows that two production sentinel fits have no valid sweep geometry even without sweep rejection; their numerical gradients are zero. This narrows follow-up work but does not identify the optimizer trajectory's cause. All calibration evidence is recorded simulation; no hardware measurements or new collection were made.

## Changes and scope

Removed `--flex` and `--spring-k-multiplier` from autocal, ellipse_active and `calibrate.py ellipse`, including main/variant parser entries, API keyword parameters, worker payloads, cost construction, metadata and validation forwarding. Removed the correction and tension arrays from `EllipseCostFunction`; it no longer disables JAX for an inert flex object. Updated callers and tests to the current signatures. This is a deliberate removal, not a deprecated no-op switch. User KEEP features remain intact.

The legacy `calibrate.py point` elasticity model, raw tension metadata role swaps, and `scripts/plot_sweep_fits.py`'s explicit diagnostic correction retain their independent inputs/callers. The standalone `autocal/flex.py` helper is still used by that plotting script. The noise model's `sigma_flex_mm` estimates uncertainty; it is a different mechanism and remains. No claim is made that tension-dependent elasticity is unimportant or that external tension datasets never existed.

Zero-tension collection now rejects insufficient/degenerate force samples instead of recording the last unprojected sample under `raw_angles_zero_tension_deg`. The existing successful projection path is unchanged. Spool rewriting requires the projected field when requested. The fit entry point validates this also with radius/buildup fitting disabled. Raw input remains usable by fitting without the projection option. Previously mislabeled historical fields cannot be detected retroactively by this validation.

Old flex-disabled stage artifacts remain replayable after removing their obsolete option keys. Explicit new overrides of removed options and old flex-enabled fits/histories fail clearly instead of silently substituting a different model. This extends the replay tool's existing migration rules for removed flags.

Origin inference and the shared low-anchor-plane constraint are unchanged. Lane 05's failed comparisons do not establish a superior policy. No simulation engine, motor control sequence, cable transport or scene dynamics changed. The collector change rejects invalid measurement results; it adds no movement and changes no force/settling sequence.

## Evidence and checks

Base revision is recorded in the [source manifest](../../output/research/sessions/2026-10-22-autocal-physical-model-cleanup/source.json). Raw evidence is in the [integration session](../../output/research/sessions/2026-10-22-autocal-physical-model-cleanup/).

| Check | Result | Evidence |
| --- | --- | --- |
| C05-01: raw objective equivalence after removal | 72/72 exact: twelve fixtures × filtering on/off × geometry scale 0.99/1/1.01. Costs, valid sweep counts and residual vectors unchanged; old flex-on also has identical costs. No per-point tensions in these fixtures. | [Results](../../output/research/sessions/2026-10-22-autocal-physical-model-cleanup/verified/results.json), [probe source](verify.py) |
| C05-02: complete replay of fifth_hp3 and layered_manual_some_bad_points | Both EXACTLY EQUAL to references. Same iterations and selected parameters; 16 sweeps/320 points consumed in both reference and generated runs. | [Regression output](../../output/research/sessions/2026-10-22-autocal-physical-model-cleanup/regression.log) |
| C05-03: Python autocal tests | 466 passed, 2 deselected after updating obsolete callers. Later stage-artifact migration changes: 40 affected tests passed, including two new migration cases. | [Suite output](../../output/research/sessions/2026-10-22-autocal-physical-model-cleanup/python-tests.log), [follow-up checks](../../output/research/sessions/2026-10-22-autocal-physical-model-cleanup/followup-tests.log) |
| C05-04: collector projection and encoder settling | Two suites, 24 tests passed. New missing/constant-force cases reject before recording; valid projection records the projected field. No physical simulator movement. | [Collector output](../../output/research/sessions/2026-10-22-autocal-physical-model-cleanup/collector-tests.log) |
| C05-05: saved production sentinel geometries | Iterations 4 and 5 (6/7 sweeps): all 6/7 sweep geometries invalid, cost 100, gradient norm zero with sweep rejection on AND off. | [Saved-geometry probe](../../output/research/sessions/2026-10-22-autocal-physical-model-cleanup/verified/results.json) |

C05-01 checks objective equivalence, not physical accuracy or speed. It uses raw fixture lengths at specified anchor matrices; C05-02 separately exercises production spool fitting and selection. Timing from tests/concurrent work is not a speed benchmark. Lane 05's reported approximately eightfold slowdown was a single-run observation; this integration claims removal of that disabling path, not a measured speedup of normal default runs.

The first restricted Python run had socket-permission failures and one obsolete CLI test. The CLI test was updated; the complete suite then passed with local socket access. The first saved-geometry probe mistakenly used raw lengths against fitted geometry. Its [initial results](../../output/research/sessions/2026-10-22-autocal-physical-model-cleanup/probe/results.json) are preserved but excluded from conclusions. The corrected probe uses each artifact's `dataset_for_estimation`. The final probe reruns on locally copied, hash-verified lane artifacts. Its recomputed ordinary costs need not reproduce every historical stage setting, so saved and recomputed values are reported separately. Invalid-geometry counts and the on/off comparison are the narrow tested claim.

## Solver follow-up

The cost-100 condition means too few valid/retained constraints, which can result from invalid geometry before sweep trimming. Disabling rejection does not rescue the two saved production geometries checked here. The finite constant also gives a zero numerical gradient in their neighborhoods. This is consistent with optimizer stagnation, but a final-geometry probe cannot establish when or why the optimizer entered that region, and it does not classify all six alternative lane-05 failures.

Next bounded experiment: replay one failed constrained-plane arm and one origin arm with the existing solver trace, logging predicted-ellipse validity, kept sweep count and step acceptance from their initial guesses through the first invalid region. Compare a feasible baseline warm start and the default cold start with sweep filtering held fixed. Only then test rejection off on the same starts. Preserve both failed and successful trajectories and score every valid fit on the same untouched sweeps. No sentinel magnitude or filtering default is changed here.

## Reproduction and provenance

The small [verification script](verify.py) compares source from a pinned Git revision with the modified cost and optionally evaluates saved geometries. It runs no optimizers:

```bash
.venv/bin/python research/autocal-physical-model-cleanup/verify.py \
  --baseline-revision 00375083 \
  --output output/research/sessions/2026-10-22-autocal-physical-model-cleanup/recheck \
  --lane-stages output/research/sessions/2026-10-22-autocal-physical-model-cleanup/lane05-evidence
```

The two full replays used:

```bash
.venv/bin/python autocal/tools/regress_calibration_logs.py --jobs 1 \
  --dataset-name fifth_hp3_dataset --dataset-name layered_manual_some_bad_points
```

Lane 05 was read from `/home/torbjorn/repos/hp-sim5-worktrees/research-eaee3cb5/output/research/sessions/2026-10-22-autocal-flags-05-physical-model/` because that session was not present under this checkout's output directory. Selected original documents, raw results and four frozen production artifacts were copied into [lane05-evidence](../../output/research/sessions/2026-10-22-autocal-physical-model-cleanup/lane05-evidence/) with an [origin/hash manifest](../../output/research/sessions/2026-10-22-autocal-physical-model-cleanup/lane05-evidence/manifest.json). The copied report retains its original historical links; consult the manifest for original paths. Original records were not edited.

## References

- Lane 05 [report](../../output/research/sessions/2026-10-22-autocal-physical-model-cleanup/lane05-evidence/report.md), [proposal](../../output/research/sessions/2026-10-22-autocal-physical-model-cleanup/lane05-evidence/proposed-changes.md), [activation probe](../../output/research/sessions/2026-10-22-autocal-physical-model-cleanup/lane05-evidence/flex_activation.json), [trial table](../../output/research/sessions/2026-10-22-autocal-physical-model-cleanup/lane05-evidence/trials.json): previous investigation's evidence and limitations.
- [Original handoff](../autocal-flag-audit/handoffs/05-physical-model.md): assigned scope and budget.
- [Ellipse cost](../../autocal/ellipse_cost.py), [collector](../../autocal/control/primitives/uncalibrated_actions.mjs), [spool model](../../autocal/spool_model.py), [stage replay](../../autocal/tools/replay_stage.py): final implementation.
- [Fixture provenance](../../autocal/data/references/README.md): recorded simulation truth and acquisition conventions.
