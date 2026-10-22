# Lane 02 integration: remove unused and misleading fit controls

Status: complete, 2026-10-22.

## Abstract

Lane 02 found an unused anchor-fit threshold and a sigma override that did not
control per-point normalization. Four bounded frozen-fit replays confirmed that
changing threshold 250 to 0.01 preserves anchors, radii and both costs exactly;
the 1 mm sigma override preserved parameters and normalized cost but changed raw
cost. The cleanup removes both options and their plumbing, consolidates paired
filter booleans, clarifies filtering/noise help and preserves old stage-artifact
replay. A post-change replay exactly matches the control. Three complete fixture
regressions preserve final and iteration parameters, scores and collection counts;
267 targeted tests pass. A provenance check also resolves the reported layered
accuracy gap: Lane 02's own final consensus is 1.58 mm anchor RMSE, matching the
later October reference; its 7.68 mm result is an individual fit. No filtering,
solver or stopping default was changed, and no accuracy or speed gain is claimed.
Evidence is recorded simulation and source tracing, not new hardware or collection.

## Changes and compatibility

- Remove `--threshold` and `residual_threshold` from the autocal anchor-fit path,
  including solver workers, fitting/planning/refinement calls and the cost object.
  The cost object stored the number but never read it; the spool prefit validity
  calls explicitly use infinity. The elliptical `calibrate.py` entry point and
  optimizer diagnostic plotter forwarded the same unused number, so their options
  are removed too. The standalone `ellipse_fitting.py` threshold and debug-tool
  validity gates remain functional and unchanged.
- Remove `--sigma-used-mm`, its fitting/variant plumbing and the incorrect override
  branch. It changed a residual-scale floor rather than replacing each point's
  sigma. Retain computed `sigma_used_mm` diagnostics and `--sigma-floor-mm`.
  Old dataset override metadata is discarded during annotation and ignored by the
  cost model. Extreme legacy overrides may therefore produce different results;
  this is removal of the unsupported behavior, not proof of equivalence for it.
- Use `argparse.BooleanOptionalAction` for pointwise filtering, sweep filtering and
  robust diagnostics. Both existing spellings and main defaults are retained;
  full-auto filtering overrides still default to `None` (inherit). The differently
  named global/per-sweep MAD and noise-mean/raw aliases are preserved and covered
  by re-enable tests.
- Describe pointwise filtering as staged pseudo-Huber loss plus final 3-MAD trim,
  and squared residuals when disabled. State that outlier-ratio sweep rejection
  needs pointwise inliers. Explain that sigma-floor changes rescale normalized
  costs. Do not silently substitute a new sweep metric.
- Replay migrates saved version-1 fit options by dropping the two removed keys.
  Explicit new override requests for either removed key fail with a concise
  parser error. Other unknown fit options still fail; no generic keyword-swallowing
  wrapper was added. Python callers and CLI commands must remove obsolete inputs.

Filtering loss/trim separation, sweep rejection redesign and noise-floor units
remain research questions. Lane 02's initial screen did not measure held-out
prediction, so it does not justify changing robust defaults or hard-coding a
winning filter policy.

## Bounded investigations

Source baseline: `87093cdd13f6410b4fd5d3a19c7ce5bdfad9dd26`. No firmware,
physics, motor, cable transport or scene dynamics implementation was changed.

| ID | Question | Result | Evidence |
| --- | --- | --- | --- |
| T1 | Does the unused threshold affect a layered frozen fit? | 0.01 versus saved 250 gives identical anchors, radii, raw and normalized costs | [Before checks](../output/research/sessions/2026-10-22-autocal-filtering-integration/before-checks.json) |
| S1 | Is sigma-used entirely inert in the fit output? | No: 1 mm preserves anchors/radii and normalized cost 216.8664530720463; raw cost changes 0.6610055028693824 to 0.6825968215144017 | Same before checks |
| C1 | Does the cleanup preserve the default frozen fit? | Post-change anchors, radii and both costs exactly match control | [After checks](../output/research/sessions/2026-10-22-autocal-filtering-integration/after-checks.json) |
| H1 | Does the claimed layered accuracy gap establish a fallback regression? | No: compares an individual fit with final consensus; both historical provenance and Lane 02's own summary contradict that inference | [Summary comparison](../output/research/sessions/2026-10-22-autocal-filtering-integration/summary-comparison.json), [revision ancestry](../output/research/sessions/2026-10-22-autocal-filtering-integration/fallback-history.json) |
| V1 | Are parser inheritance, artifacts and numerical paths preserved? | 258 targeted tests and 9 plotting tests pass | [Core tests](../output/research/sessions/2026-10-22-autocal-filtering-integration/targeted-tests.log), [plot tests](../output/research/sessions/2026-10-22-autocal-filtering-integration/plot-tests.log) |
| R1 | Are complete replay outputs preserved? | All three fixture regressions pass; final and iteration parameter deltas zero, unchanged scores, 21 sweeps/350 points | [Regression](../output/research/sessions/2026-10-22-autocal-filtering-integration/regression.log) |

These four fits are activation/equivalence controls, not an optimizer accuracy
matrix. Timings were not used as evidence. The initial targeted test run exposed
a test-helper call missing required noise-model arguments; that failure was fixed
before the final passing test run. No numerical trial failed or timed out.

### The layered reference comparison

From rounded final console parameters, against the runner's simulated truth:

| Result | Anchor RMSE, mm |
| --- | ---: |
| Lane 02 individual frozen control | 7.683 (Lane 02 report) |
| Lane 02 full-auto final consensus | 1.583 |
| October 15 original reference final consensus | 1.758 |
| October 15 second reference final consensus | 1.583 |

The first reference records revision `294e24c8`, which predates addition of TRF
fallback (`bdd28703`). The second records `efb22f0f`, after the opt-in change
(`de58e634`), and matches Lane 02's baseline summary exactly. Those two references
also have different stopping histories. The default frozen fit is not the final
applied consensus, so using its error against a selected historical result is not
a controlled solver comparison. This resolves the reported gap as a comparison
issue; it does not prove equivalence of all historical or enabled-fallback paths.

Also retain the [Lane 01 truth correction](autocal-solver-lane01-integration.md):
nominal HP3 effective-anchor truth agrees with the historical and current scene.
A fitted reference is not replacement ground truth. Unrecorded original browser
overrides remain a limitation, but poor fit agreement alone does not establish a
scene-frame mismatch.

## Sources and next work

- [Lane 02 source report copy](../output/research/sessions/2026-10-22-autocal-filtering-integration/lane02/report.md),
  [proposals](../output/research/sessions/2026-10-22-autocal-filtering-integration/lane02/proposed-changes.md)
  and [ideas](../output/research/sessions/2026-10-22-autocal-filtering-integration/lane02/ideas.md):
  original findings and limitations. Originals were read in worktree
  `/home/torbjorn/repos/hp-sim5-worktrees/research-1b0b6208/output/research/sessions/2026-10-22-interactive-research-51d5356b/`;
  this checkout did not have them at the initially reported symlinked path.
- [Notebook and commands](../output/research/sessions/2026-10-22-autocal-filtering-integration/research.md):
  scope, bounded trials and reproduction. Probe scripts/results are retained locally.
- [Fixture provenance](../autocal/data/references/README.md), reference-log headers,
  [cost implementation](../autocal/ellipse_cost.py),
  [parser](../autocal/_autocal_common.py), and
  [stage replay](../autocal/tools/replay_stage.py): physical assumptions and changed paths.

Next, separate loss shape from final trim in a bounded research harness and
validate on untouched sweeps with the same candidate-owned spool conventions.
A coherent bad-sweep control is needed before removing sweep filtering. Neither
proposal was tested or adopted here. References were not regenerated.
