# Hangprinter unknowns and the cheapest useful experiments

Status: three numerical experiments complete; scoped finding separately reviewed.

The most useful next step is to check what information calibration actually uses and where its physical length scale comes from. More encoder samples can help weakly measured directions, but cannot recover information the model discards or supply a missing independent length standard.

This program ran three cheap experiments on current calibration sources. The final combined numerical work took 2.97 seconds after imports, with no fits, collection or physics advancement. The launcher started an unused HP3 world. The research and review work takes longer than the numerical runs. All new accuracy-related evidence is synthetic or recorded simulation.

## What changed our understanding

| Experiment | Fresh observation | Decision it changes |
| --- | --- | --- |
| E1, fixed-encoder association | Reversing per-point fixed-angle observations changes no finite NumPy or JAX residual. Each condition retains 75 synthetic or 60 frozen HP3 observations. Controls changing aggregate fixed length or drive measurements change residuals by 0.16 to 1.13 mm. | Preserve the distinction between aggregate fixed length and its pointwise association. Test a per-point fixed-length objective before buying more samples of the same kind. |
| E2, physical scale | Scaling anchors, spool radii and the dimensionful buildup coefficient together by 0.95 or 1.05 preserves clean synthetic consistency below 1e-12 mm RMS over 300 observations. Scaling anchors alone produces 13 to 14 mm RMS. Holding nonzero buildup or one radius fixed also breaks the invariance. | Audit which millimetre quantities are independently measured. A nominal fixed quantity or a prior constrains the answer without establishing its physical accuracy. |
| E3, excitation versus a length standard | Twelve diverse synthetic sweeps improve the weakest observable local sensitivity by 7.51 times over three sweeps. Duplicating the original data does not. Both encoder-only cases retain rank 13 of 14. Adding one assumed radius observation gives local rank 14 of 14. | Collect diverse sweeps for the directions they can measure. Obtain an independent length observation to address this common-scale ambiguity. |

These findings concern unfiltered, nonrobust residual sensitivity and a specified free-radius, free-buildup synthetic model. They do not show a flat complete production objective. Noise floors, priors, bounds and quantities held fixed can all select a scale. A noisy residual magnitude also changes with length units even when the exact-consistency equations admit a scale symmetry. The first noise-normalization control hit the production 0.05 mm floor; the archive retains that inactive control and the corrected experiment.

[The numerical report](programs/program-dcd0c57c6b084a818d5d41b672ca78e1/selected/output/research/sessions/attempt-49e6fcc028c24a3b93e352420321ec0d/report.md) gives the definitions, controls, all tested hypotheses and limits. Its mm residual is an approximate geometric residual, not a measured Cartesian error or encoder forecast error. Local rank does not prove global uniqueness or reliable recovery from a bad initial estimate.

## Important questions still open

The rows below combine inspected prior reports with the new experiments. Proposed follow-ups have not been run in this program.

| What we still do not understand | Cheapest observation that could change a decision | Evidence and limit |
| --- | --- | --- |
| How much real calibration bias comes from discarding fixed-encoder association? | Prototype a per-point fixed-length residual offline. Generate known fixed-axis variation, compare the existing and proposed residuals, then check NumPy/JAX derivatives and frozen prediction. | E1 establishes information loss in the tested path. It does not measure the bias or prove the proposed objective improves accuracy. The [prior encoder audit](../output/research/2026-10-09-slip-recovery-and-encoder-model/report.md) also reports a failed run with almost constant fixed encoders, so this cannot explain every failure. |
| Which spool radius and buildup values are physical measurements rather than nominal assumptions? | Measure cable displacement per encoder-angle change at the relevant winding state, or independently measure one effective radius, before collecting more calibration sweeps. | E2/E3 show the information the synthetic model lacks. Fixed nonzero buildup supplies scale only to the extent that it is known accurately. |
| Does the actual applied consensus predict truly unseen observations? | Find one compatible later whole sweep outside the union of all anchor and spool training sets, then score both frozen applied models on exactly that sweep. Stop if provenance is missing. | The [autocal agenda](autocal-agenda.md) reports mixed anchor accuracy and missing common holdout coverage. This program did not resolve that gap or run another ranking comparison. |
| How much bias comes from cable stretch, winding, friction or effector rotation? | Repeat a fixed-pose encoder observation at two preloads and both approach directions, with independently measured body motion. Vary one cause at a time. | The [physical-model audit](autocal-physical-model-cleanup/report.md) reports missing tension inputs in its fixtures. Commanded force does not establish branch tension. The [HP3 truth audit](autocal-solver-lane01-integration.md) leaves substantial fixture bias unresolved. |
| Which simulated tracking limits persist under timestep and solver refinement? | Replay one frozen short command schedule at several substeps, retain body pose and encoder lag, and compare with an analytic single-spool control. | [Squares](hp4-squares-substeps.md) reports substep-sensitive tracking; [logo stability](hp4-logo-stability.md) leaves coupled rolling transport, damping and terminal tracking limits. These are prior findings, not fresh dynamics runs here. |
| Does better simulated calibration improve a real Hangprinter? | Measure a short closed path to three independent Cartesian targets before and after calibration, repeating measurements to estimate noise. Keep target measurements out of fitting. | Hardware transfer remains untested. [Native collection verification](native-collection-verification.md) establishes responsive simulated observations and parity, not fitted geometry accuracy. |

## The next physical experiment

An independent cable-displacement measurement is the cheapest physical experiment suggested by these results. A ruler or tape and the existing encoder are enough to test the common scale, subject to measurement resolution. This is proposed, not executed or timed.

Use the same preload and winding region as the calibration data. Mark a cable segment, record its encoder angle, move enough to measure roughly 200 mm of displacement, and record displacement and the second angle. Repeat in the reverse direction and repeat the pair. Estimate effective millimetres per degree from the slope, using the same gearing and mechanical-advantage convention as calibration. Compare it with the configured conversion and retain the direction-dependent spread. If 200 mm travel has 1 mm measurement uncertainty, that contribution alone is about 0.5%; angle uncertainty and repeatability must also be included.

A discrepancy would move effort toward conversion, winding and load dependence before more encoder-only sweeps. Agreement would bound this scale explanation at the measurement's resolution and leave pointwise modeling, excitation and other systematic errors open. A single measurement can test the common scale; it cannot establish every spool's winding law or the full machine's geometric accuracy.

## Evidence and reproduction

A fresh separate reviewer supported keeping the scoped report after inspecting the fixed evidence and verifying its hashes. It did not independently rerun the numerics. The [actual review](programs/program-dcd0c57c6b084a818d5d41b672ca78e1/selected/output/research/managers/program-dcd0c57c6b084a818d5d41b672ca78e1/controller-ff62e086e97b45b983aacfa15e150001/review.json) records those limits.

The live workflow also exposed a cloud reviewer output-path mismatch. Cloud review runtime files now live under `output/research/reviews/`, where the existing bridge permits final responses. A transport regression and the focused review/client suites pass, 44 tests. This tooling repair changes no calibration or dynamics source.

Program `program-dcd0c57c6b084a818d5d41b672ca78e1` preserves its assignment, setup failures, fixed submission, review and decisions in the [program archive](programs/program-dcd0c57c6b084a818d5d41b672ca78e1/report.md). The measured source is `c0f111c310cc562b7122bf606a59c60de758adbb`. Its calibration sources match base revision `7fc26ba4`; their only intervening commit changes cloud toolchain installation.

The [selected session](programs/program-dcd0c57c6b084a818d5d41b672ca78e1/selected/output/research/sessions/attempt-49e6fcc028c24a3b93e352420321ec0d/) contains the analysis, exact inputs, raw arrays, compact summary, source hashes and compressed oversized JSON. Copy `inputs.json` into a fresh output directory before replaying:

```bash
session=research/programs/program-dcd0c57c6b084a818d5d41b672ca78e1/selected/output/research/sessions/attempt-49e6fcc028c24a3b93e352420321ec0d
replay_output=$(mktemp -d /tmp/hangprinter-unknowns.XXXXXX)
cp "$session/inputs.json" "$replay_output/inputs.json"
timeout 120s .venv/bin/python -B "$session/analysis.py" all \
  --repo-root "$PWD" --output "$replay_output"
```

This work delivers research evidence and a decision map. It implements no calibration or dynamics change and establishes no hardware accuracy gain.
