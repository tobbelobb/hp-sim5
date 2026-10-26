# Hangprinter unknowns and cheap discriminating experiments

Status: complete for this bounded iteration; scoped scientific finding separately reviewed.

## Abstract

This program asks which important Hangprinter uncertainties deserve the cheapest next experiment. Three fresh offline tests compare current NumPy and JAX calibration residuals, synthetic scale changes and local Jacobians. Permuting fixed-angle observations changes no residual across complete 75-point synthetic and 60-point frozen simulated conditions, while positive controls respond. Scaling geometry, radii and dimensionful buildup together preserves clean synthetic consistency below 1e-12 mm RMS. Diverse sweeps improve the weakest observable local sensitivity by 7.51 times but leave rank 13 of 14. An assumed independent radius observation raises local rank to 14. The final numerical work took 2.97 seconds after imports. The first noise-normalization control hit a production floor and was corrected; it remains preserved. The resulting decision prioritizes pointwise modeling and auditing independently measured length quantities. Broader stretch, winding, dynamics, held-out prediction and hardware-transfer questions remain open. A fresh separate reviewer supports keeping the scoped report. No fits, collection, physics advancement, production change or hardware improvement is claimed.

## Results and scope

The worker session is `output/research/sessions/attempt-49e6fcc028c24a3b93e352420321ec0d`. The same program retains the genuine cloud worker and will bind a separate fresh reviewer to a fixed submission. Read the selected worker `report.md`, `summary.json`, source manifest and raw E1/E2/E3 measurements for methods and units. E1 includes a frozen simulated prefix plus synthetic controls. E2/E3 are synthetic source-level interventions. The millimetre residual is a Sampson approximation divided by summed absolute lengths, not measured body displacement or a sensor prediction.

E1 shows lost pointwise association, with active aggregate and drive controls. E2 separates an exact-consistency symmetry from noisy objective weighting. Its floor-limited initial discriminator is not discarded. E3 distinguishes duplicated data, diverse excitation and one independent dimensional observation. These tests support model auditing; they do not justify a new selector, solver setting, radius value or dynamics patch.

## Broader allocation

The human-facing synthesis is `research/hangprinter-unknowns-2026-10-26.md`. It lists uncertainties in pointwise calibration, physical scale, genuinely unseen prediction, stretch/winding/rotation bias, timestep-dependent dynamics and hardware transfer. Its later experiments are explicitly proposed and not tested. Prior-report source identities are in [sources.json](sources.json).

The cheapest physical discriminator proposed is independently measuring cable displacement per encoder-angle change under matching winding/preload and conversion conventions, repeated in both directions. It tests a common length standard; it cannot identify all spool laws or prove full-machine accuracy. A separate offline per-point fixed-length objective experiment should quantify whether E1 matters to actual fitted geometry.

## Budget and failures

Worker cap 18 minutes, review cap 6 minutes, active allocation 27 minutes plus 190 seconds cleanup. Zero fresh fits, new collections or simulation movement. The final E1/E2/E3 numerical phase took 2.97 seconds after imports; that is not total human research time or a full-autocal timing claim.

Two startup attempts failed before any actor or scientific work. Documented toolchain sourcing and permitted localhost networking corrected them. [Startup failures](startup-failures/) preserves their logs and exit observations. Launchers report their cleanup path returned. No failure was reclassified as a scientific negative result.

## Review and decision

Fresh reviewer `01a17957-3cc3-76c7-874b-8c241a30205c` kept the fixed scientific submission `submission-47731e6acc4d406faee0c63befec2c42` with finding supported, code readiness report_only and no material discrepancy. The controller applied review `review-07a6b19f34d149988b748081393136d2`. [Actual reviewer response](review.json) distinguishes inspection and integrity verification from numerical reproduction. All nineteen selected asset hashes and the submission identity were verified. No extra scientific run was requested.

The model route is promising for an offline per-point prototype and dimensional-information audit. Extra encoder-only collection is parked until a named uncertainty survives those steps; E3's observable-direction improvement remains useful. Production calibration defaults stay in place. The user-facing decision map includes proposed tests for the other unresolved questions.

The first review launch failed before any actor because its private-store output path violated the cloud bridge's output-only restriction. A narrow cloud runtime-path fix and regression were committed as e573a8fd; 44 focused review/client tests passed. The successful review used the same unchanged fixed scientific evidence. [Validation receipt](validation.txt), [integrity check](integrity-check.json), [failure log](startup-failures/reviewer-output-path.log).
