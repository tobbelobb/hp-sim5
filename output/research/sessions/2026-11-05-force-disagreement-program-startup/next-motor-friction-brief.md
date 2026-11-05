# Proposed next discriminator: motor friction and noise

Status: proposed, not commissioned or tested. Dispatch only after the current
fixed-evidence review, using a new finite assignment. This is not a restart of
the guide-friction investigation.

The guide-friction stress found a simulation-model limitation. A distinct next
question uses existing torque-mode motor Coulomb/stiction terms, with every guide
coefficient zero, to test the human's robustness claim without that mechanism.

Keep HP3, known 30 mm radii, zero buildup, no layering, original default collection
policy and automatic stopping. Reuse the completed nominal and matched noise-only
reference evidence from the current assignment, after checking identities.

Declare one development and one evaluation world before numerical work. Vary
per-motor Coulomb and stiction magnitudes independently over 0.5-2x the modeled
defaults, plus Gaussian encoder-read noise 0.02-0.05 deg. These are hypothetical
modest parameter perturbations, not hardware-characterized ranges. Preserve
stiction speed/cogging/windage unless a new specific question requires them.
Verify ECS and packed native fields and report effective defaults when native
NaN selects fallback. The model uses smoothed friction, not ideal breakaway.

Test current defaults on each world. If all complete accurately, retain defaults
and report the finite tested scope, not a hardware-reliability claim. If the
development world fails, choose at most one bounded force/measurement recovery
using noisy encoders and current tuning; freeze before evaluation-world accuracy
is inspected. Do not tune against test truth, relax physical guards or default
switch solvers. Any anchor >20 mm fails, every anchor <5 mm is good. Retain failed
collection separately from bad accepted geometry, and intermediate solver tails
separately from final applied results.

Candidate finite budget: 75 worker minutes, 15 review minutes, 10 startup/manager
minutes, 190 seconds cleanup; at most 5 collections, 60 fresh fit calls, 4M steps
and 8000 sim seconds per collection, 20M total steps, 1400 measurement points. Use
sequential native jobs, no recording for the numerical matrix, preserved full
command/noise journals and passive initial/final snapshots. Reserve last 10 worker
minutes for reports/cleanup. These numerical counters are contracts, not claimed
hard aggregate quotas. A later allocation may narrow this proposal.

Expected result is a concrete table of per-anchor errors, maxima, acceptance,
failures, steps/simulated time and verified physical/noise parameters. A supported
small improvement is useful; a scoped negative result or finite no-failure screen
is also useful. No automatic merge or push.

## Proposed declared worlds

Use A/B/C/D factor order and all guide coefficients zero. These fixed example
worlds are selected before outcome inspection, not fitted to geometry.

| World | Split | Coulomb factors | Stiction factors | Read sigma deg | Seed |
| --- | --- | --- | --- | --- | --- |
| motor-dev | development | 2, .5, 1.5, 1 | 2, .5, 1, 1.5 | .03 | 4101 |
| motor-eval | evaluation | .5, 1.5, 2, .75 | 1.5, 2, .5, 1 | .05 | 4201 |

Recompute the effective default torque in Nm per motor before multiplying;
expected stock values are .001 Nm Coulomb and .0015 Nm stiction, subject to
loaded-state verification. Keep stiction speed, cogging and windage unchanged.
The matched noise-only default from the first study supplies reference context,
not a matched same-seed arm for these new worlds.
