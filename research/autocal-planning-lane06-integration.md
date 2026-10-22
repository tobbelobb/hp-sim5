# Lane 06 integration: simpler planning controls and explicit final selection

Status: complete, 2026-10-22.

The planning/stopping CLI loses seven unsupported or redundant controls. Cost and
ridge-dominated standard deviations no longer trigger acceptance. Grid/range
controls replace explicit candidate lists in the user interface; differentiation,
exclusion and shortlist defaults no longer travel through the full-auto loop.
Independent stage APIs remain available for reproducible research.

The unspecified side of a one-sided range now uses the machine's ordinary range,
and invalid ranges are rejected instead of silently replacing the user's bounds.
When spacing exhausts a grid, the fallback relaxes spacing while preserving
exclusion of observed sweeps. The ridge remains 1e-6, with finite positive CLI
validation; no new information model or grid default is claimed superior.

Final logs identify the actual held-out-selected iteration/run. The seventh HP3
artifact discrepancy was attribution: iteration 2 had the best running training
rank, whereas held-out selection chose iteration 4. Its printed M669 matches its
saved anchors and the reported approximately 202 mm truth error. There is no
geometry mismatch to repair in that artifact.

The spread-stop environment override is now documented in autocal-flags.md;
non-finite values fall back to the default. Patience, finite spread-stop values,
consensus and sparse-recovery selection policies otherwise retain their defaults.

Evidence and replay/test outcomes will be recorded in the
[integration session](../output/research/sessions/2026-10-22-autocal-lane06-integration/report.md).
The [preceding lane report](../output/research/sessions/2026-10-22-autocal-flags-06-planning-stopping/report.md)
was produced at an earlier revision; current-source controls are recorded separately.

## Validation and retained policies

Six paired no-collection controls used an eight-iteration ceiling. Disabling
spread stopping left two final calibrations unchanged; the outlier fixture's
combined anchor/radius error improved 7.009 → 6.705 mm, while its anchor error
worsened 4.672 → 5.272 mm. This is mixed recorded-simulation evidence, so the
spread and patience defaults remain unchanged. Raising the iteration ceiling
does not bypass other stop rules.

The suite passed 533 tests, with seven slow tests excluded by repository defaults.
All four selected calibration regressions passed (five-point, bad-points, seventh
HP3 and fixed-radius HP4), with unchanged parameter comparisons and 27 replayed
sweeps / 470 points on both sides. This establishes compatibility on those
fixtures, not broader accuracy or acquisition improvements. No runtime physics
changed and no fresh collection was performed.
