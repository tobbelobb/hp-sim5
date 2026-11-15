# Autocal hardening and robustness

## Current understanding

Reviewed commits `1254e..5f718` and the original Fable conversation. The reported 1.4 mm result is one full HP3 simulation, not a robustness bound. Independent review found an incorrect profile lookup key and invalid assumptions for extending the Hangprinter reachability and circle models to four-high and cube layouts. See the layout investigation report in this program.

The parent implementation enables home offsets with an explicit off switch, preserves candidate-specific offsets in held-out validation, and retires zero-tension collection while keeping historical replay. Rough geometry now requires two-start agreement, convergence, full rank and <=25 mm estimated parameter standard deviation with a conservative 0.5 mm line-error floor. It collects more informative rough sweeps or stops after nine extras. This is a local statistical check, not a guarantee against arbitrary systematic bias. On historical c1 prefixes, three sweeps fail at 76 mm and four pass at 22 mm.

The home investigation established a translation gauge and existing absolute firmware flex correction. It implemented correction only with independent extension evidence; current collection does not provide that evidence. Configured stiffness alone cannot justify automatic physical-home translation.

## Open questions and alternatives

- Does the checked rough pass improve end-to-end accuracy and remain useful across HP3, HP4, worn bearings and modest compliance/noise variation?
- How much do ellipse filtering, independent point fitting, offsets and force profiles help on the same observed data? Preserve failures and compare absolute and translation-aligned errors.
- Can home extension evidence be measured cheaply without relying on nominal stiffness? Avoid adding an unidentifiable translation to ordinary results.
- Four-high needs gravity-supported nonnegative equilibrium and branch handling. Cube corners needs an active/slack set, all fixed constraints and multi-dimensional tension allocation. The HP3 predictor cannot simply be enabled there.

## Next decision

Finish production simulator comparisons on the combined candidate, inspect failures, and obtain fresh review before claiming an accuracy improvement. Keep the scope of synthetic and hardware evidence separate. Parent owns integration and topic synthesis; researchers own their reports and small evidence. No hardware validity is claimed.
