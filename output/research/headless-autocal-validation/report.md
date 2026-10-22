# Production JS headless full-auto autocalibration

The final fresh HP3 global-radius autocal completed after nine iterations and
eleven sweeps/220 points, applied M669/M666 and returned a 39.02 mm radius. Its
ideal fit score (1.039) does not establish parameter accuracy: the fitted D
height differs from the nominal preset. Independent
Chromium/Node collection with fixed forces/span produced sixty points per
backend; maximum differences were 0.02 degrees raw angles and 0.005236 mm lengths,
within stated 0.03 degree/0.01 mm tolerances. Timestamp offsets following settling
boundaries are reported separately. Bootstrap SIGTERM retained completed points
and stopped owned services. Failed initial scene loading and overly strict
comparison trials remain available. Both extended tests passed after correcting
the last physical wall sleep. Full-auto used 6487.832 simulated seconds and
435.65 wall seconds. All evidence is simulated.

See [notebook](research.md), [abstract](abstract.md), and
[trial artifacts](../../headless-verification/hp3/).
