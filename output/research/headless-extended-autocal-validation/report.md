# Headless extended autocal recording validation

The production headless JavaScript world completed default full-auto calibration
with one finalized extended-reference RRD: eight sweeps, 160 points, 267,711
physics observations and 616,025 events in 1525.41 wall seconds. Archive checks
verified exact final datasets/logs, all measurements, paired command outcomes,
stage artifacts and explicit source clocks. Live reset, 25×, external-recorder,
disconnection and interruption checks passed. Failed trials exposed a drain
wait and a backlog-sensitive disconnect; their partial evidence is retained.
The separate Chromium/Node comparison reproduced a one-point 0.16-degree
discrepancy, including with the original headless service. Tolerances were not
relaxed. These are simulated recording-tool checks, not evidence of calibration
accuracy or hardware behavior.

The requested feature is complete. The fast regression runs passed 78 Python and
104 JavaScript tests. The combined slow suite passed its full-auto/archive
assertions but failed the separate parity assertion; interruption passed.

Primary evidence is the [archive verification](../../headless-extended-verification-no-ping/test_fresh_hp3_full_auto_globa0/hp3/rrd-verification.json).
The run applied radius 39.01 mm and reported fit score 7.245 with D height
1835.96 mm, compared with the preset's nominal 1900 mm. These figures describe
the actual solver result and limit any inference about calibration accuracy.

See [the abstract](abstract.md) for tested hypotheses, limitations and references,
and [the notebook](research.md) for the failed trial and acknowledgement fix.
