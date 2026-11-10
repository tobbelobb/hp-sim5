# Trial-003 workspace-size timeout

Objective: diagnose the next manual browser capture failure and fix the cause.
Accepted steering: preserve requested versus observed speed semantics; do not disturb the browser. The recording manifest is finalized. No browser or firmware service was changed during investigation.

Hypothesis: the active workspace movement outlasts the ordinary 30-second quieting budget. E1 inspects recorded collector commands and browser encoder replies. E2 replays the recorded commands and poll schedule in an isolated production JS physics world. E3 uses the patched measurement routine on the replayed world. E4 checks finite selected-clock and wall deadlines, slow/fast simulation and transport cleanup with synthetic clocks.

Budget: three isolated replays of at most 1,700 simulation seconds each; no hardware or live-browser movement. No optimization or full autocal run requested for verification.

E1 confirms failure in measureMaxTravelMm, after successful force tuning and tighten/release. Final forces: [0.0113635757312, 9.66368350162, 9.66368350162, position]. Encoder A travels about 1,157 degrees by timeout; no force-release command follows the failed wait. E2 finds a 37.7-second wait with the recorded polling schedule. The first replay with regular 500-ms polling found a brief quiet window at 28 seconds; this was not sufficient to reproduce the browser failure. The jitter-aware replay is the relevant result, and is preserved in replay.mjs and replay-results.json.

E4: 16 suites, 79 tests passed. Ordinary settling retains its 30-second default; active size measurement and sweep pullout get a bounded 120-second selected-clock budget. The wall deadline remains independent and tolerances unchanged. Size measurement finally holds all motors; the CLI attempts a hold before disconnecting after other collection errors. Progress now names these active phases.

E3: the patched measurement routine completes in 38.16 simulation seconds including its 0.46-second ramp, measures 611.35 mm raw travel, and issues position hold to all four motors. Results are in measurement-probe-results.json.

Next decision: retry the full manual browser collection. Full calibration success and hardware behavior remain untested.
