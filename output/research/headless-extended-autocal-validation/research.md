# Headless extended autocal recording validation

Objective: let standalone `autocal.py --headless-sim` preserve physics, collection
communication, numerical artifacts and Python logs in one RRD without a browser.

Constraints: production JavaScript physics, existing collector and optimizer;
explicit source clocks; no shortened defaults for the automatic-completion check.
No hardware or calibration-accuracy claim. No movement/compute budget supplied;
each full-auto integration test has a 1800-second process deadline.

2026-10-09: implemented opt-in `--extended-reference` with an owned recorder and
support for an external recorder URL. Reused browser snapshots, sampling and
acknowledgements. Headless boundary events and clock paths identify their source.

Trial 001: default fresh HP3 full-auto collection failed during force tuning.
The headless wait tried to drain all events within ten seconds despite ongoing
acknowledgements. The RRD and manifests are preserved in
`../../headless-extended-verification/`. The interruption trial in that directory
also hit this bug before its first completed point; it did not test interruption.

Fix: wait only for physics capacity during stepping; finalization drains all
physics/events. The acknowledgement watchdog resets on progress, with an
independent wall deadline. Added a regression for a continuing event stream.

Trial 002: default full-auto in `../../headless-extended-verification-progress/`
disconnected during high-rate force-tuning traffic after 1425.22 simulated
seconds. The partial RRD finalized with no rejected messages. Default WebSocket
keepalive interacting with receive backpressure is the suspected cause; the
old receiver silently caught connection-close exceptions, so the close reason
was not retained. Added close-reason diagnostics and disabled keepalive in
extended mode, which already has acknowledgement watchdogs.

Trial 004: default full-auto completed normally in
`../../headless-extended-verification-no-ping/`: eight sweeps, 160 points,
5354.192 simulated seconds, 1525.41 wall seconds. Its single finalized RRD has
267,711 physics observations, 616,025 events and no rejected messages. Direct
archive checks verified all 160 measurements, 13,212 paired collector command
outcomes, seven stage snapshots, exact final JSON/text/JSONL artifacts and exact
concatenated timestamped text writes. See `hp3/rrd-verification.json` under the
fresh-run directory. Later source-label/provenance additions have separate fast
live checks; trial hashes identify the code actually loaded at startup.

The combined slow suite failed its subsequent matched-input parity check at one
point (second sweep, point index five): maximum raw/mean encoder difference
0.16 degrees, length difference 0.0837758 mm and timestamp difference 502 ms.
Other encoder differences were at most 0.01 degrees. Existing tolerances remain
0.03 degrees and 0.01 mm. The full RRD assertions and interruption test passed.

Trial 005: focused current-service parity rerun in
`../../headless-extended-parity-repeat/` reproduced the exact differences.
Trial 006: the diagnostic using `HEAD:scripts/autocal_headless.mjs` in
`../../headless-extended-parity-baseline/` reproduced exactly the same parity
differences, including the 502-ms clock offset. This supports a pre-existing
comparison discrepancy for these inputs. Original service/harness source and
revision/hash identity are retained there. Temporary executable source copies
were removed from the production/test directories. Sandbox port binding required
approval for that diagnostic.

Trial 003: real bootstrap interruption passed in 53.44 seconds in
`../../headless-extended-interruption-progress/`. Partial measurements and a
finalized incomplete RRD survived SIGTERM; all owned services exited.

Fast lifecycle/source-clock checks passed, including 25×, scene reset, external
every-step/full geometry, disconnection failure and service ownership. Final
artifacts are being retained in `../../headless-extended-fast-final/`.
The final source-label/provenance checks are retained in
`../../headless-extended-fast-provenance/`.

Outcome: requested recording capability is complete. Final fast Python checks
passed 78 tests; targeted JavaScript checks passed 104. The slow suite's full-auto
and RRD assertions passed, but its parity assertion failed; interruption passed.
No tolerance was relaxed. Backend parity for the failing point is separate
follow-up work, not a reason to claim calibration accuracy from this recording.
