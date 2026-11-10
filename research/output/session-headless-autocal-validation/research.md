# Headless full-auto autocal verification

Objective: implement browser-free production JS collection for the unchanged
full-auto loop, then verify independent matched Chromium/headless measurements
and fresh HP3 automatic completion with global radius fitting.

Hypotheses: a controlled fixed-step clock produces matched collection in Node
and Chromium; the existing HP3 full-auto loop completes normally using this
backend with the user's global-radius settings.

Constraints: one world per run, preserve production collection defaults and
solver options, preserve partial points and failed trials, close owned services.
No hardware accuracy claim. No algorithm or candidate-ranking changes.

Experiment IDs: HP3-FULL-01 (initial unbaked scene, failed), HP3-FULL-02 (fresh
HP3 full run after using the production cable baker), HP3-BROWSER-01 (planned
independent Chromium bootstrap collection). Initial sandbox socket failure is
retained separately. Compute budget: existing full-auto limit of 20 iterations;
collection/movement budget: production defaults, no manual stop or artificial
acceptance thresholds. Verification trials may be repeated only after failures
or material source changes.

Artifacts: ../../headless-verification/hp3/ contains all full-auto logs, datasets,
partial-point journals and numbered service manifests. Next decision: check
normal automatic acceptance and compare all bootstrap measurements in Chromium.

HP3-FULL-02 completed automatically after 6 iterations and 8 sweeps, applying
M669 and M666 (global radius 39.16 mm); 5679.692 simulated seconds in 382.94
wall seconds. The solver reports concerning quality (25.01); completion is
not an accuracy claim. HP3-FULL-03 repeated this through the integration test
with final firmware readback, 386.57 wall seconds and the same simulation time.

HP3-BROWSER-01 ran independent autotuning and selected a different span; its
measurements differ by up to 1.60745 mm and are not a matched-input parity test.
HP3-MATCHED-01/02 held forces and 300 mm span fixed in fresh Chromium and Node
worlds, measuring 60 points each. Maximum differences: 0.02 deg raw angles,
0.00523599 mm lengths, 0.01 deg noise means; absolute timestamps differ by
1000 ms following a settling-window boundary. The initial strict timestamp
assertion failed. Measurement tolerance is now three RRF reporting quanta
(0.03 deg) and 0.01 mm, with finite/monotonic simulation timestamps and 2 ms
sample-duration tolerance; absolute collection timestamp differences are
diagnostic. This is explicit tolerance selection based on observed resolution
and independent settling, not a claim of identical trajectories.

HP3-INTERRUPT-01 passed: SIGTERM during bootstrap retained completed-point
journals, recorded an interrupted manifest, and stopped the Node collector,
physics/bridge and owned RRF. The final integration suite repeats fresh
full-auto, matched-input collection with documented tolerances and interruption
cleanup. HP3-FINAL-01 passed both extended tests in 498.59 seconds, including
matched 60-point collection under the documented tolerance contract.

Final review found the size-tuning ramp's remaining 100 ms `baseSleep` call.
It now uses the simulation clock when available, retaining the original browser
fallback. HP3-FINAL-CLOCK-01 repeats the whole extended suite after this physical
wait correction; both tests passed in 573.81 seconds. Full-auto accepted at
iteration 9 with 11 sweeps/220 points, radius 39.02 mm and reported fit score
1.039. Simulation time was 6487.832 seconds; wall time was 435.65 seconds for
the full-auto portion. Matched-input parity again measured 0.02 deg maximum
angle difference and 0.00523599 mm length difference. Partial-point interruption
and process cleanup passed. Collector regressions passed 55 tests after the
correction. Fast Python regressions passed 75 tests.

Status: complete. The nominal D height comparison (1900 mm preset vs 1852.29 mm
fit) remains a parameter-accuracy limitation; this implementation does not alter
the fitter or investigate that discrepancy. Final evidence is in
../../headless-verification/final-clock/ and the tracked report is
../../../research/headless-autocal-verification.md.
