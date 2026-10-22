# Count physical cable loads once per solve

User authorized trying the preferred fix and fresh simulations; reference-007 is stopped and there are no competing user runs. Retain one default substep, force selection, and settling thresholds. No hardware.

Hypothesis: repeated solver visits incorrectly multiply motor load torque, stiffness and damping. Candidate counts the first visit, matching existing first-iteration physical force telemetry. Mirrors JS, Rust native/Wasm, Python, Warp. No more iterations or physics substeps.

Budget: 30 s fixed-torque comparison, analytic fixed-geometry regression, focused backend/physics tests, isolated preflight, then fresh headless HP4 full-autocal bounded to one full-auto iteration at speedup1 and one substep, events-only recording. Speedup1 keeps requested headless waits on actual simulation time. Source previous report ../2026-10-16-hp4-roller-propagation/report.md.

E01: same fixed-torque probe at one step advances84.8mm in30s versus previous56.8mm; K36 instead of72. Not yet evidence of full-autocal success.
E02: JS343 tests passed; native6passed1GCskip;Wasm99passed; opt-in loaded-state stability passed. Existing Python assertion expecting triple fixed load was updated to the corrected invariant; new fixed geometry fixture tests count independence and analytic stiffness/damping/torque.21targeted tests passed.
E03: isolated launcher preflight and socket check passed. First headless CLI trial stopped before motion because --physics-substeps acknowledgement was discarded by the service/bridge. Fixed the response plumbing without relaxing validation; added unit and integration checks.
E04: bounded fresh autocal retry currently running in autocal-retry/. No result available yet. Warp and acknowledgement checks pending. No performance claim against a differently optimized native artifact.

Final steering: stop runs and commit quickly. E04 already completed fourth sweep and reached configured max steps. No timeout; fit underconstrained after1Nforce fallback. Shutdown attempt found no active matching autocal. No further trials. Final checks and limitations in report.
