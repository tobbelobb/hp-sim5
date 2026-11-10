# HP4 motor load bookkeeping: one contribution per physical constraint

Recording motor cable load torque, stiffness and damping once per constraint, rather than per solver iteration, improves a matched one-step 30 s pull from56.8to84.8mm without extra integration work. The correction is mirrored in JavaScript, Rust native/Wasm, Python and Warp. A fresh headless one-substep autocal run completed three bootstrap sweeps (60 points) plus a fourth sweep, then stopped at its configured one-iteration limit. It did not time out. Force tuning fell back to1N and the fit remained underconstrained, so this is collection validation, not successful calibration. Default substeps, forces and settling criteria are unchanged.

Topics: HP4, motor loads, solver iterations, implicit stiffness, autocal.

See [report](report.md) for evidence and limitations.
