# Force realisation in autocal data: fit the realised force, then compliance

[Simulation evidence](report.md), reviewed (accept with caveats) and revised. The sim's force error is torque-mode cogging. Fitted jointly with anchors and k, a per-motor force model is needed:
- **Model.** Blind period scan, per-motor offset/scale contrasts, and direction terms.
- **Without it.** An unmodelled single-motor offset leaks into k and top z; constant per-motor offsets are not unobservable, as first claimed.
- **Blind test, HP3e.** On non-50/turn ripple (live 137/turn) with a 0.5 N offset on one motor, the scan finds the period and recovers the offset. Max error is 0.17 mm mean over 7 held-out cases (6/7 < 0.3 mm), against 3.8 mm for plain joint k.
- **Oracle-basis results.** HP3e 0.16 mm, HP5r 0.74 mm.
- **Not done.** The HP5r blind test. External force calibration becomes a cross-check and the source of the absolute scale.
