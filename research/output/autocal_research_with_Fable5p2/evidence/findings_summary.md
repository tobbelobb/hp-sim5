---
name: autocal-stretch-bias
description: "HP3 autocal's ~7 mm anchor error is unmodelled line stretch (0.1–0.6 mm) amplified by an ill-conditioned fit; found 2026-11-14"
metadata:
  node_type: memory
  type: project
  originSessionId: 6be52c02-0bfa-4f95-a23e-a262242026a8
  modified: 2026-11-14T21:08:15.869Z
---

Found 2026-11-14 on HP3 default `--sim` / `--headless-sim` runs (simulation only, nothing changed in the repo):
- Replacing the encoder lengths with true geometric lengths from the recorded effector pose gives 0.00 mm anchor error with the same sweeps, so the geometry model and optimiser are exact. The whole error is encoder length minus true geometry, which is line stretch under the 1–5.6 N measurement tensions. Reference state at encoder origin: ABC slack, D carries the 0.8 N effector.
- The fit treats flex as noise only (`sigma_flex`), never corrects it. The stretch is smooth, so anchors absorb it and residual RMS stays 0.05–0.08 mm ("near-perfect" score while 7–26 mm off).
- Conditioning: bootstrap sweeps (D fixed at 0, one low anchor at about +400 mm) have weak directions with gain 120–730 mm per mm of coherent error; D.z is weakest. At least 90 % of the stretch must be removed for < 1 mm.
- HP3 default `solverIterations` 1 makes ABC lines act like EA ≈ 8.5–12.7 kN instead of the configured 20 kN; `--solver-iterations 20` restores about 22 kN. It halves XY error but D.z got worse (−26 mm).
- `--project-zero-tension` made it worse (D.z +52 mm, residual 0.24 mm).
- Dataset gotcha: in the second sub-sweep of each sweep the torque-mode motor is `drive_anchor`, not `sensor_anchor`. Realised sensor tension is up to 0.35 N above commanded.

**Why:** the user had tried for a long time to get anchors below 1 mm and suspected the optimiser.
**How to apply:** judge autocal by anchor error against scene truth, not by fit score. Fast diagnosis: headless `--max-steps 1 --recording-mode sampled`, read the RRD with `rerun.chunk.RrdReader`, compare `mu × mm_per_degree` with geometry from the Effector translation. Related: [[hp4-solver-convergence-and-flex-ma]], [[cable-verification-tools]].

Follow-up 2026-11-14 (scratch study, nothing changed in the repo). User constraints: raise solver iterations only in autocal runs; autocal must not rely on configured stiffness (should work at 12 kN with 20 kN configured); favourite direction is stretch-insensitive sweeps, any sweep type allowed.
- Theory: at a pose the tensions have one free dof (pretension along the null vector n of the line directions), and stretch enters the constraint residual as sum n_i·stretch_i, so it never averages out. Uniform strain is harmless (scale ~1e-4). Harm comes from (a) reference taken with ABC slack while measurements are tensioned, (b) tension varying between poses and lines.
- No pose set alone fixes the rigid fit (best volumetric sets still 6–12 mm). Fix that worked: poses spread over height (D shortened, z 0–1200) + sensor force chosen per pose so the n-weighted mean per-strand tension is constant (needs no stiffness) + one extra fit parameter (length offset on one line). Real simulator, 55 poses, direct motor commands, own LSQ fit: shape error 0.12–0.62 mm at 20 iterations, 1.2 mm at 1 iteration; constant force gives 2.6 mm with the offset, 8.2 mm without.
- The offset is equivalent to a common anchor translation: fitted home sits ~1–2 mm lower in z than the slack-line home (lower lines are near horizontal, so 0.3 mm of stretch moves the effector ~2 mm vertically).
- Floor: torque-mode motor realises ±0.36 N off the commanded force (cogging), independent of force level.
- D is always in the fixed set (`mustBeInFixedSet: [3]`) and the 2026-11-14 run only ever chose D delta 0 (negative deltas are allowed but scored slightly lower), so every pose lay on one sphere around D; that is why D.z was the weakest direction.
