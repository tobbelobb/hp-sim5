# Fuzzing the pure numerical and geometry helpers for invariant violations

Program: `program-2026-11-17-which-new-minimal-reproducible-cases-exist-where-hp-sim5-returns-b3c9c0cc`
Session: `session-2026-11-17-fuzz-the-pure-numerical-and-geometry-helpers-for-invariant-viola-23ae4ea3`
Repository revision: `c2450975` · Node v24.14.1 · Python 3.12.3 (numpy 2.4.3, scipy 1.17.1, pxr/OpenUSD 26.3)
No source file was modified. Everything below is reproducible with `./run_all.sh` (finishes in under one second).

## Question

Do the pure numerical and geometry helpers in `src/js/cable_joints_3d/` (`quaternion.js`,
`vector3.js`, `geometry3.js`, `planar_math.js`, `inertia_tensor.js`, `cable_shape.js`) and the
unit/angle/frame conversions in `src/js/usd/` and `hp-sim-3d/app/` violate their mathematical
invariants — quaternion normalization, `q q⁻¹ = e`, inverse rotations, orthonormal rotation
matrices with `det = +1`, symmetric positive-definite inertia tensors satisfying the principal-moment
triangle inequality and the parallel-axis theorem, planar angle wrapping and `atan2` quadrants,
degenerate inputs, and agreement with an independent numpy/pxr reference — beyond floating-point noise?

## Method

`fuzz_invariants.mjs` runs 51 invariant checks over deterministic seeded inputs (mulberry32; the
seeds are in the file, so every run is byte-identical). Counts: 4000–6000 quaternion cases,
3000–4000 tangent cases per family, 10 552 circle–circle common tangents, 20 000
segment–sphere cases, 3000 random point-mass bodies for the inertia identities, 600 cable shapes
at 256 segments, and 3000 spool frame decompositions.

Tolerances are stated per check and are set a few ulp above the observed rounding floor for the
input magnitudes used (positions up to 5 m, tensors 0.1–5 kg·m²). A check fails only if it exceeds
its tolerance; the tolerance column is printed next to each measurement in `logs/fuzz_invariants.txt`.

`check_with_numpy_pxr.py` is the independent reference: `pxr` (the real USD implementation) builds
each `xformOp:rotate*ABC` matrix, numpy recomputes quaternion→matrix and `R I Rᵀ`, `numpy.linalg.inv` /
`pinv` recompute the inverses, and scipy fits a catenary to the JS cable samples and integrates its
arc length with `quad`. `emit_js_reference.mjs` dumps the JS-side inputs and outputs.

Two deliberately minimal repros (`repro_rotate_zero_axis.mjs`, `repro_normalize_range.mjs`) exit
non-zero with an assertion message while their defect is present. `probe_limitations.mjs` measures
the graceful-degradation boundaries; those measurements are quoted under *Limitations* and are **not**
claimed as defects.

## Findings

### F1 — `Vector3.rotateAroundAxis()` is not a rotation for a zero-length axis (genuine, latent)

`src/js/cable_joints_3d/vector3.js:32-45` normalizes the axis, then applies Rodrigues' formula. When
the axis is `(0,0,0)` the normal stays `(0,0,0)`, so `dot = 0` and `cross = 0` and the result is
`v · cos(angle)`.

```
node repro_rotate_zero_axis.mjs      # exit 1 while the bug is present
```

| quantity | expected | actual | error | tolerance |
|---|---|---|---|---|
| `norm(R(0, π/2)·(1,2,3))` | 3.7416573868 | 2.2911043711e-16 | 3.7417 (100 %) | 3.7e-12 |
| `R(0, π/2)·(1,2,3)` | (1, 2, 3) | (6.12e-17, 1.22e-16, 1.84e-16) | 3.74 | 1e-12 per component |
| `R(0, π)·(1,2,3).x` | 1 | −1 | 2.0 | 1e-12 |
| `R(axis=1e-310, 0.5)·(1,2,3)` | finite | (NaN, NaN, NaN) | — | must be finite |

**Why it is a bug, not style.** A rotation must satisfy `|R v| = |v|`; the method performs its own
axis normalization, so it claims to accept any non-zero axis, and it silently returns a scaled vector
(100 % magnitude error, not noise). Every other zero-normal guard in the codebase takes a defined
fallback instead (`geometry3.js:14`, `inertia_tensor.js:267`, `hangprinter_spools.js:35`,
`cable_joints_core.js _projectPointToPlane`), so the missing guard is an inconsistency as well.
Throwing would also be an acceptable contract; returning `v·cos θ` is not.

**Strongest alternative explanation.** No live call site in `src/`: `cable_joints_core.js:485
_rotateAroundAxis` is unused, and it passes a plane normal that is already guarded against zero. The
only callers anywhere are archived research prototypes under
`output/research/sessions/2026-10-20-interactive-research-62300644/*-core.mjs`, which pass guarded
plane normals too — so no current simulation result is affected. That is true, and it is why F1 is
labelled *latent*. It remains a genuine defect of a public helper that any new caller can hit.

### F2 — `normalize()` returns a non-unit value outside the representable length range (genuine, latent)

`Quaternion.normalize()` (`quaternion.js:69-84`) computes `lengthSq` first, which overflows to
`Infinity` for any component ≥ ~1e154, so `l = 1/Infinity = 0` and every component becomes 0 — a
silent zero quaternion, not the identity. `Vector3.normalize()` (`vector3.js:46`) computes `1/l`, which
overflows to `Infinity` for a subnormal length, producing `Infinity` and `NaN` components.

```
node repro_normalize_range.mjs       # exit 1 while the bug is present
```

| input | expected norm² / length | actual |
|---|---|---|
| `Quaternion(1e200,0,0,1).normalize()` | 1 | (0, 0, 0, 0), norm² = 0 |
| same quaternion used in `transformVector((1,2,3))` | length 3.742 | (0, 0, 0), length 0 |
| `setFromAxisAngle((1e200,0,0),0.5).normalize()` | 1 | (0, 0, 0, 0) |
| `Vector3(1e-310,0,0).normalize()` | (1, 0, 0) | (Infinity, NaN, NaN) |

**Why it is a bug.** The `l === 0` guard is the only degenerate case the code intends to handle, and
the postcondition "normalize returns a unit vector" fails catastrophically (100 % error, then
`transformVector` collapses every vector to the origin). The fix is a two-step rescale by the largest
component — standard practice, not a fundamental limit of doubles.

**Strongest alternative explanation.** No reachable path produces such magnitudes: all quaternions come
from products of unit quaternions that are renormalized every step, and no scene coordinate is near
1e±300. This is why F2 is also labelled latent, and why it should not change any trust in current results.

## Negative results (invariants that hold)

Full machine-readable list in `results.json` → `negative_results` (55 entries). Highlights, worst error
over the seeded campaign:

* **Quaternions** — `|q₁q₂|² = 1` (1.1e-15); `q q⁻¹ = e` (6.7e-16); rotate-then-unrotate recovers the
  input (1.6e-14); `(a*b)` rotating `v` equals `a` rotating `b` rotating `v` (2.3e-15);
  `transformVector` is exactly `q (0,v) q⁻¹` (0.0); `transformVectorInto` is safe when the destination
  aliases the input (0.0). There is **no `slerp`** anywhere in `src/`, `hp-sim-3d/`, `tests/` or
  `scripts/`, so the slerp part of the question has no code to fuzz.
* **Rotation matrices** — `RᵀR = I` to 2.4e-15 and `det R = +1` to 2.2e-15 over 4000 random
  quaternions; `R` and `q·v` agree with numpy to 4.7e-16 / 1.3e-15 and with `pxr.Gf.Rotation` to 6.7e-16.
* **USD frame conversion** — `readOrientationAttribute` reproduces pxr's
  `xformOp:rotate{XYZ,XZY,YXZ,YZX,ZXY,ZYX}` matrices to ≤ 8.9e-16 (48 cases, float32-authored input),
  and USD does compose as `R_last···R_first`, i.e. `Rz Ry Rx` for `rotateXYZ`, exactly as the reader
  assumes. (The separate *translation/xformOpOrder* and `metersPerUnit` defects are pre-existing findings
  of the sibling session `…-hunt-for-wrong-results-in-machine-geometry-scene-config-loading-f6d4333b`
  and are not re-reported here.)
* **Tangent geometry** — the tangent point lies on the circle/sphere to 3.7e-14, the span is
  perpendicular to the radius to 8.1e-13, and the direction of travel equals the wrap velocity
  (−n×r for cw) to 3.7e-14 for point↔circle, 9.0e-15 for all 10 552 circle–circle tangents, and
  5.3e-14 for the in-plane part of sphere–sphere tangents. This is a strong check of the branch
  signs for all four `(cw, pointIsFirst)` combinations.
* **Planar angles** — `signedArcLengthOnWheel` equals the wrapped `atan2` difference (2.7e-15), is
  ≥ 0 when `force_positive`, and the cw and ccw arcs sum to `2πr` (0.0), including the ±π boundary;
  `rightOfLine`/`rightOfPlane` match the analytic cross/triple products in all 8000 cases including
  collinear points.
* **Inertia** — world tensors are symmetric (8.9e-16) and positive definite (no negative eigenvalue);
  `invertMatrix3(W)·W = I` (5.3e-15); `parallelAxisTensor` sums equal a direct point-mass sum
  (7.1e-15); the principal-moment triangle inequality `I₃ ≤ I₁+I₂` is violated by at most 3.6e-15
  relative over 3000 random composite bodies (floating-point level);
  `effectiveInertiaAboutWorldAxis = uᵀ R I Rᵀ u` (2.5e-15),
  `applyWorldInverseInertia = R I⁻¹ Rᵀ v` (3.8e-15), `inverseInertiaQuadraticForm = vᵀ I⁻¹ v` (1.6e-15).
  For exactly singular rod tensors over 3000 random orientations the Jacobi pseudo-inverse satisfies
  `W P W = W` to 7.8e-16 and agrees with `numpy.linalg.pinv(rcond=1e-15)` to 4.4e-16.
* **Cable shape** — endpoints exact (0.0), taut cable exactly straight (0.0), no NaN/Inf, and an
  independent scipy least-squares fit puts every sampled point of a slack cable on one catenary to
  1.1e-14 relative, with a numerically integrated arc length equal to the rest length to 4.7e-14.
  That also validates the bisection in `solveCatenaryScale` far more tightly than the module's
  segment-count-limited rendering.
* **Frames in `hp-sim-3d`** — `decompose/composeSpoolOrientation` round-trips to 7.2e-16, the
  constrained orientation is a pure twist about the spool axis (6.6e-16), the angular-velocity
  projection is exact (3.6e-15), and `estimateEffectorRotation` recovers an applied rotation over 2000
  synthetic frames to 1.7e-14 (quaternion distance).
* **Degenerate inputs** — zero mass, zero tensor, zero-length segments, duplicate points and
  `closestPointOnSegment(p, a, a)` all return finite, well-defined values (`parallelAxisTensor(0,·) = 0`,
  `closestPointOnSegment(p,a,a) = a`, `segments = 0` yields only the end point).

## Limitations (measured, not claimed as defects)

1. **Interior tangent fallback.** `_tangentPointCircle` returns the radial nearest point when the
   attachment point projects inside the guide circle. Measured: `|T−C| = r` but
   `|cos(radius, span)| = 1.000000` (a true tangent gives 0) for `|p| = 0.999, 0.9, 0.5, 0` radii; in
   3D a point 3.04 m from the centre whose in-plane projection is 0.5 m also gives `|cos| = 1`. No
   tangent exists there, so this is a defensive branch — but it is silent, and a caller that believes
   it received a tangent can be off by up to 90° in attachment direction. Realistically reachable only
   for a cable endpoint whose projection lands within the guide radius.
2. **Near-singular pseudo-inverse band.** `M P M = M` holds to 9.8e-16 when `I_min/I_max ≤ 1e-16`
   (the direction is correctly dropped) and degrades to 1.2e-1 at `1e-15`, 1.1e-3 at `1e-13`,
   8.6e-7 at `1e-10`, 8.1e-11 at `1e-6`, with `max|P|` growing to 1e15. This is the FP noise floor of
   the eigendecomposition meeting the `3·ε·max|diag|` truncation threshold — conditioning, not a bug.
   All shipped `physics:inertiaTensor` values are isotropic diagonals (ratios 1), and a 1 m rod of
   1 mm radius (ratio 1e-6) still yields a 1e-10-accurate inverse.
3. **`angleTo` resolution.** `acos` loses everything below ~1e-8 rad: `angleTo((1,0,0),(1,1e-8,0)) = 0`
   instead of 1e-8. Irrelevant against the 0.5° scene tolerances.
4. **Parabolic cable fallback.** For a horizontal span ≤ 1e-5 m the catenary solver bails out to the
   parabolic branch, which sags along `up`. A vertical cable with `L = 1.5·straight` then renders with
   `polyline = 1.000000` instead of 1.5 (−33.3 %); at `h = 1e-4` it is already −0.0 %. Realistic spans
   (0.206 m) with slack 1e-6 … 1e-2 are −0.0000 % … −0.0003 %. This matches the documented
   "parabolic approximation when nearly vertical".
5. **Multi-turn wraps.** `signedArcLengthOnWheel` returns the preferred-direction arc modulo `2π`
   (`sweep 2π+0.3` and `4π+0.3` both → 0.3 rad). Two attachment points cannot determine a multi-turn
   wrap, so this is a model limit; authored `cablePath:stored` carries it instead.
6. **No physical validation of authored tensors.** `normalizeInertiaTensor(diag(1,1,10))` (which
   violates `I₃ ≤ I₁+I₂`) is used verbatim. It is a shape/type coercion with finite fallbacks by design;
   shipped tensors are isotropic, and tensors produced by `parallelAxisTensor` addition satisfy the
   inequality automatically.
7. **Unfinished audit.** I attempted to audit every baked attachment point of the shipped scenes.
   Reconstructing each rolling body's centre from `xformOp:translate` produced `|P−C|/r` errors up to
   13 % on `hp4`/`hp5_pulley_reduced`, traced to `pinhole_gate` steps where link index `i` is not
   bounded by `jointResults[i].body0Path`. That audit is therefore not reported; it says nothing about
   the helpers.
8. **Scope.** `cable_joints_core.js` also contains pure angle conversions (`_localTwistAngle`,
   `_deltaAngleAroundAxis`, `_rotationVectorBetween`) that are not exported; testing them would require
   re-implementing them, which would not test the shipped code, so they are out of scope here.

## What would change the conclusion

* Any live call path that can pass a zero or subnormal axis to `Vector3.rotateAroundAxis`, or a
  quaternion/vector outside `[1e-154, 1e154]` to `normalize()`, would promote F1/F2 from latent to
  load-bearing.
* A shipped scene (or a UI upload path) that reaches the interior-projection tangent branch would turn
  limitation 1 into a measurable geometry error; the 90°-class residuals above show the magnitude it
  would have.
* If a future body authors a principal-moment ratio near 1e-14, limitation 2 becomes operationally
  relevant (inverse inertias ~1e14).
