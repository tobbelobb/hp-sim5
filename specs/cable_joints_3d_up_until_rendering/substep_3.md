# Substep 3: Tangent Circle-Circle HTML Parity in 3D

**Goal**
Match the 2D HTML test cases for circle-circle tangents using 3D planar geometry, establishing parity for `tangentCircleCircleHTML.test.js` in 3D.

**Deliverables**
- `tests/js/cable_joints_3d/tangentCircleCircleHTML.test.js` that mirrors the 2D test cases but uses `Vector3` with `z = 0` and `planeNormal = (0,0,1)`.
- Any minor adjustments to `tangentFromSphereToSphere` needed for numerical precision or cw/ccw handling.

**Implementation Notes**
- Keep expected values identical to the 2D tests, with `z = 0`.
- Use the same cwA/cwB cases (TT, TF, FT, FF) and match the same tangent branch selection as the 2D implementation.
- If tolerance differences appear, prefer adjusting the 3D implementation to match 2D rather than loosening test thresholds.

**Files To Touch**
- `tests/js/cable_joints_3d/tangentCircleCircleHTML.test.js`
- `src/js/cable_joints_3d/geometry3.js` (only if needed)

**Acceptance Criteria**
- New `tangentCircleCircleHTML.test.js` passes with the same numerical expectations as 2D, plus `z = 0` checks.

**Risks / Open Questions**
- Exact numerical parity depends on the 2D tangent formulas and the projection basis. If the basis changes sign, cw/ccw will flip.

