# Substep 2: Plane-Based 3D Tangents (Point-Sphere + Sphere-Sphere)

**Goal**
Implement deterministic 3D tangent calculations by reducing to 2D in a specified plane, then lifting results back to 3D. This enables `tangentFunctions.test.js` in 3D.

**Deliverables**
- Expand `src/js/cable_joints_3d/geometry3.js` with `tangentFromPointToSphere`, `tangentFromSphereToPoint`, `tangentFromSphereToSphere`, and helpers for plane basis, 2D projection, and 3D lifting.
- Add `tests/js/cable_joints_3d/tangentFunctions.test.js` mirroring the 2D tests, with all points on a known plane (e.g., `z = 0`) and `planeNormal = (0,0,1)`.

**Implementation Notes**
- Use the plane normal to define a right-handed basis `(u, v, n)`.
- Project points to 2D using dot products with `u` and `v`.
- Reuse the 2D tangent logic from `src/js/cable_joints/geometry.js` to compute tangent points in 2D, then lift back with `u` and `v`.
- Resolve cw/ccw ambiguity using the plane normal and right-hand rule so tests are deterministic.
- Reject or warn when `planeNormal` is degenerate.

**Files To Touch**
- `src/js/cable_joints_3d/geometry3.js`
- `tests/js/cable_joints_3d/tangentFunctions.test.js`

**Acceptance Criteria**
- New 3D tangent tests pass with tolerances comparable to 2D tests.
- Existing `geometry3.test.js` and `vector3.test.js` still pass.

**Risks / Open Questions**
- Multiple valid tangents exist in 3D; the API must encode a deterministic choice based on `planeNormal` and `cw`.
- If two spheres have non-parallel axes in future, a single plane may be ambiguous; for now, tests will constrain to a known plane.
