# Cable Joints 3D Masterplan (Up Until Rendering)

**Summary**
Build a 3D version of the Cable Joints physics core by porting the 2D math, ECS, and cable-joint logic into `src/js/cable_joints_3d`, then replicate the 2D tests in `tests/js/cable_joints_3d`. The first milestones are math and geometry parity, then plane-based 3D tangents (point-sphere and sphere-sphere), then broader cable-joint systems. Rendering is explicitly out of scope for this plan.

**Scope**
- Port and generalize core math and geometry to 3D.
- Add a minimal 3D ECS and common systems needed by tests.
- Implement plane-based 3D tangents consistent with the Cable Joints assumptions.
- Replicate 2D tests in 3D one-by-one, starting with the four specified tests.
- Keep the 3D core renderer-agnostic.

**Out of Scope (For Now)**
- Any Three.js rendering or integration code.
- Integration test expectation changes.
- Full 3D collision and contact system beyond what tests require.

**Key Assumptions From Cable Joints (see ai_docs/CableJoints/CableJoints.md)**
- Cables are modeled by their effect on bodies, not by explicit cable segments.
- Cable motion is planar, in a plane perpendicular to the wheel’s rotation axis.

**3D Generalization Strategy**
- Represent each cable segment’s local "wrap plane."
- Use plane-based reductions: project a 3D problem into 2D, use the existing 2D math, then lift back into 3D.
- The plane normal is explicit in APIs where tangents are computed, to make the choice deterministic and testable.
- Refer to analyzing the existing 2D code in `src/js/cable_joints/*` for minute details on how everything's implemented.
- The function `setupScene` in `./examples/js/slideprinter/setupScene.js` as well as how it's used in hp-sim/assets/hp-sim.js can be enlightening to learn how the current 2D system works.

**Three.js Integration Considerations (Later)**
- Keep core types as plain JS classes without Three.js dependencies.
- Avoid per-frame allocations inside update loops to align with `render-avoid-allocations` and `memory-reuse-objects` guidance from the three-best-practices skill.
- Provide optional adapters in a separate layer (later) to convert between core types and `THREE.Vector3`/`THREE.Quaternion`.

**Test Strategy**
- Mirror 2D tests under `tests/js/cable_joints_3d` with 3D equivalents.
- Focus on deterministic, plane-constrained geometry so expected values remain precise.
- Add new tests only as needed to validate 3D-specific behavior (plane normal handling, ambiguity resolution).

**Substep Overview**
1. 3D foundations: math, minimal ECS, gravity system, and vector tests.
2. Plane-based tangent math in 3D with deterministic handedness.
3. Parity of tangent circle-circle HTML cases in 3D.
4. Scaffold 3D cable-joint core pieces for upcoming test replication (no rendering).

