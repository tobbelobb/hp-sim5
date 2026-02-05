# Substep 4: 3D Cable-Joint Scaffolding (No Rendering)

**Goal**
Prepare the 3D core for the next wave of test replications beyond the initial tangent/math tests, while still excluding rendering.

**Deliverables**
- Decide and document the 3D equivalents of key 2D components, including `PositionComponent` with `Vector3`, `OrientationComponent` with `Quaternion`, and cable link components with plane-normal metadata.
- Establish 3D system stubs for attachment caching and movement updates, mirroring 2D naming and structure.
- Identify the next set of 2D tests to port, in order, and which 3D systems they will drive.

**Implementation Notes**
- Keep physics core data independent of rendering, consistent with later Three.js integration.
- For future plane selection, encode the cable segment’s “wrap plane normal” in the cable link or constraint component.
- Minimize allocations inside system updates to align with `render-avoid-allocations` and `memory-reuse-objects` guidance.

**Files To Touch (Scaffold)**
- `src/js/cable_joints_3d/ecs.js`
- `src/js/cable_joints_3d/commonSystems.js`
- `src/js/cable_joints_3d/cable_joints_core.js` (new, minimal)
- `src/js/cable_joints_3d/cable_attachment_cache_system.js` (new, minimal)

**Candidate Next Tests To Port**
- `tests/js/cable_joints/closestPointOnSegment.test.js`
- `tests/js/cable_joints/lineSegmentCircleIntersection.test.js` (3D sphere variant)
- `tests/js/cable_joints/signedArcLengthOnWheel.test.js` (3D arc length on sphere or around a cylinder in a plane)
- `tests/js/cable_joints/cableAttachmentUpdateSystem_updateAttachmentPoints.test.js`

**Acceptance Criteria**
- Clear mapping from 2D components and systems to 3D equivalents.
- A documented plan for the next test ports after the initial four.

**Risks / Open Questions**
- 3D attachment update logic depends on a robust plane choice per cable segment; defer implementation details until plane selection is settled.
