# Substep 1: 3D Foundations (Vector3 + Gravity System)

**Goal**
Establish the minimal 3D foundation required for the first tests: `vector3.test.js` and a new 3D `gravitySystem.test.js`.

**Deliverables**
- `src/js/cable_joints_3d/ecs.js` with a minimal ECS and 3D components.
- `src/js/cable_joints_3d/commonSystems.js` with a 3D `GravitySystem`.
- `tests/js/cable_joints_3d/gravitySystem.test.js` mirroring the 2D test logic, but using `Vector3`.

**Implementation Notes**
- Make them similar to equally named files in `src/js/cable_joints`.
- Keep it DRY (don't repeat yourself). Reuse logic from `src/js/cable_joints` instead of writing separate identical logic in `src/js/cable_joints_3d`.
- However, keep changes in `src/js/cable_joints` fairly small, and don't break the existing 2D code relying on `src/js/cable_joints`. If large changes are needed, prefer writing new completely separate logic for the 3d version.
- Include components required by the tests: `World`, `VelocityComponent`, `GravityAffectedComponent`, and optionally `PositionComponent` for consistency.
- Use `Vector3` for gravity and velocity.
- Maintain the same runtime behavior as 2D: skip if gravity resource is undefined, and skip entities without `VelocityComponent`.

**Files To Touch**
- `src/js/cable_joints_3d/ecs.js`
- `src/js/cable_joints_3d/commonSystems.js`
- `tests/js/cable_joints_3d/gravitySystem.test.js`

**Acceptance Criteria**
- `tests/js/cable_joints_3d/vector3.test.js` passes unchanged.
- New `tests/js/cable_joints_3d/gravitySystem.test.js` passes.

**Risks / Open Questions**
- Naming conventions for 3D ECS modules should be stable for later use; keep them parallel to 2D but within the 3D directory.

