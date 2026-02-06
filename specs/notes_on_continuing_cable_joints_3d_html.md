# Notes On Continuing `cable_joints_3d.html`

This is a handoff note for the 3D HTML demo in `tests/html/cable_joints_3d.html`. It lists observed bugs and likely causes, plus pointers for parity with the 2D reference (`tests/html/cable_joints.html`) and the 3D core/tests.

## Baseline References
- 2D reference demo: `tests/html/cable_joints.html`
- 3D core and tests: `src/js/cable_joints_3d/`, `tests/js/cable_joints_3d/`
- 3D spec notes: `specs/cable_joints_3d_up_until_rendering/*`

## Current State (Post-Alignment)
- The 3D demo now mirrors the 2D topology: **two dynamic balls + one static rolling wheel**.
- Start positions are aligned with the 2D reference (`ball1: (1.05, 0.75)`, `obs1: (1.5, 1.5)`, `ball2: (1.6, 0.55)` with `z = 0`).
- Link types and `cw` are aligned with 2D (`['hybrid-attachment', 'rolling', 'hybrid-attachment']`, `cw = [true, true, true]`).
- 3D now uses the XPBD velocity update path (`PrevFinalPosSystem` + `PBDVelocityUpdateSystem`) instead of ad-hoc render-loop velocity updates.

## Remaining Issues + Likely Causes

1. **Right/left ball can still intersect the wheel (or each other).**
   - Symptom: The right ball drops, the cable begins to wrap, and then geometry intersects (left ball sinks into the wheel).
   - Likely cause: No collision system is present in 3D. The 2D demo relies on:
     - `PBDBallBallCollisions`
     - `PBDBallObstacleCollisions`
     - `BallObstacleBumpSystem`
   - None of these exist yet in `src/js/cable_joints_3d/`. Without collision and push-out, bodies can overlap indefinitely.

2. **Cable slack / friction / hybrid edge behavior is incomplete.**
   - Symptom: Slack behavior and hybrid link transitions feel off or unstable compared to 2D.
   - Likely cause: 3D has no `CableSlackSystem` and no `CableFrictionSystem`. The 2D demo runs:
     - `CableSlackSystem` to prevent negative rest lengths / slack misbehavior
     - `CableFrictionSystem` to update stored length on rolling links with friction
   - These systems are missing in `src/js/cable_joints_3d/`.

3. **Rotation-driven cable changes are missing or inert.**
   - Symptom: Rotational effects (hybrid links, friction on rotating links) do not accumulate.
   - Likely cause: 3D does not have `PrevFinalOrientationSystem`, `AngularMovementSystem`, or `PBDAngularVelocityUpdateSystem`. Orientation deltas are used in `CableAttachmentUpdateSystem` to compute arc-length deltas, but the bodies never rotate unless you manually update orientation.

4. **System order differs from the 2D reference.**
   - Symptom: Subtle differences in cable behavior vs 2D.
   - Likely cause: The 2D demo order is:
     1. `PrevFinalPosSystem`
     2. `PrevFinalOrientationSystem`
     3. `GravitySystem`
     4. `MovementSystem`
     5. `AngularMovementSystem`
     6. `CableAttachmentUpdateSystem`
     7. `CableAttachmentCacheSystem`
     8. `CableSlackSystem`
     9. `PBDCableConstraintSolver`
     10. collision systems
     11. `CableFrictionSystem`
     12. `PBDVelocityUpdateSystem`
     13. `PBDAngularVelocityUpdateSystem`
   - The 3D demo does not currently mirror this full chain. It uses `CableAttachmentCacheSystem` **before** `CableAttachmentUpdateSystem` (which aligns with the 3D spec/tests), while the 2D HTML runs cache after update. That mismatch is a likely source of subtle parity issues and should be resolved intentionally (either follow the spec in both, or match the 2D demo exactly).

## Principles For Continuing Work
- **Match the 2D demo topology and positions first.** Use the same coordinates with `z = 0` to validate parity.
- **Keep link ordering and `cw` semantics identical** to 2D. The `_effectiveCW` rules are easy to get wrong; see the 2D tests and comments in `tests/js/cable_joints/cableAttachmentUpdateSystem_updateAttachmentPoints.test.js`.
- **Stick to plane-based geometry** (`geometry3.js`). All tangents and arc-lengths should use the link’s `cablePlaneNormal`.
- **Port systems one-for-one** from 2D when behavior depends on them (slack, friction, collisions, velocity/ang velocity updates).

## Action Checklist (Suggested)
1. Port and wire the missing systems (slack, friction, collisions).
2. Add angular movement + angular velocity updates once orientation matters for rolling/friction.
3. Re-verify and standardize system order between 2D and 3D.
