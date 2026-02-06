# Notes On Continuing `cable_joints_3d.html`

This is a handoff note for the 3D HTML demo in `tests/html/cable_joints_3d.html`. It lists observed bugs and likely causes, plus pointers for parity with the 2D reference (`tests/html/cable_joints.html`) and the 3D core/tests.

## Baseline References
- 2D reference demo: `tests/html/cable_joints.html`
- 3D core and tests: `src/js/cable_joints_3d/`, `tests/js/cable_joints_3d/`
- 3D spec notes: `specs/cable_joints_3d_up_until_rendering/*`

## Observed Bugs + Likely Causes

1. **Endpoints start above the central wheel and the two joint segments cross.**
   - Symptom: The initial state has both endpoints elevated above the wheel and the two cable segments cross each other.
   - Likely cause: The 3D demo uses different starting positions than the 2D reference. In 2D, the wheel is above the balls, and both balls start below it (`ball1: (1.05, 0.75)`, `obs1: (1.5, 1.5)`, `ball2: (1.6, 0.55)` in `tests/html/cable_joints.html`). The 3D demo currently places the wheel at `(0, 0, 0)` and the endpoints above it, which flips the geometry and yields crossing tangents.
   - Also likely: `cw` choice + link types are not aligned with the 2D demo. The 2D demo uses `['hybrid-attachment', 'rolling', 'hybrid-attachment']` and `cw = [true, true, true]`. The 3D HTML currently uses `['attachment', 'rolling', 'hybrid-attachment']`, which alters the tangent selection and can produce crossings when combined with the changed positions.

2. **Left ball is stationary.**
   - Symptom: The left endpoint never moves (even when gravity should affect it).
   - Likely cause: The left endpoint is implemented as a static anchor (mass `< 0`, no `GravityAffectedComponent`). In the 2D reference there are **two dynamic balls** and one static obstacle. The 3D HTML currently uses only one dynamic ball and one static anchor.
   - Fix direction: Mirror the 2D topology: two dynamic balls (`MassComponent > 0`, `GravityAffectedComponent`) + one static rolling wheel.

3. **Right ball falls, cable wraps briefly, then the left ball disappears into the wheel.**
   - Symptom: The right ball drops, the cable begins to wrap, and then geometry intersects (left ball sinks into the wheel).
   - Likely cause: No collision system is present in 3D. The 2D demo relies on:
     - `PBDBallBallCollisions`
     - `PBDBallObstacleCollisions`
     - `BallObstacleBumpSystem`
   - None of these exist yet in `src/js/cable_joints_3d/`. Without collision and push-out, bodies can overlap indefinitely.

4. **Velocity behavior is inconsistent after constraint corrections.**
   - Symptom: Motion can look “sticky” or energy can vanish after constraint corrections.
   - Likely cause: The 3D demo previously computed velocities manually in the render loop and skipped the XPBD-style velocity update system used in 2D.
   - Fix direction: Use a 3D port of `PBDVelocityUpdateSystem` (now added to `src/js/cable_joints_3d/commonSystems.js`) together with `PrevFinalPosSystem` and `PrevFinalPosComponent`. This keeps velocities consistent with PBD position corrections.

5. **Cable slack / friction / hybrid edge behavior is incomplete.**
   - Symptom: Slack behavior and hybrid link transitions feel off or unstable compared to 2D.
   - Likely cause: 3D has no `CableSlackSystem` and no `CableFrictionSystem`. The 2D demo runs:
     - `CableSlackSystem` to prevent negative rest lengths / slack misbehavior
     - `CableFrictionSystem` to update stored length on rolling links with friction
   - These systems are missing in `src/js/cable_joints_3d/`.

6. **Rotation-driven cable changes are missing or inert.**
   - Symptom: Rotational effects (hybrid links, friction on rotating links) do not accumulate.
   - Likely cause: 3D does not have `PrevFinalOrientationSystem`, `AngularMovementSystem`, or `PBDAngularVelocityUpdateSystem`. Orientation deltas are used in `CableAttachmentUpdateSystem` to compute arc-length deltas, but the bodies never rotate unless you manually update orientation.

7. **System order differs from the 2D reference.**
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
   - The 3D demo does not currently mirror this full chain. Matching order matters because attachment updates use cached previous poses.

## Principles For Continuing Work
- **Match the 2D demo topology and positions first.** Use the same coordinates with `z = 0` to validate parity.
- **Keep link ordering and `cw` semantics identical** to 2D. The `_effectiveCW` rules are easy to get wrong; see the 2D tests and comments in `tests/js/cable_joints/cableAttachmentUpdateSystem_updateAttachmentPoints.test.js`.
- **Stick to plane-based geometry** (`geometry3.js`). All tangents and arc-lengths should use the link’s `cablePlaneNormal`.
- **Port systems one-for-one** from 2D when behavior depends on them (slack, friction, collisions, velocity/ang velocity updates).

## Action Checklist (Suggested)
1. Replace the static anchor with a dynamic ball. Add a second dynamic ball so the topology matches 2D.
2. Use the same start positions as `tests/html/cable_joints.html` and a consistent scale.
3. Set `linkTypes` and `cw` to match the 2D demo exactly.
4. Port and wire the missing systems (slack, friction, collisions).
5. Add angular movement + angular velocity updates once orientation matters for rolling/friction.
6. Re-verify the system order against the 2D demo.
