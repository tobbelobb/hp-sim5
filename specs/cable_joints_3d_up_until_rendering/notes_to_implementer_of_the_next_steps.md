# Notes to Implementer: 3D CableAttachmentUpdate + Solver Port

This is a concise guide for porting the remaining 2D cable‑joint logic to 3D. The goal is to preserve the 2D behavior and tests, but with plane‑based 3D geometry and Quaternion orientation.

## Current 3D Baseline
- 3D math and geometry live in `src/js/cable_joints_3d/`.
- `geometry3.js` already provides plane‑projected versions of:
  - `tangentFromPointToSphere`, `tangentFromSphereToPoint`, `tangentFromSphereToSphere`
  - `signedArcLengthOnWheel` (projected to wrap plane)
  - `rightOfPlane` (projected “right‑of‑line”)
- `CableLinkComponent` has `cablePlaneNormal` which should be the wrap plane normal for that link.
- `CableJointComponent`, `CablePathComponent`, and `createCablePaths` are ported to 3D.
- 3D ECS uses Vector3 + Quaternion and reuses the 2D `World` implementation.

## Core Invariants to Preserve
- Each joint is an upper‑distance constraint between two attachments.
- Cable rest length is split into segment rest lengths and stored lengths on rolling links.
- Total rest length per path should remain constant:
  - `sum(joint.restLength) + sum(path.stored) = path.totalRestLength`
- `cw` is directional and depends on the effective traversal direction.
  - `_effectiveCW` flips `cw` for link index 0 when traveling from a circle; see 2D tests. There’s a "gotcha" comment in tests/js/cable_joints/cableAttachmentUpdateSystem_updateAttachmentPoints.test.js, lines 340 - 342:
  ```js
  // NOTE: This !cwStart is the biggest gotcha in the whole code base.
  //       cw/ccw are treated differently for hybrid links at position A.
  const t0 = tangentFromCircleToCircle(posStart, r, !cwStart, posA, r, cwA); // !cwStart because of _effectiveCW
  ```

## CableAttachmentUpdateSystem Port (3D)
**Files to study:**
- `src/js/cable_joints/cable_joints_core.js`
- `tests/js/cable_joints/cableAttachmentUpdateSystem_*`

**What it does:**
1. `_updateAttachmentPoints`: recompute attachments based on current positions/orientations and adjust stored/rest lengths.
2. `_mergeJoints`: remove a rolling link if stored length becomes negative.
3. `_splitJoints`: insert a rolling link when a segment intersects a wheel.
4. `_updateHybridLinkStates`: switch `hybrid` ↔ `hybrid-attachment` at ends.

**3D port pointers**
- Use the plane‑based geometry in `geometry3.js` for all tangents and arc lengths.
- Use `CableLinkComponent.cablePlaneNormal` as the wrap plane normal. For wheels it should align with the wheel’s rotation axis, and the wrap plane is perpendicular to that axis.
- Keep attachment updates planar. If the body has a full 3D orientation, you still need the rotation component around the wrap axis for `deltaAngle` effects on stored length. This is the most delicate part.

### `_updateAttachmentPoints` details
- For each joint, compute new tangent attachments based on link types:
  - `attachment → rolling`: point‑to‑circle tangent
  - `rolling → attachment`: circle‑to‑point tangent
  - `rolling → rolling`: circle‑circle tangents
  - `hybrid`/`hybrid-attachment`: propagate previous attachment by translation + rotation, then tangent if needed
- Stored length deltas are `sA` and `sB` computed via `signedArcLengthOnWheel` using prev/current attachments relative to prev/current centers.
- If link is `hybrid` or has friction, add a rotation term to s:
  - `s += (cw ? deltaAngle * radius : -deltaAngle * radius)`
- In 3D, compute `deltaAngle` as rotation around the wrap plane normal.
  - Do not use full quaternion angle in arbitrary axis; project the rotation onto the axis.

### `_mergeJoints` details
- Trigger when `path.stored[i+1] < 0` on an intermediate rolling link.
- Recompute tangents for the merged joint depending on link types.
- Adjust `stored`, `restLength`, and attachments so total rest length stays constant.
- Update `path.jointEntities`, `path.stored`, `path.cw`, `path.linkTypes` and destroy the merged joint entity.
- In 3D, use plane‑based tangents and arc lengths with the correct plane normal (derive from the link being rolled over).

### `_splitJoints` details
- Detect intersection of segment with wheel. Use `lineSegmentSphereIntersection` in 3D.
- Determine `cw` of the new splitter using `rightOfPlane` in the wrap plane.
- Compute tangents for the two new segments (A→splitter, splitter→B).
- Compute new stored length `s` on the splitter wheel. Reject if `s <= 0` or almost full wrap.
- Split the original rest length into two new rest lengths so tension is preserved:
  - `newRestLengthAS = available * dAS / (dAS + dSB)`
  - `newRestLengthSB = available * dSB / (dAS + dSB)`
- Update the original joint to A→splitter, add a new joint for splitter→B, and insert a rolling link into the path.
- In 3D, all tangents and arc lengths should be computed in the same wrap plane.

### `_updateHybridLinkStates` details
- For `hybrid` at ends: if stored length is negative, switch to `hybrid-attachment` and rewind attachment along the wheel.
- For `hybrid-attachment` at ends: check whether the cable crosses a tangent and should become `hybrid`.
- In 3D, the rotation “rewind” must rotate the attachment point around the wheel axis (wrap plane normal) about the wheel center.

## PBDCableConstraintSolver (3D)
**Files to study:**
- `src/js/cable_joints/cable_joints_core.js`
- `tests/js/cable_joints/pbdCableConstraintSolver.test.js`

**Key behavior:**
- Enforces *only* upper distance constraints; corrections happen when current length > rest length.
- Precomputes local offsets of attachment points for each joint at the start of the solve.
- Iterates forward and backward to reduce bias.

**3D port pointers**
- Replace 2D local transforms with Quaternion:
  - local = inverse(orientation) * (worldPoint - position)
  - worldPoint = position + orientation * local
- Replace scalar angular gradients with 3D torque vectors:
  - `gradAng = r × dir`
- Inertia should be a tensor in 3D. If not implemented, start with a scalar “effective inertia about wrap axis” to match 2D behavior, and document the approximation.
- Orientation corrections should update quaternions using small‑angle axis‑angle increments.

## PBDResolveCableOverCorrections (3D)
**Files to study:**
- `src/js/cable_joints/pbdResolveCableOverCorrections.js`
- `tests/js/cable_joints/pbdResolveCableOverCorrections.test.js`

**Key behavior:**
- Detects joints that were stretched *before* but slack *after* attachment updates.
- Computes corrective deltas for each over‑corrected joint, then averages per‑body corrections if a body has multiple slack joints.

**3D port pointers**
- Use the same “pre vs post” attachment logic as 2D, but with 3D `calculateAttachmentPoints`.
- Translation corrections are vectors; angular corrections should be axis‑angle deltas (or small quaternion increments) and averaged per entity.

## System Ordering (Important)
- `CableAttachmentCacheSystem` must run before `CableAttachmentUpdateSystem` so prev positions/orientations are valid.
- Attachment update must run before PBD solving.
- Over‑correction resolver runs after the solver to undo excessive corrections when segments go slack.

## Testing Strategy
- Port 2D tests one‑for‑one using planar (`z = 0`) setups and explicit plane normals.
- Keep the same expected values to ensure 3D logic matches 2D behavior.
- Validate that total rest length remains constant after attachment updates, merges, and splits.

