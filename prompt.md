We're writing a physics simulation for the Hangprinter Project.

In flipper.html you find a good example of the architecture we want,
the Entity Component System (ECS) architecture.

It uses position based dynamics (PBD) in its physics engine, which is good.

In cable_joints/cableJoints.pdf you find the description of a new
kind of constraint that we want, called "Cable Joints".

Implement Cable joints inside flipper.html.

Each Cable Joint itself should be an entity, so that connected entities can know which cable joint connects them.
There should also be a Cable entity that stores a list of CableJoint entities. Useful for pinhole logic and potentially split/merge.

Components needed:
Standard Rigid Body Components: PositionComponent, OrientationComponent, VelocityComponent, MassComponent, InertiaComponent on the connected objects.
Shape/Contact Components: RadiusComponent (for wheels), ConvexHullComponent (for general shapes), WheelOrientationComponent (stores the winding direction preference from Fig 3/4).
Cable Link Type Component (on connected object entities): A component like CableLinkComponent { type: 'rolling' | 'attachment' | 'pinhole' | 'hybrid', spoolLength: float (for hybrid/spool) } attached to the entities being connected at the point where the cable interacts.
CableJointComponent (on the Cable Joint entity): This is the core data for the constraint.
  entityA: EntityID
  entityB: EntityID
  restLength: float (This is dn, the dynamic max distance)
  attachmentPointA_local: Vector2 (Optional: Store cached local coords, or recalculate)
  attachmentPointB_local: Vector2 (Optional)
isActive: bool (For merge/split)
CablePathComponent (on a 'Cable' entity): Store an ordered list of CableJoint entities, useful for pinhole logic and potentially split/merge.

Systems: (Order is crucial!)
CableAttachmentUpdateSystem (Runs BEFORE the main physics solver):
Queries for: Active CableJointComponent entities.
Reads: Position, Orientation, Radius/Shape, WheelOrientation of entityA and entityB. Also reads the CableLinkComponent type for A and B.
Logic:
  Calculates the current world-space tangent attachment points based on orientations and link types (using logic from the paper's Appendix/Figures 3, 6).
  Calculates the distance (s1, s2) the attachment points moved along the surfaces since the last step.
  Updates the CableJointComponent.restLength (dn) based on s1, s2 and the winding direction (Fig 2).
  Handles hybrid link state changes (updates CableLinkComponent.type and spoolLength, potentially triggers merge).
Writes: Updated CableJointComponent.restLength. (Maybe updated cached attachment points). Updated CableLinkComponent state.
CablePinholeSystem (Runs AFTER Attachment Update, BEFORE solver):
Queries for: Entities with CableLinkComponent(type=pinhole). Might use CablePathComponent to find adjacent joints easily.
Reads: CableJointComponent.restLength and current distance d for the two joints adjacent to the pinhole.
Logic: Implements Algorithm 1 (lines 9-15) - balances rest lengths dn to maintain equal tension across the pinhole.
Writes: Updated CableJointComponent.restLength for the adjacent joints.
ConstraintSolverSystem (e.g., PBD, impulse solver - the main physics step):
This system processes all constraints, including collisions and standard joints.
It needs to be extended to recognize CableJointComponent.
Reads: CableJointComponent (including the up-to-date restLength), current Position/Orientation of entityA/entityB, potentially the latest attachment points (or recalculates them).
Logic: Calculates the current distance d between the attachment points. If d > restLength, it applies corrections (position projections for PBD, impulses for impulse-based solvers) to entityA and entityB to enforce the unilateral distance constraint.
Writes: Updated Position/Orientation/Velocity of the connected entities.
CableSplitSystem (Could run after solver/movement):
Queries for: Active CableJointComponents.
Logic: Performs geometric checks (raycast/intersection) between the line segment of the cable joint and other potential interacting bodies. If an intersection occurs (Fig 11b), create a new CableJointComponent entity, split the restLength, update link types, and potentially re-activate merged joints.
Writes: New CableJointComponent data, updated CableLinkComponents, potentially modifies CablePathComponent.

TODO:
 - Introduce the CableJointComponent, CableLinkComponent, etc.
 - Add the CableAttachmentUpdateSystem, CablePinholeSystem, etc., to run before the existing CollisionSystem.
 - Create a distinct PBDConstraintSolverSystem to:
   * Iterate over CableJointComponents in addition to collision contacts.
   * For each Cable Joint, calculate the current distance d between attachments.
   * If d > CableJointComponent.restLength, apply position projections to the connected entities, just like it does for resolving collision penetrations.

In essence, Cable Joints become another type of constraint that the core physics solver system needs to handle, but they require dedicated pre-solver systems to update their dynamic properties (attachment points and rest length).

To test that the cable joints work, connect the two balls, obstacle 4 (index 3), and obstacle 3 (index 2) with a cable.
