I'm working on the simulation logic behind @hangprinter-org/index.html sim-app and @examples/js/slideprinter/index.html. Let's trace down the logic of the latter, but the former works analogously.
We see that setupScene in @examples/js/slideprinter/setupScene.js gets the stage from the usda file slideprinter.usda. This usda file specifies that the three spools are kept in place by three `DistancePhysicsJoint`s.
We see in the setupScene.js that a DistanceConstraintComponent is created for each pair of entities with a DistancePhysicsJoint between them. The DistanceConstraintComponent is defined in @@src/python/cable_joints/ecs.py .
It is used by the XPBDDistanceConstraintSystem defined in @@src/python/cable_joints/common_systems.py . This system is used in setupScene.js like this: `world.registerSystem(new XPBDDistanceConstraintSystem());` .
This system's equivalent python implementation has a test in @tests/python/cable_joints/test_xpbd_distance_constraint_system.py that you can read for reference to learn what the system does.

As you see the simulation runs an XPBD simulation and the distance contraint is Gaussian, not Jacobian. As it stands now, the solution for keeping the triangle similar to a rigid body has two weaknesses:

 - when the triangle rotates, the attached objects don't rotate. This is physically wrong, they should rotate.
 - We get discretization-rotation, ie that the triangle gets a rotational impulse because one distance constraint is treated individually first, then the next, etc.

My desired solution would do something like: The movements of all the spools get summed up and averaged, the combined rotation gets calculated, and the same translation and rotation gets applied to all the spools at once.
The three spools are somehow grouped under a rigid body element in the .usda file.

For knowledge about how rigid bodies should be implemented in XPDF, see @ai_docs/PBDBodies/PBDBodies.md  and @ai_docs/Smallsteps/Smallsteps.md . For knowledge about how to group bodies and constraints together this way, explore @ai_docs/usd/usd_geom_schema.md , @ai_docs/usd/usd_physics_schema.md , @ai_docs/usd/usd_physx_schemas.md .
There's also the glossary at @ai_docs/usd/USD_glossary_full.md if you wonder about the meaning of those
usd-related words.
