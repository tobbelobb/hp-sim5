Notes

The flipper scene is defined in flipper_scene.usda. It contains Xforms for balls, obstacles, three cable joints, and a CablePath definition with link types, CW settings, and stored lengths

ai_docs/how_to_create_CableJointAPI_and_use_usd.md shows how to traverse a stage and manipulate prims using the USD Python API. It then demonstrates building a simple scene with UsdGeom and UsdPhysics.Scene. Later sections outline how to apply PhysX APIs such as PhysxSceneAPI, PhysicsCollisionAPI, and creating a RevoluteJoint. Finally, it explains defining a custom CableJointAPI schema and applying it in USDA or Python

In flipper_overlay.html, a JavaScript ECS version of the flipper setup appears (duplicate of the main scene) where CableJointComponent and CablePathComponent are created and systems registered

Plan

1. Load and Inspect the Existing USDA Scene

 - Use Usd.Stage.Open to read flipper_scene.usda.

 - Traverse or get prims using stage.Traverse() or GetPrimAtPath to access all objects.

 - These APIs match the examples at the top of the documentation file

2. Define Geometry and Transform Hierarchy with Standard USD APIs

 - Create a new stage (or modify the opened one) using Usd.Stage.CreateNew if building from scratch.

 - Use UsdGeom.Xform.Define for groups such as /World or /FlipperScene.

 - Use geometry schemas like UsdGeom.Sphere (for ball visuals or collision) or UsdGeom.Capsule etc.

 - The snippet at lines 45‑76 in the doc shows the typical pattern for defining these prims and a physics scene

3. Add Physics Scene and Physics Properties via UsdPhysics (Built‑in Physics API)

 - Define UsdPhysics.Scene under the root Xform, setting gravity direction and magnitude.

 - Apply UsdPhysics.CollisionAPI and UsdPhysics.RigidBodyAPI on each ball and obstacle to mark them as rigid bodies with collision shapes.

 - Create revolute joints for the flippers using UsdPhysics.RevoluteJoint.Define, setting body relationships and joint attributes (axis, limits) as illustrated in the documentation

4. (Optional) Enable PhysX Features for Higher Fidelity

 - Apply PhysxSchema.PhysxSceneAPI to the physics scene to configure solver type and other PhysX-specific attributes (e.g., continuous collision detection).

 - This is less preferred, but the doc lines 100‑104 show how to apply the API and set attributes like solverType

5. Create and Register a Custom CableJointAPI

 - Implement a codeless or generated API schema named CableJointAPI as described in the documentation.

 - The schema should expose at least:

  - cableJoint:restLength

  - cableJoint:attachPointA

  - cableJoint:attachPointB

 - The schema definition example is at lines 158‑176

 - Register the schema with a schema.usda file and plugInfo.json so that it can be applied like any other API.

 - Use CableJointAPI.Apply(prim) to author joint data either directly in Python or by writing USDA text.

6. Represent Cable Joints and Cable Paths in USD

 - Each joint prim under /FlipperScene should have CableJointAPI applied and its attributes set according to the data currently stored in customData fields (entityA, entityB, attachA/B, restLength) in flipper_scene.usda.

 - The CablePath prim can be a new schema (e.g., CablePathAPI) or use generic attributes to store arrays of joint names, link types, clockwise flags, and stored values, mirroring lines 100‑108 of flipper_scene.usda

7. Hook Up with Existing Simulation Code

 - When the Python server (flipper_server.py) loads the scene, read the CableJointAPI properties via the USD API.

 - Populate the ECS world with CableJointComponent and CablePathComponent using these attributes (replacing the current customData parsing).

 - The same approach can be mirrored in JavaScript (as in flipper.html and flipper_overlay.html) by parsing the USDA via a small USD parser or by using usd-core libraries if available.

8. Persist the Full Scene

 - After all prims and API attributes are authored, call stage.GetRootLayer().Save() to write out a single .usda representing the complete flipper setup with standard USD, physics, and optional PhysX data.

This plan prioritizes standard USD schema usage first, then optional physics and PhysX APIs, and introduces a custom CableJointAPI so that cable joints and paths are represented natively within the USD scene. This matches the guidance and examples given in ai_docs/how_to_create_CableJointAPI_and_use_usd.md and allows flipper_scene.usda plus the server/browser implementations to share a consistent USD-based representation.
