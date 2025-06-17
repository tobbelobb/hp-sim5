Scraped from https://openusd.org/release/api/usd_physics_page_front.html on jun 17, 2025

### Table of Contents

- Purpose and Scope
- Overall Design Concerns
  - Rigid Body Simulation Primer
  - USD Implementation
    - Disambiguation
    - Fundamental Editing Capabilities
    - Physics Scenes
    - Types
    - Units
    - Default Values
    - Rigid Bodies
    - Interaction with the USD hierarchy
    - Sleep
    - Kinematic Bodies
    - Animation of Attributes
    - Body Mass Properties
    - Collision Shapes
    - Turning Meshes into Shapes
    - Physics Materials
    - Plane Shapes
    - Collision Filtering
    - Pairwise Filtering
    - Joints
    - Joint Reference Frames
    - Jointed Bodies
    - Joint Collision Filtering
    - Breaking and Disabling Joints
    - Joint Subtypes
    - Joint Limits and Drives
    - Articulations
  - Examples
    - Box on Box
    - Box on Quad
    - Spheres with Materials
    - Group Filtering
    - Pair Filtering
    - Joint
    - Distance Joint
- Physics Parsing Utils Overview.
  - Purpose and Scope
  - Specification
  - Physics Parsing Parameters
  - Physics Parsing Determinism
  - Physics Object Descriptor
  - Physics Scene Descriptor
    - Physics Scene Simulation Owner Behavior
  - Physics RigidBody Material Descriptor
    - Physics RigidBody Material Simulation Owner Behavior
  - Physics Collision Group Descriptor
    - Physics Collision Group Simulation Owner Behavior
  - Physics RigidBody Descriptor
    - Physics RigidBody Simulation Owner Behavior
  - Physics Shape Descriptor
  - Physics Sphere Shape Descriptor
  - Physics Capsule Shape Descriptor
  - Physics Cylinder Shape Descriptor
  - Physics Cone Shape Descriptor
  - Physics Cube Shape Descriptor
  - Physics Plane Shape Descriptor
  - Physics Mesh Shape Descriptor
  - Physics Sphere Points Shape Descriptor
  - Physics Custom Shape Descriptor
    - Physics Collision Simulation Owner Behavior
  - Physics Joint Descriptor
  - Physics Fixed Joint Descriptor
  - Physics Revolute Joint Descriptor
  - Physics Prismatic Joint Descriptor
  - Physics Spherical Joint Descriptor
  - Physics Distance Joint Descriptor
  - Physics D6 Joint Descriptor
  - Physics Custom Joint Descriptor
    - Physics Joint Simulation Owner Behavior
  - Physics Articulation Descriptor
    - Physics Articulation Simulation Owner Behavior

**UsdPhysics** defines the physics-related prim and property schemas that together form a physics simulation representation.

**Rigid Body Physics in USD Overview.**

# Purpose and Scope

While at its launch USD was primarily targeted at film and VFX pipelines, it has been adopted into many other spaces and applications, including the authoring, interchange, and delivery of interactive 3D graphics content. Computer games and 3D web applications are examples of new domains for the integration of this content. In these consumer-facing applications, real-time physics allows realistic user interaction with virtual objects. In professional and academic applications, there are a number of use cases for real-time physics in areas such as mechanical engineering, architecture, artificial intelligence and robotics where vehicles or robots are designed, tested and trained in simulation. This schema extends USD to represent the real-time physics data required to enable these applications.

The realm of simulation is broad. This schema is intended as a baseline initial extension to USD that enables the minimum set of common concepts required to represent rigid body physics. Future iterations and extensions will incrementally add capabilities as needs are identified.

# Overall Design Concerns

The initial usdPhysics schema concerns rigid body simulations. Rigid body simulations are the most broadly applicable category we could identify, with common and long standing uses across all disciplines described above.

## Rigid Body Simulation Primer

Rigid body simulators take as input a list of rigid bodies and a list of constraints. Given the state, or the state history of the bodies at a specific time, they compute the updated state of the bodies a moment in time later, with the general desire being that the bodies' movement while constrained by the constraints obeys the laws of physics. One can invoke a sequence of such simulation updates to generate an animation.

A rigid body can be described by its pose (position and orientation in a well defined frame of reference), as well as its mass distribution (specified by a center of mass position, total mass, and an inertia tensor). The body will also have a velocity (linear and angular vectors). Pose and velocity are both inputs and outputs of the simulation update.

Constraints describe physical limits between bodies. They can take many forms, but fall primarily into two categories:

- **Explicit constraints**, often called joints, which create a fixed spatial relationship between two rigid bodies. One example is a requirement that one body never rotate relative to the other body, even if relative translation is possible.

- **Implicit constraints**, most commonly contacts, which are generally created 'behind the scenes' by the simulator to ensure that e.g. solid objects do not pass through each other. For the simulator to derive these constraints, each body must be provided with a collision representation (called 'collision shape' or 'collider') and physical material properties.

Simulations often share a set of global parameters that influence the simulation of all bodies. It is generally possible to simultaneously create multiple simulations, each with their own set of parameter settings.

## USD Implementation

### Disambiguation

First, it is clear that some terminology commonly used by the physics simulation community, such as 'scene', 'joint', and 'material' have different meanings than in VFX, and are already in use in other contexts by USD, so all of the usdPhysics schema classes are prefixed with 'Physics' and make use of namespacing to avoid any ambiguity.

### Fundamental Editing Capabilities

A primary assumption in designing this schema was that one of the most common use cases will be to add physics behavior to existing USD content. Furthermore, the conventional wisdom was that to maximize the performance of USD implementations, it is best to avoid inflating the number of USD objects in a scene. Accordingly, the best approach is to attach new API schemas that contain physics attributes to existing USD objects whenever this makes sense. In rare cases there is no object already available to which simulation attributes can be attached in a rational manner, and in these cases, usdPhysics makes use of new USD IsA schemas.

It is vital that any operation to add physics can also be undone, which can be accomplished in most cases using UsdPrim::RemoveAPI().

Similarly, in editor use cases it is a common capability to temporarily be able to mute/disable properties, without deleting them outright. Removing an API will make it be no longer "present", but all data authored to that API still remains. USD allows entire objects to deactivate via an active flag, but it can not be animated over time - which cannot be done with just RemoveAPI(). For a few cases where animating mute behavior is a really common use case, we have defined a boolean enable attribute. (Note that we initially wanted to have the enable flag in a base class for the classes that need it, but this creates problems when multiple enableable APIs are applied to an object. In this case USD only creates a single shared enable flag, which is not what we want.)

### Physics Scenes

A primary requirement is that multiple independent physics simulations can be described within a single USD stage. We found the best way to do this is to create a UsdPhysicsScene class. It was proposed to use the USD layers concept to partition physics into separate scenes, but the dominant layering idioms are workflow oriented, as opposed to primitive organization. It makes sense to leave those operational principles to a a studio's data design, rather than overload the concept for simulations. In case there are multiple scenes in a stage, bodies are assigned to specific scenes using a rel from body to scene. If there is only one unique scene, an explicit rel is unnecessary, and bodies are assumed to be associated with the singleton scene. It is not possible to put a single body into multiple scenes, as data races could occur, and it would be impossible to resolve the correct state of body given conflicting simulation.

Scenes can define a gravity vector attribute which accelerates all contained bodies appropriately. Gravity is provided as a separate direction vector and magnitude. This allows for the independent specification of default direction (which is opposite the stage up axis), and default magnitude (earth gravity).

### Types

USD differentiates between base and role value types. We tried to use the available role types whenever applicable. For example, a velocity is a vector3f rather than a float3.

We chose to use single rather than double precision floats as widely available real time physics simulation software universally use single precision types for best performance, and the use of double or extended precision is only warranted for positions in extremely large spaces, which is already accommodated through the use of USD's built-in UsdGeomXformable type.

### Units

In terms of units, physics makes use of USD's established concepts of distance and time, and also adds the concept of mass. All of the physical quantities we use can be decomposed into a product of these three basic types. USD does not in general prescribe units for distance and time. It however has the concept of metersPerUnit ( UsdGeomTokensType::metersPerUnit](https://openusd.org/release/api/struct_usd_geom_tokens_type.html#abb367d764f195f37437cf61190f8260b)) and timeCodesPerSecond ( [UsdStage::SetTimeCodesPerSecond()](https://openusd.org/release/api/class_usd_stage.html#a61c421fcf601c28f2efc5597b8058e22 "Sets the stage's timeCodesPerSecond value.")) metadata which makes it possible to scale content authored at different scales correctly relative to each other when bringing them into a shared scene. However, the class [UsdGeomPointInstancer](https://openusd.org/release/api/class_usd_geom_point_instancer.html "Encodes vectorized instancing of multiple, potentially animated, prototypes (object/instance masters)...") stores velocity and angular velocity attributes that are specified in units per second and degrees per second, respectively. This indicates a particular preference for degrees and seconds in USD, and we wish to stay consistent with this. This physics extension therefore adopts degrees as the standard angular unit, seconds as the standard time unit, and continues to respect arbitrary units for distance. Additionally we adopt arbitrary mass units and add a kilogramsPerUnit ( [UsdPhysicsTokensType::kilogramsPerUnit metadata which remains consistent with scaling conversions into the International System of Units.

In the schema we indicate the units for each specified quantity as an expression using the terms 'distance', 'degrees', 'mass' and 'seconds' as defined above. A USD stage can be composed by referencing a number of USD files that may each have differing units; however, the specification of unit is per-stage\*\*, so correctives must be applied to referenced data to bring it into the units of the referencing stage, as part of the act of adding the reference. In the case of linear units, given the hierarchical nature of both references and composition of transformations in a scenegraph, it is possible to "correct" for differing units by applying a scale transformation on the prim where the reference is added. However, for a change of mass units, no such easy corrective is possible; instead, each measurement must be adjusted in an overriding opinion.

Similarly, any simulation outputs can be converted back into their original units before being written back to USD.

A few metric helper APIs ( UsdPhysicsGetStageKilogramsPerUnit()](https://openusd.org/release/api/usd_physics_2metrics_8h.html#a63ba908d086f1124ede1061c390c2e15), [UsdPhysicsStageHasAuthoredKilogramsPerUnit()](https://openusd.org/release/api/usd_physics_2metrics_8h.html#a1a8f0d08d433f5e0cab9e07982b592ce), [UsdPhysicsSetStageKilogramsPerUnit()](https://openusd.org/release/api/usd_physics_2metrics_8h.html#a7e2ade44ed9ee0214aafa30e48044a90), [UsdPhysicsMassUnitsAre() are provided to access and set the stage level kilogramsPerUnit metadata.

### Default Values

For some attributes in these schemas we desire the ability to signal that we want the simulator to supply a reasonable default value, whatever that may be for the particular simulator. Since USD prefers for all schema attributes to provide "fallback" (i.e. default) values, we specify default values for these attributes explicitly, typically with sentinel values that lie outside of the legal range of values for each particular attribute. For example, if an attribute is normally required to be non-negative, we use -1.0 to request a certain default behavior. Sometimes the attribute can use the entire floating point range, in which case we reserve what is effectively +/- infinity at the edges of this range as sentinels. We will use the floating point 'inf' literal which USD supports in files and schemas to denote this. We document such default sentinel behavior on a case by case basis in the schema.

### Rigid Bodies

We represent physics rigid bodies using the UsdPhysicsRigidBodyAPI](https://openusd.org/release/api/class_usd_physics_rigid_body_a_p_i.html "Applies physics body attributes to any UsdGeomXformable prim and marks that prim to be driven by a si..."), which can be applied to any [UsdGeomXformable](https://openusd.org/release/api/class_usd_geom_xformable.html "Base class for all transformable prims, which allows arbitrary sequences of component affine transfor..."). [UsdGeomXformable is the suitable base class as it provides a placement in space via the xform which is also a fundamental property of physics bodies.

Rigid bodies have linear and angular velocity attributes that are specified in local space, to be consistent with velocities in point instancers and a prim's xform.

Bodies can specify a simulationOwner scene rel for the aforementioned multi-scene simulation scenario.

### Interaction with the USD hierarchy

If a prim in a USD scene graph hierarchy is marked with UsdPhysicsRigidBodyAPI](https://openusd.org/release/api/class_usd_physics_rigid_body_a_p_i.html "Applies physics body attributes to any UsdGeomXformable prim and marks that prim to be driven by a si..."), the behavior is such that with [UsdPhysicsRigidBodyAPI, all chidlren of the marked prim are assumed to be part of this rigid body, and move rigidly along with the body. This is consistent with the common behavior one expects during hand-animation of a sub-tree. If aggregate properties of the entire rigid body must be computed, such as total mass or the entirety of its collision volume, then the contents of the entire subtree are considered.

Note that it is of course permitted to change/animate the transforms in such a sub-tree, in which case any derived quantities in the physics engine such as center of mass or relative shape poses will be updated. Such animation will however not generate momentum. For example, rapidly animating rigid portions of Luxo Jr. will not cause the lamp to jump, since to compute such behavior we would need to capture the relative masses of multiple independent portions of the lamp, which is not possible if the whole is treated as a single rigid assembly. The correct approach would be to model each of the rigid portions of the lamp as independent rigid bodies, and connect these with joints, which we will discuss later.

It is not possible to have nested bodies. PhysicsRigidBodyAPIs applied to anything in the subtree under a prim that already has a UsdPhysicsRigidBodyAPI are ignored. An exception is if a prim has an resetXformStack op. In this case it ignores rigid body parenting, and a rigid body API can then be used to make it dynamic.

### Sleep

Large terrestrial simulations have the common property that all objects will eventually fall to the ground and come to rest. It is common for rigid body simulation software to have a notion of 'sleeping' such bodies to improve performance. This means that interactions cease to be updated when an equilibrium state is reached, and start to be updated again once the equilibrium state has somehow been disturbed. It is also possible to start off simulations in a sleeping state. We provide UsdPhysicsTokensType::physicsStartsAsleep to support this. We have considered exposing the runtime sleep state of each body in the simulation so that it would be visible to USD when the simulation deactivated a body, and to let USD force a body to sleep during simulation. We decided against this since the precise deactivation rules are an implementation detail that can vary significantly between simulations, so we prefer to keep this as a hidden implementation detail for the time being.

### Kinematic Bodies

In games and VFX it is often desirable to have an animator take full control over a body, even as it interacts with other physics driven bodies. We call such bodies 'kinematic'. Kinematic bodies still 'pull on' joints and 'push on' touching rigid bodies, but their xform is only read, but not written, by the physics simulator, letting the animation system write their xforms. We support such bodies using the UsdPhysicsTokensType::physicsKinematicEnabled attribute. Kinematic bodies are not exactly the same thing as an animated static body with a collider: The simulation infers a continuous velocity for the kinematic body from the keyframing, and this velocity will be imparted to dynamic bodies during collisions.

### Animation of Attributes

We worked with the assumption that every attribute on every class that is not explicitly marked with "uniform" can be animated. Obviously erratic changing of some parameters could make some simulations explode in practice, we believe this is highly implementation dependent and not a reason to generally forbid attribute animation.

### Body Mass Properties

We opted to decouple mass properties from UsdPhysicsRigidBodyAPI](https://openusd.org/release/api/class_usd_physics_rigid_body_a_p_i.html "Applies physics body attributes to any UsdGeomXformable prim and marks that prim to be driven by a si...") and place them in a separate [UsdPhysicsMassAPI](https://openusd.org/release/api/class_usd_physics_mass_a_p_i.html "Defines explicit mass properties (mass, density, inertia etc.)."). [UsdPhysicsMassAPI](https://openusd.org/release/api/class_usd_physics_mass_a_p_i.html "Defines explicit mass properties (mass, density, inertia etc.).") is not required in cases where the mass properties of an object can be derived from collision geometry (discussed further down in this document) and the [UsdPhysicsMaterialAPI](https://openusd.org/release/api/class_usd_physics_material_a_p_i.html "Adds simulation material properties to a Material."). Most commonly, [UsdPhysicsMassAPI](https://openusd.org/release/api/class_usd_physics_mass_a_p_i.html "Defines explicit mass properties (mass, density, inertia etc.).") is applied in addition to [UsdPhysicsRigidBodyAPI.

Unlike UsdPhysicsRigidBodyAPI](https://openusd.org/release/api/class_usd_physics_rigid_body_a_p_i.html "Applies physics body attributes to any UsdGeomXformable prim and marks that prim to be driven by a si..."), it is also possible to apply [UsdPhysicsMassAPI multiple times in a USD scene graph subtree, in order to make it possible to accumulate the mass of rigid components.

The mass of an object may be specified in multiple ways, and several conflicting settings are resolved using a precedence system that will initially seem rather complex yet but is actually intuitive and practical:

- Parents' explicit total masses override any mass properties specified further down in the subtree.

- Density has lower precedence than mass, so explicit mass always overrides implicit mass that can be computed from volume and density.

- A density in a child overrides a density specified in a parent for all of the subtree under the child.

- A density specified via UsdPhysicsMassAPI](https://openusd.org/release/api/class_usd_physics_mass_a_p_i.html "Defines explicit mass properties (mass, density, inertia etc.)."), even if it is inherited from a prim higher in the tree, overrides any density specified via a material (see [UsdPhysicsMaterialAPI.

- Implicit mass at any prim is the computed volume of collision geometry at that prim times the locally effective density, plus the implicit masses of all children in the subtree.

- Density is assumed to be 1000.0 kg/m3 (approximately the density of water) for volume computation when no other density is specified locally, or in bound materials either locally or higher up in the tree, and this value is converted into the collider's native units prior to being used for mass computation.

- Mass is assumed to be 1.0 in the mass units used when none is provided explicitly, and there are no collision volumes to derive from.

Implementing this rule set is potentially nontrivial, but it's important to get right. We plan to make the pseudocode of a mass computation system available that relies on the underlying physics system to compute the volume of collision geometry.

### Collision Shapes

Our design for collision shapes defines a UsdPhysicsCollisionAPI](https://openusd.org/release/api/class_usd_physics_collision_a_p_i.html "Applies collision attributes to a UsdGeomXformable prim.") that may be attached to objects of type [UsdGeomGprim](https://openusd.org/release/api/class_usd_geom_gprim.html "Base class for all geometric primitives.") representing graphics geometry. Specifically, we suggest the support of [UsdGeomCapsule](https://openusd.org/release/api/class_usd_geom_capsule.html "Defines a primitive capsule, i.e."), [UsdGeomCone](https://openusd.org/release/api/class_usd_geom_cone.html "Defines a primitive cone, centered at the origin, whose spine is along the specified axis,..."), [UsdGeomCube](https://openusd.org/release/api/class_usd_geom_cube.html "Defines a primitive rectilinear cube centered at the origin."), [UsdGeomCylinder](https://openusd.org/release/api/class_usd_geom_cylinder.html "Defines a primitive cylinder with closed ends, centered at the origin, whose spine is along the speci..."), [UsdGeomSphere](https://openusd.org/release/api/class_usd_geom_sphere.html "Defines a primitive sphere centered at the origin.") and [UsdGeomMesh, though the precise set of supported geoms might be implementation specific. Note also that some implementations might support some of these shapes using potentially faceted convex approximations.

As we have alluded to earlier, a subtree under a UsdPhysicsRigidBodyAPI](https://openusd.org/release/api/class_usd_physics_rigid_body_a_p_i.html "Applies physics body attributes to any UsdGeomXformable prim and marks that prim to be driven by a si...") prim may contain multiple collision shape prims (or 'colliders') that are required to resolve the motion of the body as it touches other bodies. For example, a teapot is a single rigid body (the top level prim is marked with [UsdPhysicsRigidBodyAPI](https://openusd.org/release/api/class_usd_physics_rigid_body_a_p_i.html "Applies physics body attributes to any UsdGeomXformable prim and marks that prim to be driven by a si...")), but it may be composed of multiple Mesh and other Geoms at and under this prim. Each of these parts can gain a [UsdPhysicsCollisionAPI which instructs the system to make this shape's geom into a collider for the purposes of physics simulation.

It is also possible to have PhysicsCollisionAPIs on prims that are not under a UsdPhysicsRigidBodyAPI](https://openusd.org/release/api/class_usd_physics_rigid_body_a_p_i.html "Applies physics body attributes to any UsdGeomXformable prim and marks that prim to be driven by a si..."). These are treated as static colliders – shapes that are not moved by physics, but they can still collide with bodies, at which point they are interpreted as having zero velocity and infinite mass. Since static colliders do not have a corresponding [UsdPhysicsRigidBodyAPI](https://openusd.org/release/api/class_usd_physics_rigid_body_a_p_i.html "Applies physics body attributes to any UsdGeomXformable prim and marks that prim to be driven by a si..."), in this case it is possible for the [UsdPhysicsCollisionAPI](https://openusd.org/release/api/class_usd_physics_collision_a_p_i.html "Applies collision attributes to a UsdGeomXformable prim.") **itself** to specify a simulationOwner scene. For any collider that **is** associated with a [UsdPhysicsRigidBodyAPI, the collider's simulation owner attribute is ignored.

According to USD rules, UsdGeomGprim(s)](https://openusd.org/release/api/class_usd_geom_gprim.html "Base class for all geometric primitives.") must generally be leaf prims, and because [UsdPhysicsCollisionAPI](https://openusd.org/release/api/class_usd_physics_collision_a_p_i.html "Applies collision attributes to a UsdGeomXformable prim.") can only be applied to [UsdGeomGprim](https://openusd.org/release/api/class_usd_geom_gprim.html "Base class for all geometric primitives."), it means that there is no opportunity to inherit [UsdPhysicsCollisionAPI attributes down the scene graph. If a mesh is composed of submeshes, all of the submeshes are considered to be part of the collider.

The current design has the drawback that it is not possible to add multiple colliders to a single geom object directly. To add multiple colliders one must create a parent Xform (which receives the UsdPhysicsRigidBodyAPI, and then add the original geom as a child, and add any additional colliders as additional children. This is a bit more invasive than we would prefer, but the only alternative would be to make colliders Is-A schemas rather than APIs, which there was a desire to avoid to prevent the number of USD objects from increasing a great deal.

### Turning Meshes into Shapes

Simple USD Prims like Sphere, Cylinder, Cube, Cone and Capsule can be used for physics simulation directly with the simple addition of a UsdPhysicsCollisionAPI](https://openusd.org/release/api/class_usd_physics_collision_a_p_i.html "Applies collision attributes to a UsdGeomXformable prim."). [UsdGeomMesh](https://openusd.org/release/api/class_usd_geom_mesh.html "Encodes a mesh with optional subdivision properties and features.") is a bit tricky because the state of the art in simulating arbitrary meshes in real time comes with some tradeoffs that users generally want control over. To support this, we allow [UsdPhysicsMeshCollisionAPI](https://openusd.org/release/api/class_usd_physics_mesh_collision_a_p_i.html "Attributes to control how a Mesh is made into a collider.") to be applied to [UsdGeomMesh(es)](https://openusd.org/release/api/class_usd_geom_mesh.html "Encodes a mesh with optional subdivision properties and features.") only, alongside the [UsdPhysicsCollisionAPI, a simplified mesh, a set of convex hulls, a single convex hull, a bounding box or a bounding sphere. If an implementation does not support a particular kind of approximation, it is recommended that it falls back to the most similar supported option.

Note that use of the subdivision attribute ( UsdGeomMesh::GetSubdivisionSchemeAttr() can cause features such as corners and edges to be removed from the graphical representation. In order to ensure the physics representation accurately matches the graphical representation, this attribute should be accounted for when generating physics colliders.

Collision meshes may be specified explicitly (for example one that was processed by a particular decimator) by adding the custom collider mesh as a sibling to the original graphics mesh, UsdGeomImageable](https://openusd.org/release/api/class_usd_geom_imageable.html "Base class for all prims that may require rendering or visualization of some sort.") _purpose_ to "guide" so it does not render, and apply [UsdPhysicsCollisionAPI](https://openusd.org/release/api/class_usd_physics_collision_a_p_i.html "Applies collision attributes to a UsdGeomXformable prim.") and [UsdPhysicsMeshCollisionAPI to it specifying no approximation.

### Physics Materials

Just like graphics, physics uses material properties. These are primarily used to inform friction and collision restitution behavior, in addition to being one of several ways to specify object density as discussed earlier. All these properties are stored in the UsdPhysicsMaterialAPI, which can be applied to a USDShadeMaterial prim as we believe it to be practical to add physics properties to an established USD material library.

USD Physics materials are bound in the same way as graphics materials using UsdShadeMaterialBindingAPI](https://openusd.org/release/api/class_usd_shade_material_binding_a_p_i.html "UsdShadeMaterialBindingAPI is an API schema that provides an interface for binding materials to prims..."), either wih no material purpose or with a specific "physics" purpose. Note that this approach also permits binding different materials to [UsdGeomSubset(s)](https://openusd.org/release/api/class_usd_geom_subset.html "Encodes a subset of a piece of geometry (i.e."). Not all physics simulations support different materials per [UsdGeomSubset, and it's possible that all but one subset per collider will be ignored by the implementation.

The unitless coefficients dynamicFriction and staticFriction are defined by the Coulomb friction model. The coefficient of restitution is the ratio of the final to initial relative velocity between two objects after they collide. These three properties actually should be defined for each combination of two materials, but this is generally considered impractical. Common practice in real time physics is to define them on each material and then to use a simple formula to combine them, for example by taking the product or the minimum. Currently the default behavior we propose is to average the values, which is the default behavior in popular real time game engines. In the future other combine modes should be exposed.

### Plane Shapes

Implicit plane shapes are a very common physics primitive used primarily for testing simple simulations. There are plans to add a Plane class to USD as a UsdGeomGPrim. We look forward to supporting such plane shapes as static colliders when they become available.

### Collision Filtering

Even in the simplest practical applications, the need to ignore some collisions occurs often. One might need the sword of a game character to pass through an enemy rather than to bounce off, while wanting it to bounce off walls, for example.

We define a CollisionGroup as an IsA schema with a UsdCollectionAPI](https://openusd.org/release/api/class_usd_collection_a_p_i.html "A general purpose API schema used to describe a collection of prims and properties within a scene.") applied, that defines the membership of colliders (objects with a [UsdPhysicsCollisionAPI with which it needs to not collide. Colliders not in any CollisionGroup collide with all other colliders in the scene, except for those colliders which have disabled collision by default.

For behavioral or performance reasons, it is sometimes useful to configure a collider whose collision is disabled by default. To support this, the CollisionGroup has an invertFilteredGroups option; when this value is set, an implementation should disable collisions against all other colliders except for those in the referenced filteredGroups. Note that the collisions which were not disabled may still be further restricted by additional collision groups or pair filters.

When composing a stage from multiple USD scenes, it may be desirable to merge CollisionGroups, particularly if the stage composition is not known ahead of time. For example, a stage composed of multiple characters, each with a ragdoll as well as a character "controller" - the character controllers should not collide with the ragdoll instances. A CollisionGroup contains an optional mergeGroupName, and all groups with matching name should be considered to be part of a single group, whose members and filter groups should be the union of the merged groups. This allows the character file to define a group which disables the controller-versus-ragdoll collisions; by assigning a mergeGroupName to this group, the controller can be filtered against ragdoll bodies in all other instances of the character.

Care should be taken when merging groups with differing invertFilteredGroups options; merging of groups should only ever cause collision pairs to become disabled - i.e. a filter cannot re-enable a pair that has been disabled by any other group. Consider an inverted group which references only GroupX (i.e. disables collisions with everything except those in GroupX). When merging this group with a non-inverting group referencing the same GroupX (i.e. disables collisions against GroupX) - the merged group will collide with nothing, since the combined rules of the merged groups will filter out any other body.

### Pairwise Filtering

Sometimes group based filtering is insufficiently powerful to take care of some filtering special cases. One would for example set up group based filtering such that bodies of human characters collide against extremities like arms and legs, generally assuming that these arms and legs belong to different humans than the bodies. One however often doesn't want the extremities of a particular human to collide with its own body, which is hard to avoid during a lot of constant close proximity movement. To cover this case we have the FilteringPairsAPI, which specifies a multi-target relationship to other objects with which collisions are explicitly disabled. This pairwise filtering has precedence over group based filtering.

The FilteringPairsAPI can be applied to objects with a UsdPhysicsRigidBodyAPI](https://openusd.org/release/api/class_usd_physics_rigid_body_a_p_i.html "Applies physics body attributes to any UsdGeomXformable prim and marks that prim to be driven by a si..."), [UsdPhysicsCollisionAPI](https://openusd.org/release/api/class_usd_physics_collision_a_p_i.html "Applies collision attributes to a UsdGeomXformable prim."), or [UsdPhysicsArticulationRootAPI.

It is sufficient to have a rel from an object A to an object B, to get the filtering behavior. In this case the backwards rel from B to A is implicit and not necessary.

### Joints

Joints are generally fixed attachments that can represent the way a drawer is attached to a cabinet, a wheel to a car, or links of a robot to each-other. Here we try to focus on a set of capabilities that are common to most simulation packages and sufficiently expressive for a large number of applications.

Mathematically, jointed assemblies can be modeled either in maximal (world space) or reduced (relative to other bodies) coordinates. Both representations have pros and cons. We are proposing a USD representation that will work with both approaches.

### Joint Reference Frames

The joint base type is the IsA class UsdPhysicsJoint. Joints don't necessarily have a single unique Xform in space, rather, they are defined by two distinct frames, one relative to each of the two bodies which they connect.

These two frames might not work out to be the same position and orientation in world space because of either the permitted relative movement of the joint (think of a car suspension moving up and down: the joint frame of the suspension is constant relative to both the car body and the car axle, yet the axle and undercarriage move relative to each other) or the error of approximate simulations that can permit the joint to slightly pull apart when subjected to significant forces or velocities.

It does not make sense to derive UsdPhysicsJoint](https://openusd.org/release/api/class_usd_physics_joint.html "A joint constrains the movement of rigid bodies.") from [UsdGeomXformable](https://openusd.org/release/api/class_usd_geom_xformable.html "Base class for all transformable prims, which allows arbitrary sequences of component affine transfor...") because [UsdGeomXformable has only a single Xform, and a joint has many. We could have created an asymmetrical solution where the secondary xform is added on, or split the joint object into two separate joint frames that are parented into the scene graph and are then somehow pairwise cross referenced, but we opted to go with an entirely new class that has all the information we need in a symmetrical fashion.

### Jointed Bodies

A joint defines a pair of relationships to UsdGeomXformable(s)](https://openusd.org/release/api/class_usd_geom_xformable.html "Base class for all transformable prims, which allows arbitrary sequences of component affine transfor..."), to which we will refer as the 'bodies'. Simulation of the joint is possible if at least one of these has a [UsdPhysicsRigidBodyAPI](https://openusd.org/release/api/class_usd_physics_rigid_body_a_p_i.html "Applies physics body attributes to any UsdGeomXformable prim and marks that prim to be driven by a si...") on it, or on an ancestor. If either rel is not defined, it is treated as equivalent to attaching to the static world frame, though it is recommended to always work with two well defined [UsdGeomXformable(s).

The joint space relative to each body is a translation and orientation only, scaling is not supported. (This is a general tension between graphics and physics. Real world objects cannot scale arbitrarily, and simulations therefore tend to not support scaling during rigid body simulation). For this reason we don't use a general USD xform that is too flexible for our needs, but rather a separate position and orientation quaternion. (Note however that this local joint space is fixed in the prim's local space, which of course CAN be scaled using the prim's own Xform scaling. This means that if a doorknob is attached to a door at a particular position, it will continue to appear in the same correct position on the door regardless of how the door is scaled, without having to adjust the joint position.)

We generally desire to have the two joint frames line up in world space, at least along their constrained degrees of freedom. This condition can be violated if either body is moved in world space, either by changing its own or one of its parents' transforms, or if either body rels is changed. As a result it is desirable to recompute the joint frames when the connected bodies or their world space transforms have changed.

### Joint Collision Filtering

It is common practice to disable collisions between the jointed bodies so that their collision shapes don't interfere. It is therefore the default behavior that collisions between the two subtrees indicated by the two body rel-s are ignored. This default filtering behavior can be opted out of using the joint's collisionEnabled attribute. No filtering occurs if either rel is undefined.

### Breaking and Disabling Joints

Breaking force is a practical property useful for all joints; joints can break when sufficient force is applied. For example a door can be ripped off its hinges. This can be modeled using the breakForce and breakTorque attributes.

Joints can entirely be temporarily disabled just like rigid bodies or colliders. Contrary to breaking, which is a (within a simulation run irreversible) simulated behavior, disabling is a request to not simulate the joint at all.

### Joint Subtypes

It is possible to derive a number of specific joint types, however the Joint API itself can represent a generic configurable joint, so in that sense it is not an abstract type.

The subtypes UsdPhysicsSphericalJoint](https://openusd.org/release/api/class_usd_physics_spherical_joint.html "Predefined spherical joint type (Removes linear degrees of freedom, cone limit may restrict the motio..."), [UsdPhysicsRevoluteJoint](https://openusd.org/release/api/class_usd_physics_revolute_joint.html "Predefined revolute joint type (rotation along revolute joint axis is permitted.)") and [UsdPhysicsPrismaticJoint](https://openusd.org/release/api/class_usd_physics_prismatic_joint.html "Predefined prismatic joint type (translation along prismatic joint axis is permitted....") both define a primary axis (Following the USD axis definition pattern established in e.g. [UsdGeomCapsule](https://openusd.org/release/api/class_usd_geom_capsule.html "Defines a primitive capsule, i.e.") and [UsdGeomCylinder and a top and bottom motion limit along it.

UsdPhysicsDistanceJoint](https://openusd.org/release/api/class_usd_physics_distance_joint.html "Predefined distance joint type (Distance between rigid bodies may be limited to given minimum or maxi...") defines a min and max distance between the attachment points. The [UsdPhysicsFixedJoint has no additional properties and simply locks all relative degrees of freedom.

### Joint Limits and Drives

Instead of using one of the predefined joint subtypes, it is also possible to compose a custom joint from a set of limits and drives. Limits and drives are multi-apply schemas, so one can apply multiple instances, one for each degree of freedom. The degree of freedom is specified via the TfToken

The limit API further contains optional low and high limit attributes.

The drive API allows joints to be motorized along degrees of freedom. It may specify either a force or acceleration drive (The strength of force drives is impacted by the mass of the bodies attached to the joint, an acceleration drive is not). It also has a target value to reach, and one can specify if the target is a goal position or velocity. One can limit the maximum force the drive can apply, and one can specify a spring and damping coefficient.

The resulting drive force or acceleration is proportional to

stiffness \* (targetPosition - p) + damping \* (targetVelocity - v)

where p is the relative pose space motion of the joint (the axial rotation of a revolute joint, or axial translation for a prismatic joint) and v is the rate of change of this motion.

For all limits that specify ranges, a "low" limit larger than the "high" limit means the joint motion along that axis is locked.

### Articulations

Earlier we mentioned that we support reduced coordinate joints, which require some additional specification. We decided to do this with a minimal extension of the above maximal joints. Any prim of the USD scene graph hierarchy may be marked with an UsdPhysicsArticulationRootAPI](https://openusd.org/release/api/class_usd_physics_articulation_root_a_p_i.html "PhysicsArticulationRootAPI can be applied to a scene graph node, and marks the subtree rooted here fo..."). This informs the simulation that any joints found in the subtree should preferentially be simulated using a reduced coordinate approach. For floating articulations (robotics jargon for something not bolted down, e.g. a wheeled robot or a quadcopter), this API should be used on the root body (typically the central mass the wheels or rotors are attached to), or a direct or indirect parent prim. For fixed articulations (robotics jargon for e.g. a robot arm for welding that is bolted to the floor), this API can be on a direct or indirect parent of the root joint which is connected to the world, or on the joint itself. If there are multiple qualifying bodies or joints under an [UsdPhysicsArticulationRootAPI prim, each is made into a separate articulation root.

This should in general make it possible to uniquely identify a distinguished root body or root joint for the articulation. From this root, a tree of bodies and joints is identified that is not to contain loops (which may be closed by joint collections). If loops are found, they may be broken at an arbitrary location. Alternatively, a joint in the loop may use its excludeFromArticulation attribute flag to denote that it wishes to remain a maximal joint, and at this point the loop is then broken.

## Examples

### Box on Box

### Box on Quad

#usda 1.0

#More advanced example showing mesh collisions

#and center of mass offset. We expect to have the

#quad be represented as a convex mesh and have the

#cube come to rest on it, balanced on one of its corners.

(

defaultPrim = "World"

endTimeCode = 100

metersPerUnit = 0.01

startTimeCode = 0

timeCodesPerSecond = 24

upAxis = "Z"

kilogramsPerUnit = 1

)

def Xform "World"

{

def PhysicsScene "PhysicsScene"

{

}

#Added a mass API so we can offset the center of mass.

def Cube "BoxActor" (

prepend apiSchemas = \["PhysicsCollisionAPI", "PhysicsRigidBodyAPI", "PhysicsMassAPI"\]

)

{

#explicit mass

float physics:mass = 10.0

#Offset center of mass so the cube settles on its corner

point3f physics:centerOfMass = (40.0, 40.0, 40.0)

color3f\[\] primvars:displayColor = \[(0.2784314, 0.4117647, 1)\]

double size = 25

double3 xformOp:translate = (0, 0, 500)

uniform token\[\] xformOpOrder = \["xformOp:translate"\]

}

#A quad mesh that serves as the ground.

def Mesh "Ground" (

prepend apiSchemas = \["PhysicsCollisionAPI", "PhysicsMeshCollisionAPI"\]

)

{

#Approximate with a convex hull, if we remove this, it will

#be used directly

uniform token physics:approximation = "convexHull"

uniform token subdivisionScheme = "none"

uniform bool doubleSided = 1

int\[\] faceVertexCounts = \[4\]

int\[\] faceVertexIndices = \[0, 1, 2, 3\]

normal3f\[\] normals = \[(0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1)\]

point3f\[\] points = \[(-1, 1, 0), (1, 1, 0), (1, -1, 0), (-1, -1, 0)\]

color3f\[\] primvars:displayColor = \[(0.5, 0.5, 0.5)\]

texCoord2f\[\] primvars:st = \[(0, 1), (1, 1), (1, 0), (0, 0)\] (

interpolation = "varying"

)

float3 xformOp:scale = (750, 750, 750)

uniform token\[\] xformOpOrder = \["xformOp:scale"\]

}

def SphereLight "SphereLight"

{

float intensity = 30000

float radius = 150

double3 xformOp:translate = (650, 0, 1150)

uniform token\[\] xformOpOrder = \["xformOp:translate"\]

}

}

### Spheres with Materials

#usda 1.0

#Shows two spheres, one with high

#and one with low bounce, by using materials.

#Also uses a trimesh ground quad.

(

defaultPrim = "World"

endTimeCode = 100

metersPerUnit = 0.01

startTimeCode = 0

timeCodesPerSecond = 24

upAxis = "Z"

#New mass scaling

kilogramsPerUnit = 1.0

)

def Xform "World"

{

def PhysicsScene "PhysicsScene"

{

}

def Sphere "RegularSphere" (

prepend apiSchemas = \["PhysicsCollisionAPI", "PhysicsRigidBodyAPI"\]

)

{

rel material:binding:physics = </World/Looks/RegularMaterial>

color3f\[\] primvars:displayColor = \[(0.2784314, 0.4117647, 1)\]

double3 xformOp:translate = (0, 0, 500)

float3 xformOp:scale = (25, 25, 25)

uniform token\[\] xformOpOrder = \["xformOp:translate", "xformOp:scale"\]

}

def Sphere "BouncySphere" (

prepend apiSchemas = \["PhysicsCollisionAPI", "PhysicsRigidBodyAPI"\]

)

{

rel material:binding:physics = </World/Looks/BouncyMaterial> (

bindMaterialAs = "weakerThanDescendants"

)

color3f\[\] primvars:displayColor = \[(0.8784314, 0.2117647, 0.1)\]

double3 xformOp:translate = (300, 0, 500)

float3 xformOp:scale = (25, 25, 25)

uniform token\[\] xformOpOrder = \["xformOp:translate", "xformOp:scale"\]

}

def Mesh "Ground" (

prepend apiSchemas = \["PhysicsCollisionAPI"\]

)

{

int\[\] faceVertexCounts = \[4\]

int\[\] faceVertexIndices = \[3, 2, 1, 0\]

uniform token subdivisionScheme = "none"

normal3f\[\] normals = \[(0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1)\]

point3f\[\] points = \[(-1, 1, 0), (1, 1, 0), (1, -1, 0), (-1, -1, 0)\]

color3f\[\] primvars:displayColor = \[(0.5, 0.5, 0.5)\]

texCoord2f\[\] primvars:st = \[(0, 1), (1, 1), (1, 0), (0, 0)\] (

interpolation = "varying"

)

float3 xformOp:scale = (750, 750, 750)

uniform token\[\] xformOpOrder = \["xformOp:scale"\]

}

def Scope "Looks"

{

def Material "RegularMaterial" (

prepend apiSchemas = \["PhysicsMaterialAPI"\]

)

{

double physics:density = 10

float physics:restitution = 0.1

}

def Material "BouncyMaterial" (

prepend apiSchemas = \["PhysicsMaterialAPI"\]

)

{

double physics:density = 10

float physics:restitution = 0.8

}

}

def SphereLight "SphereLight"

{

float intensity = 30000

float radius = 150

double3 xformOp:translate = (650, 0, 1150)

uniform token\[\] xformOpOrder = \["xformOp:translate"\]

}

}

### Group Filtering

#usda 1.0

#Shows two boxes that collide with a ground box

#but do not collide with each other thanks to

#group based filtering.

(

defaultPrim = "World"

endTimeCode = 100

metersPerUnit = 0.01

startTimeCode = 0

timeCodesPerSecond = 24

upAxis = "Z"

kilogramsPerUnit = 1.0

)

def Xform "World"

{

def PhysicsScene "PhysicsScene"

{

}

def Cube "Box1" (

prepend apiSchemas = \["PhysicsCollisionAPI", "PhysicsRigidBodyAPI"\]

)

{

color3f\[\] primvars:displayColor = \[(0.2784314, 0.4117647, 1)\]

double size = 25

double3 xformOp:translate = (0, 0, 50)

uniform token\[\] xformOpOrder = \["xformOp:translate"\]

}

def Cube "Box2" (

prepend apiSchemas = \["PhysicsCollisionAPI", "PhysicsRigidBodyAPI"\]

)

{

color3f\[\] primvars:displayColor = \[(0.2784314, 0.4117647, 1)\]

double size = 25

double3 xformOp:translate = (0, 0, 100)

uniform token\[\] xformOpOrder = \["xformOp:translate"\]

}

def PhysicsCollisionGroup "DynamicGroup" (

prepend apiSchemas = \["CollectionAPI:colliders"\]

)

{

prepend rel collection:colliders:includes = \[\
\
</World/Box1>,\
\
</World/Box2>\
\
\]

prepend rel physics:filteredGroups = \[\
\
</World/DynamicGroup>\
\
\]

}

def Cube "Ground" (

prepend apiSchemas = \["PhysicsCollisionAPI"\]

)

{

color3f\[\] primvars:displayColor = \[(0.5, 0.5, 0.5)\]

float3 xformOp:scale = (750, 750, 10)

uniform token\[\] xformOpOrder = \["xformOp:scale"\]

}

def SphereLight "SphereLight"

{

float intensity = 30000

float radius = 150

double3 xformOp:translate = (650, 0, 1150)

uniform token\[\] xformOpOrder = \["xformOp:translate"\]

}

}

### Pair Filtering

#usda 1.0

(

defaultPrim = "World"

endTimeCode = 100

metersPerUnit = 0.01

startTimeCode = 0

timeCodesPerSecond = 24

upAxis = "Z"

kilogramsPerUnit = 1.0

)

def Xform "World"

{

def PhysicsScene "PhysicsScene"

{

}

def Cube "Box1" (

prepend apiSchemas = \["PhysicsCollisionAPI", "PhysicsRigidBodyAPI"\]

)

{

color3f\[\] primvars:displayColor = \[(0.2784314, 0.4117647, 1)\]

double size = 25

double3 xformOp:translate = (0, 0, 50)

uniform token\[\] xformOpOrder = \["xformOp:translate"\]

}

def Cube "Box2" (

prepend apiSchemas = \["PhysicsCollisionAPI", "PhysicsRigidBodyAPI", "PhysicsFilteredPairsAPI"\]

)

{

prepend rel physics:filteredPairs = </World/Box1>

color3f\[\] primvars:displayColor = \[(0.2784314, 0.4117647, 1)\]

double size = 25

double3 xformOp:translate = (0, 0, 100)

uniform token\[\] xformOpOrder = \["xformOp:translate"\]

}

def Cube "Ground" (

prepend apiSchemas = \["PhysicsCollisionAPI"\]

)

{

color3f\[\] primvars:displayColor = \[(0.5, 0.5, 0.5)\]

float3 xformOp:scale = (750, 750, 10)

uniform token\[\] xformOpOrder = \["xformOp:scale"\]

}

def SphereLight "SphereLight"

{

float intensity = 30000

float radius = 150

double3 xformOp:translate = (650, 0, 1150)

uniform token\[\] xformOpOrder = \["xformOp:translate"\]

}

}

### Joint

#usda 1.0

#Shows a joint which is driven to rotate

#around the vertical axis with a constant

#speed.

(

defaultPrim = "World"

endTimeCode = 100

metersPerUnit = 0.01

startTimeCode = 0

timeCodesPerSecond = 24

upAxis = "Z"

kilogramsPerUnit = 1.0

)

def Xform "World"

{

def PhysicsScene "physicsScene"

{

float3 gravity = (0, 0, -1000)

}

def Cube "StaticBox" (

prepend apiSchemas = \["PhysicsCollisionAPI"\]

)

{

color3f\[\] primvars:displayColor = \[(0.64705884, 0.08235294, 0.08235294)\]

double size = 100

quatf xformOp:orient = (1, 0, 0, 0)

float3 xformOp:scale = (0.1, 1, 0.1)

double3 xformOp:translate = (0, 0, 1000)

uniform token\[\] xformOpOrder = \["xformOp:translate", "xformOp:orient", "xformOp:scale"\]

}

def Cube "DynamicBox" (

prepend apiSchemas = \["PhysicsCollisionAPI", "PhysicsRigidBodyAPI"\]

)

{

color3f\[\] primvars:displayColor = \[(0.2784314, 0.64705884, 1)\]

double size = 100

quatf xformOp:orient = (1, 0, 0, 0)

float3 xformOp:scale = (0.1, 1, 0.1)

double3 xformOp:translate = (0, 120, 1000)

uniform token\[\] xformOpOrder = \["xformOp:translate", "xformOp:orient", "xformOp:scale"\]

}

#Joint with 5 limits and one drive

def PhysicsJoint "D6Joint" (

prepend apiSchemas = \["PhysicsLimitAPI:transX", "PhysicsLimitAPI:transY", "PhysicsLimitAPI:transZ", "PhysicsLimitAPI:rotX", "PhysicsLimitAPI:rotY", "PhysicsDriveAPI:rotZ"\]

)

{

rel physics:body0 = </World/StaticBox>

rel physics:body1 = </World/DynamicBox>

float limit:rotX:physics:high = -1

float limit:rotX:physics:low = 1

float limit:rotY:physics:high = -1

float limit:rotY:physics:low = 1

float limit:transX:physics:high = -1

float limit:transX:physics:low = 1

float limit:transY:physics:high = -1

float limit:transY:physics:low = 1

float limit:transZ:physics:high = -1

float limit:transZ:physics:low = 1

float drive:rotZ:physics:targetVelocity = 10.0

float drive:rotZ:physics:damping = 9999.0

point3f physics:localPos0 = (0, 60, 0)

point3f physics:localPos1 = (0, -60, 0)

quatf physics:localRot0 = (1, 0, 0, 0)

quatf physics:localRot1 = (1, 0, 0, 0)

}

def SphereLight "SphereLight"

{

float intensity = 30000

float radius = 150

double3 xformOp:translate = (650, 0, 1150)

uniform token\[\] xformOpOrder = \["xformOp:translate"\]

}

}

### Distance Joint

#usda 1.0

#Shows a dynamic box connected

#to a fixed box with a distance joint.

(

defaultPrim = "World"

endTimeCode = 100

metersPerUnit = 0.01

startTimeCode = 0

timeCodesPerSecond = 24

upAxis = "Z"

kilogramsPerUnit = 1.0

)

def Xform "World"

{

def PhysicsScene "physicsScene"

{

}

def Cube "StaticBox" (

prepend apiSchemas = \["PhysicsCollisionAPI"\]

)

{

color3f\[\] primvars:displayColor = \[(0.64705884, 0.08235294, 0.08235294)\]

double size = 100

quatf xformOp:orient = (1, 0, 0, 0)

float3 xformOp:scale = (0.1, 1, 0.1)

double3 xformOp:translate = (0, 0, 1000)

uniform token\[\] xformOpOrder = \["xformOp:translate", "xformOp:orient", "xformOp:scale"\]

}

def Cube "DynamicBox" (

prepend apiSchemas = \["PhysicsCollisionAPI", "PhysicsRigidBodyAPI"\]

)

{

color3f\[\] primvars:displayColor = \[(0.2784314, 0.64705884, 1)\]

double size = 100

float3 velocity = (0, 0, 0)

quatf xformOp:orient = (1, 0, 0, 0)

float3 xformOp:scale = (0.1, 1, 0.1)

double3 xformOp:translate = (0, 120, 1000)

uniform token\[\] xformOpOrder = \["xformOp:translate", "xformOp:orient", "xformOp:scale"\]

}

def DistancePhysicsJoint "DistanceJoint"

{

rel physics:body0 = </World/StaticBox>

rel physics:body1 = </World/DynamicBox>

float3 physics:localPos0 = (0, 60, 0)

float3 physics:localPos1 = (0, -60, 0)

quatf physics:localRot0 = (1, 0, 0, 0)

quatf physics:localRot1 = (1, 0, 0, 0)

float physics:maxDistance = 50

float physics:minDistance = 10

}

def SphereLight "SphereLight"

{

float intensity = 30000

float radius = 150

double3 xformOp:translate = (650, 0, 1150)

uniform token\[\] xformOpOrder = \["xformOp:translate"\]

}

}

# Physics Parsing Utils Overview.

## Purpose and Scope

When the UsdPhysics schema is applied to the stage, processing the stage UsdPrims and creating a consistent physics representation can be challenging, especially when dealing with different simulators. The UsdPhysics Parsing Utilities are designed to help achieve a consistent physics representation as defined in the USD stage. The stage, or generally a vector of paths, is parsed, and the function returns a callback or dictionary with parsed physics descriptors. Additionally, it can leverage the concept of simulation owners to partition the stage for multiple simulation engines.

## Specification

The UsdPhysics schemas add physics capabilities to USD; however, deducing and parsing the physics information can be non-trivial and may be misinterpreted. This section aims to specify what data are returned by the parsing utilities and what to expect in the parsed physics descriptors.

## Physics Parsing Parameters

The parsing utility function takes several input parameters that need to be described:

- The stage to be parsed.
- Include paths vector. This vector define what all paths will be included for parsing. The parsing will construct a UsdPrimRange](https://openusd.org/release/api/class_usd_prim_range.html "An forward-iterable range that traverses a subtree of prims rooted at a given prim in depth-first ord...") from each [UsdPrim](https://openusd.org/release/api/class_usd_prim.html "UsdPrim is the sole persistent scenegraph object on a UsdStage, and is the embodiment of a \"Prim\" as ...") found in the given paths (the [UsdPrimRange. Its possible to exclude some paths through an exclude vector, see exclude path vector parameter.
- Report callback function that reports the descriptors back to the calling code. The report function returns an array of the same descriptors to be processed efficiently. Note that in python instead of a callback function all the descriptors are returned in a dictionary, where the key is the descriptor type and the value is a tuple of UsdPrim paths array and descriptors array.
- User data is an optional parameter. Clients of ReportFn can optionally provide userData which is a Simulator aware data structure wrapped in VtValue. A client's implementation can then unpack any parsed physics object description to the Simulator aware data structures in their implementation of UsdPhysicsReportFn.
- Optional exclude paths vector, this vector defines what UsdPrims are pruned from the UsdPrimRange traversal done during parsing.
- Optional custom physics tokens. There can be multiple optional custom physics tokens defined. The custom tokens can hold three arrays of tokens that define a custom behavior. The custom geometry tokens specify if a certain geometry should be treated as a custom geometry that the calling code will handle. The custom joint tokens specify which joints are custom and the calling code needs to handle them. The custom point instancer tokens specify what point instancer hierarchies should be ignored in addition to standard point instancers. Note that point instancer hierarchies are skipped. It is expected that for a point instancer that one does parse individual prototypes.
- Optional simulation owners can be provided to restrict parsing to only certain simulation owners. It is expected that simulation owners are PhysicsScene primitives. The parsing will then restrict parsing only to those scenes and objects that do have a simulation owner set to those scenes. If empty SdfPath is provided, objects without a simulation owner are also returned. The behavior for simulation owners is described below for each type.

## Physics Parsing Determinism

The parsing should be deterministic; it's important that each subsequent run reports the same information in the same order. This is especially crucial for elements like articulation roots. In some cases, the parsing sacrifices performance to achieve deterministic results.

## Physics Object Descriptor

The base physics object descriptor is never returned, all types are derived from this base class. This base class contains this physics information:

- Type of the physics descriptor, PhysicsObjectType::Enum defines all types available.
- UsdPrim path that this descriptor was parsed from.
- IsValid boolean that specifies whether a returned described is valid or not. Note that the descriptors are processed in batches and if validation fails the descriptor is invalidated rather then removed from the array.

## Physics Scene Descriptor

For each UsdPhysicsScene prim a PhysicsDesc is reported, this descriptor contains:

- Gravity direction, if there is no authored value or sentinel value provided a default direction is computed based on the stage upAxis.
- Gravity magnitude, if there is no authored value or sentinel value provided a default Earth gravity is computed based on the stage meters per unit value.

### Physics Scene Simulation Owner Behavior

If the parsing utility is provided with a list of allowed simulation owners, the behavior changes. The returned PhysicsSceneDesc objects are restricted to only those UsdPhysicsScene](https://openusd.org/release/api/class_usd_physics_scene.html "General physics simulation properties, required for simulation.") prims whose paths match one of the provided simulation owners. It is expected that the [UsdPhysicsScene prim path is the simulation owner that defines where each body or collision belongs.

## Physics RigidBody Material Descriptor

For each prim with UsdPhysicsMaterialAPI](https://openusd.org/release/api/class_usd_physics_material_a_p_i.html "Adds simulation material properties to a Material.") a [UsdPhysicsRigidBodyMaterialDesc is reported, this descriptor contains:

- Static friction defined in USD.
- Dynamic friction defined in USD.
- Restitution defined in USD.
- Density defined in USD, note that density is expected to be specified in the stage unit's.

### Physics RigidBody Material Simulation Owner Behavior

Since prims can be added/enabled during runtime, all physics materials are returned and not restricted by the simulation owners.

## Physics Collision Group Descriptor

For each prim with UsdPhysicsCollisionGroup a possible PhysicsCollisionGroupDesc is created. In a subsequent pass the collision groups descriptors merge based on the mergeGroups attribute. The final list contains the merged groups and contains this information:

- Inverted filtered groups boolean specifying whether the group does not (default true) collide with the provided list of filtered groups.
- Filtered groups list, list of groups that this group should not collide by default (can be inverted by boolean above).
- If group should be merged, the specified merge group name.
- Merged groups list, list of all groups paths that were merged together.

### Physics Collision Group Simulation Owner Behavior

Since prims can be added/enabled during runtime, all physics collision groups are returned and not restricted by the simulation owners.

## Physics RigidBody Descriptor

For each UsdGeom.Xformable prim with UsdPhysicsRigidBodyAPI, a physics rigid body descriptor is reported. Note that nesting rigid bodies is not allowed; therefore, an error will be reported if attempted. To nest rigid bodies, it is necessary to add resetXformOpStack on the rigid body. The reported descriptor contains this information:

- List of collisions that belong to this rigid body.
- List of simulation owners for this rigid body, the behavior is described below.
- List of filtered collisions, the list contains all paths that the rigid body should not collide with. Note that this can be an arbitrary path in the USD hierarchy. it's up to the implementation to check what objects are created under the hierarchy.
- World position of the rigid body.
- World orientation of the rigid body.
- World scale of the rigid body.
- Rigid body enabled boolean for fast toggle between a dynamic and static body.
- Kinematic body boolean, kinematic body is expected to be driven by transformation changes.
- Starts asleep boolean, specifies if a body should be awake or not when the sim starts.
- Initial angular velocity (note that the expected unit's are in degree and body local space).
- Initial linear velocity (note that the expected velocity is in body local space).

### Physics RigidBody Simulation Owner Behavior

The rigid body's simulation owners list is compared with the provided requested simulation owners list. If a match is found, the rigid body is reported; otherwise, the rigid body is skipped.

## Physics Shape Descriptor

Physics shape descriptor is the base class for all specific shapes reported. In general for each prim with UsdPhysicsCollisionAPI a derived physics shape descriptor is returned. The base shape descriptor contains this shared information:

- Rigid body that the collision belongs to, if the rigid body path is not valid, it's considered a static collision.
- Local position relative to the rigid body primitive, if static collision, the local position is a world position.
- Local orientation relative to the rigid body primitive, if static collision, the local orientation is a world orientation.
- Local scale relative to the rigid body primitive, if static collision, the local scale is a world position.
- List of used materials (path to a prim with material API). This list contains invalid SdfPath.
- List of simulation owners for this collision, the behavior is described below.
- List of filtered collisions, the list contains all paths that the collision should not collide with. Note that this can be an arbitrary path in the USD hierarchy. It's up to the implementation to check what objects are created under the hierarchy.
- List of collision groups this collision belongs to, note that only collision groups that are part of the current parsing include paths are checked for a scoped parse, this needs to be checked for already existing groups reported by full parse.
- Boolean to specify whether collision is enabled or disabled, this can be used to create objects in physics and then avoid runtime creation on the objects while toggling the USD attribute that enables/disables the collision.

Note that many collision representations do not support non-uniform scale, behavior for each type is described below. While non-uniform scale works for some representations, scale of the orientation (skew) never works, and warnings should be reported.

## Physics Sphere Shape Descriptor

If physics collision API is applied to a UsdGeom.Sphere primitive, then a physics sphere shape descriptor is reported. The descriptor contains this additional information:

- The sphere radius. Note that the radius already contains scale, so it's the expected radius in world space. Also, since the sphere can't have non-uniform scale, if non-uniform scale is found a warning is reported. In case of a non-uniform scale the maximum of the scale per axis value is used to scale the UsdGeom.Sphere radius.

## Physics Capsule Shape Descriptor

If physics collision API is applied to a UsdGeom.Capsule primitive, then a physics capsule shape descriptor is reported. The descriptor contains this additional information:

- The axis around which the capsule is represented.
- The capsule half height, measured from the center to the cylindrical top, where the hemispherical sphere based on radius is added. Note that the height contains already scale so it's the expected height in world space.
- The radius of the capsule, note that the radius contains already scale so it's the expected radius in world space. Note since the capsule can't have generic non-uniform scale, if non-uniform scale around radius is found a warning is reported. The scale along the provided axis does scale the height, while scale around the other two axes define the radius scale, the maximum of those is used in case of a non-uniform scale.

## Physics Cylinder Shape Descriptor

If physics collision API is applied to a UsdGeom.Cylinder primitive, then a physics cylinder shape descriptor is reported. The descriptor contains this additional information:

- The axis around which the cylinder is represented.
- The cylinder half height, measured from the center to the cylindrical top. Note that the height contains already scale so it's the expected height in world space.
- The radius of the cylinder, note that the radius contains already scale so it's the expected radius in world space. Note since the cylinder can't have generic non-uniform scale, if non-uniform scale around radius is found a warning is reported. The scale along the provided axes does scale the height, while scale around the other two axis define the radius scale, the maximum of those is used in case of a non-uniform scale.

## Physics Cone Shape Descriptor

If physics collision API is applied to a UsdGeom.Cone primitive, then a physics cone shape descriptor is reported. The descriptor contains this additional information:

- The axis around which the cone is represented.
- The cone half height, measured from the center to the cone top. Note that the height contains already scale so it's the expected height in world space.
- The radius of the cone, note that the radius contains already scale so it's the expected radius in world space. Note since the cone can't have generic non-uniform scale, if non-uniform scale around radius is found a warning is reported. The scale along the provided axes does scale the height, while scale around the other two axis define the radius scale, the maximum of those is used in case of a non-uniform scale.

## Physics Cube Shape Descriptor

If physics collision API is applied to a UsdGeom.Cube primitive, then a physics cube shape descriptor is reported. The descriptor contains this additional information:

- The half extent of the cube including scale. Non-uniform scale in this case is supported as the cube size is multiplied by the world scale.

## Physics Plane Shape Descriptor

If physics collision API is applied to a UsdGeom.Plane primitive, then a physics plane shape descriptor is reported. The descriptor contains this additional information:

- The axis where the plane normal is pointing.

## Physics Mesh Shape Descriptor

If physics collision API is applied to a UsdGeom.Mesh primitive, then a physics mesh shape descriptor is reported. The descriptor contains this additional information:

- The desired approximation for the mesh geometry.
- The mesh world scale, note that the points are not scaled, hence for meshes we need a separate scale information.
- Hint whether the mesh is double sided or not.

## Physics Sphere Points Shape Descriptor

If physics collision API is applied to a UsdGeom.Points primitive, then a physics sphere points shape descriptor is reported. The descriptor contains this additional information:

- The list of sphere points to be created, each sphere point is defined by a center and radius in world coordinate. Note that non-uniform scale is not supported, the max per axis scale is applied.

## Physics Custom Shape Descriptor

Custom collisions can be defined on UsdPrims, if a custom shape token is provided to the parsing function then if the prim type or any of the prim applied APIs matches the custom shape token a custom shape descriptor is reported, these collisions are then handled by the application. The custom shape descriptor contains:

- Custom geometry token that was provided to the parsing function, this identifies the custom geometry.

- The list of sphere points to be created, each sphere point is defined by a center and radius in world coordinate. Note that non-uniform scale is not supported, the max per axis scale is applied.

### Physics Collision Simulation Owner Behavior

Collisions can have their own list of simulation owners to which they can belong. However, since they are associated with a rigid body, the behavior changes depending on whether or not they belong to a rigid body. If a collision belongs to a rigid body and one of the rigid body's simulation owners matches the requested simulation owners list, then the collision is reported. If the rigid body's simulation owners do not match, the collision is not reported. If a collision does not belong to a rigid body, then the collision's simulation owners list is compared with the provided requested simulation owners list, and if a match is found, the collision is reported; otherwise, the collision is skipped.

## Physics Joint Descriptor

Common base physics joint descriptor, this base class should not be instantiated. All joint types are derived from this base descriptor. For parsing, any UsdPhysics.Joint prim and its derived types a joint descriptor is reported. This descriptor contains:

- Path relationship defined in body0 rel, this path does not necessarily have to be the body path, it's possible to use some child prim as the relationship. Empty SdfPath means, the joint does have a world anchor, note that at least one dynamic body should be defined for body0 or body1.
- Path relationship defined in body1 rel, this path does not necessarily have to be the body path, it's possible to use some child prim as the relationship. Empty SdfPath means, the joint does have a world anchor, note that at least one dynamic body should be defined for body0 or body1.
- Path to body0, this does not have to be the path defined in the body0 rel, this is the path to the body that the joint is constraining. All poses defined in the descriptor are relative to the rigid body rather then the rel prim.
- Path to body1, this does not have to be the path defined in the body1 rel, this is the path to the body that the joint is constraining. All poses defined in the descriptor are relative to the rigid body rather then the rel prim.
- Local position against the body0 world frame.
- Local orientation against the body0 world frame.
- Local position against the body1 world frame.
- Local orientation against the body1 world frame.
- Boolean to quickly toggle the joint enabled/disable state.
- Boolean that defines whether collision between the constrained bodies is enabled or disabled.
- Boolean that can exclude joint from articulation, this is required in cases where articulation topology would create closed loops, those are in many cases not supported.
- Break force, when the joint breaks.
- Break torque when the joint breaks.

## Physics Fixed Joint Descriptor

For each UsdPhysicsFixedJoint a fixed joint descriptor is reported. The fixed joint does not have any additional parameters to be reported.

## Physics Revolute Joint Descriptor

For each UsdPhysicsRevoluteJoint a revolute joint descriptor is reported. The revolute joint adds this information:

- The revolute joint axis.
- The revolute joint limit, it can be enabled or disabled. Additionally a pair of low high angle is reported (note that the units are by default degree as per USD spec).
- The revolute joint drive, the drive that can be added to the revolute joint. Drive does have target position, target velocity, stiffness and damping. The expected drive formula looks like this - force = spring \* (target position - position) + damping \* (targetVelocity - velocity)

## Physics Prismatic Joint Descriptor

For each UsdPhysicsPrismaticJoint a prismatic joint descriptor is reported. The prismatic joint adds this information:

- The prismatic joint axis.
- The prismatic joint limit, it can be enabled or disabled. Additionally a pair of low high angle is reported.
- The prismatic joint drive, the drive that can be added to the prismatic joint. Drive does have target position, target velocity, stiffness and damping. The expected drive formula looks like this - force = spring \* (target position - position) + damping \* (targetVelocity - velocity)

## Physics Spherical Joint Descriptor

For each UsdPhysicsSphericalJoint a spherical joint descriptor is reported. The spherical joint adds this information:

- The spherical joint axis.
- The spherical joint limit, it can be enabled or disabled. Additionally a cone0 and cone1 angle is reported (note that the units are by default degree as per USD spec). Note that in python the limit is always just lower/upper, the lower value is cone0 value and upper value is the cone1 value.

## Physics Distance Joint Descriptor

For each UsdPhysicsDistanceJoint a distance joint descriptor is reported. The distance joint adds this information:

- Boolean stating whether the minimum limit is enabled as both limits can be enabled disabled independently.
- Boolean stating whether the maximum limit is enabled as both limits can be enabled disabled independently.
- The limit, for minimum and maximum definition. Note that in python the lower value defines the minimum, while the upper value defines the maximum. Both values are only valid if the bools above are true.

## Physics D6 Joint Descriptor

For each UsdPhysicsJoint a D6 joint descriptor is reported. The D6 joint is a generic joint that can be setup per axis. The parsing reports these:

- List of limit pairs, where first is the DOF, while the second is the actual limit. Note that for the DOF, there can be three states, the limit is not set - free, limit is set lower is lower then upper then the limit is valid from lower to upper. If upper is lower then lower, then the DOF is considered locked.
- List of drive pairs, where first is the DOF, while the second is the drive.

## Physics Custom Joint Descriptor

For each derived UsdPhysicsJoint type that matches the custom joint token provided to the parsing custom joint descriptor is reported. This type does not have any additional information.

### Physics Joint Simulation Owner Behavior

Joints do not have own simulation owner, the simulation owner is decided based on the bodies the joint constrained. If both rigid bodies belong to the same simulation owner and the simulation owner is passed as the argument to parsing, the joint is returned.

## Physics Articulation Descriptor

For each UsdPhysicsArticulationRootAPI, an articulation descriptor is returned. The articulation root API serves as a hint for the solver that the topology below the articulation root should be solved as a reduced coordinate system. Note that it is not possible to nest articulation root APIs within a hierarchy; doing so will result in an error. The articulation descriptor contains this information:

- List of root paths is provided, with each path in this list indicating that a topology starts from this root. There can be multiple topologies found under one articulation root. Note that the root can be reported as either a rigid body path, in which case the articulated topology is considered floating, or as a joint path, in which case the articulated topology is considered fixed-based.
- List of filtered collisions for this articulation.
- List of joints that belong to this articulation.
- List of bodies that belong to this articulation

It is expected that the articulation descriptor information is used to construct a reduced coordinate topology, starting from the reported roots and using the provided list of joints and bodies to construct the topology. Note that in the case of closed loops, the joints are usually excluded from the articulation; however, they are still reported in the list of joints.

### Physics Articulation Simulation Owner Behavior

Articulations do not have their own simulation owner; the simulation owner is determined based on the bodies that belong to the articulation. If all bodies have the same simulation owner, then the articulation is reported and should be created. If there is no alignment among the bodies regarding where they should be simulated, the articulation is not reported.
