# Logic to Implement

Extended Position Based Dynamics (XPBD) time stepping/physics simulation framework.
We will use substepping as our main method for reducing errors and enhancing stability.

Cable Joints, Matthias Müller et al contains the logic we want to use for simulating cables.
We have a home-rolled version of this logic, including merge/split and hybrid features, implemented in javascript.

We might want a home-rolled extension to Universal Scene Description (USD) language,
to get be able to express Cable Joints in some language as close to USD as possible.

We ultimately want to express the and logic of the Hangprinter Roboti in USD.

# Software Architecture

## Entity Component System (ECS)

Entities: Simple identifiers (IDs). No data, no logic.
Components: Plain old data structures (PODs). Represent a single aspect of an entity (e.g., Position, Velocity, RenderableMesh, CollidableBody, AffectedByGravity). They contain state, not logic.
Systems: Contain the logic. They operate on entities that possess a specific set of components (e.g., PhysicsSystem queries for entities with Position, Velocity, Mass, CollidableBody; RenderingSystem queries for Position, RenderableMesh).

### ECS combined with Vertical Slice Architecture

We want to separate our program into features,
and have all files and documentation belonging to one feature inside one directory,
so we can give the AI assistant all the context it needs in order to work on a feature with a simple
```
/read ./the_feature/*
```
adding one whole directory to the context window.
No manually curating context snippets or bloating the context window.

This means we copy lots of utility code into each feture directory.
This is ok, as long as we isolate our features from each other.

# Libraries

Nvidia Warp library.
@wp.array for the components of ECS.
We'll have to use something else to manage the entity ids.

Probably VGA. Probably not Isaac Sim for visualization?
We'll learn more about this in later Ten Minute Physics lectures.

Eventually we'll probably want to integrate with Nvidia Omniverse.
That is Omniverse Kit, Isaac Sim via extensions, and Isaac Lab for Reinforcement Learning work.
We will not accept a >10s slow read-evaluate-print cycle, just to get Omniverse during development though.
Find something more light weight initially.

# Tasks
 - Build a working Slideprinter
 - Translate into Python
 - Translate into Python with Warp
