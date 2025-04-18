 # Cable Joints Physics Engine

 This is a pure-JavaScript physics engine for simulating cables wrapping and sliding over pulleys/wheels, built on a Position-Based Dynamics (PBD) approach with an Entity-Component-System (ECS) core.

 ## 1. Purpose
 - A physics engine for cables interacting with rolling wheels and other obstacles.
 - Position-Based Dynamics (PBD) solver ensuring cable length constraints.
 - Supports generic ECS-based entities/components/systems.
 - Includes interactive demos (e.g., pin-ball flipper) and unit tests.

 ## 2. Top-Level Structure
 - `package.json` / `package-lock.json`
   - No runtime dependencies; Jest for unit tests.
   - `npm test` runs the full Jest suite under `tests/`.
 - `cable_joints/`
   - `cable_joints_core.js` — main library (geometry, ECS core, systems, solver, renderer).
   - `cable_joints.html` — browser demo harness.
   - `cable_joints_test.html` — in-browser smoke tests.
   - `cableJoints.pdf` — accompanying research paper.
 - `tests/` — Jest tests for geometry, components, systems, and solver logic.
 - Root HTML demos:
   - `flipper.html`, `flipper_replay3.html`, `flipper_replay4.html` — pin-ball/flipper demos driven by input replays.
 - Documentation & notes:
   - `TODO`, `ideas_about_what_to_do_from_here.md`, research PDFs, and other design notes.

 ## 3. Core Library (`cable_joints_core.js`)
 ### a. Geometry Utilities
 - `Vector2` — 2D vector operations (add, subtract, dot, normalize, etc.).
 - `closestPointOnSegment`, `lineSegmentCircleIntersection`, `rightOfLine`.
 - Tangent-point algorithms:
   - `tangentFromPointToCircle`, `tangentFromCircleToPoint`, `tangentFromCircleToCircle`.
 - `signedArcLengthOnWheel` — compute signed arc length around a wheel.

 ### b. ECS Core
 - `World` — entity/component storage, resources, system registration, update loop.
 - Add/get/remove/query components on entities.

 ### c. Components
 - `PositionComponent`, `VelocityComponent`.
 - `MassComponent`, `RestitutionComponent`, `RadiusComponent`.
 - Tags: `BallTagComponent`, `ObstacleTagComponent`, `GravityAffectedComponent`, etc.
 - `CableJointComponent` — segment constraint between two entities with rest length.
 - `CablePathComponent` — aggregates cable joints, link types (`attachment` vs. `rolling`), arc-length storage, and total rest length.
 - Render, pause, error, and input-log components.

 ### d. Systems
 - `GravitySystem` — applies gravity.
 - `MovementSystem` — integrates positions by velocities.
 - `PBDBallBallCollisions` / `PBDBallObstacleCollisions` — PBD-based collision resolution.
 - `CableAttachmentUpdateSystem` — computes and updates tangent points when cables wrap around wheels.
 - `PBDCableConstraintSolver` — enforces total cable-length constraint via PBD gradient-based corrections.
 - `InputReplaySystem` — feeds recorded user inputs into the simulation.
 - `RenderSystem` — draws entities and cables onto an HTML5 canvas.

 ## 4. Unit Tests (`tests/`)
 - Geometry: segment-circle intersection, closest point, tangent functions, signed arc length, left/right of line.
 - Components: `CablePathComponent` constructor behavior.
 - Systems: gravity, movement, ball-ball/obstacle collisions.
 - Tangent-point cases: all combinations of circle-to-circle (TT, TF, FT, FF).

 ## 5. Demos & Usage
 1. `npm install`
 2. `npm test` — run all unit tests.
 3. Open `cable_joints/cable_joints.html` or `flipper.html` in a browser for interactive demos.

 ---  
 _This README was generated to provide an overview of the Cable Joints codebase._  