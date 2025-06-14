# Cable Joints Physics Engine (JavaScript & Python)

A physics engine for simulating cables, built on a Position-Based Dynamics (PBD) approach with an Entity-Component-System (ECS) core. It is implemented in JavaScript and Python. It's long term purpose is to simulate a cable driven robot (Hangprinter) in a way that can be integrated into Isaac Lab, Omniverse, Warp etc.

 ## 1. Purpose
 - A physics engine for cables interacting with rolling wheels and other obstacles.
 - Position-Based Dynamics (PBD) solver ensuring cable length constraints.
 - Supports generic ECS-based entities/components/systems.
 - Includes interactive demos (e.g., pin-ball flipper) and unit tests.

## 2. Top-Level Structure (JavaScript version)
 - `package.json` / `package-lock.json`
   - No runtime dependencies; Jest for javascript unit tests.
   - `npm test` runs the full Jest suite under `tests/`.
 - `cable_joints/`
   - Modular ES modules: `vector2.js`, `geometry.js`, `ecs.js`, `commonSystems.js`, `createCablePaths.js`, `debugUtils.js`, `renderSystem.js`.
   - UMD bundle: `cable_joints_core.js` - main library (geometry, ECS core, systems, solver, renderer).
   - `cable_joints.html` - browser demo harness.
   - `cable_joints_test.html` - in-browser smoke tests.
   - `hybrid/` - experimental hybrid examples.
 - `tests/` - Jest tests for geometry, components, systems, and solver logic.
 - Root HTML js based demos:
   - `flipper.html` - pin-ball/flipper basic demo.
   - `flipper_with_attached_beads.html`
   - `flipper_with_sliding_beads.html`
 - How to run js demos:
   - `cd hp-sim5`
   - `python -m http.server`
   - You might have to do `npm install` the first time?
   - Open for example http://127.0.0.1:8000/flipper.html in browser. Or:
 - Js based demos are also available at [tobbelobb.github.io/hp-sim5/flipper](https://tobbelobb.github.io/hp-sim5/flipper), [tobbelobb.github.io/hp-sim5/flipper_with_sliding_beads](https://tobbelobb.github.io/hp-sim5/flipper_with_sliding_beads), [tobbelobb.github.io/hp-sim5/flipper_with_attached_beads](https://tobbelobb.github.io/hp-sim5/flipper_with_attached_beads)
 - Slideprinter demo at slideprinter/slideprinter.html

  ## 3. Python Port
  A complete Python port of the cable joints engine is available in the `python/` directory, mirroring the JavaScript implementation with equivalent ECS, geometry, and PBD systems.
  - Directory structure:
    - Core modules: `python/vector2.py`, `python/geometry.py`, `python/ecs.py`, `python/common_systems.py`, `python/create_cable_paths.py`, `python/pbd_cable_constraint_solver.py`, and related components.
    - `python/tests/` - pytest-based test suite covering the port.
  - Dependencies:
    - numpy
    - pytest
    - websockets
    - warp-lang[extras]
  - Usage:
    1. Install dependencies: `pip install numpy pytest websockets warp-lang[extras]`
    2. Run Python tests: `pytest python/tests`
    3. (Optional) Run the Python-driven flipper demo:
      - Start server: `python flipper_server.py`
      - Open `flipper_python.html` in a browser.
    4. (Optional) To play the (non-warp) and js driven flipper both at the same time, to test
        their equivalence, first start the `python -m http.server` to start the js engine, then `python flipper_server.py`
        to start the Python engine. Then open flipper_overlay.html in a browser.
    5. (Optional) Run the Python-with-Warp-driven flipper demo:
      - Start server: `python flipper_server.py --warp`
      - Open `flipper_python_warp.html` in a browser.
    6. (Optional) `python slideprinter_usd_demo.py` demonstrates loading a USD scene via Warp.
       If the optional `pxr` and `warp` modules are not installed the script falls
       back to a tiny built‑in parser. Edit the script to parse either
       `slideprinter/slideprinter.usda` or the provided `flipper_scene.usda` to
       experiment with different setups.
    7. (Optional) `python -m python.hanging_pendulum` will generate a USD file showing a simple
       3D pendulum simulated with Warp. The output can be viewed with any USD viewer.
       See `ai_docs/Warp/README.md` for Warp installation details.
    8. (Optional) `python -m python.slideprinter_warp` will generate `slideprinter.usd`
       demonstrating a basic Slideprinter setup rendered with Warp. The script
       defaults to running on `cuda:0`; pass `--device cpu` to run on the CPU.


  ## 4. Core Library (`cable_joints_core.js`)
 ### a. Geometry Utilities
 - `Vector2` - 2D vector operations (add, subtract, dot, normalize, etc.).
 - `closestPointOnSegment`, `lineSegmentCircleIntersection`, `rightOfLine`.
 - Tangent-point algorithms:
   - `tangentFromPointToCircle`, `tangentFromCircleToPoint`, `tangentFromCircleToCircle`.
 - `signedArcLengthOnWheel` - compute signed arc length around a wheel.

 ### b. ECS Core
 - `World` - entity/component storage, resources, system registration, update loop.
 - Add/get/remove/query components on entities.

 ### c. Components
 - `PositionComponent`, `VelocityComponent`.
 - `MassComponent`, `RestitutionComponent`, `RadiusComponent`.
 - Tags: `BallTagComponent`, `ObstacleTagComponent`, `GravityAffectedComponent`, etc.
 - `CableJointComponent` - segment constraint between two entities with rest length.
 - `CablePathComponent` - aggregates cable joints, link types (`attachment`, `rolling`, `hybrid`, `hybrid-attachment`, `pinhole`), arc-length storage, and total rest length.
 - Render, pause, error, and input-log components.

 ### d. Systems
 - `GravitySystem` - applies gravity.
 - `MovementSystem` - integrates positions by velocities.
 - `PBDBallBallCollisions` / `PBDBallObstacleCollisions` - PBD-based collision resolution.
 - `CableAttachmentUpdateSystem` - computes and updates tangent points when cables wrap around wheels.
 - `PBDCableConstraintSolver` - enforces total cable-length constraint via PBD gradient-based corrections.
 - `InputReplaySystem` - feeds recorded user inputs into the simulation.
 - `RenderSystem` - draws entities and cables onto an HTML5 canvas.

## 5. Unit Tests (`tests/`)
 - Geometry: segment-circle intersection, closest point, tangent functions, signed arc length, left/right of line.
 - Components: `CablePathComponent` constructor behavior.
 - Systems: gravity, movement, ball-ball/obstacle collisions.
 - Tangent-point cases: all combinations of circle-to-circle (TT, TF, FT, FF).

-## 6. 3D Visualization
- `cable_joints_3d/cable_joints_3d.html` demonstrates rendering cable joint points using Three.js.
- Utility modules `vector3.js` and `geometry3.js` provide basic 3D math helpers and now live under `cable_joints_3d/`.
- The Python side includes `python/vector3.py` and `python/geometry3.py` for analogous 3D helpers.
