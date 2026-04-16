# Research Questions

## Context
Focus on the 3D scene-loading path, ECS components, and physics systems that define body state, cable constraints, rigid groups, and distance constraints. Trace how authored USD scene data becomes runtime entities and how positional, rotational, and velocity updates flow across a simulation step.

## Questions
1. How does `public/usd_scenes/slideprinter_hexagon.usda` get imported into runtime ECS entities in `hp-sim-3d/app/setupScene.js`, and which authored prims, relationships, and attributes currently define the spool/pinhole assembly, its rigid group, and its constraints?
2. How are rigid-group members represented at runtime today, including which components are attached to the member entities versus the group entity, and how does `RigidGroupSystem` currently compute and enforce group motion?
3. How does `PBDCableConstraintSolver` trace cable-path data into attachment points, gradients, mass/inertia terms, and position/orientation corrections, and at which points does it operate on individual entities rather than any aggregate body state?
4. How do `XPBDDistanceConstraintSystem`, `RigidGroupSystem`, and `PBDCableConstraintSolver` interact within the registered system order, and what correction ordering, iteration structure, or state handoff could explain amplified impulses or energy injection across grouped members?
5. How are linear and angular state cached and reconstructed across a frame—especially via `PrevFinalPos*`, `PrevFinalOrientation*`, `PBDVelocityUpdateSystem`, and `PBDAngularVelocityUpdateSystem`—and what assumptions do those systems make about bodies being individual entities rather than compound ones?
6. What inertia and mass abstractions exist in the 3D ECS today, including the use of scalar `MomentOfInertiaComponent`, effective-axis inertia in scene setup, and any comments or code paths that discuss limits of the current model for general 3D rigid bodies or rigid groups?
