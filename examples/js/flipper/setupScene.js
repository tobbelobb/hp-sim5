import Vector2 from '../../../src/js/cable_joints/vector2.js';
import {
  getAttribute,
  getChild,
  getChildren,
  getRelationship,
  materialProperties
} from "../../../src/js/usd/stage.js";
import {
  PositionComponent,
  VelocityComponent,
  RadiusComponent,
  MassComponent,
  GravityAffectedComponent,
  OrientationComponent,
  AngularVelocityComponent,
  MomentOfInertiaComponent,
  PrevFinalOrientationComponent,
  PrevFinalPosComponent,
  RestitutionComponent,
  CoefficientOfFrictionComponent,
  SimulationErrorStateComponent,
  RenderableComponent,
} from "../../../src/js/cable_joints/ecs.js";
import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent,
  linecolor1,
  CableAttachmentUpdateSystem,
  PBDCableConstraintSolver
} from '../../../src/js/cable_joints/cable_joints_core.js';
import {
  PBDResolveCableOverCorrections
} from '../../../src/js/cable_joints/pbdResolveCableOverCorrections.js';

import { CableAttachmentCacheSystem } from '../../../src/js/cable_joints/cable_attachment_cache_system.js';
import { CableSlackSystem } from '../../../src/js/cable_joints/cable_slack_system.js';
import { CableFrictionSystem } from '../../../src/js/cable_joints/cable_friction_system.js';
import {
  ScoreComponent,
  BallTagComponent,
  ObstacleTagComponent,
  ObstaclePushComponent,
  BorderComponent,
  ScoreSystem,
  ScoreDisplaySystem,
  FlipperTagComponent,
  FlipperStateComponent,
  FlipperTipComponent,
  FlipperMotionSystem,
  FlipperTipLinkSystem,
  OverlayRadiusAndCircleSectorSystem,
  PBDUnifiedContactManifoldSystem,
  PauseStateComponent,
  InputReplaySystem,
  InputSystem
} from './flipper_common.js';

import { BallObstacleBumpSystem } from './ball_obstacle_bump_system.js';
import { BallBorderOrFlipperVelocityContactSystem } from './ball_border_or_flipper_velocity_contact_system.js';
import { LineLayersBumperVelocityContactSystem } from './line_layers_bumper_velocity_contact_system.js';
import { RenderSystem } from './renderSystem.js';
import {
  PrevFinalPosSystem,
  PrevFinalOrientationSystem,
  GravitySystem,
  MovementSystem,
  AngularMovementSystem,
  PBDVelocityUpdateSystem,
  PBDAngularVelocityUpdateSystem,
} from '../../../src/js/cable_joints/commonSystems.js';
import { ensureCableFeatureFlags } from './cable_feature_flags.js';

export function setupScene(world, stage, canvas) {
  world.clear(); // Clear entities and components

    // --- Set Canvas Size from CSS ---
    canvas.width = canvas.clientWidth;
    canvas.height = canvas.clientHeight;

    // --- Resources ---
    const physicsScene = stage.GetPrimAtPath("/World/PhysicsScene");
    const gravityDir = getAttribute(physicsScene, "physics:gravityDirection");
    const gravityMag = getAttribute(physicsScene, "physics:gravityMagnitude");
    const gravity = new Vector2(gravityDir[0] * gravityMag, gravityDir[1] * gravityMag);
    const dt = 1.0 / stage.ast.descriptor.assignments.find(s => s.type === 'assignment' && s.identifier === 'timeCodesPerSecond').value;

    world.setResource('gravity', gravity);
    world.setResource('dt', dt);
    world.setResource('pauseState', new PauseStateComponent(true));
    world.setResource('debugRenderPoints', {});
    world.setResource('errorState', new SimulationErrorStateComponent(false));
    world.setResource('grabbedBall', null);
    world.setResource('ball_obstacle_contacts', []);
    world.setResource('ball_border_contacts', []);
    world.setResource('ball_flipper_contacts', []);
    world.setResource('ball_ball_contacts', []);
    ensureCableFeatureFlags(world);
    const simWidth = 1.0;
    const simHeight = 1.7;
    world.setResource('simWidth', simWidth);
    world.setResource('simHeight', simHeight);
    const cScale = canvas.height / simHeight;
    world.setResource('cScale', cScale);

    const sceneRoot = stage.GetPrimAtPath("/World/FlipperScene");
    const nameToEntityId = {};

    // Border Entity
    const borderPrim = getChild(sceneRoot, "Border");
    if (borderPrim) {
        const points = getAttribute(borderPrim, "points");
        if (!points) {
            console.error("Failed to get 'points' for Border prim. The USD parser may not support 'point3f[]'.");
            world.getResource('errorState').hasError = true;
            return;
        }
        const borderEntity = world.createEntity();
        const borderPoints = points.slice(0, 8).map(p => new Vector2(p[0], p[1]));
        world.addComponent(borderEntity, new BorderComponent(borderPoints));
        const { color, friction, restitution } = materialProperties(stage, borderPrim);
        world.addComponent(borderEntity, new RenderableComponent('border', color || '#000000'));
        if (restitution !== null) world.addComponent(borderEntity, new RestitutionComponent(restitution));
        if (friction !== null) world.addComponent(borderEntity, new CoefficientOfFrictionComponent(friction));
    } else {
        console.error("Could not find Border prim in scene.");
        world.getResource('errorState').hasError = true;
        return;
    }

    // Process Prims
    for (const prim of getChildren(sceneRoot)) {
        const tags = getAttribute(prim, "ecs:tags") || [];
        if (!tags.length) continue;

        const posArr = getAttribute(prim, "xformOp:translate");
        if (!posArr) continue;
        const pos = new Vector2(posArr[0], posArr[1]);
        const { color, friction, restitution } = materialProperties(stage, prim);

        if (tags.includes("Ball")) {
            const ent = world.createEntity();
            const radius = getAttribute(prim, "radius");
            const mass = getAttribute(prim, "physics:mass");
            const inertiaTensor = getAttribute(prim, "physics:inertiaTensor");
            if (radius === null || mass === null || inertiaTensor === null) {
                console.warn(`Skipping Ball prim ${prim.name} due to missing attributes.`);
                continue;
            }
            const inertia = inertiaTensor[2][2];
            world.addComponent(ent, new BallTagComponent());
            world.addComponent(ent, new PositionComponent(pos.x, pos.y));
            world.addComponent(ent, new VelocityComponent(0.0, 0.0));
            world.addComponent(ent, new RadiusComponent(radius));
            world.addComponent(ent, new MassComponent(mass));
            world.addComponent(ent, new GravityAffectedComponent());
            world.addComponent(ent, new RenderableComponent('circle', color || '#a0a0a0'));
            world.addComponent(ent, new OrientationComponent(0.0));
            world.addComponent(ent, new AngularVelocityComponent(0.0));
            world.addComponent(ent, new MomentOfInertiaComponent(inertia));
            world.addComponent(ent, new PrevFinalOrientationComponent(0.0));
            world.addComponent(ent, new PrevFinalPosComponent(pos.x, pos.y));
            if (restitution !== null) world.addComponent(ent, new RestitutionComponent(restitution));
            if (friction !== null) world.addComponent(ent, new CoefficientOfFrictionComponent(friction));
            if (getAttribute(prim, "cable:linkable")) {
                world.addComponent(ent, new CableLinkComponent(pos.x, pos.y));
            }
            nameToEntityId[prim.name] = ent;
        } else if (tags.includes("Obstacle")) {
            const ent = world.createEntity();
            const radius = getAttribute(prim, "radius");
            if (radius === null) {
                console.warn(`Skipping Obstacle prim ${prim.name} due to missing radius.`);
                continue;
            }
            const push = getAttribute(prim, "obstacle:pushVel") || 0.0;
            const angVelAttr = getAttribute(prim, "physics:angularVelocity");
            const angVel = angVelAttr ? angVelAttr[2] * Math.PI / 180.0 : 0.0;
            const inertiaAttr = getAttribute(prim, "physics:inertiaTensor");
            const inertia = inertiaAttr ? inertiaAttr[2][2] : 0.0;

            world.addComponent(ent, new ObstacleTagComponent());
            world.addComponent(ent, new PositionComponent(pos.x, pos.y));
            world.addComponent(ent, new MassComponent(-1.0));
            world.addComponent(ent, new RadiusComponent(radius));
            world.addComponent(ent, new ObstaclePushComponent(push));
            world.addComponent(ent, new RenderableComponent('circle', color || '#ffffff'));
            if (friction !== null) world.addComponent(ent, new CoefficientOfFrictionComponent(friction));
            if (angVel !== 0.0) {
                world.addComponent(ent, new OrientationComponent(0.0));
                world.addComponent(ent, new AngularVelocityComponent(angVel));
                world.addComponent(ent, new MomentOfInertiaComponent(inertia));
                world.addComponent(ent, new PrevFinalOrientationComponent(0.0));
            }
            if (getAttribute(prim, "cable:linkable")) {
                world.addComponent(ent, new CableLinkComponent(pos.x, pos.y));
            }
            nameToEntityId[prim.name] = ent;
        } else if (tags.includes("Flipper")) {
            const ent = world.createEntity();
            const length = getAttribute(prim, "flipper:length");
            const restAngle = getAttribute(prim, "flipper:restAngle");
            const maxRotation = getAttribute(prim, "flipper:maxRotation");
            const angularVelocity = getAttribute(prim, "flipper:angularVelocity");
            if (length === null || restAngle === null || maxRotation === null || angularVelocity === null) {
                console.warn(`Skipping Flipper prim ${prim.name} due to missing attributes.`);
                continue;
            }
            const geom = getChild(prim, "Geom");
            const radius = geom ? getAttribute(geom, "radius") : 0.0;

            world.addComponent(ent, new FlipperTagComponent());
            world.addComponent(ent, new PositionComponent(pos.x, pos.y));
            world.addComponent(ent, new RadiusComponent(radius));
            world.addComponent(ent, new FlipperStateComponent(length, restAngle, maxRotation, angularVelocity));
            world.addComponent(ent, new RenderableComponent('flipper', color || '#FF0000'));
            if (restitution !== null) world.addComponent(ent, new RestitutionComponent(restitution));
            if (getAttribute(prim, "cable:linkable")) {
                world.addComponent(ent, new CableLinkComponent(pos.x, pos.y));
            }
            nameToEntityId[prim.name] = ent;

            const tipPrim = getChild(prim, "Tip");
            if (tipPrim) {
                const tip = world.createEntity();
                world.addComponent(tip, new PositionComponent());
                world.addComponent(tip, new RadiusComponent(radius));
                world.addComponent(tip, new FlipperTipComponent(ent));
                if (getAttribute(tipPrim, "cable:linkable")) {
                    world.addComponent(tip, new CableLinkComponent());
                }
                if (friction !== null) {
                    world.addComponent(tip, new CoefficientOfFrictionComponent(friction));
                }
            }
        }
    }

    // Score Entity
    const scoreEntity = world.createEntity();
    world.addComponent(scoreEntity, new ScoreComponent(0));

    // Cable Setup
    const cablePathPrim = getChild(sceneRoot, "CablePath");
    if (cablePathPrim) {
        const jointPaths = getRelationship(cablePathPrim, "cablePath:joints");
        const jointEntityMap = {};

        for (const jointPath of jointPaths) {
            const jointPrim = stage.GetPrimAtPath(jointPath);
            if (!jointPrim) continue;

            const body0Path = getRelationship(jointPrim, "physics:body0")[0];
            const body1Path = getRelationship(jointPrim, "physics:body1")[0];
            const entityA = nameToEntityId[body0Path.split('/').pop()];
            const entityB = nameToEntityId[body1Path.split('/').pop()];
            const restLength = getAttribute(jointPrim, "restLength");
            const attachAArr = getAttribute(jointPrim, "localPos0");
            const attachBArr = getAttribute(jointPrim, "localPos1");

            if (!entityA || !entityB || restLength === null || !attachAArr || !attachBArr) {
                console.warn(`Skipping CableJoint ${jointPrim.name} due to missing data.`);
                continue;
            }

            const attachALocal = new Vector2(attachAArr[0], attachAArr[1]);
            const attachBLocal = new Vector2(attachBArr[0], attachBArr[1]);

            const joint = world.createEntity();
            const jointComponent = CableJointComponent.fromLocal(
              world,
              entityA,
              entityB,
              restLength,
              attachALocal,
              attachBLocal
            );
            world.addComponent(joint, jointComponent);
            world.addComponent(joint, new RenderableComponent('line', linecolor1));
            jointEntityMap[jointPrim.name] = joint;
        }

        const cablePath = world.createEntity();
        const jointNames = getRelationship(cablePathPrim, "cablePath:joints").map(p => p.split('/').pop());
        const jointEntities = jointNames.map(jointName => jointEntityMap[jointName]).filter(Boolean);
        const linkTypes = getAttribute(cablePathPrim, "cablePath:linkTypes");
        const clockwise = getAttribute(cablePathPrim, "cablePath:clockwise");
        const stored = getAttribute(cablePathPrim, "cablePath:stored");
        const cableHalfWidth = getAttribute(cablePathPrim, "cablePath:halfWidth");
        const clockwiseFlags = Array.isArray(clockwise)
          ? clockwise.map((value) => value === true || value === 1 || value === '1' || value === 'true')
          : null;
        const pathComp = new CablePathComponent(
          world,
          jointEntities,
          linkTypes ? [...linkTypes] : null,
          clockwiseFlags,
          getAttribute(cablePathPrim, "stiffness") || Infinity,
          stored ? [...stored] : null,
          cableHalfWidth ?? 0.0
        );
        world.addComponent(cablePath, pathComp);
    }

    // --- Systems Registration (Order Matters!) ---
    if (world.systems.length === 0) {
        // 1. Cache state from previous step
        world.registerSystem(new PrevFinalPosSystem());
        world.registerSystem(new PrevFinalOrientationSystem());

        // 2. Handle user input and non-physics state changes
        const inputSystemInstance = new InputSystem(canvas, world);
        world.registerSystem(inputSystemInstance);
        world.registerSystem(new InputReplaySystem([], inputSystemInstance));
        world.registerSystem(new FlipperMotionSystem());

        // 3. PREDICTION: Apply forces and integrate velocity to get predicted positions
        world.registerSystem(new GravitySystem());
        world.registerSystem(new MovementSystem());
        world.registerSystem(new AngularMovementSystem());

        // 4. Update derived geometry and cable state
        world.registerSystem(new FlipperTipLinkSystem());
        world.registerSystem(new CableAttachmentUpdateSystem());
        world.registerSystem(new OverlayRadiusAndCircleSectorSystem());
        world.registerSystem(new CableAttachmentCacheSystem());
        world.registerSystem(new CableSlackSystem());

        // 5. POSITIONAL SOLVERS: Correct predicted positions to satisfy constraints.
        world.registerSystem(new PBDCableConstraintSolver());
        world.registerSystem(new PBDResolveCableOverCorrections());
        world.registerSystem(new PBDUnifiedContactManifoldSystem());

        // 6. POST-SOLVE CABLE DYNAMICS: Handle friction-based slip using accurate tension
        world.registerSystem(new CableFrictionSystem());

        // 7. UPDATE VELOCITY: Derive final velocities from the position changes
        world.registerSystem(new PBDVelocityUpdateSystem());
        world.registerSystem(new PBDAngularVelocityUpdateSystem());

        // 8. VELOCITY SOLVERS: Apply restitution and dynamic friction
        world.registerSystem(new BallObstacleBumpSystem());
        world.registerSystem(new LineLayersBumperVelocityContactSystem());
        world.registerSystem(new BallBorderOrFlipperVelocityContactSystem());

        // 9. Game Logic
        world.registerSystem(new ScoreSystem());
        world.registerSystem(new ScoreDisplaySystem('score'));
        const cScale = world.getResource('cScale');
        const simHeight = world.getResource('simHeight');
        world.registerSystem(new RenderSystem(canvas, cScale, simHeight, 1.0, 0.5, 0.85, 0.3));
    }
}
