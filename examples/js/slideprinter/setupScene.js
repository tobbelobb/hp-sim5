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
  OrientationComponent,
  AngularVelocityComponent,
  MomentOfInertiaComponent,
  PrevFinalOrientationComponent,
  PrevFinalPosComponent,
  RestitutionComponent,
  CoefficientOfFrictionComponent,
  SimulationErrorStateComponent,
  RenderableComponent,
  DistanceConstraintComponent,
  RigidGroupComponent,
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
  PauseStateComponent,
} from '../flipper/flipper_common.js';
import {
  InputSystem,
  SpoolTagComponent,
  RemoteInputSystem,
  RemoteSpoolSystem,
  SpoolStateComponent,
  StepperMotorComponent,
  StepperMotorSystem,
  ExtruderComponent,
  ExtruderSystem
} from './slideprinter_common.js';
import { RenderSystem } from '../flipper/renderSystem.js';
import {
  PrevFinalPosSystem,
  PrevFinalOrientationSystem,
  GravitySystem,
  MovementSystem,
  AngularMovementSystem,
  PBDVelocityUpdateSystem,
  PBDAngularVelocityUpdateSystem,
  XPBDDistanceConstraintSystem,
  RigidGroupSystem,
} from '../../../src/js/cable_joints/commonSystems.js';

export function setupScene(world, stage, canvas, options = {}) {
    const isRemote = options.remote || false;

    if (!isRemote) {
        world.clear();
    }

    canvas.width = canvas.clientWidth;
    canvas.height = canvas.clientHeight;

    const simHeight = 1.7;
    const cScale = canvas.height / simHeight;
    const simWidth = canvas.width / cScale;

    if (!isRemote) {
        // This block remains unchanged, it's for local simulation.
        const physicsScene = stage.GetPrimAtPath("/World/PhysicsScene");
        const gravityDir = getAttribute(physicsScene, "physics:gravityDirection");
        const gravityMag = getAttribute(physicsScene, "physics:gravityMagnitude");
        const gravity = new Vector2(gravityDir[0] * gravityMag, gravityDir[1] * gravityMag);
        const dt = 1.0 / stage.ast.descriptor.assignments.find(s => s.type === 'assignment' && s.identifier === 'timeCodesPerSecond').value;

        world.setResource('gravity', gravity);
        world.setResource('dt', dt);
    }

    world.setResource('simWidth', simWidth);
    world.setResource('simHeight', simHeight);
    world.setResource('pauseState', new PauseStateComponent(false));
    world.setResource('debugRenderPoints', {});
    world.setResource('errorState', new SimulationErrorStateComponent(false));
    world.setResource('grabbedBall', null);

    if (!isRemote) {
        // This block remains unchanged, it's for local simulation.
        const sceneRoot = stage.GetPrimAtPath("/World/SlideprinterScene");
        const nameToEntityId = {};
        const jointPrims = [];
        const pathPrims = [];
        const distanceJointPrims = [];
        const rigidGroupPrims = [];

        // Discover prims and create body entities in a single pass
        for (const prim of getChildren(sceneRoot)) {
            // Check for API schemas to identify paths
            const apiSchemas = getAttribute(prim, "apiSchemas");
            if (apiSchemas && apiSchemas.includes("CablePathAPI")) {
                pathPrims.push(prim);
            }

            if (prim.type === 'definition' && prim.defType === 'CableJoint') {
                jointPrims.push(prim);
            }

            if (prim.type === 'definition' && prim.defType === 'DistancePhysicsJoint') {
                distanceJointPrims.push(prim);
            }

            // Detect rigid groups either by explicit defType or a members rel
            if (prim.type === 'definition' && (prim.defType === 'RigidGroup')) {
                rigidGroupPrims.push(prim);
            } else {
                const maybeMembers = getRelationship(prim, 'rigidGroup:members');
                if (maybeMembers && maybeMembers.length > 0) {
                    rigidGroupPrims.push(prim);
                }
            }

            // Check for tags to identify bodies (Spools, Anchors)
            const tags = getAttribute(prim, "ecs:tags") || [];
            if (tags.length > 0) {
                const posArr = getAttribute(prim, "xformOp:translate");
                if (!posArr) continue;
                const pos = new Vector2(posArr[0], posArr[1]);
                const { color, friction, restitution } = materialProperties(stage, prim);

                if (tags.includes("Spool")) {
                    const ent = world.createEntity();
                    const radius = getAttribute(prim, "radius");
                    const mass = getAttribute(prim, "physics:mass");
                    const inertiaTensor = getAttribute(prim, "physics:inertiaTensor");
                    const velArr = getAttribute(prim, "physics:velocity");
                    const angVelArr = getAttribute(prim, "physics:angularVelocity");

                    if (radius === null || mass === null || inertiaTensor === null || !velArr || !angVelArr) {
                        console.warn(`Skipping Spool prim ${prim.name} due to missing attributes.`);
                        continue;
                    }
                    const inertia = inertiaTensor[2][2];
                    const vel = new Vector2(velArr[0], velArr[1]);
                    const angVel = angVelArr[2];

                    world.addComponent(ent, new SpoolTagComponent());
                    const axisName = prim.name.slice(-1).toUpperCase();
                    world.addComponent(ent, new SpoolStateComponent(axisName));
                    world.addComponent(ent, new StepperMotorComponent());
                    world.addComponent(ent, new PositionComponent(pos.x, pos.y));
                    world.addComponent(ent, new VelocityComponent(vel.x, vel.y));
                    world.addComponent(ent, new RadiusComponent(radius));
                    world.addComponent(ent, new MassComponent(mass));
                    world.addComponent(ent, new RenderableComponent('circle', color || '#a0a0a0'));
                    world.addComponent(ent, new OrientationComponent(0.0));
                    world.addComponent(ent, new AngularVelocityComponent(angVel));
                    world.addComponent(ent, new MomentOfInertiaComponent(inertia));
                    world.addComponent(ent, new PrevFinalOrientationComponent(0.0));
                    world.addComponent(ent, new PrevFinalPosComponent(pos.x, pos.y));
                    if (restitution !== null) world.addComponent(ent, new RestitutionComponent(restitution));
                    if (friction !== null) world.addComponent(ent, new CoefficientOfFrictionComponent(friction));
                    if (getAttribute(prim, "cable:linkable")) {
                        world.addComponent(ent, new CableLinkComponent(pos.x, pos.y));
                    }
                    nameToEntityId[prim.name] = ent;
                } else if (tags.includes("Anchor")) {
                    const ent = world.createEntity();
                    world.addComponent(ent, new PositionComponent(pos.x, pos.y));
                    world.addComponent(ent, new RadiusComponent(0.01));
                    world.addComponent(ent, new MassComponent(-1.0));
                    world.addComponent(ent, new RenderableComponent('circle', color || '#aaaaaa'));
                    if (getAttribute(prim, "cable:linkable")) {
                        world.addComponent(ent, new CableLinkComponent(pos.x, pos.y));
                    }
                    nameToEntityId[prim.name] = ent;
                } else if (tags.includes("Pinhole")) {
                    const ent = world.createEntity();
                    const radius = getAttribute(prim, "radius");
                    const mass = getAttribute(prim, "physics:mass");
                    const inertiaTensor = getAttribute(prim, "physics:inertiaTensor");
                    const velArr = getAttribute(prim, "physics:velocity");
                    const angVelArr = getAttribute(prim, "physics:angularVelocity");

                    world.addComponent(ent, new PositionComponent(pos.x, pos.y));
                    if (velArr !== null) {
                        const vel = new Vector2(velArr[0], velArr[1]);
                        world.addComponent(ent, new VelocityComponent(vel.x, vel.y));
                    }
                    if (radius !== null) {
                        world.addComponent(ent, new RadiusComponent(radius));
                    }
                    world.addComponent(ent, new MassComponent(mass));
                    world.addComponent(ent, new RenderableComponent('circle', color || '#cccccc'));
                    if (angVelArr !== null) {
                        world.addComponent(ent, new OrientationComponent(0.0));
                        world.addComponent(ent, new PrevFinalOrientationComponent(0.0));
                        const angVel = angVelArr[2];
                        world.addComponent(ent, new AngularVelocityComponent(angVel));
                    }
                    if (inertiaTensor !== null) {
                        const inertia = inertiaTensor[2][2];
                        world.addComponent(ent, new MomentOfInertiaComponent(inertia));
                    }
                    world.addComponent(ent, new PrevFinalPosComponent(pos.x, pos.y));
                    if (restitution !== null) world.addComponent(ent, new RestitutionComponent(restitution));
                    if (friction !== null) world.addComponent(ent, new CoefficientOfFrictionComponent(friction));
                    if (getAttribute(prim, "cable:linkable")) {
                        world.addComponent(ent, new CableLinkComponent(pos.x, pos.y));
                    }
                    nameToEntityId[prim.name] = ent;
                }
            }
        }

        // Build rigid groups first (if any)
        const entityToRigidGroup = {};
        for (const prim of rigidGroupPrims) {
            const memberPaths = getRelationship(prim, 'rigidGroup:members');
            if (!memberPaths || memberPaths.length === 0) continue;
            const memberNames = memberPaths.map(p => p.split('/').pop());
            const memberEntities = memberNames
                .map(name => nameToEntityId[name])
                .filter(id => id !== undefined);
            if (memberEntities.length >= 2) {
                const groupEnt = world.createEntity();
                world.addComponent(groupEnt, new RigidGroupComponent(memberEntities, 1.0));
                for (const e of memberEntities) {
                    entityToRigidGroup[e] = groupEnt;
                }
            }
        }

        // Process discovered DistancePhysicsJoints (skip those internal to a rigid group)
        for (const prim of distanceJointPrims) {
            const body0PathRel = getRelationship(prim, "physics:body0");
            const body1PathRel = getRelationship(prim, "physics:body1");

            if (body0PathRel == null || body1PathRel == null || body0PathRel.length === 0 || body1PathRel.length === 0) {
                continue;
            }

            const body0Name = body0PathRel[0].split('/').pop();
            const body1Name = body1PathRel[0].split('/').pop();

            if (nameToEntityId[body0Name] == null || nameToEntityId[body1Name] == null) {
                continue;
            }

            const entityA = nameToEntityId[body0Name];
            const entityB = nameToEntityId[body1Name];

            const minDistance = getAttribute(prim, "physics:minDistance");
            const maxDistance = getAttribute(prim, "physics:maxDistance");

            if (minDistance !== null && maxDistance !== null) {
                if (Math.abs(minDistance - maxDistance) < 1e-6) {
                    // Skip if both bodies belong to the same rigid group
                    if (!entityToRigidGroup[entityA] || entityToRigidGroup[entityA] !== entityToRigidGroup[entityB]) {
                        const restLength = (minDistance + maxDistance) / 2.0;
                        const constraintEntity = world.createEntity();
                        world.addComponent(constraintEntity, new DistanceConstraintComponent(entityA, entityB, restLength, 0.0));
                        world.addComponent(constraintEntity, new RenderableComponent('line', 'green'));
                    }
                }
            }
        }

        // Process discovered CableJoints
        const jointEntityMap = {};
        for (const prim of jointPrims) {
            const body0Path = getRelationship(prim, "physics:body0")[0];
            const body1Path = getRelationship(prim, "physics:body1")[0];
            const entityA = nameToEntityId[body0Path.split('/').pop()];
            const entityB = nameToEntityId[body1Path.split('/').pop()];
            const restLength = getAttribute(prim, "restLength");
            const attachAArr = getAttribute(prim, "localPos0");
            const attachBArr = getAttribute(prim, "localPos1");

            if (entityA === null || entityB === null || restLength === null || attachAArr === null || attachBArr === null) {
                console.warn(`Skipping CableJoint ${prim.name} due to missing data.`);
                console.log(`entityA: ${entityA}, entityB: ${entityB}, restLength: ${restLength}, attachAArr: ${attachAArr}, attachBarr: ${attachBArr}`);
                continue;
            }

            const attachA = new Vector2(attachAArr[0], attachAArr[1]);
            const attachB = new Vector2(attachBArr[0], attachBArr[1]);

            const joint = world.createEntity();
            world.addComponent(joint, new CableJointComponent(entityA, entityB, restLength, attachA, attachB));
            world.addComponent(joint, new RenderableComponent('line', linecolor1));
            jointEntityMap[prim.name] = joint;
        }

        // Process discovered CablePaths
        for (const prim of pathPrims) {
            const cablePath = world.createEntity();
            const jointPaths = getRelationship(prim, "cablePath:joints");
            if (!jointPaths) continue;

            const jointNames = jointPaths.map(p => p.split('/').pop());
            const jointEntities = jointNames.map(jointName => jointEntityMap[jointName]).filter(Boolean);
            const linkTypes = getAttribute(prim, "cablePath:linkTypes");
            const clockwise = getAttribute(prim, "cablePath:clockwise");
            const stored = getAttribute(prim, "cablePath:stored");
            const stiffness = getAttribute(prim, "stiffness");

            const pathComp = new CablePathComponent(
              world,
              jointEntities,
              linkTypes ? [...linkTypes] : null,
              clockwise ? [...clockwise] : null,
              stiffness || Infinity,
              stored ? [...stored] : null
            );
            world.addComponent(cablePath, pathComp);
        }

        const extruderEntity = world.createEntity();
        world.addComponent(extruderEntity, new ExtruderComponent());
    }

    if (world.systems.length === 0) {
      const pauseBtn = document.getElementById("pauseBtn");
      let inputSys;
      if (isRemote) {
          const ws = options.ws;
          inputSys = new RemoteInputSystem(canvas, world, ws);
      } else {
          inputSys = new InputSystem(canvas, world, pauseBtn);
      }
      inputSys.scaleMultiplier = 1.1;
      inputSys.viewOffsetX = 0.0;
      inputSys.viewOffsetY = -0.0;
      world.registerSystem(inputSys);

      if (!isRemote) {
          // 1. Cache state from previous step
          world.registerSystem(new PrevFinalPosSystem());
          world.registerSystem(new PrevFinalOrientationSystem());

          // 2. Handle non-physics state changes
          world.registerSystem(new RemoteSpoolSystem());
          world.registerSystem(new StepperMotorSystem());

          // 3. PREDICTION: Apply forces and integrate velocity to get predicted positions
          world.registerSystem(new GravitySystem());
          world.registerSystem(new MovementSystem());
          world.registerSystem(new AngularMovementSystem());

          // 4. Update derived geometry and cable state
          world.registerSystem(new CableAttachmentUpdateSystem());
          world.registerSystem(new CableAttachmentCacheSystem());
          world.registerSystem(new CableSlackSystem());

          // 5. POSITIONAL SOLVERS: Correct predicted positions to satisfy constraints.
          world.registerSystem(new PBDCableConstraintSolver());
          world.registerSystem(new PBDResolveCableOverCorrections());
          // Enforce rigid motion for grouped spools (if any)
          world.registerSystem(new RigidGroupSystem());
          //world.registerSystem(new XPBDDistanceConstraintSystem());

          // 6. POST-SOLVE CABLE DYNAMICS: Handle friction-based slip using accurate tension
          world.registerSystem(new CableFrictionSystem());

          // 7. UPDATE VELOCITY: Derive final velocities from the position changes
          world.registerSystem(new PBDVelocityUpdateSystem());
          world.registerSystem(new PBDAngularVelocityUpdateSystem());

          // 8. VELOCITY SOLVERS: Apply restitution and dynamic friction
          // Velocity-level solvers (which might also do positional adjustments)

          // 9. Game Logic or similar. Counters and stuff
          world.registerSystem(new ExtruderSystem());
      }

      const renderSystem = new RenderSystem(
          canvas,
          cScale,
          simHeight,
          inputSys.scaleMultiplier,
          inputSys.viewOffsetX,
          inputSys.viewOffsetY,
          0.2
      );
      world.setResource('renderSystem', renderSystem);
    }
}
