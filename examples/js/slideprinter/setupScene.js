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
} from "../../../src/js/cable_joints/ecs.js";
import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent,
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
  SpoolTagComponent
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
} from '../../../src/js/cable_joints/commonSystems.js';
import { tangentFromPointToCircle } from '../../../src/js/cable_joints/geometry.js';

export function setupScene(world, stage, canvas) {
  world.clear();

    canvas.width = canvas.clientWidth;
    canvas.height = canvas.clientHeight;

    const simHeight = 1.7;
    const cScale = canvas.height / simHeight;
    const simWidth = canvas.width / cScale;

    const physicsScene = stage.GetPrimAtPath("/World/PhysicsScene");
    const gravityDir = getAttribute(physicsScene, "physics:gravityDirection");
    const gravityMag = getAttribute(physicsScene, "physics:gravityMagnitude");
    const gravity = new Vector2(gravityDir[0] * gravityMag, gravityDir[1] * gravityMag);
    const dt = 1.0 / stage.ast.descriptor.assignments.find(s => s.type === 'assignment' && s.identifier === 'timeCodesPerSecond').value;

    world.setResource('gravity', gravity);
    world.setResource('dt', dt);
    world.setResource('simWidth', simWidth);
    world.setResource('simHeight', simHeight);
    world.setResource('pauseState', new PauseStateComponent(true));
    world.setResource('debugRenderPoints', {});
    world.setResource('errorState', new SimulationErrorStateComponent(false));
    world.setResource('grabbedBall', null);

    const sceneRoot = stage.GetPrimAtPath("/World/SlideprinterScene");
    const spools = {};
    const anchors = {};

    for (const prim of getChildren(sceneRoot)) {
        const tags = getAttribute(prim, "ecs:tags") || [];
        if (!tags.length) continue;

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
            spools[prim.name] = ent;
        } else if (tags.includes("Anchor")) {
            const ent = world.createEntity();
            const radius = 0.01;
            world.addComponent(ent, new PositionComponent(pos.x, pos.y));
            world.addComponent(ent, new RadiusComponent(0.01));
            world.addComponent(ent, new MassComponent(-1.0));
            world.addComponent(ent, new RenderableComponent('circle', color || '#aaaaaa'));
            if (getAttribute(prim, "cable:linkable")) {
                world.addComponent(ent, new CableLinkComponent(pos.x, pos.y));
            }
            anchors[prim.name] = ent;
        }
    }

    const spoolNames = Object.keys(spools).sort();
    const anchorNames = Object.keys(anchors).sort();

    if (spoolNames.length === anchorNames.length) {
        const turns = 5.0;
        const cableStiffness = 20000.0;

        for (let i = 0; i < spoolNames.length; i++) {
            const spoolEntity = spools[spoolNames[i]];
            const anchorEntity = anchors[anchorNames[i]];

            const spoolRadius = world.getComponent(spoolEntity, RadiusComponent).radius;
            const initialStoredLength = turns * spoolRadius * Math.PI * 2.0;

            const anchorPos = world.getComponent(anchorEntity, PositionComponent).pos;
            const spoolPos = world.getComponent(spoolEntity, PositionComponent).pos;

            const joint = world.createEntity();
            const tang = tangentFromPointToCircle(anchorPos, spoolPos, spoolRadius, true);
            const local_attach_A = tang.a_attach.clone().subtract(anchorPos);
            const local_attach_B = tang.a_circle.clone().subtract(spoolPos);
            const initialDist = tang.a_attach.distanceTo(tang.a_circle);

            world.addComponent(joint, new CableJointComponent(
                anchorEntity, spoolEntity, initialDist, local_attach_A, local_attach_B
            ));
            world.addComponent(joint, new RenderableComponent('line', 'yellow'));

            const cable = world.createEntity();
            const cableComp = new CablePathComponent(
                world, [joint], ['attachment', 'hybrid'], [true, true],
                cableStiffness, [0.0, initialStoredLength]
            );
            world.addComponent(cable, cableComp);
        }
    }

    const spoolEntities = spoolNames.map(name => spools[name]);
    if (spoolEntities.length === 3) {
        const createDistanceConstraintEntity = (entityA, entityB, compliance = 0.0) => {
            const constraintEntity = world.createEntity();
            const restLength = world.getComponent(entityA, PositionComponent).pos.distanceTo(world.getComponent(entityB, PositionComponent).pos);
            world.addComponent(constraintEntity, new DistanceConstraintComponent(entityA, entityB, restLength, compliance));
            world.addComponent(constraintEntity, new RenderableComponent('line', 'green'));
        };
        createDistanceConstraintEntity(spoolEntities[0], spoolEntities[1]);
        createDistanceConstraintEntity(spoolEntities[1], spoolEntities[2]);
        createDistanceConstraintEntity(spoolEntities[2], spoolEntities[0]);
    }

    if (world.systems.length === 0) {
      world.registerSystem(new PrevFinalPosSystem());
      world.registerSystem(new PrevFinalOrientationSystem());

      const pauseBtn = document.getElementById("pauseBtn");
      const inputSys = new InputSystem(canvas, world, pauseBtn);
      inputSys.scaleMultiplier = 0.5;
      inputSys.viewOffsetX = 0.0;
      inputSys.viewOffsetY = -0.5;
      world.registerSystem(inputSys);

      world.registerSystem(new GravitySystem());
      world.registerSystem(new MovementSystem());
      world.registerSystem(new AngularMovementSystem());

      world.registerSystem(new CableAttachmentUpdateSystem());
      world.registerSystem(new CableAttachmentCacheSystem());
      world.registerSystem(new CableSlackSystem());

      world.registerSystem(new PBDCableConstraintSolver());
      world.registerSystem(new PBDResolveCableOverCorrections());
      world.registerSystem(new XPBDDistanceConstraintSystem());

      world.registerSystem(new CableFrictionSystem());

      world.registerSystem(new PBDVelocityUpdateSystem());
      world.registerSystem(new PBDAngularVelocityUpdateSystem());

      world.registerSystem(new RenderSystem(canvas, cScale, simHeight, inputSys.scaleMultiplier, inputSys.viewOffsetX, inputSys.viewOffsetY, 0.2));
    }
}
