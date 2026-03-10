import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
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
  EncoderComponent,
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
  MachineTagComponent,
} from "../../../src/js/cable_joints_3d/ecs.js";
import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent,
  linecolor1,
  CableAttachmentUpdateSystem,
  PBDCableConstraintSolver
} from '../../../src/js/cable_joints_3d/cable_joints_core.js';
import {
  PBDResolveCableOverCorrections
} from '../../../src/js/cable_joints_3d/pbdResolveCableOverCorrections.js';

import { CableAttachmentCacheSystem } from '../../../src/js/cable_joints_3d/cable_attachment_cache_system.js';
import { CableSlackSystem } from '../../../src/js/cable_joints_3d/cable_slack_system.js';
import { CableFrictionSystem } from '../../../src/js/cable_joints_3d/cable_friction_system.js';
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
import { RenderSystem3D } from '../../../src/js/cable_joints_3d/render_system_3d.js';
import {
  PrevFinalPosSystem,
  PrevFinalOrientationSystem,
  EncoderUpdateSystem,
  GravitySystem,
  MovementSystem,
  AngularMovementSystem,
  PBDVelocityUpdateSystem,
  PBDAngularVelocityUpdateSystem,
  XPBDDistanceConstraintSystem,
  RigidGroupSystem,
} from '../../../src/js/cable_joints_3d/commonSystems.js';

const DEFAULT_PLANE_NORMAL = new Vector3(0, 0, 1);

export function setupScene(world, stage, canvas, options = {}) {
    const existingRenderSystem = world.getResource('renderSystem');
    if (existingRenderSystem instanceof RenderSystem3D && !options.append) {
        existingRenderSystem.resetVisuals();
    }

    const isRemote = options.remote || false;
    const append = Boolean(options.append);
    const palette = options.palette || null;
    const namespace = typeof options.namespace === 'string' && options.namespace.length > 0 ? options.namespace : null;
    const scenePrimPath = typeof options.scenePrimPath === 'string' && options.scenePrimPath.length > 0
        ? options.scenePrimPath
        : '/World/SlideprinterScene';

    const sceneRootPath = scenePrimPath.endsWith('/') ? scenePrimPath.slice(0, -1) : scenePrimPath;
    const sceneRoot = !isRemote ? stage?.GetPrimAtPath(scenePrimPath) : null;

    if (!isRemote && !sceneRoot) {
        console.warn(`setupScene: Unable to find scene root at ${scenePrimPath}.`);
        return;
    }
    const machineId = namespace || 'default';

    let machineColors = world.getResource('machineColors');
    if (!machineColors || typeof machineColors.set !== 'function') {
        machineColors = new Map();
        world.setResource('machineColors', machineColors);
    }
    if (!append) {
        machineColors.clear();
    }
    machineColors.set(machineId, {
        tintColor: options.tintColor || null,
        extrusionColor: options.extrusionColor || null,
    });

    function normalizeToPlainArray(value) {
        if (Array.isArray(value)) {
            return value;
        }
        if (value && typeof ArrayBuffer !== 'undefined' && ArrayBuffer.isView && ArrayBuffer.isView(value)) {
            return Array.from(value);
        }
        return value;
    }

    function parseRigidGroupRenderSegments(rawValue) {
        if (rawValue === null || rawValue === undefined) {
            return null;
        }
        let parsed = rawValue;
        if (typeof parsed === 'string') {
            const trimmed = parsed.trim();
            if (!trimmed) {
                return null;
            }
            try {
                parsed = JSON.parse(trimmed);
            } catch (err) {
                console.warn('setupScene: failed to parse rigidGroup:renderIndices JSON', err);
                return null;
            }
        }
        parsed = normalizeToPlainArray(parsed);
        if (!Array.isArray(parsed)) {
            return null;
        }

        const segments = [];
        const pushSequence = (sequence) => {
            const flattened = normalizeToPlainArray(sequence);
            if (!Array.isArray(flattened)) {
                return;
            }
            const indices = [];
            for (const value of flattened) {
                const asNumber = Number(value);
                if (Number.isInteger(asNumber)) {
                    indices.push(asNumber);
                }
            }
            if (indices.length >= 2) {
                segments.push(indices);
            }
        };

        const hasNested = parsed.some((entry) => {
            const normalized = normalizeToPlainArray(entry);
            return Array.isArray(normalized);
        });

        if (hasNested) {
            for (const entry of parsed) {
                pushSequence(entry);
            }
        } else {
            pushSequence(parsed);
        }

        return segments.length > 0 ? segments : null;
    }

    function readNumericAttribute(primNode, attributeName) {
        const rawValue = getAttribute(primNode, attributeName);
        if (typeof rawValue === 'number') {
            return Number.isFinite(rawValue) ? rawValue : null;
        }
        if (rawValue === null || rawValue === undefined) {
            return null;
        }
        const parsed = Number(rawValue);
        return Number.isFinite(parsed) ? parsed : null;
    }

    function addGravityIfDynamic(entityId, mass) {
        if (typeof mass === 'number' && Number.isFinite(mass) && mass > 0.0) {
            world.addComponent(entityId, new GravityAffectedComponent());
        }
    }

    function scopedKey(relativeName) {
        const key = relativeName || '';
        return namespace ? `${namespace}::${key}` : key;
    }

    function scopedKeyFromPath(fullPath) {
        if (typeof fullPath !== 'string') {
            return scopedKey(fullPath);
        }
        let relative = fullPath;
        if (sceneRootPath && fullPath.startsWith(sceneRootPath)) {
            relative = fullPath.slice(sceneRootPath.length);
            if (relative.startsWith('/')) {
                relative = relative.slice(1);
            }
        } else if (fullPath.startsWith('/')) {
            relative = fullPath.slice(1);
        }
        const sanitized = relative.replace(/\//g, '::');
        return scopedKey(sanitized || relative);
    }

    if (!isRemote && !append) {
        world.clear();
    }

    canvas.width = canvas.clientWidth;
    canvas.height = canvas.clientHeight;

    const simHeight = 1.7;
    const cScale = canvas.height / simHeight;
    const simWidth = canvas.width / cScale;

    if (!isRemote && !append) {
        // This block remains unchanged, it's for local simulation.
        const physicsScene = stage.GetPrimAtPath("/World/PhysicsScene");
        const gravityDir = getAttribute(physicsScene, "physics:gravityDirection");
        const gravityMag = getAttribute(physicsScene, "physics:gravityMagnitude");
        const gravity = new Vector3(
            gravityDir[0] * gravityMag,
            gravityDir[1] * gravityMag,
            (gravityDir[2] || 0) * gravityMag
        );
        const dt = 1.0 / stage.ast.descriptor.assignments.find(s => s.type === 'assignment' && s.identifier === 'timeCodesPerSecond').value;

        world.setResource('gravity', gravity);
        world.setResource('dt', dt);
        world.setResource('defaultPlaneNormal', DEFAULT_PLANE_NORMAL.clone());
    }

    if (!append) {
        world.setResource('simWidth', simWidth);
        world.setResource('simHeight', simHeight);
        world.setResource('pauseState', new PauseStateComponent(false));
        world.setResource('debugRenderPoints', {});
        world.setResource('errorState', new SimulationErrorStateComponent(false));
        world.setResource('grabbedBall', null);
    }

    if (!isRemote) {
        // This block remains unchanged, it's for local simulation.
        const nameToEntityId = {};
        const jointPrims = [];
        const pathPrims = [];
        const distanceJointPrims = [];
        const rigidGroupPrims = [];
        let extruderPrim = null;
        let extruderAuthoredPos = null;
        let extruderCenterPaths = [];

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
                const pos = new Vector3(posArr[0], posArr[1], posArr[2] || 0);
                const centerPaths = getRelationship(prim, 'machine:centerSources');
                if (tags.includes("Extruder") || centerPaths.length > 0) {
                    extruderPrim = prim;
                    extruderAuthoredPos = pos.clone();
                    extruderCenterPaths = centerPaths;
                    continue;
                }
                const { color, friction, restitution } = materialProperties(stage, prim);
                const primKey = scopedKey(prim.name);

                if (tags.includes("Spool")) {
                    const ent = world.createEntity();
                    world.addComponent(ent, new MachineTagComponent(machineId));
                    const radius = getAttribute(prim, "radius");
                    const mass = getAttribute(prim, "physics:mass");
                    const inertiaTensor = getAttribute(prim, "physics:inertiaTensor");
                    const velArr = getAttribute(prim, "physics:velocity");
                    const angVelArr = getAttribute(prim, "physics:angularVelocity");

                    if (radius === null || mass === null || inertiaTensor === null || !angVelArr) {
                        console.warn(`Skipping Spool prim ${prim.name} due to missing attributes.`);
                        continue;
                    }
                    const inertia = inertiaTensor[2][2];
                    const angVel = angVelArr[2] || 0.0;

                    world.addComponent(ent, new SpoolTagComponent());
                    const axisName = prim.name.slice(-1).toUpperCase();
                    world.addComponent(ent, new SpoolStateComponent(axisName));
                    const holdingTorque = readNumericAttribute(prim, "stepper:holdingTorque");
                    const numPolePairs = readNumericAttribute(prim, "stepper:numPolePairs");
                    const dampingCoeff = readNumericAttribute(prim, "stepper:dampingCoeff");
                    const maxSpeedRad = readNumericAttribute(prim, "stepper:maxSpeedRad");
                    const stepperComponent = new StepperMotorComponent();
                    if (holdingTorque !== null) {
                        stepperComponent.holdingTorque = holdingTorque;
                    }
                    if (numPolePairs !== null) {
                        stepperComponent.numPolePairs = Math.round(numPolePairs);
                    }
                    if (dampingCoeff !== null) {
                        stepperComponent.dampingCoeff = dampingCoeff;
                    }
                    if (maxSpeedRad !== null) {
                        stepperComponent.maxSpeedRad = maxSpeedRad;
                    }
                    world.addComponent(ent, stepperComponent);
                    world.addComponent(ent, new PositionComponent(pos.x, pos.y, pos.z));
                    if (velArr !== null) {
                      const vel = new Vector3(velArr[0], velArr[1], velArr[2] || 0);
                      world.addComponent(ent, new VelocityComponent(vel.x, vel.y, vel.z));
                    } else {
                      world.addComponent(ent, new VelocityComponent(0.0, 0.0, 0.0));
                    }
                    world.addComponent(ent, new RadiusComponent(radius));
                    world.addComponent(ent, new MassComponent(mass));
                    addGravityIfDynamic(ent, mass);
                    const spoolColor = palette?.spool ?? color ?? '#a0a0a0';
                    world.addComponent(ent, new RenderableComponent('circle', spoolColor));
                    world.addComponent(ent, new OrientationComponent(0.0, 0.0, 0.0, 1.0));
                    world.addComponent(ent, new EncoderComponent());
                    world.addComponent(ent, new AngularVelocityComponent(0.0, 0.0, angVel));
                    world.addComponent(ent, new MomentOfInertiaComponent(inertia));
                    world.addComponent(ent, new PrevFinalOrientationComponent(0.0, 0.0, 0.0, 1.0));
                    world.addComponent(ent, new PrevFinalPosComponent(pos.x, pos.y, pos.z));
                    if (restitution !== null) world.addComponent(ent, new RestitutionComponent(restitution));
                    if (friction !== null) world.addComponent(ent, new CoefficientOfFrictionComponent(friction));
                    if (getAttribute(prim, "cable:linkable")) {
                        world.addComponent(ent, new CableLinkComponent(pos.x, pos.y, pos.z, null, DEFAULT_PLANE_NORMAL));
                    }
                    nameToEntityId[primKey] = ent;
                } else if (tags.includes("Anchor")) {
                    const ent = world.createEntity();
                    world.addComponent(ent, new MachineTagComponent(machineId));
                    world.addComponent(ent, new PositionComponent(pos.x, pos.y, pos.z));
                    world.addComponent(ent, new RadiusComponent(0.01));
                    world.addComponent(ent, new MassComponent(-1.0));
                    const anchorColor = palette?.anchor ?? color ?? '#aaaaaa';
                    world.addComponent(ent, new RenderableComponent('circle', anchorColor));
                    if (getAttribute(prim, "cable:linkable")) {
                        world.addComponent(ent, new CableLinkComponent(pos.x, pos.y, pos.z, null, DEFAULT_PLANE_NORMAL));
                    }
                    nameToEntityId[primKey] = ent;
                } else if (tags.includes("Pinhole") || tags.includes("Attachment")) {
                    const ent = world.createEntity();
                    world.addComponent(ent, new SpoolTagComponent());
                    world.addComponent(ent, new MachineTagComponent(machineId));
                    const radius = getAttribute(prim, "radius");
                    const mass = getAttribute(prim, "physics:mass");
                    const inertiaTensor = getAttribute(prim, "physics:inertiaTensor");
                    const velArr = getAttribute(prim, "physics:velocity");
                    const angVelArr = getAttribute(prim, "physics:angularVelocity");

                    world.addComponent(ent, new PositionComponent(pos.x, pos.y, pos.z));
                    if (velArr !== null) {
                        const vel = new Vector3(velArr[0], velArr[1], velArr[2] || 0);
                        world.addComponent(ent, new VelocityComponent(vel.x, vel.y, vel.z));
                    } else {
                        world.addComponent(ent, new VelocityComponent(0.0, 0.0, 0.0));
                    }
                    if (radius !== null) {
                        world.addComponent(ent, new RadiusComponent(radius));
                    }
                    world.addComponent(ent, new MassComponent(mass));
                    addGravityIfDynamic(ent, mass);
                    const pinholeColor = palette?.pinhole ?? color ?? '#cccccc';
                    world.addComponent(ent, new RenderableComponent('circle', pinholeColor));
                    if (angVelArr !== null) {
                        world.addComponent(ent, new OrientationComponent(0.0, 0.0, 0.0, 1.0));
                        world.addComponent(ent, new PrevFinalOrientationComponent(0.0, 0.0, 0.0, 1.0));
                        const angVel = angVelArr[2] || 0.0;
                        world.addComponent(ent, new AngularVelocityComponent(0.0, 0.0, angVel));
                    }
                    if (inertiaTensor !== null) {
                        const inertia = inertiaTensor[2][2];
                        world.addComponent(ent, new MomentOfInertiaComponent(inertia));
                    }
                    world.addComponent(ent, new PrevFinalPosComponent(pos.x, pos.y, pos.z));
                    if (restitution !== null) world.addComponent(ent, new RestitutionComponent(restitution));
                    if (friction !== null) world.addComponent(ent, new CoefficientOfFrictionComponent(friction));
                    if (getAttribute(prim, "cable:linkable")) {
                        world.addComponent(ent, new CableLinkComponent(pos.x, pos.y, pos.z, null, DEFAULT_PLANE_NORMAL));
                    }
                    nameToEntityId[primKey] = ent;
                }
            }
        }

        // Build rigid groups first (if any)
        const entityToRigidGroup = {};
        const extruderCenterEntityIds = (() => {
            const rels = normalizeToPlainArray(extruderCenterPaths);
            if (!Array.isArray(rels) || rels.length === 0) {
                return [];
            }
            return rels
                .map((path) => nameToEntityId[scopedKeyFromPath(path)])
                .filter((id) => id !== undefined);
        })();
        const resolveAveragePosition = (entityIds) => {
            if (!Array.isArray(entityIds) || entityIds.length === 0) {
                return null;
            }
            const sum = new Vector3();
            let count = 0;
            for (const entityId of entityIds) {
                const pos = world.getComponent(entityId, PositionComponent)?.pos;
                if (pos) {
                    sum.add(pos);
                    count += 1;
                }
            }
            if (count === 0) {
                return null;
            }
            return sum.scale(1.0 / count);
        };
        const extruderCenterOffset = (() => {
            if (!extruderPrim || !extruderAuthoredPos || extruderCenterEntityIds.length === 0) {
                return null;
            }
            const averagePos = resolveAveragePosition(extruderCenterEntityIds);
            if (!averagePos) {
                return null;
            }
            return extruderAuthoredPos.clone().subtract(averagePos);
        })();
        for (const prim of rigidGroupPrims) {
            const memberPaths = getRelationship(prim, 'rigidGroup:members');
            if (!memberPaths || memberPaths.length === 0) continue;
            const memberEntities = memberPaths
                .map(path => nameToEntityId[scopedKeyFromPath(path)])
                .filter(id => id !== undefined);
            if (memberEntities.length >= 2) {
                const groupEnt = world.createEntity();
                const renderIndicesAttr = getAttribute(prim, 'rigidGroup:renderIndices');
                const renderSegments = parseRigidGroupRenderSegments(renderIndicesAttr);
                const groupComponent = new RigidGroupComponent(memberEntities, 1.0, renderSegments);
                world.addComponent(groupEnt, groupComponent);
                world.addComponent(groupEnt, new MachineTagComponent(machineId));
                world.addComponent(groupEnt, new RenderableComponent('line', palette?.rigidGroup ?? palette?.distanceConstraint ?? '#55ff88'));
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

            const body0Key = scopedKeyFromPath(body0PathRel[0]);
            const body1Key = scopedKeyFromPath(body1PathRel[0]);

            if (nameToEntityId[body0Key] == null || nameToEntityId[body1Key] == null) {
                continue;
            }

            const entityA = nameToEntityId[body0Key];
            const entityB = nameToEntityId[body1Key];

            const minDistance = getAttribute(prim, "physics:minDistance");
            const maxDistance = getAttribute(prim, "physics:maxDistance");

            if (minDistance !== null && maxDistance !== null) {
                if (Math.abs(minDistance - maxDistance) < 1e-6) {
                    // Skip if both bodies belong to the same rigid group
                    if (!entityToRigidGroup[entityA] || entityToRigidGroup[entityA] !== entityToRigidGroup[entityB]) {
                        const restLength = (minDistance + maxDistance) / 2.0;
                        const constraintEntity = world.createEntity();
                        world.addComponent(constraintEntity, new MachineTagComponent(machineId));
                        world.addComponent(constraintEntity, new DistanceConstraintComponent(entityA, entityB, restLength, 0.0));
                        const distanceColor = palette?.distanceConstraint ?? 'green';
                        world.addComponent(constraintEntity, new RenderableComponent('line', distanceColor));
                    }
                }
            }
        }

        // Process discovered CableJoints
        const jointEntityMap = {};
        for (const prim of jointPrims) {
            const body0Path = getRelationship(prim, "physics:body0")[0];
            const body1Path = getRelationship(prim, "physics:body1")[0];
            const entityA = nameToEntityId[scopedKeyFromPath(body0Path)];
            const entityB = nameToEntityId[scopedKeyFromPath(body1Path)];
            const restLength = getAttribute(prim, "restLength");
            const attachAArr = getAttribute(prim, "localPos0");
            const attachBArr = getAttribute(prim, "localPos1");

            if (entityA === null || entityB === null || restLength === null || attachAArr === null || attachBArr === null) {
                console.warn(`Skipping CableJoint ${prim.name} due to missing data.`);
                console.log(`entityA: ${entityA}, entityB: ${entityB}, restLength: ${restLength}, attachAArr: ${attachAArr}, attachBarr: ${attachBArr}`);
                continue;
            }

            const attachALocal = new Vector3(attachAArr[0], attachAArr[1], attachAArr[2] || 0);
            const attachBLocal = new Vector3(attachBArr[0], attachBArr[1], attachBArr[2] || 0);

            const joint = world.createEntity();
            world.addComponent(joint, new MachineTagComponent(machineId));
            const jointComponent = CableJointComponent.fromLocal(
              world,
              entityA,
              entityB,
              restLength,
              attachALocal,
              attachBLocal
            );
            world.addComponent(joint, jointComponent);
            const cableColor = palette?.cable ?? linecolor1;
            world.addComponent(joint, new RenderableComponent('line', cableColor));
            jointEntityMap[scopedKey(prim.name)] = joint;
        }

        // Process discovered CablePaths
        for (const prim of pathPrims) {
            const cablePath = world.createEntity();
            const jointPaths = getRelationship(prim, "cablePath:joints");
            if (!jointPaths) continue;

            const jointNames = jointPaths.map((p) => scopedKeyFromPath(p));
            const jointEntities = jointNames.map(jointName => jointEntityMap[jointName]).filter(Boolean);
            const linkTypes = getAttribute(prim, "cablePath:linkTypes");
            const clockwise = getAttribute(prim, "cablePath:clockwise");
            const stored = getAttribute(prim, "cablePath:stored");
            const stiffness = getAttribute(prim, "stiffness");
            const cableHalfWidth = getAttribute(prim, "cablePath:halfWidth");

            const pathComp = new CablePathComponent(
              world,
              jointEntities,
              linkTypes ? [...linkTypes] : null,
              clockwise ? [...clockwise] : null,
              stiffness || Infinity,
              stored ? [...stored] : null,
              cableHalfWidth ?? 0.0
            );
            world.addComponent(cablePath, pathComp);
            world.addComponent(cablePath, new MachineTagComponent(machineId));
        }

        const existingExtruder = world.query([ExtruderComponent]);
        if (existingExtruder.length === 0) {
            const extruderEntity = world.createEntity();
            world.addComponent(extruderEntity, new ExtruderComponent());
        }
        {
            let extruderComp = null;
            for (const extruderEntity of world.query([ExtruderComponent])) {
                extruderComp = world.getComponent(extruderEntity, ExtruderComponent);
                if (extruderComp) {
                    break;
                }
            }
            if (extruderComp) {
                if (!extruderComp.centerSources || typeof extruderComp.centerSources !== 'object') {
                    extruderComp.centerSources = {};
                }
                if (!extruderComp.centerOffsets || typeof extruderComp.centerOffsets !== 'object') {
                    extruderComp.centerOffsets = {};
                }
                if (extruderCenterEntityIds.length > 0) {
                    extruderComp.centerSources[machineId] = extruderCenterEntityIds.slice();
                } else if (extruderComp.centerSources[machineId]) {
                    delete extruderComp.centerSources[machineId];
                }
                if (extruderCenterOffset) {
                    extruderComp.centerOffsets[machineId] = extruderCenterOffset.clone();
                } else if (extruderComp.centerOffsets[machineId]) {
                    delete extruderComp.centerOffsets[machineId];
                }
            }
        }

        const remoteSpoolSystem = world.systems.find((system) => system instanceof RemoteSpoolSystem);
        if (remoteSpoolSystem && typeof remoteSpoolSystem.resetAxisMapping === 'function') {
            remoteSpoolSystem.resetAxisMapping();
        }
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

          world.registerSystem(new RigidGroupSystem());

          // 4. Update derived geometry and cable state
          world.registerSystem(new CableAttachmentUpdateSystem());
          world.registerSystem(new CableAttachmentCacheSystem());
          world.registerSystem(new CableSlackSystem());

          // 5. POSITIONAL SOLVERS: Correct predicted positions to satisfy constraints.
          world.registerSystem(new PBDCableConstraintSolver());
          world.registerSystem(new PBDResolveCableOverCorrections());

          // Enforce rigid motion for grouped spools (if any)
          world.registerSystem(new XPBDDistanceConstraintSystem());

          // 6. POST-SOLVE CABLE DYNAMICS: Handle friction-based slip using accurate tension
          world.registerSystem(new CableFrictionSystem());

          // 7. UPDATE VELOCITY: Derive final velocities from the position changes
          world.registerSystem(new PBDVelocityUpdateSystem());
          world.registerSystem(new PBDAngularVelocityUpdateSystem());

          // 8. VELOCITY SOLVERS: Apply restitution and dynamic friction
          // Velocity-level solvers (which might also do positional adjustments)

          // 9. Game Logic or similar. Counters and stuff
          world.registerSystem(new ExtruderSystem());
          world.registerSystem(new EncoderUpdateSystem());
      }

      const renderSystem = new RenderSystem3D(canvas, {
          planeNormal: DEFAULT_PLANE_NORMAL,
          targetX: 0.0,
          targetY: 0.0,
          cameraZ: 2.2 / Math.max(0.2, inputSys.scaleMultiplier),
          initialOrbitAzimuth: -Math.PI * 0.25,
          initialOrbitPolar: 1.05,
          controlsEnabled: false,
          renderOnSimulationStep: false
      });
      renderSystem.setViewTransform?.({
          scaleMultiplier: inputSys.scaleMultiplier,
          offsetX: inputSys.viewOffsetX,
          offsetY: inputSys.viewOffsetY,
      });
      world.setResource('renderSystem', renderSystem);
    } else if (existingRenderSystem instanceof RenderSystem3D) {
      existingRenderSystem.setCanvasSize(canvas.clientWidth, canvas.clientHeight);
    }
}
