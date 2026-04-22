import Vector3 from '../../src/js/cable_joints_3d/vector3.js';
import {
  getAttribute,
  getChild,
  getChildren,
  getRelationship,
  materialProperties
} from "../../src/js/usd/stage.js";
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
  RigidBodyComponent,
  RigidBodyMemberComponent,
  MachineTagComponent,
} from "../../src/js/cable_joints_3d/ecs.js";
import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent,
  linecolor1,
  CableAttachmentUpdateSystem,
  PBDCableConstraintSolver
} from '../../src/js/cable_joints_3d/cable_joints_core.js';
import {
  PBDResolveCableOverCorrections
} from '../../src/js/cable_joints_3d/pbdResolveCableOverCorrections.js';

import { CableAttachmentCacheSystem } from '../../src/js/cable_joints_3d/cable_attachment_cache_system.js';
import { CableSlackSystem } from '../../src/js/cable_joints_3d/cable_slack_system.js';
import { CableFrictionSystem } from '../../src/js/cable_joints_3d/cable_friction_system.js';
import {
  PauseStateComponent,
} from '../../example_apps/js/flipper/flipper_common.js';
import {
  InputSystem,
  RemoteInputSystem,
} from './hangprinter_input.js';
import { ExtruderComponent, ExtruderSystem } from './hangprinter_extruder.js';
import { RemoteSpoolSystem } from './remoteSpoolSystem.js';
import {
  SpoolTagComponent,
  SpoolStateComponent,
  normalizeSpoolAxisLocal,
} from './hangprinter_spools.js';
import { StepperMotorComponent, StepperMotorSystem } from './hangprinter_stepper_motor.js';
import { RenderSystem3D } from '../../src/js/cable_joints_3d/render_system_3d.js';
import {
  PrevFinalPosSystem,
  PrevFinalOrientationSystem,
  EncoderUpdateSystem,
  GravitySystem,
  MovementSystem,
  AngularMovementSystem,
  PBDVelocityUpdateSystem,
  PBDAngularVelocityUpdateSystem,
  RigidBodySyncSystem,
  XPBDDistanceConstraintSystem,
} from '../../src/js/cable_joints_3d/commonSystems.js';
import Quaternion from '../../src/js/cable_joints_3d/quaternion.js';
import { initializeRigidBodySyncState } from '../../src/js/cable_joints_3d/rigid_bodies.js';

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

    function readVectorAttribute(primNode, attributeName) {
        const rawValue = getAttribute(primNode, attributeName);
        if (!rawValue || !Array.isArray(rawValue) || rawValue.length < 2) {
            return null;
        }
        const x = Number(rawValue[0]);
        const y = Number(rawValue[1]);
        const z = Number(rawValue[2] ?? 0.0);
        if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
            return null;
        }
        return new Vector3(x, y, z);
    }

    function readQuaternionAttribute(primNode, attributeName) {
        const rawValue = getAttribute(primNode, attributeName);
        if (!rawValue || !Array.isArray(rawValue) || rawValue.length < 4) {
            return null;
        }
        const w = Number(rawValue[0]);
        const x = Number(rawValue[1]);
        const y = Number(rawValue[2]);
        const z = Number(rawValue[3]);
        if (!Number.isFinite(w) || !Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
            return null;
        }
        return new Quaternion(x, y, z, w).normalize();
    }

    function readRotateYXZAttribute(primNode, attributeName) {
        const rawValue = getAttribute(primNode, attributeName);
        if (!rawValue || !Array.isArray(rawValue) || rawValue.length < 3) {
            return null;
        }
        const xDeg = Number(rawValue[0]);
        const yDeg = Number(rawValue[1]);
        const zDeg = Number(rawValue[2]);
        if (!Number.isFinite(xDeg) || !Number.isFinite(yDeg) || !Number.isFinite(zDeg)) {
            return null;
        }
        const degToRad = Math.PI / 180.0;
        const qx = new Quaternion().setFromAxisAngle(new Vector3(1.0, 0.0, 0.0), xDeg * degToRad);
        const qy = new Quaternion().setFromAxisAngle(new Vector3(0.0, 1.0, 0.0), yDeg * degToRad);
        const qz = new Quaternion().setFromAxisAngle(new Vector3(0.0, 0.0, 1.0), zDeg * degToRad);
        return new Quaternion()
            .multiplyQuaternions(qz, new Quaternion().multiplyQuaternions(qx, qy))
            .normalize();
    }

    function readOrientationAttribute(primNode) {
        const xformOpOrder = normalizeToPlainArray(getAttribute(primNode, 'xformOpOrder'));
        const rotationOrder = Array.isArray(xformOpOrder) ? xformOpOrder : [];
        if (rotationOrder.length > 0) {
            const combined = new Quaternion();
            let found = false;
            for (const opName of rotationOrder) {
                let opRotation = null;
                if (opName === 'xformOp:orient') {
                    opRotation = readQuaternionAttribute(primNode, opName);
                } else if (opName === 'xformOp:rotateYXZ') {
                    opRotation = readRotateYXZAttribute(primNode, opName);
                }
                if (!opRotation) {
                    continue;
                }
                combined.multiplyQuaternions(opRotation, combined).normalize();
                found = true;
            }
            if (found) {
                return combined;
            }
        }
        return (
            readQuaternionAttribute(primNode, 'xformOp:orient')
            || readRotateYXZAttribute(primNode, 'xformOp:rotateYXZ')
        );
    }

    function readSpoolAxisLocal(primNode) {
        const axis =
            readVectorAttribute(primNode, 'spool:axisLocal')
            || readVectorAttribute(primNode, 'machine:axisLocal')
            || readVectorAttribute(primNode, 'physics:rotationAxis')
            || DEFAULT_PLANE_NORMAL;
        return normalizeSpoolAxisLocal(axis);
    }

    function effectiveInertiaAboutAxis(inertiaTensor, axisLocal) {
        if (!Array.isArray(inertiaTensor) || inertiaTensor.length < 3) {
            return null;
        }
        const axis = normalizeSpoolAxisLocal(axisLocal);
        const rows = inertiaTensor.map((row) => Array.isArray(row) ? row : []);
        const xx = Number(rows[0][0] ?? 0.0);
        const xy = Number(rows[0][1] ?? 0.0);
        const xz = Number(rows[0][2] ?? 0.0);
        const yx = Number(rows[1][0] ?? 0.0);
        const yy = Number(rows[1][1] ?? 0.0);
        const yz = Number(rows[1][2] ?? 0.0);
        const zx = Number(rows[2][0] ?? 0.0);
        const zy = Number(rows[2][1] ?? 0.0);
        const zz = Number(rows[2][2] ?? 0.0);
        const projectedX = (xx * axis.x) + (xy * axis.y) + (xz * axis.z);
        const projectedY = (yx * axis.x) + (yy * axis.y) + (yz * axis.z);
        const projectedZ = (zx * axis.x) + (zy * axis.y) + (zz * axis.z);
        const inertia =
            (axis.x * projectedX)
            + (axis.y * projectedY)
            + (axis.z * projectedZ);
        return Number.isFinite(inertia) ? inertia : null;
    }

    function addGravityIfDynamic(entityId, mass) {
        if (typeof mass === 'number' && Number.isFinite(mass) && mass > 0.0) {
            world.addComponent(entityId, new GravityAffectedComponent());
        }
    }

    function computeRigidBodyAggregateState(memberEntities) {
        const com = new Vector3(0.0, 0.0, 0.0);
        const linearVelocity = new Vector3(0.0, 0.0, 0.0);
        let totalMass = 0.0;

        for (const entityId of memberEntities) {
            const pos = world.getComponent(entityId, PositionComponent)?.pos;
            const mass = world.getComponent(entityId, MassComponent)?.mass ?? 0.0;
            const vel = world.getComponent(entityId, VelocityComponent)?.vel;
            if (!pos || !(mass > 0.0)) {
                continue;
            }
            com.add(pos, mass);
            if (vel) {
                linearVelocity.add(vel, mass);
            }
            totalMass += mass;
        }

        if (totalMass > 0.0) {
            com.scale(1.0 / totalMass);
            linearVelocity.scale(1.0 / totalMass);
        }

        let totalInertia = 0.0;
        for (const entityId of memberEntities) {
            const pos = world.getComponent(entityId, PositionComponent)?.pos;
            const mass = world.getComponent(entityId, MassComponent)?.mass ?? 0.0;
            const inertia = world.getComponent(entityId, MomentOfInertiaComponent)?.inertia ?? 0.0;
            if (!(mass > 0.0) || !pos) {
                continue;
            }
            const offset = pos.clone().subtract(com);
            totalInertia += inertia + (mass * offset.lengthSq());
        }

        if (!(totalInertia > 0.0)) {
            totalInertia = totalMass > 0.0 ? totalMass : 0.0;
        }

        return {
            position: com,
            velocity: linearVelocity,
            orientation: new Quaternion(),
            angularVelocity: new Vector3(0.0, 0.0, 0.0),
            mass: totalMass,
            inertia: totalInertia,
        };
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
        const existingPauseState = world.getResource('pauseState');
        const paused = typeof existingPauseState?.paused === 'boolean'
            ? existingPauseState.paused
            : false;
        world.setResource('simWidth', simWidth);
        world.setResource('simHeight', simHeight);
        world.setResource('pauseState', new PauseStateComponent(paused));
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
        const rigidBodyPrims = [];
        let extruderPrim = null;
        let extruderAuthoredPos = null;
        let extruderCenterPaths = [];
        let extruderTipOffset = new Vector3(0.0, 0.0, 0.0);
        let extruderColdEndOffset = null;

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

            // Detect rigid bodies either by explicit defType or a members rel.
            if (prim.type === 'definition' && (prim.defType === 'RigidBody' || prim.defType === 'RigidGroup')) {
                rigidBodyPrims.push(prim);
            } else {
                const maybeRigidBodyMembers = getRelationship(prim, 'rigidBody:members');
                const maybeRigidGroupMembers = getRelationship(prim, 'rigidGroup:members');
                if (
                    (maybeRigidBodyMembers && maybeRigidBodyMembers.length > 0)
                    || (maybeRigidGroupMembers && maybeRigidGroupMembers.length > 0)
                ) {
                    rigidBodyPrims.push(prim);
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
                    const tipPrim = getChild(prim, 'Tip') || getChild(prim, 'HotEnd');
                    const coldEndPrim = getChild(prim, 'ColdEnd') || getChild(prim, 'Bottom');
                    extruderTipOffset = readVectorAttribute(tipPrim, 'xformOp:translate') || new Vector3(0.0, 0.0, 0.0);
                    extruderColdEndOffset = readVectorAttribute(coldEndPrim, 'xformOp:translate');
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
                    const initialOrientation = readOrientationAttribute(prim) || new Quaternion();
                    const spoolAxisLocal = readSpoolAxisLocal(prim);

                    if (radius === null || mass === null || inertiaTensor === null || !angVelArr) {
                        console.warn(`Skipping Spool prim ${prim.name} due to missing attributes.`);
                        continue;
                    }
                    const inertia = effectiveInertiaAboutAxis(inertiaTensor, spoolAxisLocal);
                    const angVel = new Vector3(
                        Number(angVelArr[0] ?? 0.0),
                        Number(angVelArr[1] ?? 0.0),
                        Number(angVelArr[2] ?? 0.0),
                    );

                    world.addComponent(ent, new SpoolTagComponent());
                    const axisName = prim.name.slice(-1).toUpperCase();
                    world.addComponent(ent, new SpoolStateComponent(axisName, spoolAxisLocal, initialOrientation));
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
                    world.addComponent(ent, new RenderableComponent('cylinder', spoolColor));
                    world.addComponent(
                        ent,
                        new OrientationComponent(
                            initialOrientation.x,
                            initialOrientation.y,
                            initialOrientation.z,
                            initialOrientation.w,
                        ),
                    );
                    world.addComponent(ent, new EncoderComponent());
                    world.addComponent(ent, new AngularVelocityComponent(angVel.x, angVel.y, angVel.z));
                    world.addComponent(ent, new MomentOfInertiaComponent(inertia ?? 0.0));
                    world.addComponent(
                        ent,
                        new PrevFinalOrientationComponent(
                            initialOrientation.x,
                            initialOrientation.y,
                            initialOrientation.z,
                            initialOrientation.w,
                        ),
                    );
                    world.addComponent(ent, new PrevFinalPosComponent(pos.x, pos.y, pos.z));
                    if (restitution !== null) world.addComponent(ent, new RestitutionComponent(restitution));
                    if (friction !== null) world.addComponent(ent, new CoefficientOfFrictionComponent(friction));
                    if (getAttribute(prim, "cable:linkable")) {
                        world.addComponent(
                            ent,
                            new CableLinkComponent(
                                pos.x,
                                pos.y,
                                pos.z,
                                initialOrientation,
                                null,
                                spoolAxisLocal,
                            ),
                        );
                    }
                    nameToEntityId[primKey] = ent;
                } else if (tags.includes("Wheel")) {
                    const ent = world.createEntity();
                    world.addComponent(ent, new MachineTagComponent(machineId));
                    const radius = getAttribute(prim, "radius");
                    const mass = getAttribute(prim, "physics:mass");
                    const inertiaTensor = getAttribute(prim, "physics:inertiaTensor");
                    const velArr = getAttribute(prim, "physics:velocity");
                    const angVelArr = getAttribute(prim, "physics:angularVelocity");
                    const initialOrientation = readOrientationAttribute(prim) || new Quaternion();
                    const wheelAxisLocal = readSpoolAxisLocal(prim);

                    if (radius === null || mass === null || inertiaTensor === null || !angVelArr) {
                        console.warn(`Skipping Wheel prim ${prim.name} due to missing attributes.`);
                        continue;
                    }
                    const inertia = effectiveInertiaAboutAxis(inertiaTensor, wheelAxisLocal);
                    const angVel = new Vector3(
                        Number(angVelArr[0] ?? 0.0),
                        Number(angVelArr[1] ?? 0.0),
                        Number(angVelArr[2] ?? 0.0),
                    );

                    world.addComponent(ent, new SpoolStateComponent(null, wheelAxisLocal, initialOrientation));
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
                    const wheelColor = palette?.wheel ?? palette?.spool ?? color ?? '#a0a0a0';
                    world.addComponent(ent, new RenderableComponent('cylinder', wheelColor));
                    world.addComponent(
                        ent,
                        new OrientationComponent(
                            initialOrientation.x,
                            initialOrientation.y,
                            initialOrientation.z,
                            initialOrientation.w,
                        ),
                    );
                    world.addComponent(ent, new EncoderComponent());
                    world.addComponent(ent, new AngularVelocityComponent(angVel.x, angVel.y, angVel.z));
                    world.addComponent(ent, new MomentOfInertiaComponent(inertia ?? 0.0));
                    world.addComponent(
                        ent,
                        new PrevFinalOrientationComponent(
                            initialOrientation.x,
                            initialOrientation.y,
                            initialOrientation.z,
                            initialOrientation.w,
                        ),
                    );
                    world.addComponent(ent, new PrevFinalPosComponent(pos.x, pos.y, pos.z));
                    if (restitution !== null) world.addComponent(ent, new RestitutionComponent(restitution));
                    if (friction !== null) world.addComponent(ent, new CoefficientOfFrictionComponent(friction));
                    if (getAttribute(prim, "cable:linkable")) {
                        world.addComponent(
                            ent,
                            new CableLinkComponent(
                                pos.x,
                                pos.y,
                                pos.z,
                                initialOrientation,
                                null,
                                wheelAxisLocal,
                            ),
                        );
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
                    const initialOrientation = readOrientationAttribute(prim) || new Quaternion();

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
                        world.addComponent(
                            ent,
                            new OrientationComponent(
                                initialOrientation.x,
                                initialOrientation.y,
                                initialOrientation.z,
                                initialOrientation.w,
                            ),
                        );
                        world.addComponent(
                            ent,
                            new PrevFinalOrientationComponent(
                                initialOrientation.x,
                                initialOrientation.y,
                                initialOrientation.z,
                                initialOrientation.w,
                            ),
                        );
                        world.addComponent(
                            ent,
                            new AngularVelocityComponent(
                                Number(angVelArr[0] ?? 0.0),
                                Number(angVelArr[1] ?? 0.0),
                                Number(angVelArr[2] ?? 0.0),
                            ),
                        );
                    }
                    if (inertiaTensor !== null) {
                        const inertia = effectiveInertiaAboutAxis(inertiaTensor, DEFAULT_PLANE_NORMAL);
                        world.addComponent(ent, new MomentOfInertiaComponent(inertia ?? 0.0));
                    }
                    world.addComponent(ent, new PrevFinalPosComponent(pos.x, pos.y, pos.z));
                    if (restitution !== null) world.addComponent(ent, new RestitutionComponent(restitution));
                    if (friction !== null) world.addComponent(ent, new CoefficientOfFrictionComponent(friction));
                    if (getAttribute(prim, "cable:linkable")) {
                        world.addComponent(ent, new CableLinkComponent(pos.x, pos.y, pos.z, initialOrientation, DEFAULT_PLANE_NORMAL));
                    }
                    nameToEntityId[primKey] = ent;
                }
            }
        }

        // Build rigid bodies first (if any)
        const entityToRigidBody = {};
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
            if (!extruderPrim || !extruderAuthoredPos) {
                return null;
            }
            return extruderAuthoredPos.clone();
        })();
        const extruderCenterSourceOffsets = (() => {
            if (extruderCenterEntityIds.length === 0) {
                return [];
            }
            const averagePos = resolveAveragePosition(extruderCenterEntityIds);
            if (!averagePos) {
                return [];
            }
            return extruderCenterEntityIds
                .map((entityId) => {
                    const pos = world.getComponent(entityId, PositionComponent)?.pos;
                    return pos ? pos.clone().subtract(averagePos) : null;
                })
                .filter((offset) => offset instanceof Vector3);
        })();
        for (const prim of rigidBodyPrims) {
            const memberPaths = getRelationship(prim, 'rigidBody:members') || getRelationship(prim, 'rigidGroup:members');
            if (!memberPaths || memberPaths.length === 0) continue;
            const memberEntities = memberPaths
                .map(path => nameToEntityId[scopedKeyFromPath(path)])
                .filter(id => id !== undefined);
            if (memberEntities.length >= 2) {
                const bodyState = computeRigidBodyAggregateState(memberEntities);
                const bodyEnt = world.createEntity();
                const renderIndicesAttr = getAttribute(prim, 'rigidBody:renderIndices') ?? getAttribute(prim, 'rigidGroup:renderIndices');
                const renderSegments = parseRigidGroupRenderSegments(renderIndicesAttr);

                world.addComponent(bodyEnt, new MachineTagComponent(machineId));
                world.addComponent(bodyEnt, new PositionComponent(
                    bodyState.position.x,
                    bodyState.position.y,
                    bodyState.position.z,
                ));
                world.addComponent(bodyEnt, new VelocityComponent(
                    bodyState.velocity.x,
                    bodyState.velocity.y,
                    bodyState.velocity.z,
                ));
                world.addComponent(bodyEnt, new MassComponent(bodyState.mass));
                addGravityIfDynamic(bodyEnt, bodyState.mass);
                world.addComponent(bodyEnt, new RenderableComponent('line', palette?.rigidBody ?? palette?.rigidGroup ?? palette?.distanceConstraint ?? '#55ff88'));
                world.addComponent(bodyEnt, new OrientationComponent(
                    bodyState.orientation.x,
                    bodyState.orientation.y,
                    bodyState.orientation.z,
                    bodyState.orientation.w,
                ));
                world.addComponent(bodyEnt, new PrevFinalOrientationComponent(
                    bodyState.orientation.x,
                    bodyState.orientation.y,
                    bodyState.orientation.z,
                    bodyState.orientation.w,
                ));
                world.addComponent(bodyEnt, new AngularVelocityComponent(
                    bodyState.angularVelocity.x,
                    bodyState.angularVelocity.y,
                    bodyState.angularVelocity.z,
                ));
                world.addComponent(bodyEnt, new MomentOfInertiaComponent(bodyState.inertia));
                world.addComponent(bodyEnt, new PrevFinalPosComponent(
                    bodyState.position.x,
                    bodyState.position.y,
                    bodyState.position.z,
                ));
                world.addComponent(bodyEnt, new RigidBodyComponent(memberEntities, renderSegments));
                initializeRigidBodySyncState(world, bodyEnt);

                for (const entityId of memberEntities) {
                    const memberPos = world.getComponent(entityId, PositionComponent)?.pos || new Vector3(0.0, 0.0, 0.0);
                    const memberOrientation = world.getComponent(entityId, OrientationComponent)?.quaternion || new Quaternion();
                    const memberMass = world.getComponent(entityId, MassComponent)?.mass ?? 0.0;
                    const localPosition = memberPos.clone().subtract(bodyState.position);
                    const localOrientation = memberOrientation.clone().normalize();
                    world.addComponent(
                        entityId,
                        new RigidBodyMemberComponent(bodyEnt, localPosition, localOrientation, memberMass),
                    );

                    world.removeComponent(entityId, GravityAffectedComponent);
                    const massComponent = world.getComponent(entityId, MassComponent);
                    if (massComponent) {
                        massComponent.mass = 0.0;
                    }

                    const velocityComponent = world.getComponent(entityId, VelocityComponent);
                    if (velocityComponent?.vel) {
                        velocityComponent.vel.set(new Vector3(0.0, 0.0, 0.0));
                    }

                    entityToRigidBody[entityId] = bodyEnt;
                }
            }
        }

        // Process discovered DistancePhysicsJoints (skip those internal to a rigid body)
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
                    // Skip if both bodies belong to the same rigid body
                    if (!entityToRigidBody[entityA] || entityToRigidBody[entityA] !== entityToRigidBody[entityB]) {
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
            const damping = readNumericAttribute(prim, "cablePath:damping");

            const pathComp = new CablePathComponent(
              world,
              jointEntities,
              linkTypes ? [...linkTypes] : null,
              clockwise ? [...clockwise] : null,
              stiffness || Infinity,
              stored ? [...stored] : null,
              cableHalfWidth ?? 0.0,
              damping ?? 0.0
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
                if (!extruderComp.centerSourceOffsets || typeof extruderComp.centerSourceOffsets !== 'object') {
                    extruderComp.centerSourceOffsets = {};
                }
                if (!extruderComp.tipOffsets || typeof extruderComp.tipOffsets !== 'object') {
                    extruderComp.tipOffsets = {};
                }
                if (!extruderComp.coldEndOffsets || typeof extruderComp.coldEndOffsets !== 'object') {
                    extruderComp.coldEndOffsets = {};
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
                if (extruderCenterSourceOffsets.length > 0) {
                    extruderComp.centerSourceOffsets[machineId] = extruderCenterSourceOffsets.map((offset) => offset.clone());
                } else if (extruderComp.centerSourceOffsets[machineId]) {
                    delete extruderComp.centerSourceOffsets[machineId];
                }
                if (extruderTipOffset) {
                    extruderComp.tipOffsets[machineId] = extruderTipOffset.clone();
                } else if (extruderComp.tipOffsets[machineId]) {
                    delete extruderComp.tipOffsets[machineId];
                }
                if (extruderColdEndOffset) {
                    extruderComp.coldEndOffsets[machineId] = extruderColdEndOffset.clone();
                } else if (extruderComp.coldEndOffsets[machineId]) {
                    delete extruderComp.coldEndOffsets[machineId];
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
          world.registerSystem(new RigidBodySyncSystem());

          // 4. Update derived geometry and cable state
          world.registerSystem(new CableAttachmentUpdateSystem());
          world.registerSystem(new CableAttachmentCacheSystem());
          world.registerSystem(new CableSlackSystem());

          // 5. POSITIONAL SOLVERS: Correct predicted positions to satisfy constraints.
          world.registerSystem(new PBDCableConstraintSolver());
          world.registerSystem(new PBDResolveCableOverCorrections());

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

    if (!isRemote) {
        const extruderSystem = world.systems.find((system) => system instanceof ExtruderSystem);
        if (extruderSystem && typeof extruderSystem.update === 'function') {
            extruderSystem.update(world, 0);
        }
    }

    const renderSystem = world.getResource('renderSystem');
    if (renderSystem instanceof RenderSystem3D && typeof renderSystem.update === 'function') {
        renderSystem.update(world, 0);
    }
}
