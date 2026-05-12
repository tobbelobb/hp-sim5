import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import { getAttribute, materialProperties } from "../../../src/js/usd/stage.js";
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
    RenderableComponent,
    MachineTagComponent,
} from "../../../src/js/cable_joints_3d/ecs.js";
import { CableLinkComponent } from '../../../src/js/cable_joints_3d/cable_joints_core.js';
import {
    SpoolTagComponent,
    SpoolStateComponent,
} from '../hangprinter_spools.js';
import { StepperMotorComponent } from '../hangprinter_stepper_motor.js';
import Quaternion from '../../../src/js/cable_joints_3d/quaternion.js';
import {
    effectiveInertiaAboutAxis,
    readNumericAttribute,
    readOrientationAttribute,
    readSpoolAxisLocal,
} from './usdValueReaders.js';
import { scopedKey } from './sceneNaming.js';

const DEFAULT_PLANE_NORMAL = new Vector3(0, 0, 1);
const DEFAULT_PINHOLE_RADIUS = 0.002;

export function buildMachineBodies(context, registry) {
    if (context.options.remote) {
        return [];
    }
    return registry.bodyPrims
        .map((prim) => buildMachineBody(context, registry, prim))
        .filter(Boolean);
}

function buildMachineBody(context, registry, prim) {
    const tags = getAttribute(prim, "ecs:tags") || [];
    const posArr = getAttribute(prim, "xformOp:translate");
    if (tags.length === 0 || !posArr) {
        return null;
    }

    if (tags.includes("Spool")) {
        return (world) => applySpool(world, context, registry, prim, posArr);
    }
    if (tags.includes("Wheel")) {
        return (world) => applyWheel(world, context, registry, prim, posArr);
    }
    if (tags.includes("Anchor")) {
        return (world) => applyAnchor(world, context, registry, prim, posArr);
    }
    if (tags.includes("Pinhole") || tags.includes("Eyelet") || tags.includes("Attachment")) {
        return (world) => applyPinhole(world, context, registry, prim, posArr);
    }
    return null;
}

function applySpool(world, context, registry, prim, posArr) {
    const { color, friction, restitution } = materialProperties(context.stage, prim);
    const palette = context.options.palette || null;
    const machineId = context.options.namespace || 'default';
    const namespace = context.options.namespace || null;
    const pos = new Vector3(posArr[0], posArr[1], posArr[2] || 0);
    const radius = getAttribute(prim, "radius");
    const mass = getAttribute(prim, "physics:mass");
    const inertiaTensor = getAttribute(prim, "physics:inertiaTensor");
    const velArr = getAttribute(prim, "physics:velocity");
    const angVelArr = getAttribute(prim, "physics:angularVelocity");
    const initialOrientation = readOrientationAttribute(prim) || new Quaternion();
    const spoolAxisLocal = readSpoolAxisLocal(prim);

    if (radius === null || mass === null || inertiaTensor === null || !angVelArr) {
        console.warn(`Skipping Spool prim ${prim.name} due to missing attributes.`);
        return;
    }

    const ent = world.createEntity();
    world.addComponent(ent, new MachineTagComponent(machineId));
    world.addComponent(ent, new SpoolTagComponent());
    world.addComponent(ent, new SpoolStateComponent(prim.name.slice(-1).toUpperCase(), spoolAxisLocal, initialOrientation));
    world.addComponent(ent, buildStepperMotorComponent(prim));
    addBodyKinematics(world, ent, pos, velArr, mass);
    world.addComponent(ent, new RadiusComponent(radius));
    addGravityIfDynamic(world, ent, mass);
    world.addComponent(ent, new RenderableComponent('cylinder', palette?.spool ?? color ?? '#a0a0a0'));
    addOrientationState(world, ent, initialOrientation);
    world.addComponent(ent, new EncoderComponent());
    world.addComponent(ent, vectorComponent(AngularVelocityComponent, angVelArr));
    world.addComponent(ent, new MomentOfInertiaComponent(effectiveInertiaAboutAxis(inertiaTensor, spoolAxisLocal) ?? 0.0));
    addMaterialComponents(world, ent, restitution, friction);
    if (getAttribute(prim, "cable:linkable")) {
        world.addComponent(ent, new CableLinkComponent(pos.x, pos.y, pos.z, initialOrientation, null, spoolAxisLocal));
    }
    registry.nameToEntityId.set(scopedKey(namespace, prim.name), ent);
}

function applyWheel(world, context, registry, prim, posArr) {
    const { color, friction, restitution } = materialProperties(context.stage, prim);
    const palette = context.options.palette || null;
    const machineId = context.options.namespace || 'default';
    const namespace = context.options.namespace || null;
    const pos = new Vector3(posArr[0], posArr[1], posArr[2] || 0);
    const radius = getAttribute(prim, "radius");
    const mass = getAttribute(prim, "physics:mass");
    const inertiaTensor = getAttribute(prim, "physics:inertiaTensor");
    const velArr = getAttribute(prim, "physics:velocity");
    const angVelArr = getAttribute(prim, "physics:angularVelocity");
    const height = Number(getAttribute(prim, "height"));
    const initialOrientation = readOrientationAttribute(prim) || new Quaternion();
    const wheelAxisLocal = readSpoolAxisLocal(prim);

    if (radius === null || mass === null || inertiaTensor === null || !angVelArr) {
        console.warn(`Skipping Wheel prim ${prim.name} due to missing attributes.`);
        return;
    }

    const ent = world.createEntity();
    world.addComponent(ent, new MachineTagComponent(machineId));
    world.addComponent(ent, new SpoolStateComponent(null, wheelAxisLocal, initialOrientation));
    addBodyKinematics(world, ent, pos, velArr, mass);
    world.addComponent(ent, new RadiusComponent(radius));
    addGravityIfDynamic(world, ent, mass);
    world.addComponent(ent, new RenderableComponent(
        'cylinder',
        palette?.wheel ?? palette?.spool ?? color ?? '#a0a0a0',
        { height: Number.isFinite(height) && height > 0 ? height : null },
    ));
    addOrientationState(world, ent, initialOrientation);
    world.addComponent(ent, new EncoderComponent());
    world.addComponent(ent, vectorComponent(AngularVelocityComponent, angVelArr));
    world.addComponent(ent, new MomentOfInertiaComponent(effectiveInertiaAboutAxis(inertiaTensor, wheelAxisLocal) ?? 0.0));
    addMaterialComponents(world, ent, restitution, friction);
    world.addComponent(ent, new CableLinkComponent(pos.x, pos.y, pos.z, initialOrientation, null, wheelAxisLocal));
    registry.nameToEntityId.set(scopedKey(namespace, prim.name), ent);
}

function applyAnchor(world, context, registry, prim, posArr) {
    const { color } = materialProperties(context.stage, prim);
    const palette = context.options.palette || null;
    const machineId = context.options.namespace || 'default';
    const namespace = context.options.namespace || null;
    const pos = new Vector3(posArr[0], posArr[1], posArr[2] || 0);
    const ent = world.createEntity();
    world.addComponent(ent, new MachineTagComponent(machineId));
    world.addComponent(ent, new PositionComponent(pos.x, pos.y, pos.z));
    world.addComponent(ent, new RadiusComponent(0.01));
    world.addComponent(ent, new MassComponent(-1.0));
    world.addComponent(ent, new RenderableComponent('circle', palette?.anchor ?? color ?? '#aaaaaa'));
    if (getAttribute(prim, "cable:linkable")) {
        world.addComponent(ent, new CableLinkComponent(pos.x, pos.y, pos.z, null, DEFAULT_PLANE_NORMAL));
    }
    registry.nameToEntityId.set(scopedKey(namespace, prim.name), ent);
}

function applyPinhole(world, context, registry, prim, posArr) {
    const { color, friction, restitution } = materialProperties(context.stage, prim);
    const palette = context.options.palette || null;
    const machineId = context.options.namespace || 'default';
    const namespace = context.options.namespace || null;
    const pos = new Vector3(posArr[0], posArr[1], posArr[2] || 0);
    const radius = getAttribute(prim, "radius");
    const mass = getAttribute(prim, "physics:mass");
    const inertiaTensor = getAttribute(prim, "physics:inertiaTensor");
    const velArr = getAttribute(prim, "physics:velocity");
    const angVelArr = getAttribute(prim, "physics:angularVelocity");
    const initialOrientation = readOrientationAttribute(prim) || new Quaternion();

    const ent = world.createEntity();
    world.addComponent(ent, new SpoolTagComponent());
    world.addComponent(ent, new MachineTagComponent(machineId));
    addBodyKinematics(world, ent, pos, velArr, mass);
    world.addComponent(ent, new RadiusComponent(radius ?? DEFAULT_PINHOLE_RADIUS));
    addGravityIfDynamic(world, ent, mass);
    world.addComponent(ent, new RenderableComponent('circle', palette?.pinhole ?? color ?? '#cccccc'));
    if (angVelArr !== null) {
        addOrientationState(world, ent, initialOrientation);
        world.addComponent(ent, vectorComponent(AngularVelocityComponent, angVelArr));
    }
    if (inertiaTensor !== null) {
        world.addComponent(ent, new MomentOfInertiaComponent(effectiveInertiaAboutAxis(inertiaTensor, DEFAULT_PLANE_NORMAL) ?? 0.0));
    }
    addMaterialComponents(world, ent, restitution, friction);
    if (getAttribute(prim, "cable:linkable")) {
        world.addComponent(ent, new CableLinkComponent(pos.x, pos.y, pos.z, initialOrientation, DEFAULT_PLANE_NORMAL));
    }
    registry.nameToEntityId.set(scopedKey(namespace, prim.name), ent);
}

function addBodyKinematics(world, ent, pos, velArr, mass) {
    world.addComponent(ent, new PositionComponent(pos.x, pos.y, pos.z));
    if (velArr !== null) {
        world.addComponent(ent, new VelocityComponent(velArr[0], velArr[1], velArr[2] || 0));
    } else {
        world.addComponent(ent, new VelocityComponent(0.0, 0.0, 0.0));
    }
    world.addComponent(ent, new MassComponent(mass));
    world.addComponent(ent, new PrevFinalPosComponent(pos.x, pos.y, pos.z));
}

function addOrientationState(world, ent, orientation) {
    world.addComponent(ent, new OrientationComponent(orientation.x, orientation.y, orientation.z, orientation.w));
    world.addComponent(ent, new PrevFinalOrientationComponent(orientation.x, orientation.y, orientation.z, orientation.w));
}

function addGravityIfDynamic(world, entityId, mass) {
    if (typeof mass === 'number' && Number.isFinite(mass) && mass > 0.0) {
        world.addComponent(entityId, new GravityAffectedComponent());
    }
}

function addMaterialComponents(world, ent, restitution, friction) {
    if (restitution !== null) world.addComponent(ent, new RestitutionComponent(restitution));
    if (friction !== null) world.addComponent(ent, new CoefficientOfFrictionComponent(friction));
}

function buildStepperMotorComponent(prim) {
    const stepperComponent = new StepperMotorComponent();
    const holdingTorque = readNumericAttribute(prim, "stepper:holdingTorque");
    const numPolePairs = readNumericAttribute(prim, "stepper:numPolePairs");
    const dampingCoeff = readNumericAttribute(prim, "stepper:dampingCoeff");
    const maxSpeedRad = readNumericAttribute(prim, "stepper:maxSpeedRad");
    if (holdingTorque !== null) stepperComponent.holdingTorque = holdingTorque;
    if (numPolePairs !== null) stepperComponent.numPolePairs = Math.round(numPolePairs);
    if (dampingCoeff !== null) stepperComponent.dampingCoeff = dampingCoeff;
    if (maxSpeedRad !== null) stepperComponent.maxSpeedRad = maxSpeedRad;
    return stepperComponent;
}

function vectorComponent(ComponentClass, values) {
    return new ComponentClass(
        Number(values[0] ?? 0.0),
        Number(values[1] ?? 0.0),
        Number(values[2] ?? 0.0),
    );
}
