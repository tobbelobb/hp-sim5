import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import Quaternion from '../../../src/js/cable_joints_3d/quaternion.js';
import { getAttribute, getRelationship } from '../../../src/js/usd/stage.js';
import {
    AngularVelocityComponent,
    GravityAffectedComponent,
    MachineTagComponent,
    MassComponent,
    MomentOfInertiaComponent,
    OrientationComponent,
    PositionComponent,
    PrevFinalOrientationComponent,
    PrevFinalPosComponent,
    RenderableComponent,
    RigidBodyComponent,
    RigidBodyMemberComponent,
    VelocityComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';
import { initializeRigidBodySyncState } from '../../../src/js/cable_joints_3d/rigid_bodies.js';
import { parseRigidGroupRenderSegments } from './usdValueReaders.js';
import { scopedKeyFromPath } from './sceneNaming.js';
import {
    addMatrix3,
    normalizeInertiaTensor,
    parallelAxisTensor,
    transformInertiaTensorToWorld,
} from '../../../src/js/cable_joints_3d/inertia_tensor.js';

export function buildRigidBodies(context, registry) {
    if (context.options.remote) {
        return [];
    }
    return [(world) => applyRigidBodies(world, context, registry)];
}

function applyRigidBodies(world, context, registry) {
    const palette = context.options.palette || null;
    const machineId = context.options.namespace || 'default';
    const namespace = context.options.namespace || null;

    for (const prim of registry.rigidBodyPrims) {
        const memberPaths = getRelationship(prim, 'rigidBody:members') || getRelationship(prim, 'rigidGroup:members');
        if (!memberPaths || memberPaths.length === 0) continue;
        const memberEntities = memberPaths
            .map((path) => registry.nameToEntityId.get(scopedKeyFromPath(namespace, context.sceneRootPath, path)))
            .filter((id) => id !== undefined);
        if (memberEntities.length < 2) {
            continue;
        }

        const bodyState = computeRigidBodyAggregateState(world, memberEntities);
        const bodyEnt = world.createEntity();
        const renderIndicesAttr = getAttribute(prim, 'rigidBody:renderIndices') ?? getAttribute(prim, 'rigidGroup:renderIndices');
        const renderSegments = parseRigidGroupRenderSegments(renderIndicesAttr);

        world.addComponent(bodyEnt, new MachineTagComponent(machineId));
        world.addComponent(bodyEnt, new PositionComponent(bodyState.position.x, bodyState.position.y, bodyState.position.z));
        world.addComponent(bodyEnt, new VelocityComponent(bodyState.velocity.x, bodyState.velocity.y, bodyState.velocity.z));
        world.addComponent(bodyEnt, new MassComponent(bodyState.mass));
        addGravityIfDynamic(world, bodyEnt, bodyState.mass);
        world.addComponent(bodyEnt, new RenderableComponent('line', palette?.rigidBody ?? palette?.rigidGroup ?? palette?.distanceConstraint ?? '#55ff88'));
        world.addComponent(bodyEnt, new OrientationComponent(bodyState.orientation.x, bodyState.orientation.y, bodyState.orientation.z, bodyState.orientation.w));
        world.addComponent(bodyEnt, new PrevFinalOrientationComponent(bodyState.orientation.x, bodyState.orientation.y, bodyState.orientation.z, bodyState.orientation.w));
        world.addComponent(bodyEnt, new AngularVelocityComponent(bodyState.angularVelocity.x, bodyState.angularVelocity.y, bodyState.angularVelocity.z));
        world.addComponent(bodyEnt, new MomentOfInertiaComponent(bodyState.inertiaTensor));
        world.addComponent(bodyEnt, new PrevFinalPosComponent(bodyState.position.x, bodyState.position.y, bodyState.position.z));
        world.addComponent(bodyEnt, new RigidBodyComponent(memberEntities, renderSegments));
        initializeRigidBodySyncState(world, bodyEnt);

        for (const entityId of memberEntities) {
            const memberPos = world.getComponent(entityId, PositionComponent)?.pos || new Vector3(0.0, 0.0, 0.0);
            const memberOrientation = world.getComponent(entityId, OrientationComponent)?.quaternion || new Quaternion();
            const memberMass = world.getComponent(entityId, MassComponent)?.mass ?? 0.0;
            world.addComponent(
                entityId,
                new RigidBodyMemberComponent(
                    bodyEnt,
                    memberPos.clone().subtract(bodyState.position),
                    memberOrientation.clone().normalize(),
                    memberMass,
                ),
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
            registry.entityToRigidBody.set(entityId, bodyEnt);
        }
    }
}

function computeRigidBodyAggregateState(world, memberEntities) {
    const com = new Vector3(0.0, 0.0, 0.0);
    const linearVelocity = new Vector3(0.0, 0.0, 0.0);
    let totalMass = 0.0;

    for (const entityId of memberEntities) {
        const pos = world.getComponent(entityId, PositionComponent)?.pos;
        const mass = world.getComponent(entityId, MassComponent)?.mass ?? 0.0;
        const vel = world.getComponent(entityId, VelocityComponent)?.vel;
        if (!pos || !(mass > 0.0)) continue;
        com.add(pos, mass);
        if (vel) linearVelocity.add(vel, mass);
        totalMass += mass;
    }

    if (totalMass > 0.0) {
        com.scale(1.0 / totalMass);
        linearVelocity.scale(1.0 / totalMass);
    }

    let totalInertiaTensor = normalizeInertiaTensor(0.0);
    for (const entityId of memberEntities) {
        const pos = world.getComponent(entityId, PositionComponent)?.pos;
        const mass = world.getComponent(entityId, MassComponent)?.mass ?? 0.0;
        const memberInertia = world.getComponent(entityId, MomentOfInertiaComponent);
        const memberOrientation = world.getComponent(entityId, OrientationComponent)?.quaternion || new Quaternion();
        if (!(mass > 0.0) || !pos) continue;
        const offset = pos.clone().subtract(com);
        const memberTensor = memberInertia
            ? transformInertiaTensorToWorld(memberInertia.inertiaTensor, memberOrientation)
            : normalizeInertiaTensor(0.0);
        totalInertiaTensor = addMatrix3(totalInertiaTensor, memberTensor);
        totalInertiaTensor = addMatrix3(totalInertiaTensor, parallelAxisTensor(mass, offset));
    }

    const trace = totalInertiaTensor[0][0] + totalInertiaTensor[1][1] + totalInertiaTensor[2][2];
    if (!(trace > 0.0)) {
        totalInertiaTensor = normalizeInertiaTensor(totalMass > 0.0 ? totalMass : 0.0);
    }

    return {
        position: com,
        velocity: linearVelocity,
        orientation: new Quaternion(),
        angularVelocity: new Vector3(0.0, 0.0, 0.0),
        mass: totalMass,
        inertiaTensor: totalInertiaTensor,
    };
}

function addGravityIfDynamic(world, entityId, mass) {
    if (typeof mass === 'number' && Number.isFinite(mass) && mass > 0.0) {
        world.addComponent(entityId, new GravityAffectedComponent());
    }
}
