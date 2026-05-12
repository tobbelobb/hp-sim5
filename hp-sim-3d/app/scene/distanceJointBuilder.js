import { getAttribute, getRelationship } from '../../../src/js/usd/stage.js';
import {
    DistanceConstraintComponent,
    MachineTagComponent,
    RenderableComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';
import { scopedKeyFromPath } from './sceneNaming.js';

export function buildDistanceJoints(context, registry) {
    if (context.options.remote) {
        return [];
    }
    return [(world) => applyDistanceJoints(world, context, registry)];
}

function applyDistanceJoints(world, context, registry) {
    const palette = context.options.palette || null;
    const machineId = context.options.namespace || 'default';
    const namespace = context.options.namespace || null;

    for (const prim of registry.distanceJointPrims) {
        const body0PathRel = getRelationship(prim, "physics:body0");
        const body1PathRel = getRelationship(prim, "physics:body1");
        if (body0PathRel == null || body1PathRel == null || body0PathRel.length === 0 || body1PathRel.length === 0) {
            continue;
        }

        const entityA = registry.nameToEntityId.get(scopedKeyFromPath(namespace, context.sceneRootPath, body0PathRel[0]));
        const entityB = registry.nameToEntityId.get(scopedKeyFromPath(namespace, context.sceneRootPath, body1PathRel[0]));
        if (entityA == null || entityB == null) {
            continue;
        }

        const minDistance = getAttribute(prim, "physics:minDistance");
        const maxDistance = getAttribute(prim, "physics:maxDistance");
        if (minDistance === null || maxDistance === null || Math.abs(minDistance - maxDistance) >= 1e-6) {
            continue;
        }

        if (registry.entityToRigidBody.get(entityA) && registry.entityToRigidBody.get(entityA) === registry.entityToRigidBody.get(entityB)) {
            continue;
        }

        const constraintEntity = world.createEntity();
        world.addComponent(constraintEntity, new MachineTagComponent(machineId));
        world.addComponent(constraintEntity, new DistanceConstraintComponent(entityA, entityB, (minDistance + maxDistance) / 2.0, 0.0));
        world.addComponent(constraintEntity, new RenderableComponent('line', palette?.distanceConstraint ?? 'green'));
    }
}
