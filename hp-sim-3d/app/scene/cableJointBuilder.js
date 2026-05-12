import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import { getAttribute, getRelationship } from '../../../src/js/usd/stage.js';
import { MachineTagComponent, RenderableComponent } from '../../../src/js/cable_joints_3d/ecs.js';
import {
    CableJointComponent,
    linecolor1,
} from '../../../src/js/cable_joints_3d/cable_joints_core.js';
import { scopedKey, scopedKeyFromPath } from './sceneNaming.js';

export function buildCableJoints(context, registry) {
    if (context.options.remote) {
        return [];
    }
    return [(world) => applyCableJoints(world, context, registry)];
}

function applyCableJoints(world, context, registry) {
    const palette = context.options.palette || null;
    const machineId = context.options.namespace || 'default';
    const namespace = context.options.namespace || null;

    for (const prim of registry.jointPrims) {
        const body0Path = getRelationship(prim, "physics:body0")[0];
        const body1Path = getRelationship(prim, "physics:body1")[0];
        const entityA = registry.nameToEntityId.get(scopedKeyFromPath(namespace, context.sceneRootPath, body0Path));
        const entityB = registry.nameToEntityId.get(scopedKeyFromPath(namespace, context.sceneRootPath, body1Path));
        const restLength = getAttribute(prim, "restLength");
        const attachAArr = getAttribute(prim, "localPos0");
        const attachBArr = getAttribute(prim, "localPos1");

        if (entityA === null || entityB === null || restLength === null || attachAArr === null || attachBArr === null) {
            console.warn(`Skipping CableJoint ${prim.name} due to missing data.`);
            console.log(`entityA: ${entityA}, entityB: ${entityB}, restLength: ${restLength}, attachAArr: ${attachAArr}, attachBarr: ${attachBArr}`);
            continue;
        }

        const joint = world.createEntity();
        world.addComponent(joint, new MachineTagComponent(machineId));
        world.addComponent(
            joint,
            CableJointComponent.fromLocal(
                world,
                entityA,
                entityB,
                restLength,
                new Vector3(attachAArr[0], attachAArr[1], attachAArr[2] || 0),
                new Vector3(attachBArr[0], attachBArr[1], attachBArr[2] || 0),
            ),
        );
        world.addComponent(joint, new RenderableComponent('line', palette?.cable ?? linecolor1));
        registry.jointEntityMap.set(scopedKey(namespace, prim.name), joint);
    }
}
