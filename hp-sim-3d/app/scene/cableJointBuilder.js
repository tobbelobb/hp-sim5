import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import { getAttribute, getRelationship } from '../../../src/js/usd/stage.js';
import { MachineTagComponent, RenderableComponent, SceneEntityInfoComponent } from '../../../src/js/cable_joints_3d/ecs.js';
import {
    CableJointComponent,
    linecolor1,
} from '../../../src/js/cable_joints_3d/cable_joints_core.js';
import { scopedKey, scopedKeyFromPath } from './sceneNaming.js';

export function buildCableJoints(context, registry) {
    if (context.options.remote) {
        return [];
    }
    return registry.jointPrims.map((prim) => buildCableJoint(context, registry, prim));
}

function buildCableJoint(context, registry, prim) {
    const palette = context.options.palette || null;
    const machineId = context.options.namespace || 'default';
    const namespace = context.options.namespace || null;
    const jointPath = `${context.sceneRootPath}/${prim.name}`;
    const resolveBodyKey = (relationship) => {
        const targets = getRelationship(prim, relationship);
        if (targets.length !== 1) {
            throw new Error(`${jointPath}: CableJoint requires exactly one ${relationship} target.`);
        }
        const key = scopedKeyFromPath(namespace, context.sceneRootPath, targets[0]);
        if (!registry.nameToEntityId.has(key)) {
            throw new Error(`${jointPath}: CableJoint ${relationship} target ${targets[0]} did not resolve to a simulation body. Check the target path and its ecs:tags.`);
        }
        return key;
    };
    const bodyAKey = resolveBodyKey("physics:body0");
    const bodyBKey = resolveBodyKey("physics:body1");
    const restLength = getAttribute(prim, "restLength");
    const attachAArr = getAttribute(prim, "localPos0");
    const attachBArr = getAttribute(prim, "localPos1");
    const missing = Object.entries({ restLength, localPos0: attachAArr, localPos1: attachBArr })
        .filter(([, value]) => value == null)
        .map(([attribute]) => attribute);
    if (missing.length > 0) {
        throw new Error(`${jointPath}: CableJoint is missing required attributes: ${missing.join(", ")}.`);
    }

    const jointKey = scopedKey(namespace, prim.name);
    registry.jointEntityMap.set(jointKey, null);
    return (world) => {
        const joint = world.createEntity();
        world.addComponent(joint, new MachineTagComponent(machineId));
        world.addComponent(joint, new SceneEntityInfoComponent(prim.name, ['CableJoint']));
        world.addComponent(
            joint,
            CableJointComponent.fromLocal(
                world,
                registry.nameToEntityId.get(bodyAKey),
                registry.nameToEntityId.get(bodyBKey),
                restLength,
                new Vector3(attachAArr[0], attachAArr[1], attachAArr[2] || 0),
                new Vector3(attachBArr[0], attachBArr[1], attachBArr[2] || 0),
            ),
        );
        world.addComponent(joint, new RenderableComponent('line', palette?.cable ?? linecolor1));
        registry.jointEntityMap.set(jointKey, joint);
    };
}
