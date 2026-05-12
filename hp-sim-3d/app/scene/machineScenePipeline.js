import {
    createSceneReadContext,
    parseStage,
    readMachineSceneSpec,
    validateMachineSceneSpec,
} from './machineSceneSpec.js';
import { buildSceneResources } from './sceneResources.js';
import { buildMachineBodies } from './machineEntityBuilders.js';
import { buildRigidBodies } from './rigidBodyBuilder.js';
import { buildDistanceJoints } from './distanceJointBuilder.js';
import { buildCableJoints } from './cableJointBuilder.js';
import { buildCablePaths } from './cablePathBuilder.js';
import { buildExtruderBindings } from './extruderSceneBinding.js';
import { applyEntityPlan } from './entityPlanApplier.js';

export {
    applyEntityPlan,
    parseStage,
    readMachineSceneSpec,
    validateMachineSceneSpec,
};

export function buildEntityPlan(checkedSpec, options = {}) {
    const context = createSceneReadContext(checkedSpec, options);
    return {
        resources: buildSceneResources(context),
        entities: [
            ...buildMachineBodies(context),
            ...buildRigidBodies(context),
            ...buildDistanceJoints(context),
            ...buildCableJoints(context),
            ...buildCablePaths(context),
            ...buildExtruderBindings(context),
        ],
        postApply: [
            resetRemoteSpoolAxisMapping,
        ],
    };
}

function resetRemoteSpoolAxisMapping(world) {
    for (const system of world.systems) {
        if (typeof system.resetAxisMapping === 'function') {
            system.resetAxisMapping();
        }
    }
}
