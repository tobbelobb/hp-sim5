import {
    applyEntityPlan,
    buildEntityPlan,
    parseStage,
    readMachineSceneSpec,
    validateMachineSceneSpec,
} from './scene/machineScenePipeline.js';
import { registerSceneSystems } from './sceneSystems.js';

export {
    applyEntityPlan,
    buildEntityPlan,
    parseStage,
    readMachineSceneSpec,
    validateMachineSceneSpec,
} from './scene/machineScenePipeline.js';
export { registerSceneSystems } from './sceneSystems.js';

export function buildScenePlan(stage, options = {}) {
    const parsed = parseStage(stage);
    const spec = readMachineSceneSpec(parsed, options.scenePrimPath, options);
    const checked = validateMachineSceneSpec(spec);

    if (!checked.valid) {
        throw new Error(checked.warnings.join('\n'));
    }

    const namespace = typeof options.namespace === 'string' && options.namespace.length > 0
        ? options.namespace
        : null;
    return buildEntityPlan(checked, {
        ...options,
        namespace,
    });
}

export function setupScene(world, stage, canvas, options = {}) {
    applyEntityPlan(world, buildScenePlan(stage, { ...options, canvas }));
    registerSceneSystems(world, {
        canvas,
        mode: '3d',
        remote: Boolean(options.remote),
        ws: options.ws ?? null,
    });
}
