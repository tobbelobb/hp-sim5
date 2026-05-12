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

export function setupScene(world, stage, canvas, options = {}) {
    const scenePrimPath = typeof options.scenePrimPath === 'string' && options.scenePrimPath.length > 0
        ? options.scenePrimPath
        : '/World/SlideprinterScene';
    const parsed = parseStage(stage);
    const spec = readMachineSceneSpec(parsed, scenePrimPath, options);
    const checked = validateMachineSceneSpec(spec);

    if (!checked.valid) {
        for (const warning of checked.warnings) {
            console.warn(warning);
        }
        return;
    }

    const namespace = typeof options.namespace === 'string' && options.namespace.length > 0
        ? options.namespace
        : null;
    const plan = buildEntityPlan(checked, {
        ...options,
        namespace,
        palette: options.palette || null,
        canvas,
    });
    applyEntityPlan(world, plan);
    registerSceneSystems(world, {
        canvas,
        mode: '3d',
        remote: Boolean(options.remote),
        ws: options.ws ?? null,
    });
}
