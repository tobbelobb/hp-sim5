import { RenderSystem3D } from '../../src/js/cable_joints_3d/render_system_3d.js';
import {
    applyEntityPlan,
    buildEntityPlan,
    parseStage,
    readMachineSceneSpec,
    validateMachineSceneSpec,
} from './machineScenePipeline.js';
import { registerSceneSystems } from './sceneSystems.js';

export {
    applyEntityPlan,
    buildEntityPlan,
    parseStage,
    readMachineSceneSpec,
    validateMachineSceneSpec,
} from './machineScenePipeline.js';
export { registerSceneSystems } from './sceneSystems.js';

export function setupScene(world, stage, canvas, options = {}) {
    const existingRenderSystem = world.getResource('renderSystem');
    if (existingRenderSystem instanceof RenderSystem3D && !options.append) {
        existingRenderSystem.resetVisuals();
    }

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
    });
    applyEntityPlan(world, plan, canvas);
    registerSceneSystems(world, {
        canvas,
        mode: '3d',
        remote: Boolean(options.remote),
        ws: options.ws ?? null,
    });
}
