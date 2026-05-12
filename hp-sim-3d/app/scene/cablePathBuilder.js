import { getAttribute, getRelationship } from '../../../src/js/usd/stage.js';
import { MachineTagComponent } from '../../../src/js/cable_joints_3d/ecs.js';
import { CablePathComponent } from '../../../src/js/cable_joints_3d/cable_joints_core.js';
import { scopedKeyFromPath } from './sceneNaming.js';
import { readNumericAttribute } from './usdValueReaders.js';

export function buildCablePaths(context, registry) {
    if (context.options.remote) {
        return [];
    }
    return [(world) => applyCablePaths(world, context, registry)];
}

function applyCablePaths(world, context, registry) {
    const machineId = context.options.namespace || 'default';
    const namespace = context.options.namespace || null;

    for (const prim of registry.pathPrims) {
        const jointPaths = getRelationship(prim, "cablePath:joints");
        if (!jointPaths) continue;

        const jointEntities = jointPaths
            .map((path) => registry.jointEntityMap.get(scopedKeyFromPath(namespace, context.sceneRootPath, path)))
            .filter(Boolean);
        const pathComp = new CablePathComponent(
            world,
            jointEntities,
            getAttribute(prim, "cablePath:linkTypes") ? [...getAttribute(prim, "cablePath:linkTypes")] : null,
            getAttribute(prim, "cablePath:clockwise") ? [...getAttribute(prim, "cablePath:clockwise")] : null,
            getAttribute(prim, "cablePath:stiffness") || Infinity,
            getAttribute(prim, "cablePath:stored") ? [...getAttribute(prim, "cablePath:stored")] : null,
            getAttribute(prim, "cablePath:halfWidth") ?? 0.0,
            readNumericAttribute(prim, "cablePath:damping") ?? 0.0,
            readNumericAttribute(prim, "cablePath:solverIterations") ?? 1,
        );
        const cablePath = world.createEntity();
        world.addComponent(cablePath, pathComp);
        world.addComponent(cablePath, new MachineTagComponent(machineId));
    }
}
