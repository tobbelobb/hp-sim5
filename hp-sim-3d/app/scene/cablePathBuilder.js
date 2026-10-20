import { getAttribute, getRelationship } from '../../../src/js/usd/stage.js';
import { MachineTagComponent, SceneEntityInfoComponent } from '../../../src/js/cable_joints_3d/ecs.js';
import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import { CablePathComponent, PinholeGateComponent } from '../../../src/js/cable_joints_3d/cable_joints_core.js';
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
            getAttribute(prim, "cablePath:stiffness") ?? Infinity,
            getAttribute(prim, "cablePath:stored") ? [...getAttribute(prim, "cablePath:stored")] : null,
            getAttribute(prim, "cablePath:halfWidth") ?? 0.0,
            readNumericAttribute(prim, "cablePath:damping") ?? 0.0,
            readNumericAttribute(prim, "cablePath:solverIterations") ?? 1,
        );
        for (let i = 1; i < pathComp.linkTypes.length - 1; i++) {
            if (pathComp.linkTypes[i] !== 'pinhole_gate') continue;
            const jointPrim = context.stage.GetPrimAtPath(jointPaths[i - 1]);
            const portPath = getRelationship(jointPrim, 'physics:body1')[0];
            const portPrim = context.stage.GetPrimAtPath(portPath);
            const entry = registry.nameToEntityId.get(scopedKeyFromPath(namespace, context.sceneRootPath, portPath));
            const gate = world.getComponent(entry, PinholeGateComponent);
            gate.visualRoute = (getAttribute(portPrim, 'pinholeGate:visualRoute') || []).map(p => new Vector3(...p));
            gate.visible = getAttribute(portPrim, 'pinholeGate:visible') !== false;
        }
        const cablePath = world.createEntity();
        world.addComponent(cablePath, pathComp);
        world.addComponent(cablePath, new MachineTagComponent(machineId));
        world.addComponent(cablePath, new SceneEntityInfoComponent(prim.name, ['CablePath']));
    }
}
